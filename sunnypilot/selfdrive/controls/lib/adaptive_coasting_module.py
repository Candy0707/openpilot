import numpy as np
from cereal import messaging, custom
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE

from openpilot.common.constants import CV
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX

# ==========================================
# ⚙️ 全域變數定義區 (Global Configurations)
# ==========================================

# 1. 狀態機與邊界基準 (百分比)
COAST_START_PCT = 0.95      # 🟢 滑行起點 (95%)
DANGER_PCT      = 0.75      # 🔴 煞車終點 / MPC 交接點 (75%)
EXIT_PERCENT    = 1.00      # ⚪ 退出點 (100%)

# 2. 車速與動態邊界線性插值參數
V_LOW  = 20.0 * CV.KPH_TO_MS  # 🚗 低速判定基準 (20 km/h)
V_HIGH = 80.0 * CV.KPH_TO_MS  # 🏎️ 高速判定基準 (80 km/h)

SAFETY_PCT_LOW  = 0.75        # 低速時滑行線 (75%) -> 75%~95% 全滑行，無微煞車區
SAFETY_PCT_HIGH = 0.80        # 高速時滑行線 (80%) -> 80%~95% 滑行，75%~80% 微煞車
MIN_BRAKE_ZONE_M = 3.0        # 📏 舒適煞車物理底線 (滑行線 - 煞車線 > 3 公尺)

# 3. 加速度與 TTA 極限參數
MIN_RECOVERY_ACCEL  = ACCEL_MIN  # 煞車極限
MAX_RECOVERY_ACCEL  =  0.0       # 加速極限
MPC_FALLBACK_ACCEL  = -1.2       # 緊急重煞交接閾值
TARGET_V_REL        =  0.6       # 🎯 TTA 目標速差 (保留微小速差，滑順收尾)
TTA_MULTIPLIER      =  1.2       # 🚀 TTA 力道放大器
FADE_SPAN_RATIO     =  0.5       # 平滑過渡比例 (前 50% 漸進介入)

# 4. 意圖預測與訊號濾波參數
TRAJECTORY_HORIZON  = 6
INTENT_LOOKAHEAD    = 3
INTENT_V_LOW        =  0.0 * CV.KPH_TO_MS # 🛑 意圖判定低速錨點 (補回)
INTENT_V_HIGH       = 80.0 * CV.KPH_TO_MS # 🚄 意圖判定高速錨點 (補回)
INTENT_FRAMES_LOW   = 1
INTENT_FRAMES_HIGH  = 20
DEFAULT_T_FOLLOW    = 1.6
FILTER_ALPHA        = 0.2
LEAD_LOST_TICKS     = 5
EMA_ALPHA_ACCEL     = 0.4
EMA_ALPHA_DECEL     = 0.8

# 5. 車速動態打折參數 (Gain Scheduling)
DISCOUNT_V_ARR      = [0.0, 80.0]
DISCOUNT_RATIO_ARR  = [0.0, 1.0]

ACM_DEBUG = True

# 🟢 直接對接 custom.capnp 中定義的 Enum
AcmState = custom.LongitudinalPlanSP.AdaptiveCoastingModule.State


class AdaptiveCoastingModule:
    """
    自適應滑行管理模組 (ACM) - TTA 舒適煞車與解耦狀態機版
    """
    def __init__(self):
        self.intent_accelerating = False
        self.accel_intent_counter = 0

        self.filtered_d_rel = 0.0
        self.filtered_v_rel = 0.0
        self.lead_status_prev = False
        self.lead_lost_counter = 0
        self.has_lead_locked = False
        self.last_valid_d_rel = 0.0
        self.last_valid_v_rel = 0.0
        self.last_a_target_array = []
        self.last_log_state = ""

        # 提供給 longitudinal_planner 讀寫的實體變數 (CamelCase)
        self.active = False
        self.state = AcmState.noLead
        self.leadDist = 0.0
        self.targetDist = 0.0
        self.dynamicSafety = 0.0
        self.dynamicDanger = 0.0
        self.stockControl = 0.0
        self.distPercent = 0.0
        self.ttaAccelValue = 0.0
        self.ttaLimitValue = 0.0
        self.speedRatio = 1.0
        self.fadeFactor = 0.0
        self.mpcBlendRatio = 0.0
        self.mpcAccel = 0.0
        self.acmAccel = 0.0

    def update(self, sm: messaging.SubMaster, a_desired_trajectory: list[float], v_ego: float, t_follow_override: float) -> list[float]:
        # 1. 訊號預處理
        self._reset_frame_variables()
        self._update_radar_signals(sm['radarState'].leadOne)

        # 2. 物理與危險計算
        emergency_fallback = False
        emergency_str = ""
        if self.has_lead_locked:
            self._calc_dynamic_boundaries(v_ego, t_follow_override)
            emergency_fallback, emergency_str = self._check_emergencies(a_desired_trajectory, v_ego, t_follow_override)
            self._eval_acceleration_intent(a_desired_trajectory, v_ego)

            # 🟢 人類最舒適的 TTA 動態追蹤演算法
            self._calc_tta_and_blending(a_desired_trajectory[0], v_ego)
        else:
            self.intent_accelerating = False
            self.accel_intent_counter = 0

        # 3. 🚦 狀態機判定 (更新 self.state 與 self.active)
        self._determine_state_and_active(emergency_fallback)

        # 4. 軌跡派發與處理
        result, zone_str = self._process_trajectory(a_desired_trajectory, v_ego, emergency_str)

        return self._log_and_return(zone_str, result, v_ego, a_desired_trajectory[0])

    # ==========================================
    # 🧩 1. 狀態機轉移中心 (State Determination)
    # ==========================================

    def _determine_state_and_active(self, emergency_fallback):
        """集中處理所有的狀態切換邏輯，統一更新 state 與 active"""
        if not self.has_lead_locked:
            self.state = AcmState.noLead
            self.active = False
            return

        if emergency_fallback or self.distPercent < self.dynamicDanger:
            self.state = AcmState.takeover
        elif self.intent_accelerating:
            self.state = AcmState.smoothAccel
        elif self.dynamicDanger <= self.distPercent < self.dynamicSafety:
            self.state = AcmState.smoothDecel
        else:
            self.state = AcmState.coasting

        if self.distPercent >= EXIT_PERCENT or emergency_fallback:
            self.active = False
        elif self.distPercent <= COAST_START_PCT:
            self.active = True

    # ==========================================
    # 🧩 2. 獨立狀態機處理器 (State Handlers)
    # ==========================================

    def _dispatch_state_action(self, raw_mpc_a, v_ego, emergency_str):
        """將單一軌跡點派發給對應的狀態處理器"""
        if self.state == AcmState.noLead:
            return self._state_no_lead(raw_mpc_a)
        elif self.state == AcmState.takeover:
            return self._state_takeover(raw_mpc_a, emergency_str)
        elif self.state == AcmState.smoothAccel:
            return self._state_smooth_accel(raw_mpc_a)
        elif self.state == AcmState.smoothDecel:
            return self._state_smooth_decel(raw_mpc_a)
        elif self.state == AcmState.coasting:
            return self._state_coasting(raw_mpc_a, v_ego)

        return "", raw_mpc_a

    def _state_no_lead(self, raw_mpc_a):
        a_target = 0.0 if -0.4 <= raw_mpc_a < 0.0 else raw_mpc_a
        return "🟢 無車狀態(執行抹平)", a_target

    def _state_takeover(self, raw_mpc_a, emergency_str):
        zone_str = emergency_str if emergency_str else "🟠 交接MPC"
        return zone_str, raw_mpc_a

    def _state_smooth_accel(self, raw_mpc_a):
        return "🛑 加速意圖", raw_mpc_a

    def _state_smooth_decel(self, raw_mpc_a):
        return "🟡 平滑退讓", self.ttaLimitValue

    def _state_coasting(self, raw_mpc_a, v_ego):
        """🟢 單純滑行區：依據車速給予不同的滑行權限"""
        if self.distPercent >= EXIT_PERCENT:
            return "⚪ 跟車距離外", raw_mpc_a

        if v_ego < V_LOW:
            return "🟢 低速滑行(防點頭)", min(0.0, raw_mpc_a)
        else:
            return "🟢 高速滑行(純滑行)", 0.0

    # ==========================================
    # 🧩 3. 軌跡處理與濾波
    # ==========================================

    def _process_trajectory(self, a_desired_trajectory, v_ego, emergency_str):
        """將軌跡陣列送入狀態機並進行非對稱 EMA 濾波"""
        result = list(a_desired_trajectory)
        init_history = not self.last_a_target_array or len(self.last_a_target_array) != len(result)
        if init_history:
            self.last_a_target_array = [0.0] * len(result)

        log_zone_str = ""

        for i in range(len(result)):
            raw_mpc_a = result[i]

            # 將 raw_mpc_a 丟進狀態機派發器，取出最終決定的加速度
            zone_str, a_target = self._dispatch_state_action(raw_mpc_a, v_ego, emergency_str)

            # 最終安全鉗制與濾波
            a_target = np.clip(a_target, ACCEL_MIN, ACCEL_MAX)
            if not init_history:
                alpha = EMA_ALPHA_ACCEL if a_target > self.last_a_target_array[i] else EMA_ALPHA_DECEL
                a_target = (alpha * a_target) + ((1.0 - alpha) * self.last_a_target_array[i])

            if i == 0:
                log_zone_str = zone_str
                self.mpcAccel = raw_mpc_a
                self.acmAccel = a_target

            self.last_a_target_array[i] = a_target
            result[i] = a_target

        return result, log_zone_str

    # ==========================================
    # 🧩 4. 基礎訊號與物理邊界計算
    # ==========================================

    def _reset_frame_variables(self):
        self.ttaAccelValue = 0.0
        self.ttaLimitValue = 0.0
        self.distPercent = 0.0
        self.targetDist = 0.0
        self.dynamicSafety = 0.0
        self.dynamicDanger = 0.0
        self.stockControl = 0.0
        self.fadeFactor = 0.0
        self.mpcBlendRatio = 0.0
        self.speedRatio = 1.0

    def _update_radar_signals(self, lead):
        if lead.status:
            self.lead_lost_counter = 0
            self.has_lead_locked = True
            if not self.lead_status_prev:
                self.filtered_d_rel = lead.dRel
                self.filtered_v_rel = lead.vRel
            else:
                self.filtered_d_rel = (FILTER_ALPHA * lead.dRel) + ((1.0 - FILTER_ALPHA) * self.filtered_d_rel)
                self.filtered_v_rel = (FILTER_ALPHA * lead.vRel) + ((1.0 - FILTER_ALPHA) * self.filtered_v_rel)
            self.lead_status_prev = True
            self.last_valid_d_rel = self.filtered_d_rel
            self.last_valid_v_rel = self.filtered_v_rel
        else:
            if self.has_lead_locked:
                self.lead_lost_counter += 1
            if self.lead_lost_counter >= LEAD_LOST_TICKS:
                self.has_lead_locked = False
                self.lead_status_prev = False
                self.intent_accelerating = False
                self.accel_intent_counter = 0

    def _calc_dynamic_boundaries(self, v_ego, t_follow_override):
        """📏 以物理距離為目標，自動推算雙邊界與區域 B"""
        self.leadDist = self.last_valid_d_rel
        tf = t_follow_override if t_follow_override is not None else DEFAULT_T_FOLLOW

        self.targetDist = max(v_ego * tf, 1.0)
        dynamic_d_rel = max(self.leadDist - STOP_DISTANCE, 0.0)
        self.distPercent = dynamic_d_rel / self.targetDist

        v_ratio = np.clip((v_ego - V_LOW) / (V_HIGH - V_LOW), 0.0, 1.0)
        safety_pct = SAFETY_PCT_LOW + v_ratio * (SAFETY_PCT_HIGH - SAFETY_PCT_LOW)

        coast_start_m = self.targetDist * COAST_START_PCT
        danger_dist_m = self.targetDist * DANGER_PCT
        safety_dist_m = self.targetDist * safety_pct

        # 🛡️ 舒適物理底線：如果有微煞車區，保證其長度 > 3m
        if safety_pct > DANGER_PCT:
            if (safety_dist_m - danger_dist_m) < MIN_BRAKE_ZONE_M:
                safety_dist_m = danger_dist_m + MIN_BRAKE_ZONE_M

        safety_dist_m = min(safety_dist_m, coast_start_m - 0.1)

        self.dynamicSafety = safety_dist_m / self.targetDist
        self.dynamicDanger = danger_dist_m / self.targetDist
        self.stockControl = self.dynamicDanger * 0.8

    def _check_emergencies(self, a_desired_trajectory, v_ego, t_follow_override):
        tf = t_follow_override if t_follow_override is not None else DEFAULT_T_FOLLOW
        ttc = (self.leadDist / abs(self.last_valid_v_rel)) if self.last_valid_v_rel < -0.5 else 999.0
        if ttc < (tf * 1.2):
            return True, "🛑 強制退出(TTC防撞)"
        elif any(a < MPC_FALLBACK_ACCEL for a in a_desired_trajectory[:TRAJECTORY_HORIZON]):
            return True, "🛑 強制退出(原廠重煞)"
        return False, ""

    def _eval_acceleration_intent(self, a_desired_trajectory, v_ego):
        recent_traj = a_desired_trajectory[:TRAJECTORY_HORIZON]
        intent_v_ratio = np.clip((v_ego - INTENT_V_LOW) / (INTENT_V_HIGH - INTENT_V_LOW), 0.0, 1.0)
        dynamic_intent_frames = int(round(INTENT_FRAMES_LOW + intent_v_ratio * (INTENT_FRAMES_HIGH - INTENT_FRAMES_LOW)))

        moment_accel = sum(1 for a in recent_traj if a > 0.05) >= INTENT_LOOKAHEAD and self.last_valid_v_rel > 0.05
        moment_decel = sum(1 for a in recent_traj if a < -0.05) >= INTENT_LOOKAHEAD or self.last_valid_v_rel < 0.05

        self.accel_intent_counter = self.accel_intent_counter + 1 if moment_accel else 0

        if self.accel_intent_counter >= dynamic_intent_frames:
            self.intent_accelerating = True

        if moment_decel or self.distPercent >= self.dynamicSafety:
            self.intent_accelerating = False
            self.accel_intent_counter = 0

    def _calc_tta_and_blending(self, current_mpc_a, v_ego):
        """🧮 動態追蹤演算法：經典 TTA 速度匹配與 MPC 漸進融合"""
        dynamic_d_rel = max(self.leadDist - STOP_DISTANCE, 0.0)
        safe_buffer_dist = max(dynamic_d_rel - (self.targetDist * self.dynamicDanger), 0.1)

        safe_v_rel = max(abs(self.last_valid_v_rel), 1e-3)
        tta = safe_buffer_dist / safe_v_rel

        # 🟢 回歸經典的 TTA 公式 (分子正是速度差！)
        self.ttaAccelValue = - (TARGET_V_REL - self.last_valid_v_rel) / max(tta, 1.0)

        v_ego_kph = v_ego * CV.MS_TO_KPH
        self.speedRatio = float(np.interp(v_ego_kph, DISCOUNT_V_ARR, DISCOUNT_RATIO_ARR))
        self.ttaAccelValue *= TTA_MULTIPLIER * self.speedRatio

        fade_span = (self.dynamicSafety - self.dynamicDanger) * FADE_SPAN_RATIO
        self.fadeFactor = np.clip((self.dynamicSafety - self.distPercent) / max(fade_span, 1e-5), 0.0, 1.0)

        smooth_a = self.ttaAccelValue * self.fadeFactor
        self.ttaLimitValue = np.clip(smooth_a, MIN_RECOVERY_ACCEL, MAX_RECOVERY_ACCEL)

        if self.distPercent < self.dynamicSafety:
            blend_span = self.dynamicSafety - self.dynamicDanger
            self.mpcBlendRatio = np.clip((self.dynamicSafety - self.distPercent) / max(blend_span, 1e-5), 0.0, 1.0)
            if self.mpcBlendRatio > 0.0:
                self.ttaLimitValue = ((1.0 - self.mpcBlendRatio) * self.ttaLimitValue) + (self.mpcBlendRatio * current_mpc_a)

    def _log_and_return(self, state_str: str, current_result: list[float], v_ego: float, raw_mpc_a: float) -> list[float]:
        if ACM_DEBUG and (state_str != self.last_log_state or self.active):
            cloudlog.debug(
                f"[{self.__class__.__name__}] 啟動:{self.active} | 狀態:{state_str} | 加速意圖:{self.intent_accelerating} | "
                f"剩餘距離:{self.distPercent*100:.1f}% | 退讓位置:{self.dynamicSafety*100:.1f}% | 煞車位置:{self.dynamicDanger*100:.1f}% | "
                f"車速:{v_ego * CV.MS_TO_KPH:.1f} | 速差:{self.last_valid_v_rel:.1f} | "
                f"TTA計算煞車:{self.ttaAccelValue:.2f} | 最終覆寫:{raw_mpc_a:.2f} -> {current_result[0]:.2f}"
            )
            self.last_log_state = state_str
        return current_result