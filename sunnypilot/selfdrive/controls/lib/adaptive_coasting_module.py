import numpy as np
from cereal import messaging, custom
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE

from openpilot.common.constants import CV
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX

# ==========================================
# ⚙️ 全域變數定義區 (Global Configurations)
# ==========================================

# 1. 距離與狀態機閾值 (百分比)
COAST_START_PERCENT    = 0.95  # 🟢 進入點：距離小於 95% 時，ACM 狀態機啟動，準備介入滑行邏輯
EXIT_PERCENT           = 1.00  # ⚪ 退出點：距離拉開大於 100% 時，ACM 徹底休眠
# 遲滯區 (0.95 ~ 1.00)：兩條件皆不成立時維持現有狀態，刻意避免邊界震盪

# 2. 動態邊界變數 (依據物理目標距離線性插值)
COAST_END_PERCENT_FAR  = 0.85  # 🟡 高速警戒線：維持 85% 結束純滑行，保留長滑行區
COAST_END_PERCENT_NEAR = 0.90  # 🟡 低速警戒線：提早至 90% 結束純滑行，增加退讓緩衝
SAFE_DIST_PERCENT_FAR  = 0.75  # 🟠 高速防護線：維持 75% 進入原廠交接區
SAFE_DIST_PERCENT_NEAR = 0.70  # 🟠 低速防護線：下推至 70% 進入原廠交接區，最大化低速平滑空間
DYNAMIC_DIST_FAR       = 40.0  # 📏 動態拓寬上限 (公尺)，套用 FAR 邊界
DYNAMIC_DIST_NEAR      = 15.0  # 📏 動態拓寬下限 (公尺)，套用 NEAR 邊界

# 3. 加速度動作極限變數 (單位: m/s²)
COAST_MAX_BRAKE     = -0.4       # 🌊 滑行極限：在滑行區間，MPC 煞車輕於此值就強制歸零 (純滑行)
MIN_RECOVERY_ACCEL  = ACCEL_MIN  # 🛡️ 煞車極限：強制限制平滑退讓區(區域 B)的煞車力道
MAX_RECOVERY_ACCEL  =  0.0       # 🐢 加速極限：強制限制平滑退讓區(區域 B)的加速力道
MPC_FALLBACK_ACCEL  = -1.2       # 💣 危險判定閾值：近期軌跡點需要重煞時立刻轉交 MPC
TTA_MULTIPLIER      =  1.2       # 🚀 TTA 力道放大器：將計算出的 TTA 力道 放大 1.2 倍
TTA_FADE_SPAN_RATIO =  0.5       # 🎛️ TTA 漸進比例：在動態邊界內的前 50% 區間達到 100% 介入強度

# 4. 軌跡掃描與意圖預測範圍
TRAJECTORY_HORIZON  = 6     # 🔭 危險預判：取 MPC 軌跡前 6 個點 (約 0.6 秒)
INTENT_LOOKAHEAD    = 3     # 🧠 意圖預判：在 6 個點中有 3 個點成立即觸發

INTENT_V_LOW        =  0.0 * CV.KPH_TO_MS # 🛑 意圖判定低速錨點：0 km/h
INTENT_V_HIGH       = 80.0 * CV.KPH_TO_MS # 🚄 意圖判定高速錨點：80 km/h
INTENT_FRAMES_LOW   = 1                   # 🧊 低速所需連續幀數 (0km/h 時 1 幀即觸發)
INTENT_FRAMES_HIGH  = 20                  # ⚡ 高速所需連續幀數 (80km/h 時需連續 20 幀)

# 5. 物理與標定預設常數
DEFAULT_T_FOLLOW    = 1.6   # 預設跟車秒數 (若未傳入 override 時使用)
TARGET_V_REL        = 0.6   # 🎯 TTA 目標速差 (m/s)：在退讓區內只要比前車慢即可

# 6. 訊號穩定與濾波參數
FILTER_ALPHA        = 0.2   # 📡 雷達原始數據平滑係數：數值越小越平滑，有效消除雷達速差雜訊 (0.0~1.0)
LEAD_LOST_TICKS     = 5     # 🔒 鎖定幀數：雷達丟失前車需連續滿 5 幀 (約0.25秒) 才判定無車

# 7. 非對稱濾波全域變數
EMA_ALPHA_ACCEL     = 0.4   # 🚀 放煞車/加速比例 (數值小，反應慢)
EMA_ALPHA_DECEL     = 0.8   # 🛑 踩煞車/減速比例 (數值大，反應快)

# 8. 遠距加速抑制參數 (起步防暴衝)
EXCESS_DIST_ARR     = [0.0, 0.10, 0.20, 0.40]
EXCESS_RATIO_ARR    = [0.2, 0.5, 0.8, 1.0]

# 9. TTA 車速動態打折參數 (Gain Scheduling)
TTA_DISCOUNT_V_ARR     = [0.0, 80.0]    # 🚄 車速節點 (km/h)
TTA_DISCOUNT_RATIO_ARR = [0.0, 1.0]     # 📉 對應打折比例 (0km/h=0.0完全歸零，80km/h=1.0不打折)

# 10. 系統偵錯開關
ACM_DEBUG           = True  # 📝 開關：是否輸出 cloudlog 偵錯日誌


# 🟢 直接對接 custom.capnp 中定義的 Enum
AcmState = custom.LongitudinalPlanSP.AdaptiveCoastingModule.State


class AdaptiveCoastingModule:
    """
    自適應滑行管理模組 (ACM) - 解耦重構版
    """

    def __init__(self):
        # 狀態機 B：記錄目前是否處於「強烈起步/加速意圖」狀態
        self.intent_accelerating = False
        # ⏱️ 加速意圖連續幀數計數器
        self.accel_intent_counter = 0

        # 📡 訊號穩定器記憶變數
        self.filtered_d_rel = 0.0       # EMA 濾波後的距離
        self.filtered_v_rel = 0.0       # EMA 濾波後的速差
        self.lead_status_prev = False   # 記憶上一幀是否有車
        self.lead_lost_counter = 0      # 丟失前車的倒數計時器
        self.has_lead_locked = False    # 最終輸出的「穩態前車有無」標記
        self.last_valid_d_rel = 0.0     # 最後一次有效的物理距離
        self.last_valid_v_rel = 0.0     # 最後一次有效的速差

        # 🚀 記憶上一幀的最終輸出陣列，專供「非對稱濾波」使用
        self.last_a_target_array = []
        # 記憶上一次的「狀態字串」，只要狀態改變就印出
        self.last_log_state = ""

        # ==========================================
        # 📦 提供給 longitudinal_planner 與 Cap'n Proto 讀寫的實體變數 (CamelCase)
        # ==========================================
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
        """主控迴圈：作為管線的目錄，調用各個私有方法進行處理"""

        # 1. 重置瞬時變數並更新雷達濾波訊號
        self._reset_frame_variables()
        self._update_radar_signals(sm['radarState'].leadOne)

        emergency_fallback = False
        emergency_str = ""

        if self.has_lead_locked:
            # 2. 核心運算：計算動態邊界、檢查防撞、預測意圖、計算 TTA 與融合
            self._calc_dynamic_boundaries(v_ego, t_follow_override)
            emergency_fallback, emergency_str = self._check_emergencies(a_desired_trajectory, v_ego, t_follow_override)
            self._eval_acceleration_intent(a_desired_trajectory, v_ego)
            self._calc_tta_and_blending(a_desired_trajectory[0], v_ego)
        else:
            self.intent_accelerating = False
            self.accel_intent_counter = 0

        # 3. 狀態更新與軌跡分區處理
        self._update_acm_active_status(emergency_fallback)
        result, zone_str = self._process_trajectory(a_desired_trajectory, emergency_fallback, emergency_str)

        # 4. 日誌輸出與回傳
        return self._log_and_return(zone_str, result, v_ego, a_desired_trajectory[0])

    # ==========================================
    # 🧩 私有方法 (Private Methods) 解耦具體邏輯
    # ==========================================

    def _reset_frame_variables(self):
        """預設重置當前幀的計算變數 (避免舊資料殘留)"""
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
        """📡 雷達訊號預處理：前車鎖定 (Lead Lock) 與源頭 EMA 濾波"""
        if lead.status:
            # 只要雷達有看到車，重置丟失計數器，維持鎖定狀態
            self.lead_lost_counter = 0
            self.has_lead_locked = True

            # EMA 速差濾波邏輯
            if not self.lead_status_prev:
                # 若上一幀無車，瞬間同步，避免數值從 0 緩慢爬升
                self.filtered_d_rel = lead.dRel
                self.filtered_v_rel = lead.vRel
            else:
                # 正常融合：20% 新數據 + 80% 歷史數據
                self.filtered_d_rel = (FILTER_ALPHA * lead.dRel) + ((1.0 - FILTER_ALPHA) * self.filtered_d_rel)
                self.filtered_v_rel = (FILTER_ALPHA * lead.vRel) + ((1.0 - FILTER_ALPHA) * self.filtered_v_rel)

            self.lead_status_prev = True

            # 💾 寫入快取，供後續所有邏輯與閃爍時使用
            self.last_valid_d_rel = self.filtered_d_rel
            self.last_valid_v_rel = self.filtered_v_rel
        else:
            # 雷達沒看到車，啟動丟失倒數
            if self.has_lead_locked:
                self.lead_lost_counter += 1
            # 滿 5 幀依然無車，才判定前車消失
            if self.lead_lost_counter >= LEAD_LOST_TICKS:
                self.has_lead_locked = False
                self.lead_status_prev = False
                self.intent_accelerating = False
                self.accel_intent_counter = 0  # 丟失前車時重置計數器

    def _calc_dynamic_boundaries(self, v_ego, t_follow_override):
        """📏 雙邊界動態拓寬 (依據物理目標距離) 與距離計算"""
        # 全部採用鎖定與源頭濾波後的平滑數值
        self.leadDist = self.last_valid_d_rel
        tf = t_follow_override if t_follow_override is not None else DEFAULT_T_FOLLOW

        # 動態目標緩衝空間
        self.targetDist = max(v_ego * tf, 1.0)
        # 當前可用動態空間
        dynamic_d_rel = max(self.leadDist - STOP_DISTANCE, 0.0)
        # 動態空間剩餘百分比
        self.distPercent = dynamic_d_rel / self.targetDist

        # 解決低速跟車時，固定百分比換算成物理空間太短的死穴
        # 🛡️ 加入鉗制魔法 (Clamp)，強制鎖死在 0.0 ~ 1.0 之間，防止高速或極低速時數值溢出
        ratio = np.clip((self.targetDist - DYNAMIC_DIST_NEAR) / (DYNAMIC_DIST_FAR - DYNAMIC_DIST_NEAR), 0.0, 1.0)

        # 線性插值計算動態警戒線
        self.dynamicSafety = COAST_END_PERCENT_NEAR - (ratio * (COAST_END_PERCENT_NEAR - COAST_END_PERCENT_FAR))
        # 線性插值計算動態防護線
        self.dynamicDanger = SAFE_DIST_PERCENT_NEAR - (ratio * (SAFE_DIST_PERCENT_NEAR - SAFE_DIST_PERCENT_FAR))
        # 🔴 原廠完全接管控制線 (通常設在危險線的 80%)
        self.stockControl = self.dynamicDanger * 0.8

    def _check_emergencies(self, a_desired_trajectory, v_ego, t_follow_override):
        """🛡️ 防護 A：動態 TTC 預警與 MPC 原生重煞防護"""
        tf = t_follow_override if t_follow_override is not None else DEFAULT_T_FOLLOW
        # 計算 TTC (碰撞時間)：只在逼近時計算，遠離時設為 999.0 安全值
        ttc = (self.leadDist / abs(self.last_valid_v_rel)) if self.last_valid_v_rel < -0.5 else 999.0

        # 動態 TTC 閾值：將跟車秒數放大 1.2 倍作為防護底線，提早應對鬼切
        if ttc < (tf * 1.2):
            return True, "🛑 強制退出(TTC防撞)"
        elif any(a < MPC_FALLBACK_ACCEL for a in a_desired_trajectory[:TRAJECTORY_HORIZON]):
            return True, "🛑 強制退出(原廠重煞)"
        return False, ""

    def _eval_acceleration_intent(self, a_desired_trajectory, v_ego):
        """🧠 狀態機 B：動態加速意圖預測鎖定"""
        recent_traj = a_desired_trajectory[:TRAJECTORY_HORIZON]

        # 1. 依據車速線性插值計算目標幀數 (0~80 km/h -> 1~20 幀)
        intent_v_ratio = np.clip((v_ego - INTENT_V_LOW) / (INTENT_V_HIGH - INTENT_V_LOW), 0.0, 1.0)
        dynamic_intent_frames = int(round(INTENT_FRAMES_LOW + intent_v_ratio * (INTENT_FRAMES_HIGH - INTENT_FRAMES_LOW)))

        # 2. 判斷當下這一幀是否具備「瞬間加速/減速條件」
        moment_accel = sum(1 for a in recent_traj if a > 0.05) >= INTENT_LOOKAHEAD and self.last_valid_v_rel > 0.05
        moment_decel = sum(1 for a in recent_traj if a < -0.05) >= INTENT_LOOKAHEAD or self.last_valid_v_rel < 0.05

        # 3. 連續幀數累積計數 (一旦中斷就歸零，確保是真正的連續)
        self.accel_intent_counter = self.accel_intent_counter + 1 if moment_accel else 0

        # 4. 狀態機觸發與解除
        # 【觸發條件】：連續滿足動態幀數
        if self.accel_intent_counter >= dynamic_intent_frames:
            self.intent_accelerating = True

        # 【解除條件】：出現減速徵兆或距離拉開，立刻解除以保安全
        if moment_decel or self.distPercent >= self.dynamicSafety:
            self.intent_accelerating = False
            self.accel_intent_counter = 0

    def _calc_tta_and_blending(self, current_mpc_a, v_ego):
        """🧮 動態追蹤演算法：全時段連續 TTA 速度匹配與 MPC 漸進融合"""
        dynamic_d_rel = max(self.leadDist - STOP_DISTANCE, 0.0)
        # 計算距離「動態死亡線 (self.dynamicDanger)」還剩下多少真實物理空間
        safe_buffer_dist = max(dynamic_d_rel - (self.targetDist * self.dynamicDanger), 0.0)

        # TTA 計算
        safe_v_rel = max(abs(self.last_valid_v_rel), 1e-3)
        tta = safe_buffer_dist / safe_v_rel

        # 原始 TTA 公式計算出的理論煞車力道
        self.ttaAccelValue = - (TARGET_V_REL - self.last_valid_v_rel) / max(tta, 1.0)

        # 🌟 依據車速進行線性插值打折 (Gain Scheduling)
        # 速度從 0 km/h ~ 80 km/h，對應比例 1.0 ~ 0.0 (速度越慢力道越大)
        v_ego_kph = v_ego * CV.MS_TO_KPH
        self.speedRatio = float(np.interp(v_ego_kph, TTA_DISCOUNT_V_ARR, TTA_DISCOUNT_RATIO_ARR))

        # 套用 TTA 放大器與車速打折比例
        self.ttaAccelValue *= TTA_MULTIPLIER * self.speedRatio

        # 🚀 漸進比例魔法 (Fade-in)：使用動態上下界徹底消除跨界頓挫
        # 功能說明：根據 TTA_FADE_SPAN_RATIO 設定，提早讓煞車比例達到 1.0，確保進入區域 B 也能及早建立安全阻力。
        fade_span = (self.dynamicSafety - self.dynamicDanger) * TTA_FADE_SPAN_RATIO
        self.fadeFactor = np.clip((self.dynamicSafety - self.distPercent) / max(fade_span, 1e-5), 0.0, 1.0)

        # 限制 TTA 本身算出來的退讓力道
        smooth_tta_a = self.ttaAccelValue * self.fadeFactor
        self.ttaLimitValue = np.clip(smooth_tta_a, MIN_RECOVERY_ACCEL, MAX_RECOVERY_ACCEL)

        # 🌟 全區域 B 比例式過度 (MPC 漸進介入防線)
        if self.distPercent < self.dynamicSafety:
            # 1. 算出區域 B 的總長度跨度 (例如 85% - 75% = 10% 的緩坡)
            blend_span = self.dynamicSafety - self.dynamicDanger
            # 2. 計算你在這個緩坡上走了多遠 (0.0 -> 1.0)
            self.mpcBlendRatio = np.clip((self.dynamicSafety - self.distPercent) / max(blend_span, 1e-5), 0.0, 1.0)

            # 3. 🛡️ 安全防線：只有當原廠煞得比我們深時，才依照比例把原廠力道揉進來！
            if self.mpcBlendRatio > 0.0:
                self.ttaLimitValue = ((1.0 - self.mpcBlendRatio) * self.ttaLimitValue) + (self.mpcBlendRatio * current_mpc_a)

    def _update_acm_active_status(self, emergency_fallback):
        """ACM 狀態機進出判定 (處理遲滯區間 Hysteresis)"""
        if not self.has_lead_locked:
            self.active = False
        else:
            # 【有車狀態】：依據距離遲滯區間判定 (將緊急狀態也納入解除條件)
            if self.distPercent >= EXIT_PERCENT or emergency_fallback:
                self.active = False
                self.state = AcmState.takeover if emergency_fallback else AcmState.coasting
            elif self.distPercent <= COAST_START_PERCENT:
                self.active = True

    def _process_trajectory(self, a_desired_trajectory, emergency_fallback, emergency_str):
        """統一軌跡處理、分區覆寫與非對稱 EMA 濾波"""
        result = list(a_desired_trajectory)

        # 檢查歷史陣列是否需要初始化 (長度不符或首次啟動)
        init_history = not self.last_a_target_array or len(self.last_a_target_array) != len(result)
        if init_history:
            self.last_a_target_array = [0.0] * len(result)

        log_zone_str = ""

        for i in range(len(result)):
            raw_mpc_a = result[i]
            a_target = raw_mpc_a
            zone_str = ""

            # --- A. 區域邏輯處理 ---
            if not self.has_lead_locked:
                # 【無車滑行邏輯】：抹平神經質微煞車
                zone_str = "🟢 無車狀態(執行抹平)"
                if COAST_MAX_BRAKE <= a_target < 0.0: a_target = 0.0
            else:
                # 🛑 緊急防護退場 (取代提早 return，無條件交還 MPC 並讓下方濾波器持續更新)
                if emergency_fallback:
                    zone_str, a_target = emergency_str, raw_mpc_a
                # ⚪ 跟車距離外
                elif self.distPercent >= EXIT_PERCENT:
                    zone_str, a_target = "⚪ 跟車距離外", raw_mpc_a
                # 🛑 加速意圖
                elif self.intent_accelerating:
                    zone_str, a_target = "🛑 加速意圖", raw_mpc_a
                # 🟢 區域 A (動態邊界 ~ 100%)：單純滑行區
                elif self.dynamicSafety <= self.distPercent < EXIT_PERCENT:
                    zone_str, a_target = "🟢 區域A(單純滑行)", 0.0
                # 🟡 區域 B (動態防線 ~ 動態邊界)：平滑退讓區
                elif self.dynamicDanger <= self.distPercent < self.dynamicSafety:
                    zone_str, a_target = "🟡 區域B(平滑退讓)", self.ttaLimitValue
                # 🟠 區域 C (0% ~ 動態防線)：危險防護區，聽從 MPC (全面放行，不設限制)
                elif self.distPercent < self.dynamicDanger:
                    zone_str, a_target = "🟠 區域C(交接MPC)", raw_mpc_a

            # --- B. 最終安全鉗制 (Clamp)，確保不超出物理加速度極限 ---
            a_target = np.clip(a_target, ACCEL_MIN, ACCEL_MAX)

            # --- C. 輸出前平滑濾波處理 (非對稱 EMA) ---
            if not init_history:
                # 比較當下算出的 a_target 與「上一幀的歷史數值」來判斷是加速還是減速
                alpha = EMA_ALPHA_ACCEL if a_target > self.last_a_target_array[i] else EMA_ALPHA_DECEL
                # 🟢 放開煞車/加速 (數值變大)：反應慢
                # 🔴 踩深煞車/減速 (數值變小)：反應快
                a_target = (alpha * a_target) + ((1.0 - alpha) * self.last_a_target_array[i])

            # 擷取在第 0 個點把 zone_str 存起來，避免被後面 16 個預測點蓋掉
            if i == 0:
                log_zone_str = zone_str
                self._update_state_enum(emergency_fallback)

                # 寫入單點的加速度比較供發布
                self.mpcAccel = raw_mpc_a
                self.acmAccel = a_target

            # 將最終處理完的數值，同步寫回歷史紀錄與輸出陣列
            self.last_a_target_array[i] = a_target
            result[i] = a_target

        return result, log_zone_str

    def _update_state_enum(self, emergency_fallback):
        """🚦 根據當前狀態更新 Enum，直接對接 custom.capnp"""
        if not self.has_lead_locked:
            self.state = AcmState.noLead
        elif emergency_fallback or self.distPercent < self.stockControl:
            self.state = AcmState.takeover
        elif self.intent_accelerating:
            self.state = AcmState.smoothAccel
        elif self.dynamicDanger <= self.distPercent < self.dynamicSafety:
            self.state = AcmState.smoothDecel
        else:
            self.state = AcmState.coasting

    def _log_and_return(self, state_str: str, current_result: list[float], v_ego: float, raw_mpc_a: float) -> list[float]:
        """📝 防洗頻日誌輸出中心"""
        # 只有當「狀態(退出原因或區域)」跟上一次不同時，才印出 LOG
        if ACM_DEBUG and (state_str != self.last_log_state or self.active):
            cloudlog.debug(
                f"[{self.__class__.__name__}] 啟動:{self.active} | 狀態:{state_str} | 加速意圖:{self.intent_accelerating} | "
                f"剩餘距離:{self.distPercent*100:.1f}% | 減速距離:{self.dynamicSafety*100:.1f}% | 緊急距離:{self.dynamicDanger*100:.1f}% | "
                f"當前車速:{v_ego * CV.MS_TO_KPH:.1f}km/h | 速差:{self.last_valid_v_rel:.1f}m/s | "
                f"目標距離:{self.targetDist:.1f}m | 前車距離:{self.leadDist:.1f}m | "
                f"TTA極限:{self.ttaAccelValue:.2f} | 車速打折:{self.speedRatio*100:.0f}% | TTA輸出:{self.ttaLimitValue:.2f} | TTA煞車比例:{self.fadeFactor*100:.0f}% | MPC 融合比例:{self.mpcBlendRatio*100:.0f}% | "
                f"覆寫:{raw_mpc_a:.2f} -> {current_result[0]:.2f}"
            )
            self.last_log_state = state_str

        return current_result