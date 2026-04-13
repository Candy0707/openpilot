from cereal import messaging
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE

# ==========================================
# ⚙️ 全域變數定義區 (Global Configurations)
# ==========================================

# 1. 距離與狀態機閾值 (百分比)
ZERO_ACCEL_PERCENT  = 0.10  # 🛑 零加速鎖定線：跌破 10% (極限微距) 時，徹底沒收油門權限防暴衝
SAFE_DIST_PERCENT   = 0.75  # 🚨 絕對安全底線：跌破 75% 理想距離時，ACM 完全退場，交還給原生 MPC 重煞保命
COAST_START_PERCENT = 0.95  # 🟢 進入點：距離小於 95% 時，ACM 狀態機啟動，準備介入滑行邏輯
COAST_END_PERCENT   = 0.85  # 🟡 警戒線：距離小於 85% 時結束純滑行，進入「動態微煞車」把距離拉回 85%
EXIT_PERCENT        = 1.00  # ⚪ 退出點：距離拉開大於 100% 時，ACM 徹底休眠
# 遲滯區 (0.95 ~ 1.00)：兩條件皆不成立時維持現有狀態，刻意避免邊界震盪

# 2. 加速度動作極限變數 (單位: m/s²)
COAST_MAX_BRAKE     = -0.4  # 🌊 滑行極限：在 85%~100% 區間，MPC 煞車輕於此值就強制歸零 (純滑行)
MIN_RECOVERY_ACCEL  = -1.0  # 🛡️ 最小煞車極限：75%~85% 區間強制限縮的最大煞車力道，壓制神經質急煞
MAX_RECOVERY_ACCEL  =  0.2  # 🐢 緩加速極限：前車加速時限制補油門力道，確保提速比前車慢以拉開距離
MPC_FALLBACK_ACCEL  = -1.2  # 💣 危險判定閾值：近期軌跡點需要重煞時立刻轉交 MPC

# 3. 軌跡掃描與意圖預測範圍
TRAJECTORY_HORIZON  = 6     # 🔭 危險預判：取 MPC 軌跡前 6 個點 (約 0.6 秒) 預判是否有緊急重煞
INTENT_LOOKAHEAD    = 3     # 🧠 意圖預判：在 6 個點中有 3 個點成立即觸發 (過半數表決防震盪)

# 4. 物理與標定預設常數
DEFAULT_T_FOLLOW    = 1.6   # 預設跟車秒數，當外部未傳入 t_follow_override 時使用
TARGET_V_REL        = 0.6   # 🎯 TTA 目標速差 (m/s)：在退讓區內，只要比前車慢即可，讓距離自然拉開

# 5. 系統偵錯開關
ACM_DEBUG           = True  # 📝 開關：是否輸出 cloudlog 偵錯日誌


class AdaptiveCoastingModule:
    """
    自適應滑行管理模組 (ACM)
    """

    def __init__(self):
        # 狀態機 A：記錄目前是否處於 ACM 介入滑行狀態，避免在邊界值反覆橫跳造成頓挫
        self.acm_active = False
        # 狀態機 B：記錄目前是否處於「強烈起步/加速意圖」狀態 (暫停 ACM，確保敏捷性)
        self.intent_accelerating = False
        # 記憶上一次的「狀態字串」(包含強制退出或區域)，只要狀態改變就印出
        self.last_log_state = ""

    def update(
        self,
        sm: messaging.SubMaster,
        a_desired_trajectory: list[float],
        v_ego: float,
        t_follow_override: float,
    ) -> list[float]:
        class_name = self.__class__.__name__

        # 從 SubMaster 取得最新一幀的雷達狀態資料
        radar_state = sm['radarState']
        lead = radar_state.leadOne

        # 預先複製陣列，準備給最後統一覆寫使用
        result = list(a_desired_trajectory)

        # 準備給 LOG 使用的預設變數，確保無車或提早退出時也能印出 0.0，維持格式統一
        tta = 0.0
        raw_a_calc = 0.0
        dist_percent = 0.0
        d_rel = 0.0
        v_rel = 0.0
        target_dist = 0.0

        # ==========================================
        # 🛡️ 統一攔截與日誌輸出中心 (Universal Logger & Return)
        # ==========================================
        def log_and_return(state_str: str, current_result: list[float], active=False, intent=False):
            # 寫入狀態機
            self.acm_active = active
            self.intent_accelerating = intent

            # 防洗頻核心：只有當「狀態(退出原因或區域)」跟上一次不同時，才印出 LOG
            if ACM_DEBUG and state_str != self.last_log_state:
                # 無論是提早退出還是正常覆寫，全部套用這套 100% 統一的格式！
                cloudlog.debug(f"[{class_name}] 啟動:{self.acm_active} 加速意圖:{self.intent_accelerating} | "
                               f"{state_str} (距離:{dist_percent*100:.1f}%) | "
                               f"目標距離:{target_dist:.1f}m 當前距離:{d_rel:.1f}m 相對速度:{v_rel:.2f}m/s | "
                               f"TTA:{tta:.2f}s raw:{raw_a_calc:.2f} | "
                               f"覆寫: {a_desired_trajectory[0]:.2f} -> {current_result[0]:.2f}")

                # 記憶這次的狀態
                self.last_log_state = state_str

            return current_result

        # ==========================================
        # 1. 狀態計算與安全防護 (提早退出區)
        # ==========================================
        if lead.status:
            d_rel = lead.dRel
            v_rel = lead.vRel

            # 理想距離 = 車速 × 跟車秒數，並確保絕對不能低於靜止安全間距 (STOP_DISTANCE 同步 MPC)
            tf = t_follow_override if t_follow_override is not None else DEFAULT_T_FOLLOW
            target_dist = max(v_ego * tf, STOP_DISTANCE)
            dist_percent = d_rel / target_dist

            # 統一擷取近期軌跡 (供危險與意圖預判使用)
            recent_trajectory = a_desired_trajectory[:TRAJECTORY_HORIZON]

            # ------------------------------------------
            # 🛡️ 防護 A：動態 TTC 預警與 MPC 原生重煞防護
            # ------------------------------------------
            # 計算 TTC (碰撞時間)：只在逼近時計算，遠離時設為 999.0 安全值
            ttc = (d_rel / abs(v_rel)) if v_rel < -0.5 else 999.0

            # 動態 TTC 閾值：將跟車秒數放大 1.2 倍作為防護底線，提早應對鬼切
            dynamic_ttc_threshold = tf * 1.2

            # 若預計碰撞時間太短，立刻退場保命
            if ttc < dynamic_ttc_threshold:
                return log_and_return(f"🛑 強制退出(TTC防撞 {ttc:.1f}s)", result, active=False, intent=False)

            # 原廠近期軌跡已經預測到緊急重煞，立刻退場保命
            if any(a < MPC_FALLBACK_ACCEL for a in recent_trajectory):
                return log_and_return("🛑 強制退出(原廠重煞保命)", result, active=False, intent=False)

            # ------------------------------------------
            # 🧠 狀態機 B：動態加速意圖鎖定
            # ------------------------------------------
            # 觸發條件：近期軌跡出現加速意圖 AND 前車正在遠離 (v_rel > 0.0)
            if sum(1 for a in recent_trajectory if a > 0.05) >= INTENT_LOOKAHEAD and v_rel > 0.0:
                self.intent_accelerating = True

            # 解除條件：近期軌跡出現減速意圖 OR 前車正在接近 (v_rel < 0.0)
            if sum(1 for a in recent_trajectory if a < -0.05) >= INTENT_LOOKAHEAD or v_rel < 0.0:
                self.intent_accelerating = False

            # 距離已經拉開到滑行起點 (85%)
            if dist_percent >= COAST_END_PERCENT:
                self.intent_accelerating = False

            # 若系統鎖定在提速意圖，暫停 ACM 壓制，100% 放行原廠 MPC 確保起步與加速敏捷
            if self.intent_accelerating:
                return log_and_return("🛑 強制退出(加速意圖鎖定中)", result, active=False, intent=True)

            # ------------------------------------------
            # 動態追蹤演算法：全時段連續 TTA 速度匹配
            # ------------------------------------------
            # 計算距離「75% 死亡線」還剩下多少真實物理空間
            safe_buffer_dist = max(d_rel - (target_dist * SAFE_DIST_PERCENT), 0.0)

            # 🛡️ 數學防護盾：用 1e-3 墊底，徹底避開 ZeroDivisionError 當機！
            safe_v_rel = max(abs(v_rel), 1e-3)
            tta = safe_buffer_dist / safe_v_rel

            # 核心修復：全時段套用 TTA 公式
            raw_a_calc = - (TARGET_V_REL - v_rel) / max(tta, 1.0)

            # 防護 C：若 TTA 算出的所需減速度過大，交還 MPC 保命
            if raw_a_calc < MPC_FALLBACK_ACCEL:
                return log_and_return(f"🛑 強制退出(TTA極限保命 {raw_a_calc:.2f})", result, active=False, intent=False)

        else:
            # 確保無車狀態下不會有加速意圖殘留
            self.intent_accelerating = False

        # ------------------------------------------
        # 🌟 ACM 狀態機進出判定 (Hysteresis & 無車區塊)
        # ------------------------------------------
        if lead.status:
            # 【有車狀態】：依據距離遲滯區間判定
            if dist_percent >= EXIT_PERCENT:
                return log_and_return("⚪ 退出(跟車距離>100%)", result, active=False, intent=False)
            elif dist_percent <= COAST_START_PERCENT:
                self.acm_active = True
            else:
                if not self.acm_active:
                    return log_and_return("⚪ 退出(還未達到95%門檻)", result, active=False, intent=False)
        else:
            # 【無車狀態】：抹平的神經質微煞車
            self.acm_active = any(COAST_MAX_BRAKE <= a < 0.0 for a in result)
            if not self.acm_active:
                return log_and_return("⚪ 退出(前方無車且無須抹平煞車)", result, active=False, intent=False)

        # ==========================================
        # 2. 統一軌跡處理與分區覆寫 (單一輸出區塊)
        # ==========================================
        zone_str = ""

        for i in range(len(result)):
            a_target = result[i]

            if not lead.status:
                # 【無車滑行邏輯】：抹平 E2E 模型的神經質微煞車
                zone_str = "🟢 無車狀態(執行抹平)"
                if COAST_MAX_BRAKE <= a_target < 0.0:
                    a_target = 0.0
            else:
                # 【有車分區邏輯】：依照安全距離百分比分段控制
                if COAST_END_PERCENT <= dist_percent < EXIT_PERCENT:
                    # 🟢 區域 A (85% ~ 100%)：連續動態滑行區
                    zone_str = "🟢 區域A(單純滑行)"
                    a_target = 0

                elif SAFE_DIST_PERCENT <= dist_percent < COAST_END_PERCENT:
                    # 🟡 區域 B (75% ~ 85%)：平滑退讓區 (連續融合版)
                    zone_str = "🟡 區域B(平滑退讓)"
                    a_target = raw_a_calc

                elif ZERO_ACCEL_PERCENT <= dist_percent < SAFE_DIST_PERCENT:
                    # 區域 C (10% ~ 75%)：絕對危險區，保留原生急煞指令
                    zone_str = "🟠 區域C(危急防護)"

                elif dist_percent < ZERO_ACCEL_PERCENT:
                    # 🔴 區域 D (< 10%)：極限防暴衝區
                    zone_str = "🔴 區域D(預防加速)"
                    a_target = min(a_target, 0.0)

            # 將處理完的數值寫回陣列
            result[i] = a_target

        # 正常跑到最後，也一律呼叫 log_and_return 處理日誌並回傳！
        return log_and_return(zone_str, result, active=self.acm_active, intent=self.intent_accelerating)