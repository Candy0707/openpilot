from cereal import messaging
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

# 3.1 🚀 動態 TTA 介入極限 (依據車速平滑化退讓強度)
REC_ACCEL_V_HIGH    = 100.0 * CV.KPH_TO_MS  # 🚄 高速錨點
REC_ACCEL_V_LOW     = 0.0   * CV.KPH_TO_MS  # 🛑 低速錨點
REC_ACCEL_MAX_BRAKE = -1.0                  # ⚡ 高速時 TTA 允許的最大介入煞車
REC_ACCEL_MIN_BRAKE =  0.0                  # 🧊 低速時 TTA 允許的最大介入煞車

# 4. 軌跡掃描與意圖預測範圍
TRAJECTORY_HORIZON  = 6     # 🔭 危險預判：取 MPC 軌跡前 6 個點 (約 0.6 秒)
INTENT_LOOKAHEAD    = 3     # 🧠 意圖預判：在 6 個點中有 3 個點成立即觸發

# 5. 物理與標定預設常數
DEFAULT_T_FOLLOW    = 1.6   # 預設跟車秒數 (若未傳入 override 時使用)
TARGET_V_REL        = 0.6   # 🎯 TTA 目標速差 (m/s)：在退讓區內只要比前車慢即可

# 6. 訊號穩定與濾波參數
FILTER_ALPHA        = 0.2   # 📡 雷達原始數據平滑係數：數值越小越平滑，有效消除雷達速差雜訊 (0.0~1.0)
LEAD_LOST_TICKS     = 5     # 🔒 鎖定幀數：雷達丟失前車需連續滿 5 幀 (約0.25秒) 才判定無車

# 7. 非對稱濾波全域變數
EMA_ALPHA_ACCEL     = 0.4   # 🚀 放煞車比例
EMA_ALPHA_DECEL     = 0.8   # 🛑 採煞車比例

# 8. 系統偵錯開關
ACM_DEBUG           = True  # 📝 開關：是否輸出 cloudlog 偵錯日誌


class AdaptiveCoastingModule:
    """
    自適應滑行管理模組 (ACM)
    """

    def __init__(self):
        # 狀態機 A：記錄目前是否處於 ACM 介入滑行狀態
        self.acm_active = False
        # 狀態機 B：記錄目前是否處於「強烈起步/加速意圖」狀態
        self.intent_accelerating = False

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

    def update(
        self,
        sm: messaging.SubMaster,
        a_desired_trajectory: list[float],
        v_ego: float,
        t_follow_override: float,
    ) -> list[float]:
        class_name = self.__class__.__name__

        # 取得最新一幀雷達資料
        radar_state = sm['radarState']
        lead = radar_state.leadOne

        # 預先複製陣列，準備給最後統一覆寫使用
        result = list(a_desired_trajectory)

        # 給 LOG 用的預設變數
        tta = 0.0
        raw_a_calc = 0.0
        smooth_tta_a = 0.0
        dist_percent = 0.0
        dynamic_target_dist = 0.0
        dynamic_coast_end = 0.0
        dynamic_safe_dist = 0.0
        dynamic_tta_limit = REC_ACCEL_MAX_BRAKE
        fade_factor = 0.0
        d_rel = 0.0
        v_rel = 0.0

        # ==========================================
        # 🛡️ 統一攔截與日誌輸出中心
        # ==========================================
        def log_and_return(state_str: str, current_result: list[float], active=False, intent=False):
            self.acm_active = active
            self.intent_accelerating = intent

            # 防洗頻核心：只有當「狀態(退出原因或區域)」跟上一次不同時，才印出 LOG
            if ACM_DEBUG and (state_str != self.last_log_state or self.acm_active):
                cloudlog.debug(f"[{class_name}] 啟動: {self.acm_active} | 加速意圖: {self.intent_accelerating}")
                cloudlog.debug(f" ┣ 狀態: {state_str} (距離: {dist_percent*100:.1f}% | 邊界: {dynamic_coast_end*100:.1f}% | 防線: {dynamic_safe_dist*100:.1f}%)")
                cloudlog.debug(f" ┣ 物理: 目標距離: {dynamic_target_dist:.1f}m | 當前距離: {d_rel:.1f}m | 當前車速: {v_ego * CV.MS_TO_KPH:.1f}km/h | 相對速度: {v_rel:.1f}m/s" )
                cloudlog.debug(f" ┣ 運算: TTA極限: {dynamic_tta_limit:.2f} | 輸出TTA: {smooth_tta_a:.2f} | 煞車比例: {fade_factor*100:.0f}%")
                cloudlog.debug(f" ┗ 覆寫: {a_desired_trajectory[0]:.2f} -> {current_result[0]:.2f}")

                # 記憶這次的狀態
                self.last_log_state = state_str

            return current_result

        # ==========================================
        # 📡 雷達訊號預處理：前車鎖定 (Lead Lock) 與源頭濾波
        # ==========================================
        if lead.status:
            # 只要雷達有看到車，重置丟失計數器，維持鎖定狀態
            self.lead_lost_counter = 0
            self.has_lead_locked = True

            current_d_rel = lead.dRel
            current_v_rel = lead.vRel

            # EMA 速差濾波邏輯
            if not self.lead_status_prev:
                # 若上一幀無車，瞬間同步，避免數值從 0 緩慢爬升
                self.filtered_d_rel = current_d_rel
                self.filtered_v_rel = current_v_rel
            else:
                # 正常融合：20% 新數據 + 80% 歷史數據
                self.filtered_d_rel = (FILTER_ALPHA * current_d_rel) + ((1.0 - FILTER_ALPHA) * self.filtered_d_rel)
                self.filtered_v_rel = (FILTER_ALPHA * current_v_rel) + ((1.0 - FILTER_ALPHA) * self.filtered_v_rel)

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
            # 鎖定期間會自動沿用最後有效的快取數值

        has_lead = self.has_lead_locked

        # ==========================================
        # 1. 狀態計算與安全防護
        # ==========================================
        if has_lead:
            # 全部採用鎖定與源頭濾波後的平滑數值
            d_rel = self.last_valid_d_rel
            v_rel = self.last_valid_v_rel

            tf = t_follow_override if t_follow_override is not None else DEFAULT_T_FOLLOW

            # 動態目標緩衝空間
            dynamic_target_dist = max(v_ego * tf, 1.0)
            # 當前可用動態空間
            dynamic_d_rel = max(d_rel - STOP_DISTANCE, 0.0)
            # 動態空間剩餘百分比
            dist_percent = dynamic_d_rel / dynamic_target_dist

            # ------------------------------------------
            # 📏 雙邊界動態拓寬 (依據物理目標距離)
            # ------------------------------------------
            # 解決低速跟車時，固定百分比換算成物理空間太短的死穴
            ratio = (dynamic_target_dist - DYNAMIC_DIST_NEAR) / (DYNAMIC_DIST_FAR - DYNAMIC_DIST_NEAR)
            # 🛡️ 加入鉗制魔法 (Clamp)，強制鎖死在 0.0 ~ 1.0 之間，防止高速或極低速時數值溢出
            ratio = max(0.0, min(ratio, 1.0))

            # 線性插值計算動態警戒線
            dynamic_coast_end = COAST_END_PERCENT_NEAR - (ratio * (COAST_END_PERCENT_NEAR - COAST_END_PERCENT_FAR))
            # 線性插值計算動態防護線
            dynamic_safe_dist = SAFE_DIST_PERCENT_NEAR - (ratio * (SAFE_DIST_PERCENT_NEAR - SAFE_DIST_PERCENT_FAR))

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
                return log_and_return("🛑 強制退出(TTC防撞)", result, active=False, intent=False)

            # 原廠近期軌跡已經預測到緊急重煞，立刻退場保命
            if any(a < MPC_FALLBACK_ACCEL for a in recent_trajectory):
                return log_and_return("🛑 強制退出(原廠重煞)", result, active=False, intent=False)

            # ------------------------------------------
            # 🧠 狀態機 B：動態加速意圖鎖定
            # ------------------------------------------
            # 觸發條件：近期軌跡出現加速意圖 AND 前車正在遠離
            if sum(1 for a in recent_trajectory if a > 0.05) >= INTENT_LOOKAHEAD and v_rel > 0.05:
                self.intent_accelerating = True

            # 解除條件：近期軌跡出現減速意圖 OR 前車正在接近
            if sum(1 for a in recent_trajectory if a < -0.05) >= INTENT_LOOKAHEAD or v_rel < 0.05:
                self.intent_accelerating = False

            # 距離已經拉開到「動態滑行起點」
            if dist_percent >= dynamic_coast_end:
                self.intent_accelerating = False

            # 若系統鎖定在提速意圖，暫停 ACM 壓制，100% 放行原廠 MPC 確保起步與加速敏捷
            if self.intent_accelerating:
                return log_and_return("🛑 強制退出(加速意圖)", result, active=False, intent=True)

            # ------------------------------------------
            # 動態追蹤演算法：全時段連續 TTA 速度匹配
            # ------------------------------------------
            # 計算距離「動態死亡線 (dynamic_safe_dist)」還剩下多少真實物理空間
            safe_buffer_dist = max(dynamic_d_rel - (dynamic_target_dist * dynamic_safe_dist), 0.0)

            # TTA 計算與後續的煞車力道
            safe_v_rel = max(abs(v_rel), 1e-3)
            tta = safe_buffer_dist / safe_v_rel

            # 全時段套用 TTA 公式
            raw_a_calc = - (TARGET_V_REL - v_rel) / max(tta, 1.0)

            # 🚀 漸進比例魔法 (Fade-in)：使用動態上下界徹底消除跨界頓挫
            fade_factor = (dynamic_coast_end - dist_percent) / (dynamic_coast_end - dynamic_safe_dist)
            fade_factor = max(0.0, min(fade_factor, 1.0)) # 確保比例鎖死在 0~1 之間

            # ------------------------------------------
            # 🚀 動態 TTA 極限與連續速度匹配演算法
            # ------------------------------------------
            # 實作動態車速平滑化介入強度，根據 v_ego 線性插值計算 dynamic_tta_limit
            v_ratio = max(0.0, min((v_ego - REC_ACCEL_V_LOW) / (REC_ACCEL_V_HIGH - REC_ACCEL_V_LOW), 1.0))
            dynamic_tta_limit = REC_ACCEL_MIN_BRAKE + v_ratio * (REC_ACCEL_MAX_BRAKE - REC_ACCEL_MIN_BRAKE)

            # 限制 TTA 本身 (Pre-clamping)，確保 TTA 算出來的退讓力道不會在低速時過強
            smooth_tta_a = max(dynamic_tta_limit, raw_a_calc * fade_factor)

        else:
            self.intent_accelerating = False

        # ------------------------------------------
        # 🌟 ACM 狀態機進出判定
        # ------------------------------------------
        if not has_lead:
            self.acm_active = False
        else:
            # 【有車狀態】：依據距離遲滯區間判定
            if dist_percent >= EXIT_PERCENT:
                return log_and_return("⚪ 退出(距離過遠)", result, active=False, intent=False)
            elif dist_percent <= COAST_START_PERCENT:
                self.acm_active = True

        # ==========================================
        # 2. 統一軌跡處理、分區覆寫與輸出前濾波
        # ==========================================
        zone_str = ""

        # 檢查歷史陣列是否需要初始化 (長度不符或首次啟動)
        init_history = not self.last_a_target_array or len(self.last_a_target_array) != len(result)
        if init_history:
            self.last_a_target_array = [0.0] * len(result)

        for i in range(len(result)):
            a_target = result[i]

            # --- A. 區域邏輯處理 ---
            if not has_lead:
                # 【無車滑行邏輯】：抹平神經質微煞車
                zone_str = "🟢 無車狀態(執行抹平)"
                if COAST_MAX_BRAKE <= a_target < 0.0:
                    a_target = 0.0
            else:
                # 【有車分區邏輯】：依照安全距離百分比分段控制 (套用雙動態邊界)
                if dynamic_coast_end <= dist_percent < EXIT_PERCENT:
                    # 🟢 區域 A (動態邊界 ~ 100%)：單純滑行區
                    zone_str = "🟢 區域A(單純滑行)"
                    a_target = 0.0

                elif dynamic_safe_dist <= dist_percent < dynamic_coast_end:
                    # 🟡 區域 B (動態防線 ~ 動態邊界)：平滑退讓區
                    zone_str = "🟡 區域B(平滑退讓)"

                    # 🚀 終極放飛 MPC：
                    # 1. min(MPC, TTA_Clamped)：若 MPC 想給更重的煞車，系統會優先採用 MPC，確保安全下限。
                    # 2. 修改的是「當 MPC 煞車不足時，TTA 會補上的平滑介入量」。
                    combined_a = min(a_target, smooth_tta_a)

                    # 3. 確保不發生意外加速，徹底放飛 MPC 重煞
                    a_target = max(MIN_RECOVERY_ACCEL ,min(combined_a, MAX_RECOVERY_ACCEL))

                elif dist_percent < dynamic_safe_dist:
                    # 🟠 區域 C (0% ~ 動態防線)：危險防護區，聽從 MPC (故意全面放行，不設限制)
                    zone_str = "🟠 區域C(交接MPC)"

            # --- B. 輸出前平滑濾波處理 ---
            # 🚀 核心邏輯：若 MPC 想煞得比 ACM 重 (result[i] <= a_target)，絕對放行 MPC 命令，不套用濾波
            if result[i] > a_target:
                # 當前是由 ACM 主導，對 a_target 進行加速/減速非對稱濾波
                if not init_history:
                    if a_target > self.last_a_target_array[i]:
                        # 🟢 加速/放開煞車 (數值變大)：反應慢，套用 EMA_ALPHA_ACCEL
                        a_target = (EMA_ALPHA_ACCEL * a_target) + ((1.0 - EMA_ALPHA_ACCEL) * self.last_a_target_array[i])
                    else:
                        # 🔴 減速/踩重煞車 (數值變小)：反應快，套用 EMA_ALPHA_DECEL
                        a_target = (EMA_ALPHA_DECEL * a_target) + ((1.0 - EMA_ALPHA_DECEL) * self.last_a_target_array[i])
            else:
                # 🛡️ 絕對優先防線：MPC 想要更重煞車時，100% 絕對放行原廠訊號
                a_target = result[i]

            # 將最終處理完的數值，同步寫回歷史紀錄與輸出陣列
            self.last_a_target_array[i] = a_target
            result[i] = a_target

        # 正常跑到最後，也一律呼叫 log_and_return 處理日誌並回傳！
        return log_and_return(zone_str, result, active=self.acm_active, intent=self.intent_accelerating)
