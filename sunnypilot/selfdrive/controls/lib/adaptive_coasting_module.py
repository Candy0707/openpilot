import numpy as np
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

# 9. 系統偵錯開關
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

        # 給 LOG 與全域運算用的預設變數
        tta = 0.0                   # ⏱️ 預計到達緩衝區的時間 (Time To Arrival)
        raw_a_calc = 0.0            # 🧮 原始 TTA 公式計算出的理論煞車力道 (未經限幅)
        smooth_tta_a = 0.0          # 🧊 最終套用漸進比例與極限限制後的平滑微煞車力道
        dist_percent = 0.0          # 📏 目前剩餘可用距離佔「目標跟車距離」的百分比
        dynamic_target_dist = 0.0   # 🎯 依據車速與跟車秒數，動態計算出的物理目標跟車距離 (公尺)
        dynamic_coast_end = 0.0     # 🟡 動態微煞區起點 (區域 B 進入點，通常在 85%~90%)
        dynamic_safe_dist = 0.0     # 🟠 動態危險交接線 (區域 C 進入點，通常在 70%~75%)
        fade_factor = 0.0           # 🪄 漸進魔法比例係數 (0.0 ~ 1.0)，確保微煞力道平滑介入
        d_rel = 0.0                 # 🚗 經過濾波處理後的與前車相對距離 (公尺)
        v_rel = 0.0                 # 💨 經過濾波處理後的與前車相對速度 (m/s，負值代表逼近中)
        mpc_blend_ratio = 0.0       # 🌟 10% 比例式融合的漸進比例 (0.0 ~ 1.0)

        # ==========================================
        # 🛡️ 統一攔截與日誌輸出中心
        # ==========================================
        def log_and_return(state_str: str, current_result: list[float], active=False, intent=False):
            self.acm_active = active
            self.intent_accelerating = intent

            # 防洗頻核心：只有當「狀態(退出原因或區域)」跟上一次不同時，才印出 LOG
            if ACM_DEBUG and (state_str != self.last_log_state or self.acm_active):
                cloudlog.debug(
                    f"[{class_name}] 啟動:{self.acm_active} | 狀態:{state_str} | 加速意圖:{self.intent_accelerating} | "
                    f"剩餘距離:{dist_percent*100:.1f}% | 減速距離:{dynamic_coast_end*100:.1f}% | 緊急距離:{dynamic_safe_dist*100:.1f}% | "
                    f"當前車速:{v_ego * CV.MS_TO_KPH:.1f}km/h | 速差:{v_rel:.1f}m/s | "
                    f"目標距離:{dynamic_target_dist:.1f}m | 前車距離:{d_rel:.1f}m | "
                    f"TTA極限:{raw_a_calc:.2f} | TTA輸出:{smooth_tta_a:.2f} | TTA煞車比例:{fade_factor*100:.0f}% | MPC 融合比例:{mpc_blend_ratio*100:.0f}% | "
                    f"覆寫:{a_desired_trajectory[0]:.2f} -> {current_result[0]:.2f}"
                )

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
                self.accel_intent_counter = 0  # 丟失前車時重置計數器
            # 鎖定期間會自動沿用最後有效的快取數值

        has_lead = self.has_lead_locked

        # ==========================================
        # 1. 狀態計算與安全防護
        # ==========================================
        emergency_fallback = False
        emergency_str = ""

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

            # 🚨 標記緊急狀態，讓防線自然融入下方管線處理
            if ttc < dynamic_ttc_threshold:
                emergency_fallback = True
                emergency_str = "🛑 強制退出(TTC防撞)"
            elif any(a < MPC_FALLBACK_ACCEL for a in recent_trajectory):
                emergency_fallback = True
                emergency_str = "🛑 強制退出(原廠重煞)"

            # ------------------------------------------
            # 🧠 狀態機 B：動態加速意圖鎖定
            # ------------------------------------------
            # 1. 依據車速線性插值計算目標幀數 (0~80 km/h -> 1~20 幀)
            intent_v_ratio = max(0.0, min((v_ego - INTENT_V_LOW) / (INTENT_V_HIGH - INTENT_V_LOW), 1.0))
            dynamic_intent_frames = int(round(INTENT_FRAMES_LOW + intent_v_ratio * (INTENT_FRAMES_HIGH - INTENT_FRAMES_LOW)))

            # 2. 判斷當下這一幀是否具備「瞬間加速/減速條件」
            moment_accel = sum(1 for a in recent_trajectory if a > 0.05) >= INTENT_LOOKAHEAD and v_rel > 0.05
            moment_decel = sum(1 for a in recent_trajectory if a < -0.05) >= INTENT_LOOKAHEAD or v_rel < 0.05

            # 3. 連續幀數累積計數 (一旦中斷就歸零，確保是真正的連續)
            if moment_accel:
                self.accel_intent_counter += 1
            else:
                self.accel_intent_counter = 0

            # 4. 狀態機觸發與解除
            # 【觸發條件】：連續滿足動態幀數
            if self.accel_intent_counter >= dynamic_intent_frames:
                self.intent_accelerating = True

            # 【解除條件】：出現減速徵兆或距離拉開，立刻解除以保安全
            if moment_decel or dist_percent >= dynamic_coast_end:
                self.intent_accelerating = False
                self.accel_intent_counter = 0

            # ------------------------------------------
            # 動態追蹤演算法：全時段連續 TTA 速度匹配
            # ------------------------------------------
            # 計算距離「動態死亡線 (dynamic_safe_dist)」還剩下多少真實物理空間
            safe_buffer_dist = max(dynamic_d_rel - (dynamic_target_dist * dynamic_safe_dist), 0.0)

            # TTA 計算與後續的煞車力道
            safe_v_rel = max(abs(v_rel), 1e-3)
            tta = safe_buffer_dist / safe_v_rel

            # 🧮 全時段套用 TTA 公式，並乘上 1.2 倍 TTA 放大器
            raw_a_calc = - (TARGET_V_REL - v_rel) / max(tta, 1.0)
            raw_a_calc = raw_a_calc * TTA_MULTIPLIER

            # ------------------------------------------
            # 🚀 漸進比例魔法 (Fade-in)：使用動態上下界徹底消除跨界頓挫
            # ------------------------------------------
            # 功能說明：根據 TTA_FADE_SPAN_RATIO 設定，提早讓煞車比例達到 1.0，確保進入區域 B 也能及早建立安全阻力。
            fade_span = (dynamic_coast_end - dynamic_safe_dist) * TTA_FADE_SPAN_RATIO
            fade_factor = (dynamic_coast_end - dist_percent) / max(fade_span, 1e-5)
            fade_factor = max(0.0, min(fade_factor, 1.0)) # 確保比例鎖死在 0~1 之間

            # 限制 TTA 本身算出來的退讓力道
            smooth_tta_a = raw_a_calc * fade_factor
            limit_tta_a = max(MIN_RECOVERY_ACCEL, min(smooth_tta_a, MAX_RECOVERY_ACCEL))

            # 全區域 B 比例式過度 (MPC 漸進介入防線)
            if dist_percent < dynamic_coast_end:

                # 1. 算出區域 B 的總長度跨度 (例如 85% - 75% = 10% 的緩坡)
                blend_span = dynamic_coast_end - dynamic_safe_dist

                # 2. 計算你在這個緩坡上走了多遠 (0.0 -> 1.0)
                mpc_blend_ratio = (dynamic_coast_end - dist_percent) / max(blend_span, 1e-5)
                mpc_blend_ratio = max(0.0, min(mpc_blend_ratio, 1.0))

                # 取當下這一瞬間的原廠煞車指令 (第 0 個點) 作為融合基準
                current_mpc_a = a_desired_trajectory[0]

                # 3. 🛡️ 安全防線：只有當原廠煞得比我們深時，才依照比例把原廠力道揉進來！
                if mpc_blend_ratio > 0.0:
                    limit_tta_a = ((1.0 - mpc_blend_ratio) * limit_tta_a) + (mpc_blend_ratio * current_mpc_a)

        else:
            self.intent_accelerating = False
            self.accel_intent_counter = 0

        # ------------------------------------------
        # 1. ACM 狀態機進出判定
        # ------------------------------------------
        if not has_lead:
            self.acm_active = False
        else:
            # 【有車狀態】：依據距離遲滯區間判定 (將緊急狀態也納入解除條件)
            if dist_percent >= EXIT_PERCENT or emergency_fallback:
                self.acm_active = False
            elif dist_percent <= COAST_START_PERCENT:
                self.acm_active = True

        # ==========================================
        # 2. 統一軌跡處理與分區覆寫
        # ==========================================
        zone_str = ""
        log_zone_str = ""

        # 檢查歷史陣列是否需要初始化 (長度不符或首次啟動)
        init_history = not self.last_a_target_array or len(self.last_a_target_array) != len(result)
        if init_history:
            self.last_a_target_array = [0.0] * len(result)

        for i in range(len(result)):
            raw_mpc_a = result[i]
            a_target = raw_mpc_a

            # --- A. 區域邏輯處理 ---
            if not has_lead:
                # 【無車滑行邏輯】：抹平神經質微煞車
                zone_str = "🟢 無車狀態(執行抹平)"
                if COAST_MAX_BRAKE <= a_target < 0.0:
                    a_target = 0.0
            else:
                # 🛑 緊急防護退場 (取代提早 return，無條件交還 MPC 並讓下方濾波器持續更新)
                if emergency_fallback:
                    zone_str = emergency_str
                    a_target = raw_mpc_a

                # ⚪ 跟車距離外
                elif dist_percent >= EXIT_PERCENT:
                    zone_str = "⚪ 跟車距離外"
                    a_target = raw_mpc_a

                # 🛑 加速意圖
                elif self.intent_accelerating:
                    zone_str = "🛑 加速意圖"
                    a_target = raw_mpc_a

                # 🟢 區域 A (動態邊界 ~ 100%)：單純滑行區
                elif dynamic_coast_end <= dist_percent < EXIT_PERCENT:
                    zone_str = "🟢 區域A(單純滑行)"
                    a_target = 0.0

                # 🟡 區域 B (動態防線 ~ 動態邊界)：平滑退讓區
                elif dynamic_safe_dist <= dist_percent < dynamic_coast_end:
                    zone_str = "🟡 區域B(平滑退讓)"
                    a_target = limit_tta_a

                # 🟠 區域 C (0% ~ 動態防線)：危險防護區，聽從 MPC (全面放行，不設限制)
                elif dist_percent < dynamic_safe_dist:
                    zone_str = "🟠 區域C(交接MPC)"
                    a_target = raw_mpc_a

            # --- B. 最終安全鉗制 (Clamp)，確保不超出物理加速度極限 ---
            a_target = max(ACCEL_MIN, min(a_target, ACCEL_MAX))

            # --- C. 輸出前平滑濾波處理 ---
            if not init_history:
                # 比較當下算出的 a_target 與「上一幀的歷史數值」來判斷是加速還是減速
                if a_target > self.last_a_target_array[i]:
                    # 🟢 放開煞車/加速 (數值變大)：反應慢，套用 EMA_ALPHA_ACCEL
                    a_target = (EMA_ALPHA_ACCEL * a_target) + ((1.0 - EMA_ALPHA_ACCEL) * self.last_a_target_array[i])
                else:
                    # 🔴 踩深煞車/減速 (數值變小)：反應快，套用 EMA_ALPHA_DECEL
                    a_target = (EMA_ALPHA_DECEL * a_target) + ((1.0 - EMA_ALPHA_DECEL) * self.last_a_target_array[i])

            # 擷取在第 0 個點把 zone_str 存起來，避免被後面 16 個預測點蓋掉
            if i == 0:
                log_zone_str = zone_str

            # 將最終處理完的數值，同步寫回歷史紀錄與輸出陣列
            self.last_a_target_array[i] = a_target
            result[i] = a_target

        # 正常跑到最後，也一律呼叫 log_and_return 處理日誌並回傳！
        return log_and_return(log_zone_str, result, active=self.acm_active, intent=self.intent_accelerating)