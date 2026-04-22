import numpy as np
from cereal import messaging, custom
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE

from openpilot.common.constants import CV
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX

AcmState = custom.LongitudinalPlanSP.AdaptiveCoastingModule.State
"""
【AcmState 10 大精細狀態對照表】(實際定義於 custom.capnp)
- noLead (0)            : 前方無車
- fastExitSafe (1)      : 距離 > 100%，空間充裕交還原廠
- fastExitAccel (2)     : 偵測到加速意圖，放行起步
- fastExitEmergency (3) : 觸發 TTC 或原廠重煞，緊急交還原廠
- fastExitTakeover (4)  : 跌破 75% 且原廠煞車夠深，完美交接
- hysteresisCoast (5)   : 100% ~ 98% 遲滯區 (防抖動)
- pureCoasting (6)      : 98% ~ 90% 純滑行區
- shortMicroBrake (7)   : 90% ~ 75% 防點頭短距煞車 (< 3m)
- longTtaBrake (8)      : 90% ~ 75% TTA 舒適微煞車 (>= 3m)
- ttaFallback (9)       : 跌破 75% 但原廠煞太輕，強制 TTA 保底
"""

# ==========================================
# ⚙️ 全域變數定義區 (Global Configurations)
# ==========================================

# 1. 固定邊界基準 (百分比)
EXIT_PCT        = 1.00      # ⚪ 退出點：大於 100% 關閉滑行功能，全權交還 MPC
COAST_START_PCT = 0.98      # 🟢 滑行起點：98% 啟動滑行，強制設定 0.0
COAST_BRAKE_PCT = 0.90      # 🟡 微煞車點：90% 開啟微煞車功能 (防點頭或 TTA 舒適煞車)
DANGER_PCT      = 0.75      # 🔴 交接線：跌破 75% 評估交接給 MPC 執行重煞

# 2. 物理保底與極限參數
MIN_BRAKE_ZONE_M    = 3.0   # 📏 實體長度保底：90%~75% 區間大於 3m 啟用 TTA，小於則 min(MPC, 0.0)
COAST_MAX_BRAKE     = -0.4  # 🌊 無車神經質極限：無車時抹除 0.0 到 -0.4 之間的微弱煞車
MPC_FALLBACK_ACCEL  = -1.2  # 💣 緊急重煞交接閾值：原廠低於此數值時 ACM 瞬間退出
TTA_ACCEL_MIN       = -1.0  # 🛑 TTA 煞車力道壓制下限
TTA_ACCEL_MAX       = 0.0   # 🛑 TTA 煞車力道壓制上限
TARGET_V_REL        = 0.6   # 🎯 TTA 目標速差：保留微小速差以平滑收尾，不完全貼死前車速度
TTA_MULTIPLIER      = 1.2   # 🚀 TTA 力道放大器：放大基礎數學公式算出的力道，使煞車更扎實
TTA_TIME_RATIO      = 0.5   # ⏱️ TTA 時間壓縮比例：欺騙公式要求用剩餘時間的一半達成減速，逼出初期煞車力道

# 3. 意圖預測與訊號濾波參數
TRAJECTORY_HORIZON  = 6     # 掃描未來軌跡點數 (約涵蓋未來 1.2 秒的預測)
INTENT_LOOKAHEAD    = 3     # 確認意圖所需的連續點數 (避免單一雜訊誤判)
INTENT_V_LOW        = 0.0 * CV.KPH_TO_MS   # 低速判定基準 (用於動態決定確認幀數)
INTENT_V_HIGH       = 80.0 * CV.KPH_TO_MS  # 高速判定基準 (用於動態決定確認幀數)
INTENT_FRAMES_LOW   = 1     # 低速起步所需的確認幀數 (極其靈敏，防卡油門)
INTENT_FRAMES_HIGH  = 20    # 高速巡航所需的確認幀數 (防止高速風吹草動誤判)
DEFAULT_T_FOLLOW    = 1.6   # 預設跟車秒數 (當無法讀取車機設定時的保底值)
FILTER_ALPHA        = 0.2   # 雷達訊號 EMA 濾波權重 (20% 新資料，80% 舊資料，撫平跳動)
LEAD_LOST_TICKS     = 5     # 判定前車丟失的容忍幀數 (連續 5 幀沒看到才當作無車)
EMA_ALPHA_ACCEL     = 0.4   # 輸出平滑濾波：放開煞車/踩油門較慢 (40% 權重，確保舒適)
EMA_ALPHA_DECEL     = 0.8   # 輸出平滑濾波：踩下煞車較快 (80% 權重，確保安全保命)

class AdaptiveCoastingModule:
    """
    自適應滑行管理模組 (ACM) - 順序執行單體架構 (無 Log 純淨十狀態完整版)
    """
    def __init__(self):
        # 意圖與雷達記憶變數 (跨幀保持，用於濾波與連續性判斷)
        self.intent_accelerating = False             # 記錄當前是否處於加速意圖狀態
        self.accel_intent_counter = 0                # 加速意圖連續發生幀數計數器
        self.filtered_d_rel = 0.0                    # 儲存 EMA 濾波後的前車距離
        self.filtered_v_rel = 0.0                    # 儲存 EMA 濾波後的前車相對速度
        self.lead_status_prev = False                # 記錄上一幀是否有前車 (處理突然抓到車的瞬間)
        self.lead_lost_counter = 0                   # 前車丟失幀數計數器 (容忍雷達短暫雜訊)
        self.has_lead_locked = False                 # 系統最終認定的「是否有前車」旗標
        self.last_valid_d_rel = 0.0                  # 最後一次有效的前車距離 (雷達盲算保底用)
        self.last_valid_v_rel = 0.0                  # 最後一次有效的前車相對速度 (雷達盲算保底用)
        self.last_a_target_array = []                # 儲存上一幀的最終軌跡陣列 (輸出端平滑濾波核心)

        # 外部存取變數 (供 UI 介面顯示或外部模組讀取)
        self.active = False                          # ACM 是否有實際修改軌跡 (控制 UI 燈號亮滅)
        self.state = AcmState.noLead                 # 當前 ACM 處於 10 大狀態中的哪一個
        self.leadDist = 0.0                          # 當前前車實體距離 (公尺)
        self.targetDist = 0.0                        # 當前 100% 目標跟車距離 (公尺)
        self.distPercent = 0.0                       # 當前距離佔目標距離的百分比 (驅動狀態機的核心)
        self.ttaLimitValue = 0.0                     # 當前 TTA 算出的煞車極限值 (供除錯參考)
        self.mpcAccel = 0.0                          # 原廠 MPC 原始首個加速度指令
        self.acmAccel = 0.0                          # ACM 介入修飾後的首個加速度指令

    def update(self, sm: messaging.SubMaster, a_desired_trajectory: list[float], v_ego: float, t_follow_override: float) -> list[float]:
        """
        🚀 核心邏輯區：依序執行雷達濾波、物理計算、例外退出判定與常規軌跡修飾
        """
        # ==========================================
        # 步驟 1：初始化與雷達訊號濾波更新
        # ==========================================
        self.ttaLimitValue = 0.0                                 # 每幀開頭歸零 TTA 限值，防止舊資料污染
        lead = sm['radarState'].leadOne                          # 從 cereal 通訊讀取主要前車雷達資料

        if lead.status:                                          # 判斷雷達是否回報有前車
            self.lead_lost_counter = 0                           # 看到車了，重置丟失計數器
            self.has_lead_locked = True                          # 鎖定系統狀態為「有車」

            # 第一幀抓到車時直接使用真實數值，避免濾波造成的延遲導致撞車；之後進行 EMA 濾波
            if not self.lead_status_prev:
                self.filtered_d_rel = lead.dRel
                self.filtered_v_rel = lead.vRel
            else:
                self.filtered_d_rel = (FILTER_ALPHA * lead.dRel) + ((1.0 - FILTER_ALPHA) * self.filtered_d_rel)
                self.filtered_v_rel = (FILTER_ALPHA * lead.vRel) + ((1.0 - FILTER_ALPHA) * self.filtered_v_rel)

            self.lead_status_prev = True                         # 記錄此幀為有車，供下一幀比對
            self.last_valid_d_rel = self.filtered_d_rel          # 備份有效的平滑距離供運算使用
            self.last_valid_v_rel = self.filtered_v_rel          # 備份有效的平滑速度供運算使用
        else:
            if self.has_lead_locked:                             # 如果系統原本認為有車
                self.lead_lost_counter += 1                      # 丟失計數器 +1，容忍短暫的雷達遮蔽雜訊
            if self.lead_lost_counter >= LEAD_LOST_TICKS:        # 如果連續丟失超過設定的容忍幀數
                self.has_lead_locked = False                     # 正式解除有車鎖定
                self.lead_status_prev = False                    # 記錄此幀為無車
                self.intent_accelerating = False                 # 重置加速意圖旗標 (前車都不見了不用再預判)
                self.accel_intent_counter = 0                    # 重置加速意圖計數器

        # ==========================================
        # 步驟 2：物理邊界、TTA 計算與安全意圖掃描 (僅有車時)
        # ==========================================
        emergency_fallback = False                               # 初始化緊急交接旗標為安全 (False)

        if self.has_lead_locked:
            # 2.1 物理邊界計算
            self.leadDist = self.last_valid_d_rel                # 取得濾波後的實體前車距離 (公尺)
            tf = t_follow_override if t_follow_override is not None else DEFAULT_T_FOLLOW # 取得當前跟車秒數
            self.targetDist = max(v_ego * tf, 1.0)               # 計算 100% 目標距離，保底 1.0m 避開除零錯誤

            # 扣除物理極限停止距離，算出真正可用的緩衝距離
            dynamic_d_rel = max(self.leadDist - STOP_DISTANCE, 0.0)
            self.distPercent = dynamic_d_rel / self.targetDist   # 計算當前可用距離佔目標距離的百分比

            # 2.2 🚨 安全防護掃描 (TTC 與原廠重煞)
            # TTC (碰撞剩餘時間) 只在對方比我們慢時計算，否則設為 999 代表極度安全
            ttc = (self.leadDist / abs(self.last_valid_v_rel)) if self.last_valid_v_rel < -0.5 else 999.0
            # 若 TTC 低於安全底線，或掃描到未來 1.2 秒內原廠打算給出低於 -1.2 的重煞車
            if ttc < (tf * 1.2) or any(a < MPC_FALLBACK_ACCEL for a in a_desired_trajectory[:TRAJECTORY_HORIZON]):
                emergency_fallback = True                        # 觸發緊急防護旗標

            # 2.3 🚀 加速意圖掃描 (抓取起步與超車時機)
            recent_traj = a_desired_trajectory[:TRAJECTORY_HORIZON] # 截取未來 6 個原廠加速度軌跡點
            intent_v_ratio = np.clip((v_ego - INTENT_V_LOW) / (INTENT_V_HIGH - INTENT_V_LOW), 0.0, 1.0) # 計算車速所在比例
            # 線性插值決定需要幾幀來確認意圖 (低速 1 幀極靈敏，高速 20 幀防誤判)
            dynamic_intent_frames = int(round(INTENT_FRAMES_LOW + intent_v_ratio * (INTENT_FRAMES_HIGH - INTENT_FRAMES_LOW)))

            # 判斷單幀強烈加速意圖：未來軌跡有多個點加速，且與前車速差正在拉開
            moment_accel = sum(1 for a in recent_traj if a > 0.05) >= INTENT_LOOKAHEAD and self.last_valid_v_rel > 0.05
            # 判斷單幀減速意圖：未來軌跡有煞車，或前車速差正在縮小
            moment_decel = sum(1 for a in recent_traj if a < -0.05) >= INTENT_LOOKAHEAD or self.last_valid_v_rel < 0.05

            # 如果單幀有加速意圖，計數器 +1；否則歸零重新計算
            self.accel_intent_counter = self.accel_intent_counter + 1 if moment_accel else 0
            if self.accel_intent_counter >= dynamic_intent_frames:
                self.intent_accelerating = True                  # 連續計數達標，確認前車正在加速

            # 若遇到減速意圖，或是距離被拉開到 100% 以外，立刻取消加速意圖的放行特權
            if moment_decel or self.distPercent >= EXIT_PCT:
                self.intent_accelerating = False
                self.accel_intent_counter = 0

            # 2.4 🧮 預先計算特製版 TTA (進入 90% 區間以下才計算以節省算力)
            if self.distPercent < COAST_BRAKE_PCT:
                # 算出 90% 至 75% 之間的純緩衝長度 (物理公尺)
                buffer_dist = max(dynamic_d_rel - (self.targetDist * DANGER_PCT), 0.1)
                safe_v_rel = max(abs(self.last_valid_v_rel), 1e-3)                     # 取得絕對速差，保底 1e-3 避免除零

                # 魔法 1：時間壓縮 × 0.5。欺騙公式讓它以為時間只剩一半，藉此逼出更重、更早的初期煞車力道
                tta = (buffer_dist / safe_v_rel) * TTA_TIME_RATIO
                raw_tta_a = - (TARGET_V_REL - self.last_valid_v_rel) / max(tta, 1e-3)  # TTA 核心物理公式算出減速度

                # 魔法 2：套用 TTA 力道放大器 (無動態打折，直接放大)
                tta_a = np.clip(raw_tta_a * TTA_MULTIPLIER, TTA_ACCEL_MIN, TTA_ACCEL_MAX)

                # 魔法 3：90%~80% 比例漸進，80%~75% 鉗制為滿煞車。預留最後 5% 空間滿輸出，消滅 75% 交接頓挫
                fadeFactor = np.clip((COAST_BRAKE_PCT - self.distPercent) / 0.10, 0.0, 1.0)
                self.ttaLimitValue = tta_a * fadeFactor          # 乘上漸進比例，得出這一個瞬間系統允許的最大微煞車極限值

        # ==========================================
        # 步驟 3：例外狀況優先判定 (Bypass 直接退出)
        # ==========================================
        direct_return = False                                    # 初始化直接回傳旗標
        current_mpc_first_a = a_desired_trajectory[0]            # 取得原廠 MPC 當前要輸出的第一個加速度指令

        if self.has_lead_locked:
            # 按照優先級依序判斷是否要瞬間交還控制權
            if emergency_fallback:
                direct_return, self.state = True, AcmState.fastExitEmergency     # 優先級 1：緊急防撞保命
            elif self.intent_accelerating:
                direct_return, self.state = True, AcmState.fastExitAccel         # 優先級 2：前車大腳油門，放行起步
            elif self.distPercent >= EXIT_PCT:
                direct_return, self.state = True, AcmState.fastExitSafe          # 優先級 3：大於 100% 空間極度充裕
            elif self.distPercent < DANGER_PCT:
                # 優先級 4：跌破 75% 死亡線時，必須確認 MPC 煞車力道 >= ACM TTA (數字更小) 才能放心交給它
                if current_mpc_first_a <= self.ttaLimitValue:
                    direct_return, self.state = True, AcmState.fastExitTakeover

        # 若符合上述任何一個例外退出條件，直接記憶原始軌跡並 Return，繞過下方所有運算
        if direct_return:
            self.active = False                                  # 關閉 ACM 動作燈號
            self.last_a_target_array = list(a_desired_trajectory)# 偷偷記住原廠此刻軌跡，確保未來切換回介入模式時，EMA 濾波器不會產生記憶斷層
            self.mpcAccel, self.acmAccel = current_mpc_first_a, current_mpc_first_a
            return list(a_desired_trajectory)                    # 零延遲將控制權交還原廠系統

        # ==========================================
        # 步驟 4：常規軌跡逐點運算與狀態修飾 (鐵腕控制區)
        # ==========================================
        result = list(a_desired_trajectory)                      # 複製一份原廠預測軌跡準備塗改
        init_history = not self.last_a_target_array or len(self.last_a_target_array) != len(result)
        if init_history:
            self.last_a_target_array = [0.0] * len(result)       # 防呆：系統初次啟動時建立全零歷史陣列，防止 EMA 發生陣列越界崩潰

        action_triggered = False                                 # 初始化動作判定旗標

        # 對未來 6 秒內的預測軌跡陣列逐點執行邏輯判斷與覆寫
        for i in range(len(result)):
            raw_mpc_a = result[i]                                # 取出原廠在第 i 個點的預計加速度
            a_target = raw_mpc_a                                 # 預設不干涉，經過下方層層判斷決定是否塗改

            if not self.has_lead_locked:
                self.state = AcmState.noLead
                # 無車狀態：將 -0.4 到 0.0 之間的微弱煞車抹除成 0.0，消除迎風或小坡造成的幽靈神經減速
                if COAST_MAX_BRAKE <= raw_mpc_a < 0.0:
                    a_target = 0.0
            else:
                if self.distPercent >= COAST_START_PCT:
                    self.state = AcmState.hysteresisCoast
                    # 98% ~ 100% 遲滯區：若前一刻正在作動，則維持強制 <= 0.0 (剝奪加速權)，防止邊界來回切換抖動
                    if self.active:
                        a_target = min(raw_mpc_a, 0.0)

                elif self.distPercent >= COAST_BRAKE_PCT:
                    self.state = AcmState.pureCoasting
                    # 90% ~ 98% 純滑行區：無條件強制設定 0.0，享受最極致舒適的滑行
                    a_target = 0.0

                elif self.distPercent >= DANGER_PCT:
                    # 75% ~ 90% 微煞車雙軌邏輯分流
                    zone_length_m = self.targetDist * (COAST_BRAKE_PCT - DANGER_PCT) # 換算這 15% 區間的實體公尺數
                    if zone_length_m < MIN_BRAKE_ZONE_M:
                        self.state = AcmState.shortMicroBrake
                        # 實體長度小於 3m (低速塞車)：放棄計算不出來的 TTA，使用 min(MPC, 0.0) 執行防點頭邏輯
                        a_target = min(raw_mpc_a, 0.0)
                    else:
                        self.state = AcmState.longTtaBrake
                        # 實體長度大於 3m (中高速)：強制套用 TTA 極限值舒適煞車，並確保永遠不補油門
                        a_target = min(self.ttaLimitValue, 0.0)

                else:
                    self.state = AcmState.ttaFallback
                    # 跌破 75% 但因為 MPC 煞車太輕拒絕提前退出，留在這裡強制輸出 TTA 滿額重煞，直到原廠醒來
                    a_target = min(self.ttaLimitValue, 0.0)

            # 💡 動態燈號判定：只要 ACM 的決策結果與原廠 MPC 原始值不同，就視為有動作介入
            if a_target != raw_mpc_a:
                action_triggered = True

            # ==========================================
            # 步驟 5：終極硬體防護與平滑濾波輸出
            # ==========================================
            # 5.1 🛡️ 終極硬體防護：用 numpy clip 將算出的加速度限制在車輛實體能承受的極限內
            a_target = np.clip(a_target, ACCEL_MIN, ACCEL_MAX)

            # 5.2 🌊 非對稱 EMA 平滑濾波
            if not init_history:
                # 動態決定融合權重：若想加速/放開煞車(數值變大)，用 0.4 慢速過渡；若想踩下煞車(數值變小)，用 0.8 快速建立制動力
                alpha = EMA_ALPHA_ACCEL if a_target > self.last_a_target_array[i] else EMA_ALPHA_DECEL
                # 執行 EMA 指數移動平均，讓現在的值與上一刻歷史「手牽手」，打磨出絲綢般的曲線
                a_target = (alpha * a_target) + ((1.0 - alpha) * self.last_a_target_array[i])

            # 僅紀錄陣列首個點(當下即時輸出)的變化狀態，供除錯比對
            if i == 0:
                self.mpcAccel = raw_mpc_a
                self.acmAccel = a_target

            # 將這完美修飾過的值寫回歷史陣列供下一幀濾波融合，並更新到要傳回給車輛的陣列中
            self.last_a_target_array[i] = a_target
            result[i] = a_target

        # 狀態更新：只要該幀有任何塗改動作，即觸發全域作動旗標，點亮 UI 燈號
        self.active = action_triggered

        return result