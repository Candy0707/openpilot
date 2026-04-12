from cereal import messaging

# ==========================================
# ⚙️ 全域變數定義區 (Global Configurations)
# ==========================================

# 1. 距離與狀態機閾值 (百分比)
SAFE_DIST_PERCENT   = 0.75  # 🚨 絕對安全底線：跌破 75% 理想距離時，ACM 完全退場，交還給原生 MPC 重煞保命
COAST_START_PERCENT = 0.95  # 🟢 進入點：距離小於 95% 時，ACM 狀態機啟動，準備介入滑行邏輯
COAST_END_PERCENT   = 0.85  # 🟡 警戒線：距離小於 85% 時結束純滑行，進入「動態微煞車」把距離拉回 85%
EXIT_PERCENT        = 1.00  # ⚪ 退出點：距離拉開大於 100% 時，ACM 徹底休眠
# 遲滯區 (0.95 ~ 1.00)：兩條件皆不成立時維持現有狀態，刻意避免邊界震盪

# 2. 加速度動作極限變數 (單位: m/s²)
COAST_MAX_BRAKE     = -0.4  # 🌊 滑行極限：在 85%~100% 區間，MPC 煞車輕於此值就強制歸零 (純滑行)
MIN_RECOVERY_ACCEL  = -0.6  # 🛡️ 最小煞車極限：75%~85% 區間強制限縮的最大煞車力道，壓制神經質急煞
MAX_RECOVERY_ACCEL  =  0.0  # 🐢 緩加速極限：前車加速時限制補油門力道，確保提速比前車慢以拉開距離
MPC_FALLBACK_ACCEL  = -1.2  # 💣 危險判定閾值：近期軌跡點需要重煞時立刻轉交 MPC

# 3. 軌跡掃描與意圖預測範圍
TRAJECTORY_HORIZON  = 6     # 🔭 危險預判：取 MPC 軌跡前 6 個點 (約 0.6 秒) 預判是否有緊急重煞
INTENT_LOOKAHEAD    = 3     # 🧠 意圖預判：在 6 個點中有 3 個點成立即觸發 (過半數表決防震盪)

# 4. 物理與標定預設常數
DEFAULT_T_FOLLOW    = 1.6   # 預設跟車秒數，當外部未傳入 t_follow_override 時使用
TARGET_V_REL        = 0.6   # 🎯 TTA 目標速差 (m/s)：在退讓區內，只要比前車慢即可，讓距離自然拉開
STANDSTILL_GAP      = 4.0   # 🛡️ 靜止安全間距保底 (m)：確保煞停後與前車保持約一車身的物理距離


class AdaptiveCoastingManager:
    """
    自適應滑行管理模組 (ACM) - 全物理 TTA 升級版
    結合「近期軌跡意圖預測」、「純滑行區間」、「動態 TTC 防撞」與「TTA 平滑速度匹配退讓」。
    """

    def __init__(self):
        # 狀態機 A：記錄目前是否處於 ACM 介入滑行狀態，避免在邊界值反覆橫跳造成頓挫
        self.acm_active = False
        # 狀態機 B：記錄目前是否處於「強烈起步/加速意圖」狀態 (暫停 ACM，確保敏捷性)
        self.intent_accelerating = False

    def update(
        self,
        sm: messaging.SubMaster,
        a_desired_trajectory: list[float],
        v_ego: float,
        t_follow_override: float,
    ) -> list[float]:

        # 從 SubMaster 取得最新一幀的雷達狀態資料
        radar_state = sm['radarState']
        lead = radar_state.leadOne

        # 預先複製陣列，準備給最後統一覆寫使用
        result = list(a_desired_trajectory)

        # ==========================================
        # 1. 狀態計算與安全防護 (提早 Return 區塊)
        # ==========================================
        if lead.status:
            d_rel = lead.dRel
            v_rel = lead.vRel

            # 理想距離 = 車速 × 跟車秒數，並確保絕對不能低於靜止安全間距 (STANDSTILL_GAP)
            tf = t_follow_override if t_follow_override is not None else DEFAULT_T_FOLLOW
            target_dist = max(v_ego * tf, STANDSTILL_GAP)
            dist_percent = d_rel / target_dist

            # 統一擷取近期軌跡 (供危險與意圖預判使用)
            recent_trajectory = a_desired_trajectory[:TRAJECTORY_HORIZON]

            # ------------------------------------------
            # 🛡️ 防護 A：動態煞停意圖預測
            # ------------------------------------------
            # 物理運動學公式：v² = v0² + 2ad
            # 反推：要在安全間距 (STANDSTILL_GAP) 前剛好煞停，理論上需要多大的減速度？

            # 1. 算出真實可用的煞停物理空間 (扣除保底安全距離，最少給 0.5m 避免數學除以零)
            stopping_distance = max(d_rel - STANDSTILL_GAP, 0.5)

            # 2. 公式推導：a = -(v²) / 2d ，計算「理論所需煞停力道」
            a_req_to_stop = - (v_ego ** 2) / (2.0 * stopping_distance)

            # 3. 算出原廠 MPC 目前規劃的未來平均加速度
            avg_mpc_a = sum(recent_trajectory) / len(recent_trajectory)

            # 4. 動態比對大腦意圖：
            # 因為公式算的是「煞到 0」的力道。如果前車還在走，MPC 的煞車力道絕對不會達到這個值的高標 (70%)。
            # 只有當前車真的靜止，MPC 決定煞停時，兩者的物理預期才會完美重合！
            is_stopping_intent = avg_mpc_a < -0.1 and avg_mpc_a <= (a_req_to_stop * 0.7)

            if is_stopping_intent:
                self.acm_active = False
                self.intent_accelerating = False
                return result

            # ------------------------------------------
            # 🛡️ 防護 B：動態 TTC 預警與 MPC 原生重煞防護
            # ------------------------------------------
            # 計算 TTC (碰撞時間)：只在逼近時計算，遠離時設為 999.0 安全值
            ttc = (d_rel / abs(v_rel)) if v_rel < -0.5 else 999.0

            # 動態 TTC 閾值：將跟車秒數放大 1.2 倍作為防護底線，提早應對鬼切
            dynamic_ttc_threshold = tf * 1.2

            # 若預計碰撞時間太短，或原廠近期軌跡已經預測到緊急重煞，立刻退場保命
            if ttc < dynamic_ttc_threshold or any(a < MPC_FALLBACK_ACCEL for a in recent_trajectory):
                self.acm_active = False
                self.intent_accelerating = False
                return result

            # ------------------------------------------
            # 🧠 狀態機 B：動態加速意圖鎖定
            # ------------------------------------------
            # 觸發條件：近期軌跡中出現明顯加速意圖 (> 0.05)
            if sum(1 for a in recent_trajectory if a > 0.05) >= INTENT_LOOKAHEAD:
                self.intent_accelerating = True

            # 解除條件：近期軌跡中出現明顯減速意圖 (< -0.05)
            if sum(1 for a in recent_trajectory if a < -0.05) >= INTENT_LOOKAHEAD:
                self.intent_accelerating = False

            # 距離已經拉開到滑行起點 (95%)
            if dist_percent >= COAST_START_PERCENT:
                self.intent_accelerating = False

            # 若系統鎖定在提速意圖，暫停 ACM 壓制，100% 放行原廠 MPC 確保起步與加速敏捷
            if self.intent_accelerating:
                self.acm_active = False
                return result

            # ------------------------------------------
            # 🌟 動態追蹤演算法：全時段連續 TTA 速度匹配 (拔除 if 斷層)
            # ------------------------------------------
            # 計算距離「75% 死亡線」還剩下多少真實物理空間
            safe_buffer_dist = max(d_rel - (target_dist * SAFE_DIST_PERCENT), 0.0)

            # 🛡️ 數學防護盾：用 0.01 墊底，徹底避開 ZeroDivisionError 當機！
            # 這樣我們就能大膽刪除 if v_rel < 0.0，讓數學公式無縫運行。
            safe_v_rel = max(abs(v_rel), 1e-3)
            tta = safe_buffer_dist / safe_v_rel

            # 🌟 核心修復：全時段套用你原本的 TTA 公式
            raw_a_calc = - (TARGET_V_REL - v_rel) / max(tta, 1.0)

            # 防護 C：若 TTA 算出的所需減速度過大，交還 MPC 保命
            if raw_a_calc < MPC_FALLBACK_ACCEL:
                self.acm_active = False
                return result

        else:
            # 確保無車狀態下不會有加速意圖殘留
            self.intent_accelerating = False

        # ------------------------------------------
        # 🌟 ACM 狀態機進出判定 (Hysteresis & 無車區塊)
        # ------------------------------------------
        if lead.status:
            # 【有車狀態】：依據距離遲滯區間判定
            if dist_percent >= EXIT_PERCENT:
                self.acm_active = False
            elif dist_percent <= COAST_START_PERCENT:
                self.acm_active = True
        else:
            # 【無車狀態】：抹平的神經質微煞車
            self.acm_active = any(COAST_MAX_BRAKE <= a < 0.0 for a in result)

        # 沒啟動就直接回傳
        if not self.acm_active:
            return result

        # ==========================================
        # 2. 統一軌跡處理與分區覆寫 (單一輸出區塊)
        # ==========================================
        # 執行到這裡，代表處於「需要抹平的無車狀態」或是「有車且允許介入的安全狀態」

        for i in range(len(result)):
            a_target = result[i]

            if not lead.status:
                # 【無車滑行邏輯】：抹平 E2E 模型的神經質微煞車
                if COAST_MAX_BRAKE <= a_target < 0.0:
                    a_target = 0.0
            else:
                # 【有車分區邏輯】：依照安全距離百分比分段控制
                if COAST_END_PERCENT <= dist_percent < EXIT_PERCENT:
                    # 區域 A (85% ~ 100%)：滑行享受與遲滯維持區
                    a_target = 0.0

                elif SAFE_DIST_PERCENT <= dist_percent < COAST_END_PERCENT:
                    # 區域 B (75% ~ 85%)：平滑退讓區，套用 TTA 速度匹配力道
                    a_target = max(MIN_RECOVERY_ACCEL, min(MAX_RECOVERY_ACCEL, raw_a_calc))

                elif dist_percent < SAFE_DIST_PERCENT:
                    # 區域 C (< 75%)：絕對危險區，保留原生急煞指令
                    pass

            # 將處理完的數值寫回陣列
            result[i] = a_target

        return result
