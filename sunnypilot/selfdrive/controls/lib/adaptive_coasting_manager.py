from cereal import messaging

# ==========================================
# ⚙️ 全域變數定義區 (Global Configurations)
# ==========================================
# 1. 距離與狀態機閾值 (百分比)
SAFE_DIST_PERCENT = 0.75    # 🚨 絕對安全底線：跌破 75% 理想距離時，ACM 完全退場，交還給原生 MPC 重煞保命
COAST_START_PERCENT = 0.95  # 🟢 進入點：距離小於 95% 時，ACM 狀態機啟動，準備介入滑行邏輯
COAST_END_PERCENT = 0.85    # 🟡 警戒線：距離小於 85% 時結束純滑行，進入「動態微煞車」把距離拉回 85%
EXIT_PERCENT = 1.00         # ⚪ 退出點：距離拉開大於 100% 時，ACM 徹底休眠

# 2. 加速度動作極限變數 (單位: m/s²)
COAST_MAX_BRAKE = -0.4      # 🌊 滑行極限：在 85%~95% 區間，只要 MPC 煞車力道輕於 -0.4，就強制抹平為 0.0 (純滑行)
MIN_RECOVERY_ACCEL = -0.4   # 🛡️ 最小煞車極限：在 75%~85% 區間，為了壓制 MPC 神經質急煞，強制限縮的最大煞車力道
MAX_RECOVERY_ACCEL = 0.4    # 🐢 緩加速極限：前車加速時，限制我們的補油門力道，確保提速比前車慢以拉開安全距離
MPC_FALLBACK_ACCEL = -1.2   # 💣 危險判定閾值：如果預測或計算出需要低於 -1.2 的重煞，代表情況危急，立刻轉交 MPC

# 3. 起步緩衝與意圖預測變數 (Intent Prediction)
STOPPED_SPEED_MAX = 1.0       # 🛑 靜止判定車速 (約 3.6 km/h)：低於此速度視為「車輛靜止或極低速蠕行中」
INTENT_POINTS_THRESHOLD = 16  # ⚖️ 意圖判定門檻：MPC 軌跡 33 點中，超過此數量 (約 50%) 即確認意圖 (加速或減速)
INTENT_ACCEL_VAL = 0.1        # 📈 加速閥值：加速度大於此值才算一個有效的「加速點」(過濾微小雜訊)
INTENT_DECEL_VAL = -0.1       # 📉 減速閥值：加速度小於此值才算一個有效的「減速點」(過濾微小雜訊)


class AdaptiveCoastingManager:
    """
    自適應滑行管理模組 (ACM)
    結合「起步意圖預測」、「純滑行區間」與「平滑退讓防護」的高階縱向控制。
    """

    def __init__(self):
        # 狀態機 A：記錄目前是否處於 ACM 介入滑行狀態，避免邊界反覆橫跳
        self.acm_active = False
        # 狀態機 B：記錄目前是否處於「強烈起步加速意圖」狀態 (起步緩衝專用)
        self.intent_accelerating = False

    def update(self, sm: messaging.SubMaster, a_desired_trajectory: list[float], v_ego: float, t_follow_override: float) -> list[float]:
        # 取得雷達資料
        radar_state = sm['radarState']

        # ------------------------------------------
        # 1. 多目標雷達掃描與危險目標鎖定
        # ------------------------------------------
        # 兼容不同分支的雷達資料結構
        if hasattr(radar_state, 'leads'):
            leads = radar_state.leads
        else:
            leads = [radar_state.leadOne, radar_state.leadTwo]

        lead = None
        # 尋找距離我們最近、最具威脅性的前車目標
        for l in leads:
            if l.status:
                if lead is None or l.dRel < lead.dRel:
                    lead = l

        # 前方無車輛時，關閉所有狀態並直接回傳原始軌跡
        if not lead:
            self.acm_active = False
            self.intent_accelerating = False
            return a_desired_trajectory

        # 提取前車相對距離 (d_rel) 與相對速度 (v_rel)
        d_rel = lead.dRel
        v_rel = lead.vRel

        # 🌟 核心計算：前車的真實絕對速度 (v_lead)
        # 公式：前車速度 = 本車速度 + 相對速差。使用 max 確保速度不會因為雷達雜訊變成負數
        v_lead = max(0.0, v_ego + v_rel)

        # ------------------------------------------
        # 2. 基礎數據與距離百分比計算
        # ------------------------------------------
        # 取得當前的跟車秒數，並計算理想跟車距離 (強制設定最低下限為 4.0 公尺)
        tf = t_follow_override if t_follow_override is not None else 1.45
        target_dist = max(v_ego * tf, 4.0)

        # 算出目前的相對距離百分比 (實際距離 / 理想距離)
        dist_percent = d_rel / target_dist

        # ------------------------------------------
        # 3. 絕對保命防護網 (條件成立即刻退場)
        # ------------------------------------------
        # 🌟 防護 A (極簡煞停邏輯)：前車靜止或極低速蠕行
        # 只要前車速度低於 1.0 m/s (約 3.6 km/h)，視為「煞停目標」。
        # 直接把控制權 100% 交還給 MPC，完美解決紅綠燈最後一哩路「放開煞車又急煞」的問題！
        if v_lead < 1.0:
            self.acm_active = False
            self.intent_accelerating = False
            return a_desired_trajectory

        # 防護 B：極速接近中 (例如遇到靜止車，速差極大，且本車尚未減速)
        if v_rel < -1.5:
            self.acm_active = False
            self.intent_accelerating = False
            return a_desired_trajectory

        # 防護 C：MPC 原生軌跡危險預判
        # 只要未來有任何一點需要重煞 (-1.2)，立刻交還 MPC 保命
        if any(a < MPC_FALLBACK_ACCEL for a in a_desired_trajectory):
            self.acm_active = False
            self.intent_accelerating = False
            return a_desired_trajectory

        # ------------------------------------------
        # 🌟 4. 起步緩衝意圖預測 (Intent-Based Startup Buffer)
        # ------------------------------------------
        # 計算未來 33 個預測點中，明確的加速點與減速點數量
        accel_points_count = sum(1 for a in a_desired_trajectory if a > INTENT_ACCEL_VAL)
        decel_points_count = sum(1 for a in a_desired_trajectory if a < INTENT_DECEL_VAL)

        # 【觸發加速意圖】：車輛處於靜止/極低速，且未來有 50% (16點) 以上在加速
        if v_ego < STOPPED_SPEED_MAX and accel_points_count >= INTENT_POINTS_THRESHOLD:
            self.intent_accelerating = True

        # 【解除加速意圖】：偵測到前車減速 (未來有 50% 點數預測減速)，或已經拉開至安全距離
        elif decel_points_count >= INTENT_POINTS_THRESHOLD or dist_percent >= COAST_START_PERCENT:
            self.intent_accelerating = False

        # 如果目前處於「起步加速意圖」中，暫停 ACM 壓制，100% 放行原廠 MPC 以確保起步敏捷
        if self.intent_accelerating:
            self.acm_active = False
            return a_desired_trajectory

        # ------------------------------------------
        # 5. ACM 狀態機進出判定
        # ------------------------------------------
        if dist_percent >= EXIT_PERCENT:
            self.acm_active = False   # 距離拉開大於 100%，退出 ACM
        elif dist_percent <= COAST_START_PERCENT:
            self.acm_active = True    # 距離壓縮小於 95%，正式啟動 ACM

        # 狀態機未啟動且無加速意圖，放行原生軌跡
        if not self.acm_active:
            return a_desired_trajectory

        # ------------------------------------------
        # 6. 動態追蹤演算法 (PD 控制)
        # ------------------------------------------
        # 距離誤差 = 目前距離 - 警戒線距離
        distance_error = d_rel - (target_dist * COAST_END_PERCENT)

        # 核心公式：依據速差與距離誤差計算平滑拉回力道
        raw_a_calc = (v_rel * 0.5) + (distance_error * 0.15)

        # 防護 D：若動態拉回力道過大，交還 MPC 保命
        if raw_a_calc < MPC_FALLBACK_ACCEL:
            self.acm_active = False
            return a_desired_trajectory

        # ------------------------------------------
        # 7. 軌跡處理與分區覆寫
        # ------------------------------------------
        for i in range(len(a_desired_trajectory)):
            a_target = a_desired_trajectory[i]

            # 【區域 A】85% ~ 100% 滑行享受與遲滯維持區
            if COAST_END_PERCENT <= dist_percent < EXIT_PERCENT:
                if COAST_MAX_BRAKE <= a_target < 0.0:
                    a_target = 0.0
                elif a_target < COAST_MAX_BRAKE:
                    # MPC 煞得較重時，進行數學平移以消除頓挫
                    a_target = a_target - COAST_MAX_BRAKE

            # 【區域 B】75% ~ 85% 平滑退讓區 (10% 防切入緩衝空間)
            elif SAFE_DIST_PERCENT <= dist_percent < COAST_END_PERCENT:
                # 強制限縮最大煞車力道，溫柔拉回距離
                a_target = max(MIN_RECOVERY_ACCEL, min(MAX_RECOVERY_ACCEL, raw_a_calc))

            # 【區域 C】小於 75% 絕對危險區
            elif dist_percent < SAFE_DIST_PERCENT:
                # 距離被極度壓縮，放棄覆寫，交由原生 MPC 執行重煞
                pass

            # 將處理完的數值寫回軌跡陣列
            a_desired_trajectory[i] = a_target

        return a_desired_trajectory