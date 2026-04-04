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


class AdaptiveCoastingManager:
    """
    自適應滑行管理模組 (ACM)
    負責攔截並優化縱向加速度軌跡，提供 80%~95% 區間的純滑行，
    以及 70%~80% 區間的平滑退讓，同時具備多重保命防護機制。
    """

    def __init__(self):
        # 狀態機：記錄目前是否處於 ACM 介入狀態，避免在邊界值反覆橫跳 (抖動)
        self.acm_active = False

    def update(self, sm: messaging.SubMaster, a_desired_trajectory: list[float], v_ego: float, t_follow_override: float) -> list[float]:
        # 取得雷達資料
        radar_state = sm['radarState']

        # ------------------------------------------
        # 1. 多目標雷達掃描與危險目標鎖定
        # ------------------------------------------
        # 兼容不同分支的雷達資料結構 (支援 leads 陣列或獨立的 leadOne/leadTwo)
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

        # 如果前方完全沒有車輛，關閉 ACM 狀態並直接回傳原始軌跡 (交給定速巡航)
        if not lead:
            self.acm_active = False
            return a_desired_trajectory

        # 提取前車相對距離 (d_rel) 與相對速度 (v_rel)
        d_rel = lead.dRel
        v_rel = lead.vRel

        # ------------------------------------------
        # 2. 基礎數據與距離百分比計算
        # ------------------------------------------
        # 取得當前的跟車秒數 (若無設定，預設為 1.45 秒)
        tf = t_follow_override if t_follow_override is not None else 1.45
        # 計算 MPC 理想跟車距離 (以目前車速計算，並強制設定最低下限為 4.0 公尺)
        target_dist = max(v_ego * tf, 4.0)
        # 算出目前的相對距離百分比 (實際距離 / 理想距離)
        dist_percent = d_rel / target_dist

        # ------------------------------------------
        # 3. 絕對保命防護網 (條件成立即刻退場)
        # ------------------------------------------
        # 防護 A：極速接近中 (例如前方靜止車或前車急煞)
        # 如果相對速差極大 (我們比前車快 1.5 m/s 以上)，立刻交還 MPC 處理重煞
        if v_rel < -1.5:
            self.acm_active = False
            return a_desired_trajectory

        # 🌟 防護 B：MPC 原生軌跡危險預判 (掃描未來 33 點預測)
        # 這是極重要的防線！如果 MPC 預測未來幾秒內【有任何一點】的煞車力道重於 -1.2，
        # 代表神經網路已經察覺到嚴重危險，ACM 立刻無條件退場，不進行任何壓制
        if any(a < MPC_FALLBACK_ACCEL for a in a_desired_trajectory):
            self.acm_active = False
            return a_desired_trajectory

        # ------------------------------------------
        # 4. ACM 狀態機進出判定
        # ------------------------------------------
        if dist_percent >= EXIT_PERCENT:
            self.acm_active = False   # 距離拉開大於 100%，徹底退出 ACM 介入
        elif dist_percent <= COAST_START_PERCENT:
            self.acm_active = True    # 距離壓縮小於 95%，正式啟動 ACM 介入

        # 如果狀態機未啟動，直接放行 MPC 原始指令
        if not self.acm_active:
            return a_desired_trajectory

        # ------------------------------------------
        # 5. 動態追蹤演算法 (針對 70%~80% 區間計算最佳平滑拉回力道)
        # ------------------------------------------
        # 距離誤差 = 目前距離 - 目標距離 (即 80% 警戒線)
        # 若為負數，代表距離已經跌破 80%，需要煞車拉開距離
        distance_error = d_rel - (target_dist * COAST_END_PERCENT)

        # 核心 PD 控制公式：(速差 * 權重 0.5) + (距離誤差 * 權重 0.15)
        # 完美達成：前車加速時緩慢跟上，前車減速或太近時平緩煞車
        raw_a_calc = (v_rel * 0.5) + (distance_error * 0.15)

        # 防護 C：如果 ACM 自己算出來的動態拉回力道極端重 (低於 -1.2)
        # 代表這已經不是「平滑微調」能解決的狀況，直接交還給 MPC 保命
        if raw_a_calc < MPC_FALLBACK_ACCEL:
            self.acm_active = False
            return a_desired_trajectory

        # ------------------------------------------
        # 6. 軌跡處理與分區覆寫 (針對未來 33 個預測點)
        # ------------------------------------------
        for i in range(len(a_desired_trajectory)):
            a_target = a_desired_trajectory[i]

            # 【區域 A】80% ~ 100% 滑行享受與遲滯維持區
            # 只要在 ACM 活躍狀態下，大於 80% 的所有區間，都貫徹滑行邏輯
            if COAST_END_PERCENT <= dist_percent < EXIT_PERCENT:
                if COAST_MAX_BRAKE <= a_target < 0.0:
                    # 如果 MPC 的煞車力道很輕 (-0.4 到 0.0 之間) -> 抹平為 0.0 純滑行
                    a_target = 0.0
                elif a_target < COAST_MAX_BRAKE:
                    # 如果 MPC 煞得比較重 (超過 -0.4) -> 進行數學平移保留煞車力道
                    a_target = a_target - COAST_MAX_BRAKE

            # 【區域 B】70% ~ 80% 平滑退讓區 (擴大為 10% 的防護切入緩衝空間)
            elif SAFE_DIST_PERCENT <= dist_percent < COAST_END_PERCENT:
                # 落實設計理念：強制壓制 MPC 的神經質急煞！
                # 就算 MPC 被切入車嚇到給出極端煞車，只要還在這 10% 緩衝區內，
                # 我們就只使用剛剛算出的平滑力道 `raw_a_calc`，並嚴格限制在 [-0.4, 0.4] 之間，
                # 確保車輛以溫柔、不點頭的方式把距離慢慢拉回 80%。
                a_target = max(MIN_RECOVERY_ACCEL, min(MAX_RECOVERY_ACCEL, raw_a_calc))

            # 【區域 C】小於 70% 絕對危險區
            elif dist_percent < SAFE_DIST_PERCENT:
                # 距離被極度壓縮，跌破了 70% 的絕對安全底線
                # 放棄任何覆寫 (pass)，將這 33 個點的控制權 100% 交還給原生 MPC 執行重煞
                pass

            # 將處理完的數值寫回軌跡陣列
            a_desired_trajectory[i] = a_target

        # 回傳優化後的軌跡，交由外部 Planner 進行終端平滑與執行
        return a_desired_trajectory