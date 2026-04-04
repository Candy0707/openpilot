from cereal import messaging


class AdaptiveCoastingManager:
  """
  自適應滑行管理模組 (Adaptive Coasting Management, ACM) - 多目標雷達版
  掃描 radar_state.leads 中的所有偵測點，取最保守(安全)的條件來覆寫 MPC 軌跡。
  """

  def __init__(self):
    pass

  def update(self, sm: messaging.SubMaster, a_desired_trajectory: list[float], v_ego: float, t_follow_override: float) -> list[float]:
    radar_state = sm['radarState']

    # 假設你的分支 radarState 結構支援 leads 列表
    # 若為傳統結構可能需改為 [radar_state.leadOne, radar_state.leadTwo]
    leads = radar_state.leads

    # 1. 預設為最放鬆的狀態 (假設前方無障礙物)
    global_coast_limit = -1.5  # 預設允許最深 -1.5 的滑行攔截
    is_safe_to_coast = True  # 預設距離安全
    trigger_anti_surge = False  # 預設不觸發防暴衝鎖死
    valid_leads_count = 0  # 記錄有效目標數量

    # 計算動態安全門檻
    tf = t_follow_override if t_follow_override is not None else 1.45
    target_dist = max(v_ego * tf, 4.0)
    # 直接使用理想目標距離的 50% (0.5)，並保留最低 8 公尺的實體底線
    MIN_SAFE_DIST = max(8.0, target_dist * 0.5)

    # 2. 遍歷所有雷達偵測點，尋找「最危險」的條件
    for lead in leads:
      # 只處理有訊號/存在的目標
      if not lead.status:
        continue

      valid_leads_count += 1
      d_rel = lead.dRel
      v_rel = lead.vRel

      # ==========================================
      # 條件 A：只要有【任何一個】目標距離過近，就全域禁止滑行
      # ==========================================
      if d_rel <= MIN_SAFE_DIST:
        is_safe_to_coast = False

      # ==========================================
      # 條件 B：計算此目標的滑行攔截深度，並【取最嚴格值】
      # ==========================================
      if v_rel > 0.2:
        lead_coast_limit = -1.5  # 目標遠去，允許深攔截
      elif v_rel > -0.5:
        lead_coast_limit = -0.5  # 穩定相對速度，允許淺攔截
      else:
        lead_coast_limit = 0.0  # 目標急煞，禁止攔截 (交給MPC)

      # 取 max() 是因為 0.0 (不攔截) 比 -1.5 (深攔截) 更嚴格/更保守
      global_coast_limit = max(global_coast_limit, lead_coast_limit)

      # ==========================================
      # 條件 C：只要有【任何一個】目標滿足接近條件，就全域觸發防暴衝
      # ==========================================
      if v_rel < -0.1 and d_rel < (target_dist * 1.5):
        trigger_anti_surge = True

    # 3. 如果完全沒有偵測到任何目標，直接回傳原始軌跡
    if valid_leads_count == 0:
      return a_desired_trajectory

    # 4. 根據全局最嚴格的條件，處理 33 個軌跡點
    for i in range(len(a_desired_trajectory)):
      a_target = a_desired_trajectory[i]

      # 🛡️ 規則一：接近防護 (Anti-Surge) - 鎖死正向加速
      if trigger_anti_surge:
        a_target = min(a_target, 0.0)

      # 🌊 規則二：削峰平滑與主動滑行
      # 必須距離安全，且全局 coast_limit 允許攔截 (< 0.0)
      if is_safe_to_coast and global_coast_limit < 0.0:
        if global_coast_limit <= a_target < 0.0:
          a_target = 0.0
        elif a_target < global_coast_limit:
          a_target = a_target - global_coast_limit

      a_desired_trajectory[i] = a_target

    return a_desired_trajectory
