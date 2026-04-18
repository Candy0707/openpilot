"""
================================================================================
Dynamic Turn Speed Controller (DTSC) - Final Pro Edition
自動駕駛動態彎道速度控制器 (四階段狀態機 + 實車G力限速 + 幀控制防護版) 🚀
================================================================================

【系統運作核心理論 (System Architecture & Control Theory)】
本系統屏棄了傳統依賴「靜態地圖」或「單一曲率門檻」的生硬煞車方式，
改採最先進的「時空軌跡重疊 (Spatio-Temporal Overlap)」與「四階段動態狀態機」。

1. 信心濾波器 (Confidence Filter)：
   我們不盲目相信 AI 模型單一幀的預測。系統會將未來的軌跡投影到時間軸上，
   只有當模型在「連續的時間流逝中，對同一個物理位置持續預測出危險彎道特徵」時，
   信心度才會累積。這就像給了系統一雙抗雜訊的眼睛，徹底消滅了幽靈煞車。

2. 危險剝離與彎心鎖定 (Danger Extraction & Apex Targeting)：
   在可信賴的視距內，系統會像賽車手一樣去尋找「真正的入彎點」(G>0.3 或 曲率>0.005)，
   並直接抓取整段彎道最極限的「彎心 (Apex)」數據，一次性反推出該彎道的絕對安全車速。

   🌟 [防護] 實車 G 力強制介入：
   若 AI 信心累積太慢 (<60%)，但底盤實測感受到的橫向 G 力已超越舒適極限，
   系統將無視 AI 信心，強制判定為「彎道中」，立刻啟動減速接管，構築絕對安全底線。

3. 四階段狀態機與雙模控制 (4-Stage State Machine & Dual-Mode Control)：
   - [第一階段] 啟用判定：AI 預見彎道 OR 底盤 G 力超標。
   - [第二階段] 狀態定位：區分預先減速區 (Pre-decel) 與 彎中動態區 (Turning)。
   - [第三階段] 減速實作：入彎前用 TTA 空間公式；彎中單純採用「實體橫向G力」決定安全車速。
   - [第四階段] 出彎判斷：使用實體曲率並執行 20 幀連續驗證，確保 S 彎不提早加速。
================================================================================
"""

import numpy as np
from cereal import messaging
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase

# ==========================================================
# [0. 系統全域常數與物理極限設定]
# ==========================================================
# 【系統輸出權限界線】
MAX_ACCEL = 1.0   # 解鎖正向加速權限 (1.0 m/s² 代表允許系統在彎中/出彎時，給予溫和且線性的補油)
MIN_ACCEL = -3.5  # 最大允許煞車力道 (-3.5 m/s² 接近人類重踩煞車的極限，做為保命的物理底線)

# 【時間與空間網格化 (Grid Settings)】
# Openpilot 視覺模型頻率為 20Hz (即每幀 0.05 秒)，我們將未來 10 秒的預測切分為 200 格。
NUM_SLOTS = 200
DT_MDL = 0.05
LINEAR_T = np.arange(0.0, 10.0, DT_MDL)  # 建立均勻的線性時間軸 (0.0, 0.05, 0.10...)
MODEL_T = np.array(ModelConstants.T_IDXS)  # 模型原始輸出的非均勻時間軸 (越遠處點越稀疏)

# 【彎道特徵與信心門檻】
MIN_CURVATURE = 0.002    # 基礎彎道門檻：低於此值視為直路，用於累積最基礎的視野信心。
TRIGGER_LAT_ACCEL = 0.6  # 預測 G 力門檻：專門用來捕捉高速公路上「曲率微小但車速極快」的高速大緩彎。
CONF_THRESHOLD = 0.60    # 絕對信心門檻：軌跡穩定度 > 60%，系統才認定此段軌跡可信並納入計算。

# 【人類舒適度動態查表 (Comfort G-Force Lookup Table)】
# 根據不同車速 (km/h)，定義出乘客不會感到暈眩或恐懼的最大橫向 G 力 (m/s²)。
# 速度越快，容忍的 G 力通常會稍微放寬 (對應高速公路的超大彎道設計)。
COMFORT_SPEEDS_KPH = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0]
COMFORT_LAT_ACCELS = [1.2, 1.2, 1.2, 1.3, 1.4, 1.5, 1.6, 1.6, 1.8, 1.8, 1.8, 1.8, 1.8]

# 【底盤硬體物理防線】
STAT_HARD_LIMIT_LAT_ACCEL = 2.0  # 絕對極限：不論查表舒適度為何，絕不允許過彎 G 力超過此數值，防止失控。


class DynamicTurnSpeedController(TargetsBase):
  def __init__(self, CP: structs.CarParams, mpc=None):
    super().__init__(CP, mpc)
    self.log_timer = 0.0
    self.exit_frame_count = 0                     # 出彎防護計數器 (幀數控制)

    # 【狀態記憶體配置】
    self.confidence_table = np.zeros(NUM_SLOTS)   # 核心記憶：儲存未來 10 秒每一格的彎道確信度 (0.0~1.0)
    self.ui_confidence = np.zeros(len(MODEL_T))   # 視覺渲染：提供給 HUD 的 33 格信心地毯數據
    self.v_target_history = np.zeros(10)          # 輸出平滑器 (車速)
    self.a_target_history = np.zeros(10)          # 輸出平滑器 (加速度)

  def update_target(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float):
    model_msg = sm['modelV2']
    ctrl_name = self.__class__.__name__

    # 【基礎防呆機制】
    # 若 AI 模型當機、無資料，直接放行使用者設定的巡航定速，不做任何干預。
    if model_msg is None or len(model_msg.position.x) == 0:
      self.v_target, self.a_target = v_cruise, a_ego
      return super().update_target(sm, v_ego, a_ego, v_cruise)


    # ==========================================================
    # 車體狀態與虛擬軌跡萃取 (Trajectory Extraction)
    # ==========================================================
    # 讀取底盤感測器計算的當下「真實曲率」(用於退出判定)。
    cs_curvature = sm['controlsState'].curvature if sm.valid['controlsState'] else 0.0

    # 擷取 AI 模型的 33 個原始預測節點 (空間距離、速度、車頭旋轉角速度、坡度)。
    x_raw = np.array(model_msg.position.x)                      # 縱向距離
    v_raw = np.maximum(np.array(model_msg.velocity.x), 0.1)     # 預測車速
    y_rate_raw = np.abs(np.array(model_msg.orientationRate.z))  # 預測橫擺角速度 (Yaw Rate)
    pitch_raw = np.array(model_msg.orientation.y)               # 預測俯仰角 (坡度)

    # 利用線性插值 (Linear Interpolation) 將稀疏的 33 個點，均勻展開為 200 個時間格。
    x_200 = np.interp(LINEAR_T, MODEL_T, x_raw)
    pitch_200 = np.interp(LINEAR_T, MODEL_T, pitch_raw)

    # 預測曲率計算公式：k = (Yaw Rate) / Velocity
    # 分母加上 1.0 m/s 的下限，避免車輛靜止時微轉方向盤導致曲率爆炸。
    v_raw_k = np.maximum(v_raw, 1.0)
    k_raw = y_rate_raw / v_raw_k

    # 將預測的 G 力 (向心加速度) 與曲率，一併展開至 200 格陣列。
    g_200 = np.interp(LINEAR_T, MODEL_T, y_rate_raw * v_raw)
    k_200_raw = np.interp(LINEAR_T, MODEL_T, k_raw)

    # 【下坡危險補償 (Slope Compensation)】
    # 當遇到下坡 (pitch 為負)，車輛重心前移且重力提供額外向下分量，外拋風險大增。
    # 這裡人為將預測曲率放大 (最多放大 50%)，迫使系統提早且更重地踩下煞車。
    slope_factor = np.clip(np.abs(pitch_200) * 5.0, 0.0, 0.5)
    k_200_safe = k_200_raw * (1.0 + slope_factor)

    # ==========================================================
    # 信心感知器 (Confidence Perception & Temporal Shift)
    # ==========================================================
    # 【時空平移】將上一幀的信心畫布向左平移一格 (代表 0.05 秒過去了，未來逼近了當下)。
    self.confidence_table[:-1] = self.confidence_table[1:]
    self.confidence_table[-1] = 0.0

    # 【彎道特徵遮罩】找出預測軌跡中，滿足大 G 力或大曲率的節點。
    curve_mask = (g_200 > TRIGGER_LAT_ACCEL) | (k_200_safe > MIN_CURVATURE)

    # 【充放電邏輯】
    # 若特徵持續存在，信心度 +5%；若特徵消失或亂跳，信心度 -1% (緩慢放電以容錯)。
    self.confidence_table[curve_mask] += 0.05
    self.confidence_table[~curve_mask] -= 0.01
    self.confidence_table = np.clip(self.confidence_table, 0.0, 1.0)

    # ==========================================================
    # 實車動態極限與 G 力計算 (Chassis Dynamics)
    # ==========================================================
    # 透過查表，獲取當前車速下人類乘客能接受的舒適 G 力極限。
    comfort_speeds_ms = [v * CV.KPH_TO_MS for v in COMFORT_SPEEDS_KPH]
    comfort_g_limit = float(np.interp(v_ego, comfort_speeds_ms, COMFORT_LAT_ACCELS))

    # A. 實車當下安全限速：利用底盤實測曲率反推，用來應對已經在彎道中，防護底盤物理極限。
    act_k_raw = max(abs(cs_curvature), 1e-5)
    act_k_safe = act_k_raw * (1.0 + np.clip(-pitch_200[0] * 5.0, 0.0, 0.5))
    v_act_hard_g = np.sqrt(STAT_HARD_LIMIT_LAT_ACCEL / act_k_safe)
    v_act_k_safe = np.sqrt(comfort_g_limit / act_k_safe)
    v_actual_decision = min(v_act_hard_g, v_act_k_safe)

    # 🌟 即時實體橫向 G 力防護 (G-Force Override)
    # 物理公式：a_y = v² × k (當下車速平方 乘以 底盤真實曲率)
    current_lat_accel = (v_ego ** 2) * act_k_raw

    # [更嚴格條件] 必須同時滿足：實體橫向 G 力大於舒適極限 AND 實車真實曲率大於最小彎道門檻
    is_g_force_override = (current_lat_accel > comfort_g_limit) and (act_k_raw > MIN_CURVATURE)

    # ==========================================================
    # AI 動態視距與危險特徵萃取 (AI Prediction)
    # ==========================================================
    # 初始化 Log 遙測與狀態機變數
    ai_sees_curve = False
    v_model_decision = v_cruise
    k_future_max = 0.0001
    g_future_max = 0.0
    entry_sec = 0.0
    horizon_sec = 0.0
    dist_to_entry = 0.0
    dyn_horizon_dist = 0.0
    entry_conf = 0.0
    horizon_conf = 0.0

    # 從信心畫布中，篩選出大於 60% 門檻的索引值，這些是系統認可的「絕對可信軌跡」。
    valid_indices = np.where(self.confidence_table > CONF_THRESHOLD)[0]

    # 若視野內存在大於 60% 信心的軌跡，確立「AI 預測前方有彎道」
    if len(valid_indices) > 0:
      ai_sees_curve = True

      # 高信心軌跡的尾端，即為系統當下的「最遠動態視距」。
      horizon_idx = valid_indices[-1]

      # 【危險入彎點精確定位】
      # 為了避免微小曲率 (如直路上微調方向盤) 被誤判為 0.0m 入彎，
      # 必須在可信軌跡中，去尋找第一個真正具備物理危險性 (G>0.3 或 曲率>0.005) 的點。
      dangerous_pts = np.where((g_200[valid_indices] > 0.3) | (k_200_safe[valid_indices] > 0.005))[0]

      if len(dangerous_pts) > 0:
        entry_idx = valid_indices[dangerous_pts[0]] # 真正的煞車起點
      else:
        entry_idx = valid_indices[0] # 若無明顯危險，退回可信點起點

      # 將 Index 轉換為真實物理距離 (公尺) 與時間 (秒)。
      dist_to_entry = x_200[entry_idx]
      dyn_horizon_dist = x_200[horizon_idx]
      entry_sec = entry_idx * DT_MDL
      horizon_sec = horizon_idx * DT_MDL
      entry_conf = self.confidence_table[entry_idx] * 100
      horizon_conf = self.confidence_table[horizon_idx] * 100

      # 【鎖定彎心 (Apex)】
      # 從這段高信心軌跡中，直接萃取出「最極限的曲率與 G 力」，用來決定彎道的最低限速。
      k_future_max = float(np.max(k_200_safe[valid_indices]))
      g_future_max = float(np.max(g_200[valid_indices]))

      # B. 預測未來安全限速：利用找出的彎心 (Apex) 極限曲率，反推入彎前必須降到的目標車速。
      v_mod_hard_g = np.sqrt(STAT_HARD_LIMIT_LAT_ACCEL / max(k_future_max, 1e-5))
      v_mod_k_safe = np.sqrt(comfort_g_limit / max(k_future_max, 1e-5))
      v_model_decision = min(v_mod_hard_g, v_mod_k_safe)


    # ==========================================================
    # 【第一階段】 啟用/停用判斷 (Activation Check)
    # ==========================================================
    # 這是 DTSC 的核心閘門：整合「AI 預測意圖」與「實體底盤意圖」，決定最終是否啟動彎道防護。
    is_curve_ahead = False

    if ai_sees_curve:
      # 【意圖一：AI 預見彎道】(原本的 AI 意圖啟動)
      is_curve_ahead = True
      # 最終決策：整合實車現狀、預測未來與巡航定速，取最保守 (最小) 的數值作為絕對安全目標。
      v_decision_final = min(v_model_decision, v_actual_decision, v_cruise)

    elif is_g_force_override:
      # 【意圖二：底盤感受危險】(新的實體底盤啟動)
      # AI 信心不足 (<60%)，但實車底盤 G 力與曲率已同時超標！強制介入！
      is_curve_ahead = True
      v_decision_final = min(v_actual_decision, v_cruise) # 以實體方向盤限速為準
      dist_to_entry = 0.0        # 距離歸零，強迫系統直接進入彎中動態調整
      k_future_max = act_k_safe  # 極限曲率採用當下底盤實測
      g_future_max = current_lat_accel

    else:
      # 【意圖三：無危險狀態】
      v_decision_final = v_cruise

    # 車輛必須行進中才允許啟動，防止靜止蠕行時因方向盤轉動引發頓挫。
    # 只要前方的判斷亮起綠燈 (is_curve_ahead = True)，立即啟動介入。
    if v_ego > 0.1 and not self.action and is_curve_ahead:
      self.action = True


    # ==========================================================
    # 【第二階段】 判斷當前是預先減速，還是在彎中動態 (State Positioning)
    # ==========================================================
    # 若為底盤 G 力強制觸發，無視距離強迫定位為「彎中」。
    effective_dist = 0.0 if (is_g_force_override and not ai_sees_curve) else dist_to_entry

    is_pre_deceleration = effective_dist > 1.0   # 距離大於 1.0m：預先減速區
    is_in_curve_dynamic = effective_dist <= 1.0  # 距離小於 1.0m：彎中動態區


    # ==========================================================
    # 【第三階段】 實作減速 (Control Execution)
    # ==========================================================
    a_target_raw = 0.0

    # 只要仍在介入狀態 (self.action == True)，就必須執行對應的控制邏輯
    if self.action:

      if is_curve_ahead:
        # --------------------------------------------------------
        # (A) 預先減速實作：TTA 空間公式
        # --------------------------------------------------------
        if is_pre_deceleration:
          if v_ego > v_decision_final:
            # TTA 等加速度運動學公式：a = (Vf² - Vi²) / 2S
            # 準備入彎階段「嚴格禁止補油 (幽靈加速)」，強制使用 min(a, 0.0) 鎖死加速權限。
            a_req = (v_decision_final**2 - v_ego**2) / (2.0 * effective_dist)
            a_target_raw = min(a_req, 0.0)
          else:
            # 🌟 [極簡防護：模型 G 值看破威脅]
            # 若前方確實是具備離心威脅的彎道 (預測 G 值 > 舒適極限的 50%)，
            # 減速達標後嚴格鎖死油門 (0.0) 讓車輛順順滑進去。
            # 只有前方威脅解除時，才允許平滑補油提速。
            if g_future_max > (comfort_g_limit * 0.5):
              a_target_raw = 0.0
            else:
              a_target_raw = (v_decision_final - v_ego) / 2.5

        # --------------------------------------------------------
        # (B) 彎中動態實作：實車 G 力限速控制
        # --------------------------------------------------------
        elif is_in_curve_dynamic:
          # 🌟 彎中完全捨棄 AI 預測速度，單純使用「實體底盤 G 力」來計算當下的安全車速
          # v_act_k_safe 是由公式 sqrt(舒適G力 / 實體曲率) 所算出的完美過彎速度
          # 這確保了減速力道永遠柔和且符合物理極限，徹底解決極端重煞問題 (-3.5G)
          v_curve_target = min(v_act_k_safe, v_cruise)
          speed_diff_curve = v_curve_target - v_ego

          # 單純使用 1.5 秒的時間常數進行平滑追隨，體感會非常線性柔和 (允許輸出正加速度補油)
          a_target_raw = speed_diff_curve / 1.5

      else:
        # --------------------------------------------------------
        # (C) 出彎過渡期實作：平滑加速
        # --------------------------------------------------------
        # 模型已看不見彎道，但方向盤尚未完全回正 (self.action 依然 True)。
        # 【解除 min() 限制】：允許輸出正加速度。
        # 利用 2.0 秒的時間常數，平滑引導車輛加速回歸原定的巡航定速，營造出人類補油出彎的爽快感。
        speed_diff_exit = v_cruise - v_ego
        a_target_raw = speed_diff_exit / 2.0


    # ==========================================================
    # 【第四階段】 判斷出彎 (Exit Condition)
    # ==========================================================
    # 【原始退出條件】方向盤實體曲率已回歸直線 (<0.002) 且 AI 模型判定未來已無彎道。
    exit_condition_raw = (abs(cs_curvature) < MIN_CURVATURE) and (not is_curve_ahead)

    # 引入防護計數器 (改為幀數控制)
    # 避免 S 彎在轉換左右方向時，方向盤瞬間經過中心 (0.0) 導致誤判提早解除介入。
    # modelV2 運作頻率為 20Hz (一幀 = 0.05秒)
    if exit_condition_raw:
      self.exit_frame_count += 1
    else:
      self.exit_frame_count = 0

    # 必須連續 20 幀 (相當於 1.0 秒) 滿足退出條件，才敢真正解除系統！
    exit_condition = (self.exit_frame_count >= 20)

    # 若滿足退出條件，或車輛靜止，解除系統介入。
    if self.action and exit_condition:
      self.action = False
      self.exit_frame_count = 0 # 退出時清零計數器

    if v_ego <= 0.1:
      self.action = False
      self.exit_frame_count = 0


    # ==========================================================
    # [輸出控制與 10 幀環形平滑 (Ring Buffer Smoothing)]
    # ==========================================================
    # 將加速度也納入 10 幀平滑緩衝，消除硬切換與運算抖動造成的踏板抽搐
    self.a_target_history[1:] = self.a_target_history[:-1]
    self.a_target_history[0] = a_target_raw
    a_target_smoothed = float(np.mean(self.a_target_history))

    # 確保最終輸出的加速度符合硬體物理極限 (-3.5G ~ 1.0G)
    self.a_target = np.clip(a_target_smoothed, MIN_ACCEL, MAX_ACCEL)

    if self.action:
      # 【虛擬前導車速反推】
      # 為與 Openpilot 原生的縱向 MPC (模型預測控制) 完美融合，
      # 我們將決策好的目標加速度，反推為前方 4 秒處的一台「虛擬假車」的速度，引導底層執行。
      v_target_raw = np.sqrt(STAT_HARD_LIMIT_LAT_ACCEL / max(k_future_max, 1e-5)) + (self.a_target * 4.0)
      v_target_raw = np.clip(v_target_raw, 0.0, v_cruise)

      # 陣列平移：拋棄最舊的歷史數據 (index 9)，存入最新計算的目標車速 (index 0)
      self.v_target_history[1:] = self.v_target_history[:-1]
      self.v_target_history[0] = v_target_raw
      # 取 10 幀 (0.5秒) 的平均值輸出，徹底抹平微小的運算抖動，讓油門/煞車踏板絲滑無比。
      self.v_target = float(np.mean(self.v_target_history))
    else:
      # 待機狀態：持續用當前車速填滿緩衝區，確保下次系統「突然介入」的瞬間，車速無縫接軌不會頓挫。
      self.v_target_history.fill(v_ego)
      self.a_target_history.fill(a_ego) # 🌟 待機時對齊當前的實體加速度，確保介入瞬間平滑過渡
      self.v_target = v_cruise
      self.a_target = a_ego

    # ==========================================================
    # [UI 視覺渲染與遙測日誌 (Telemetry & Logging)]
    # ==========================================================
    # 將高解析的 200 格信心陣列降採樣回 33 格，提供給 UI 端繪製半透明信心地毯使用。
    self.ui_confidence = np.interp(MODEL_T, LINEAR_T, self.confidence_table)

    self.log_timer += DT_MDL

    # 狀態機紀錄：控制終端機輸出頻率 (每 0.5 秒一筆)，避免洗頻
    if self.action:
      if self.log_timer >= 0.5:
        # 配合 4 階段狀態機，精準輸出當前動態文字，方便事後透過 Log 抓蟲與調校。
        if not is_curve_ahead:
          state_str = "🏁 出彎加速"
        # G 力強制介入的專屬 Log 狀態 (AI 沒看見但底盤啟動)
        elif is_g_force_override and not ai_sees_curve:
          state_str = "⚠️ 實車Ｇ力強制介入"
        elif is_in_curve_dynamic:
          state_str = "🎯 彎中實車G力限速"
        else:
          state_str = "📉 預先減速"

        # 高解析遙測輸出 (包含秒數、距離與精準的物理極限)
        log_msg = (
          f"[{ctrl_name}] {state_str} | "
          f"V(當前/安全/輸出): {v_ego * 3.6:4.1f}/{v_decision_final * 3.6:4.1f}/{self.v_target * 3.6:4.1f} | "
          f"A(加速度輸出): {self.a_target:5.2f} | "
          f"極限(曲率/橫向G值): {k_future_max:.4f}/{g_future_max:.2f} | "
          f"入彎: {entry_sec:.1f}s/{dist_to_entry:.1f}m/{entry_conf:.0f}% | "
          f"最遠: {horizon_sec:.1f}s/{dyn_horizon_dist:.1f}m/{horizon_conf:.0f}%"
        )
        cloudlog.debug(log_msg)
        print(log_msg)
        self.log_timer = 0.0

    return super().update_target(sm, v_ego, a_ego, v_cruise)