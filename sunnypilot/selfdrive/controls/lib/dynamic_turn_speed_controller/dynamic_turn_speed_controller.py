"""
================================================================================
Dynamic Turn Speed Controller (DTSC) - Apex TTA & Telemetry Edition
自動駕駛動態彎道速度控制器 (極限值決策 + 零頓挫平滑煞車 + 遙測日誌版) 🚀
================================================================================

【架構哲學 (Architecture Philosophy)】
傳統的彎道減速往往依賴靜態地圖或單一曲率閾值，容易造成「幽靈煞車」或「入彎太晚」。
本系統採用「時空軌跡重疊 (Spatio-Temporal Overlap)」與「運動學反推 (Kinematic Inversion)」：
1. 信心建立：AI 模型必須在時間推移中，持續且穩定地在「同一個物理位置」預測出彎道特徵
   (大曲率或高 G 力)，信心度才會攀升。這有效過濾了模型的瞬間閃爍雜訊。
2. 極限決策：不看整條軌跡的平均值，而是像賽車手一樣，直接在可信視野內尋找「最刁鑽的彎心 (Apex)」，
   並用該點的極限物理量一次性決定安全車速。
3. 完美煞車：利用與彎道起點的剩餘距離，套用等加速度運動學公式 (TTA)，確保減速過程絲滑無頓挫。
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
MAX_ACCEL = 0.0   # 系統介入時最大允許的加速度 (0.0 代表本系統只負責煞車，加速交還給基礎巡航)
MIN_ACCEL = -3.5  # 最大煞車力道 (m/s²)，接近人類重踩煞車的極限，確保安全

# 時間軸設定 (Openpilot 模型頻率為 20Hz，即 DT = 0.05s)
NUM_SLOTS = 200                          # 將未來預測切分為 200 格 (總計 10 秒預測)
DT_MDL = 0.05
LINEAR_T = np.arange(0.0, 10.0, DT_MDL)  # 建立均勻的線性時間軸
MODEL_T = np.array(ModelConstants.T_IDXS)  # Openpilot 原始的非均勻時間軸 (遠處點較稀疏)

# 彎道判定與信心閾值
MIN_CURVATURE = 0.002    # 最小曲率門檻：低於此值視為大直路，避免在直線上微調方向盤被誤判
TRIGGER_LAT_ACCEL = 0.6  # 觸發信心累積的預測 G 力門檻：專門捕捉高速公路的大緩彎 (曲率小但 G 值高)
CONF_THRESHOLD = 0.60    # 絕對信心門檻：軌跡穩定度大於 60%，系統才認定此為真實彎道並啟動減速

# 舒適動態查表 (Comfort Lookup Table)
# 根據不同車速 (km/h) 定義人類乘客可容忍的舒適橫向 G 力 (m/s²)
COMFORT_SPEEDS_KPH = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0]
COMFORT_LAT_ACCELS = [1.2, 1.2, 1.2, 1.3, 1.4, 1.5, 1.6, 1.6, 1.8, 1.8, 1.8, 1.8, 1.8]

# 物理防線
STAT_HARD_LIMIT_LAT_ACCEL = 2.0  # 絕對物理極限，不論查表數值為何，系統絕不允許過彎 G 力超過 2.0G


class DynamicTurnSpeedController(TargetsBase):
  def __init__(self, CP: structs.CarParams, mpc=None):
    super().__init__(CP, mpc)
    self.log_timer = 0.0

    # 狀態記憶陣列
    self.confidence_table = np.zeros(NUM_SLOTS)   # 核心：儲存未來 10 秒每一格的彎道確信度 (0.0 ~ 1.0)
    self.ui_confidence = np.zeros(len(MODEL_T))   # UI 渲染：提供給 HUD 的 33 格信心地毯數據
    self.v_target_history = np.zeros(10)          # 輸出平滑：10 幀環形緩衝區，防止目標車速跳動

  def update_target(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float):
    model_msg = sm['modelV2']
    ctrl_name = self.__class__.__name__

    # 基礎防呆：若模型當機或無軌跡輸出，直接放行巡航設定
    if model_msg is None or len(model_msg.position.x) == 0:
      self.v_target, self.a_target = v_cruise, a_ego
      return super().update_target(sm, v_ego, a_ego, v_cruise)

    # 限制最低運算基準車速，防止後續公式出現 ZeroDivisionError (除以零)
    v_ego_clp = max(v_ego, 0.3)

    # ==========================================================
    # [1. 車體狀態與虛擬軌跡萃取]
    # ==========================================================
    # 讀取實車感測器計算的當下真實曲率
    cs_curvature = sm['controlsState'].curvature if sm.valid['controlsState'] else 0.0

    # 擷取 AI 模型的 33 個原始預測節點
    x_raw = np.array(model_msg.position.x)                      # 縱向距離
    v_raw = np.maximum(np.array(model_msg.velocity.x), 0.1)     # 預測車速
    y_rate_raw = np.abs(np.array(model_msg.orientationRate.z))  # 預測橫擺角速度 (Yaw Rate)
    pitch_raw = np.array(model_msg.orientation.y)               # 預測俯仰角 (坡度)

    # 利用線性插值 (Interpolation) 將非均勻的 33 個點，展開為 200 個均勻的時間格 (每 0.05 秒一格)
    x_200 = np.interp(LINEAR_T, MODEL_T, x_raw)
    pitch_200 = np.interp(LINEAR_T, MODEL_T, pitch_raw)

    # 曲率計算 k = (Yaw Rate) / Velocity
    # 為了避免車輛靜止或蠕行時，微小的方向盤轉動導致曲率運算結果爆炸，設定分母下限為 1.0 m/s
    v_raw_k = np.maximum(v_raw, 1.0)
    k_raw = y_rate_raw / v_raw_k

    # 將預測 G 力與曲率展開至 200 格陣列
    g_200 = np.interp(LINEAR_T, MODEL_T, y_rate_raw * v_raw)
    k_200_raw = np.interp(LINEAR_T, MODEL_T, k_raw)

    # 坡度補償：若面臨下坡 (pitch 為負)，重力分量會加劇外拋風險，因此人為放大預測曲率作為安全餘裕
    slope_factor = np.clip(np.abs(pitch_200) * 5.0, 0.0, 0.5)
    k_200_safe = k_200_raw * (1.0 + slope_factor)

    # ==========================================================
    # [2. 信心感知 (Confidence Perception)] 時空平移濾波器
    # ==========================================================
    # 將上一幀的信心畫布向左平移一格 (代表 0.05 秒過去了，未來的軌跡逼近了車頭)
    self.confidence_table[:-1] = self.confidence_table[1:]
    self.confidence_table[-1] = 0.0  # 最遠的未來補零，等待新資料

    # 定義彎道特徵遮罩：滿足「預測 G 力過大」或「預測曲率過大」任一條件即可
    curve_mask = (g_200 > TRIGGER_LAT_ACCEL) | (k_200_safe > MIN_CURVATURE)

    # 執行充放電邏輯：
    # 如果同一個物理位置在這一幀依然被判定為彎道，信心 +5%；若特徵消失，信心緩慢 -1% 避免錯殺
    self.confidence_table[curve_mask] += 0.05
    self.confidence_table[~curve_mask] -= 0.01
    self.confidence_table = np.clip(self.confidence_table, 0.0, 1.0)

    # ==========================================================
    # [3. 動態視距與極限萃取 (Dynamic Horizon & Apex Extraction)]
    # ==========================================================
    # 尋找信心畫布中，所有大於採信門檻 (60%) 的索引值
    valid_indices = np.where(self.confidence_table > CONF_THRESHOLD)[0]

    is_curve_ahead = False
    k_future_max = 0.0001
    g_future_max = 0.0
    v_decision_final = v_cruise

    # 初始化遙測與計算用變數
    entry_sec = 0.0          # 預計入彎時間 (秒)
    horizon_sec = 0.0        # 最遠視距時間 (秒)
    dist_to_entry = 0.0      # 距入彎點真實物理距離 (公尺)
    dyn_horizon_dist = 0.0   # 視距極限物理距離 (公尺)
    entry_conf = 0.0         # 入彎點信心度 (%)
    horizon_conf = 0.0       # 視距終點信心度 (%)

    # 若視野內存在大於 60% 信心的軌跡，確立「前方有彎道」
    if len(valid_indices) > 0:
      is_curve_ahead = True

      # 獲取高信心區段的頭 (入彎點) 與 尾 (最遠視距)
      entry_idx = valid_indices[0]
      horizon_idx = valid_indices[-1]

      # 計算空間距離 (公尺)
      dist_to_entry = x_200[entry_idx]
      dyn_horizon_dist = x_200[horizon_idx]

      # 計算時間距離 (秒)
      entry_sec = entry_idx * DT_MDL
      horizon_sec = horizon_idx * DT_MDL

      # 擷取當前格子的實際信心百分比
      entry_conf = self.confidence_table[entry_idx] * 100
      horizon_conf = self.confidence_table[horizon_idx] * 100

      # 🌟 核心突破：直接從這段高信心軌跡中，萃取出「最極限的彎心 (Apex)」數據
      k_future_max = float(np.max(k_200_safe[valid_indices]))
      g_future_max = float(np.max(g_200[valid_indices]))

      # ==========================================================
      # [4. 速度決策 (Speed Decision)]
      # ==========================================================
      # 將當前車速代入查表，得出人類乘客當下能接受的舒適 G 力極限
      comfort_speeds_ms = [v * CV.KPH_TO_MS for v in COMFORT_SPEEDS_KPH]
      comfort_g_limit = float(np.interp(v_ego, comfort_speeds_ms, COMFORT_LAT_ACCELS))

      # A. 實車當下安全限速 (應對車輛已經駛入彎道中的狀況)
      act_k_raw = max(abs(cs_curvature), 1e-5)
      act_k_safe = act_k_raw * (1.0 + np.clip(-pitch_200[0] * 5.0, 0.0, 0.5))
      v_act_hard_g = np.sqrt(STAT_HARD_LIMIT_LAT_ACCEL / act_k_safe)  # 物理極限車速
      v_act_k_safe = np.sqrt(comfort_g_limit / act_k_safe)            # 舒適極限車速
      v_actual_decision = min(v_act_hard_g, v_act_k_safe)

      # B. 預測未來安全限速 (應對遠方即將到來的彎心 Apex)
      v_mod_hard_g = np.sqrt(STAT_HARD_LIMIT_LAT_ACCEL / max(k_future_max, 1e-5))
      v_mod_k_safe = np.sqrt(comfort_g_limit / max(k_future_max, 1e-5))
      v_model_decision = min(v_mod_hard_g, v_mod_k_safe)

      # 最終決策：整合實車現狀、預測未來與駕駛設定的巡航時速，取最保守 (最小) 的數值
      v_decision_final = min(v_model_decision, v_actual_decision, v_cruise)

    # --- 狀態機：介入與退出條件 ---
    # 退出條件：車體已回正 (實體曲率極小) 且 模型視野內已無彎道特徵
    exit_condition = (abs(cs_curvature) < MIN_CURVATURE) and (not is_curve_ahead)

    if v_ego > 0.1:
      if self.action:
        if exit_condition:
          self.action = False
      else:
        if is_curve_ahead:
          self.action = True
    else:
      self.action = False  # 靜止或極低速時強制解除系統，避免蠕行頓挫

    # ==========================================================
    # [5. 提前減速 (TTA - Time To Arrival Braking)]
    # ==========================================================
    a_target_tta = 0.0

    if self.action and is_curve_ahead:
      # 🟩 零頓挫平滑演算法：完全信任物理距離，不再人為預扣 1 秒緩衝。
      # 這能確保分母 S 最大化，讓算出的減速度 a 極度線性柔和。
      safe_braking_dist = max(dist_to_entry, 1.0)

      # 狀況 A：需要重煞 (當前車速高於目標安全車速 0.5 m/s 以上)
      if v_ego > v_decision_final + 0.5:
        # 運動學公式：a = (Vf² - Vi²) / 2S
        # 目標是在抵達入彎點 (S) 時，車速剛好降至安全時速 (Vf)
        a_req = (v_decision_final**2 - v_ego**2) / (2.0 * safe_braking_dist)
        a_target_tta = min(a_req, 0.0) # 此階段僅允許系統踩煞車，嚴禁補油門

      # 狀況 B：彎中巡航或出彎 (已達安全速度)
      else:
        # 轉換為溫和的 P-Controller (Proportional Controller)，以 1.5 秒的反應時間平滑貼合目標車速
        a_target_tta = (v_decision_final - v_ego) / 1.5

    # ==========================================================
    # [6. 輸出控制與 10 幀環形平滑 (Ring Buffer Smoothing)]
    # ==========================================================
    # 確保輸出的加速度符合車輛物理極限防護
    self.a_target = np.clip(a_target_tta, MIN_ACCEL, MAX_ACCEL)

    if self.action:
      # 反推虛擬前導車速 (Virtual Lead Speed)
      # 為了與 Openpilot 原生的縱向 MPC 完美融合，我們將需要的加速度，反推為前方 4 秒處的一台虛擬車輛的速度
      v_target_raw = np.sqrt(STAT_HARD_LIMIT_LAT_ACCEL / max(k_future_max, 1e-5)) + (self.a_target * 4.0)
      v_target_raw = np.clip(v_target_raw, 0.0, v_cruise)

      # 環形緩衝區平移：拋棄最舊的幀 (index 9)，存入最新的幀 (index 0)
      self.v_target_history[1:] = self.v_target_history[:-1]
      self.v_target_history[0] = v_target_raw
      # 取 10 幀平均值輸出，徹底抹平微小的運算抖動
      self.v_target = float(np.mean(self.v_target_history))
    else:
      # 待機狀態：持續用當前車速填滿緩衝區，確保下次系統介入的瞬間不會產生車速跳變 (無縫接軌)
      self.v_target_history.fill(v_ego)

    # ==========================================================
    # [7. UI 視覺渲染與遙測日誌 (Telemetry & Logging)]
    # ==========================================================
    # 🟩 視覺優化：將高解析的 200 格信心陣列，降採樣回 33 格交給 HUD 渲染。
    # 乘上 0.5 將透明度設定為 50%，打造具備科技感且不刺眼的高級信心地毯。
    self.ui_confidence = np.interp(MODEL_T, LINEAR_T, self.confidence_table)

    self.log_timer += DT_MDL

    # 狀態機紀錄：控制終端機輸出頻率 (每 0.5 秒一筆)
    if self.action:
      if self.log_timer >= 0.5:
        # 動態判定車輛當前與彎道的相對位置
        if dist_to_entry < 5.0 and is_curve_ahead:
          state_str = "🎯 彎心中"
        else:
          state_str = "📉 準備入彎"

        # 🟩 高解析遙測輸出 (包含秒數、距離與信心度)
        log_msg = (
          f"[{ctrl_name}] {state_str} | "
          f"V(現/安/終): {v_ego * 3.6:4.1f}/{v_decision_final * 3.6:4.1f}/{self.v_target * 3.6:4.1f} | "
          f"A(輸出): {self.a_target:5.2f} | "
          f"極限(K/G): {k_future_max:.4f}/{g_future_max:.2f} | "
          f"入彎: {entry_sec:.1f}s/{dist_to_entry:.1f}m/{entry_conf:.0f}% | "
          f"最遠: {horizon_sec:.1f}s/{dyn_horizon_dist:.1f}m/{horizon_conf:.0f}%"
        )
        cloudlog.warning(log_msg)
        print(log_msg)
        self.log_timer = 0.0

    return super().update_target(sm, v_ego, a_ego, v_cruise)