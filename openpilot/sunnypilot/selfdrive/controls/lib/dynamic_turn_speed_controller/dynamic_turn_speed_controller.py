"""
動態過彎速度控制器 (Dynamic Turn Speed Controller - DTSC)
"""

import math
import numpy as np

from openpilot.cereal import messaging
from opendbc.car import structs
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase


# ==========================================
# 全域物理常數與演算法參數定義
# ==========================================
TARGET_LAT_G = 0.15            # 目標舒適橫向向心 G 值 (0.15G)
MIN_LAT_G = 0.08               # 陡下坡時容許橫向向心 G 值(0.08G)
DOWNHILL_G_FACTOR = 0.5        # 下坡重力分量折減權重 (k_g)
GRAVITY = 9.81                 # 重力加速度常數 (m/s^2)
MIN_CURVATURE = 0.0015         # 最小有效曲率門檻
MIN_SPEED = 0.3                # 最小有效速度

# 運動學減速參數
CONVERGENCE_TAU = 1.5          # 動態距離下限之時間常數 (秒)

# 200 格時空網格參數 (時間跨度 10 秒)
NUM_SLOTS = 200
LINEAR_T = np.arange(0.0, 10.0, DT_MDL)
MODEL_T = np.array(ModelConstants.T_IDXS)
CONF_THRESHOLD = 0.60          # 信心度確認門檻 (60%)


class DynamicTurnSpeedController(TargetsBase):
  """
  動態過彎速度控制器
  """

  def __init__(self, CP: structs.CarParams, mpc):
    super().__init__(CP, mpc)

    # --- 控制狀態記憶體配置 ---
    self.latAction = False
    self.curvature = 0.0
    self.desiredCurvature = 0.0

    self.active_target_source: str = '當前曲率'
    self.targets: dict[str, list[float]] = {
      '當前曲率': [float(V_CRUISE_MAX), 0.0],
      '目標曲率': [float(V_CRUISE_MAX), 0.0],
      '預測曲率': [float(V_CRUISE_MAX), 0.0],
      '轉向限制': [float(V_CRUISE_MAX), 0.0],
    }

    # --- 200 格時空信心表記憶體配置 ---
    self.confidence_table = np.zeros(NUM_SLOTS)

    # --- 轉向不足濾波器 ---
    # x0=1.0 (初值基準), rc=1.0s (時間常數), dt=DT_MDL (步長 0.05s)
    self.steer_filter = FirstOrderFilter(1.0, 1.0, DT_MDL)
    # 內部過濾後之轉向不足比值
    self.steer_filtered = 1.0

  @staticmethod
  def calc_v_by_curvature(curvature: float | np.ndarray, max_lat_g: float | np.ndarray = TARGET_LAT_G) -> float | np.ndarray:
    """
    依據向心加速度物理公式換算目標車速：
      a_lat = v^2 / R = kappa * v^2
      => v = sqrt(a_lat / kappa) = sqrt((max_lat_g * g) / kappa)

    參數:
      curvature (float | np.ndarray): 路徑曲率 (1/m)
      max_lat_g (float | np.ndarray): 容許橫向加速度 G 值，預設為 TARGET_LAT_G (0.15G)

    回傳:
      float | np.ndarray: 物理速限 (m/s)；小於 MIN_CURVATURE 時回傳 V_CRUISE_MAX
    """
    if isinstance(curvature, np.ndarray) or isinstance(max_lat_g, np.ndarray):
      curv_abs = np.abs(curvature)
      a_lat_limit = np.asarray(max_lat_g, dtype=float) * GRAVITY
      # 曲率小於最小曲率，表示無需減速，直接回傳最大巡航速度
      return np.where(curv_abs < MIN_CURVATURE, float(V_CRUISE_MAX), np.sqrt(a_lat_limit / np.maximum(curv_abs, MIN_CURVATURE)))

    curv_abs = abs(curvature)
    # 曲率小於最小曲率，表示無需減速，直接回傳最大巡航速度
    if curv_abs < MIN_CURVATURE:
      return float(V_CRUISE_MAX)
    a_lat_limit = float(max_lat_g) * GRAVITY
    return math.sqrt(a_lat_limit / curv_abs)

  @staticmethod
  def calc_kinematic_target(v_ego: float, v_end: float, d_end: float) -> list[float]:
    """
    依據動態運動學公式計算單步目標車速與起始加速度。

    核心公式：
      a_req = (v_end^2 - v_ego^2) / (2 * d_end)

    參數:
      v_ego (float): 當前車速 (m/s)
      v_end (float): 目標車速 (m/s)
      d_end (float): 目標距離 (m)

    回傳:
      list[float]: [目標車速 v_target, 目標加速度 a_target]
    """
    # 目標速度大於當前車速，無需減速
    if v_end >= v_ego:
      return [v_end, 0.0]
    # 動態距離
    d_calc = max(d_end, v_ego * CONVERGENCE_TAU, MIN_SPEED * CONVERGENCE_TAU)
    # 動態運動學公式求解所需等減速度
    a_target = (v_end**2 - v_ego**2) / (2.0 * d_calc)
    v_target = v_ego + a_target * DT_MDL
    return [v_target, a_target]

  def update_target(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float):
    """
    核心目標車速與加速度運算迴圈。
    """
    # 200 格信心表時空物理平移 (向左平移 1 格，新的一格補 0.0)
    self.confidence_table[:-1] = self.confidence_table[1:]
    self.confidence_table[-1] = 0.0

    # 取得 controlsState 資料
    if sm.valid['controlsState']:
      cs = sm['controlsState']
      lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
      self.latAction = lac.active
      self.curvature = cs.curvature
      self.desiredCurvature = cs.desiredCurvature

    # 橫向控制未啟用時，重置內部目標與濾波器狀態，避免重新啟用瞬間殘留舊幀數值
    if not self.latAction:
      self.action = False
      self.active_target_source = '當前曲率'
      self.steer_filter.x = 1.0
      self.steer_filtered = 1.0
      for key in self.targets:
        self.targets[key] = [float(V_CRUISE_MAX), a_ego]
      self.v_target = float(V_CRUISE_MAX)
      self.a_target = a_ego
      return super().update_target(sm, v_ego, a_ego, v_cruise)

    # 算出當前曲率與目標曲率限制速度
    v_curvature = float(self.calc_v_by_curvature(self.curvature))
    self.targets['當前曲率'] = self.calc_kinematic_target(v_ego, v_curvature, 0.0)
    v_desiredCurvature = float(self.calc_v_by_curvature(self.desiredCurvature))
    self.targets['目標曲率'] = self.calc_kinematic_target(v_ego, v_desiredCurvature, 0.0)

    # 計算「前方預測彎道」動態減速 (僅使用 modelV2 相對俯仰角進行下坡坡度補償)
    # 每幀預設重置為無限制，確保 modelV2 無效、點數不足或出彎後不會殘留舊的低速目標
    self.targets['預測曲率'] = [float(V_CRUISE_MAX), a_ego]

    if sm.valid['modelV2']:
      model = sm['modelV2']
      # 檢查 modelV2 四組陣列皆具完整 33 點，避免系統啟動初期或空幀造成 np.interp 崩潰
      n_model_pts = len(MODEL_T)
      if (
        len(model.position.x) == n_model_pts
        and len(model.velocity.x) == n_model_pts
        and len(model.orientationRate.z) == n_model_pts
        and len(model.orientation.y) == n_model_pts
      ):
        x_raw = np.array(model.position.x)
        v_raw = np.maximum(np.array(model.velocity.x), MIN_SPEED)
        y_rate_raw = np.abs(np.array(model.orientationRate.z))
        pitch_raw = np.array(model.orientation.y)
        curvatures_33 = y_rate_raw / v_raw
        x_200 = np.interp(LINEAR_T, MODEL_T, x_raw)
        v_200 = np.interp(LINEAR_T, MODEL_T, v_raw)
        pitch_200 = np.interp(LINEAR_T, MODEL_T, pitch_raw)
        k_200 = np.interp(LINEAR_T, MODEL_T, curvatures_33)

        pitch_down_200 = np.maximum(0.0, -pitch_200)
        lat_g_eff_200 = np.maximum(TARGET_LAT_G * np.cos(pitch_200) - DOWNHILL_G_FACTOR * np.sin(pitch_down_200),MIN_LAT_G)
        v_200_limit = self.calc_v_by_curvature(k_200, lat_g_eff_200)
        curve_mask = (v_ego > 3.0) & (v_200 > 3.0) & (k_200 > MIN_CURVATURE) & (v_200_limit < v_cruise)

        # 滿足條件信心累積 (+0.05)，未滿足則衰減 (-0.01)
        self.confidence_table[curve_mask] += 0.05
        self.confidence_table[~curve_mask] -= 0.01
        self.confidence_table = np.clip(self.confidence_table, 0.0, 1.0)

        valid_indices = np.where(self.confidence_table > CONF_THRESHOLD)[0]
        if len(valid_indices) > 0:
          # 取「最低速限」點，
          v_valid_limits = v_200_limit[valid_indices]
          min_v_pos = int(np.argmin(v_valid_limits))
          best_idx = valid_indices[min_v_pos]

          v_curve_pred = float(v_valid_limits[min_v_pos])
          d_pred_max = max(0.0, float(x_200[best_idx]))
          self.targets['預測曲率'] = self.calc_kinematic_target(v_ego, v_curve_pred, d_pred_max)

    # 判斷轉向需求
    clipped_speed = max(v_ego, MIN_SPEED)
    actual_lateral_accel = self.curvature * (clipped_speed ** 2)
    desired_lateral_accel = self.desiredCurvature * (clipped_speed ** 2)
    steer_raw = abs(desired_lateral_accel) / (abs(actual_lateral_accel) + 1e-3)
    turning = abs(desired_lateral_accel) > MIN_LAT_G * GRAVITY
    steer_input = float(np.clip(steer_raw, 0.8, 1.6)) if turning else 1.0
    self.steer_filtered = self.steer_filter.update(steer_input)
    steer_factor = float(np.interp(self.steer_filtered, [1.05, 1.20], [1.0, 0.0]))

    # 轉向不足輸出降速目標；
    if steer_factor < 1.0:
      v_steer = v_ego * steer_factor
      self.targets['轉向限制'] = self.calc_kinematic_target(v_ego, v_steer, 0.0)
    else:
      self.targets['轉向限制'] = [float(V_CRUISE_MAX), a_ego]

    # 仲裁結果 (取最低目標車速；同速時取最保守加速度)
    best_source, best_target = min(
      self.targets.items(),
      key=lambda item: (item[1][0], item[1][1]),
    )
    self.active_target_source = best_source
    self.v_target = best_target[0]
    self.a_target = best_target[1]
    # 判定介入狀態 (action)：上鎖看 v_ego，解鎖看 v_cruise
    self.action = bool((self.action or (self.v_target < v_ego)) and (self.v_target < v_cruise))

    return super().update_target(sm, v_ego, a_ego, v_cruise)
