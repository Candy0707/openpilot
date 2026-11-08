"""
動態過彎速度控制器 (Dynamic Turn Speed Controller - DTSC)

本模組繼承自 TargetsBase，負責在車輛行駛與即將過彎時提供動態降速目標：
1. 依據固定向心 G 值 (0.15G) 物理公式，將路徑曲率換算為舒適過彎極限速度。
2. 結合當前控制曲率與前方視覺模型 (modelV2) 預測之 33 個點完整路徑曲率。
3. 採用 200 格時空網格平移累積信心表，於確認彎道中提取「全路徑最大曲率 (最低速)」為最保守安全約束。
4. 只要轉彎判定成立即無條件套用「動態運動學即時減速度」，由 TargetsBase 進行硬體邊界鉗位。
5. 監控 EPS 方向盤橫向轉向扭矩 (僅於 torqueState 時讀取)，經一階低通濾波後於 0.6 ~ 0.9 區間線性插值至 1.0 ~ 0.0。
"""

import math
import numpy as np

from openpilot.cereal import messaging
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase


# ==========================================
# 全域物理常數與演算法參數定義
# ==========================================
TARGET_LAT_G = 0.15        # 目標舒適橫向向心 G 值 (0.15G)
GRAVITY = 9.81            # 重力加速度常數 (m/s^2)
MIN_CURVATURE = 1e-4      # 最小有效曲率門檻 (半徑大於 10,000m 視為純直線，避免除以零)

# 200 格時空網格參數 (時間跨度 10 秒)
NUM_SLOTS = 200
LINEAR_T = np.arange(0.0, 10.0, DT_MDL)
MODEL_T = np.array(ModelConstants.T_IDXS)
CONF_THRESHOLD = 0.60     # 信心度確認門檻 (60%)


def menger_curvature(ptA: tuple[float, float], ptB: tuple[float, float], ptC: tuple[float, float]) -> float:
  """
  利用三點外接圓 (Menger Curvature) 計算點 B 的離散幾何曲率。

  幾何公式：
    kappa = (2 * |(xB - xA)(yC - yA) - (yB - yA)(xC - xA)|) / (d_AB * d_BC * d_AC)
  """
  xA, yA = ptA
  xB, yB = ptB
  xC, yC = ptC

  d_AB = math.hypot(xB - xA, yB - yA)
  d_BC = math.hypot(xC - xB, yC - yB)
  d_AC = math.hypot(xC - xA, yC - yA)

  # 計算兩向量之二維叉積絕對值 (對應兩倍三角形面積)
  cross = 2.0 * abs((xB - xA) * (yC - yA) - (yB - yA) * (xC - xA))
  denom = d_AB * d_BC * d_AC

  # 避免三點重合或距離過近導致除以零
  if denom < 1e-5:
    return 0.0

  return cross / denom


class DynamicTurnSpeedController(TargetsBase):
  """
  動態過彎速度控制器子類別
  """

  def __init__(self, CP: structs.CarParams, mpc):
    super().__init__(CP, mpc)

    # --- 200 格時空信心表記憶體配置 ---
    self.confidence_table = np.zeros(NUM_SLOTS)

    # --- 方向盤橫向扭力濾波器 ---
    # x0=0 (初值), rc=0.1 (時間常數), dt=DT_MDL (步長 0.05s)
    self._torque_filter = FirstOrderFilter(0.0, 0.1, DT_MDL)

    # 內部過濾後之扭矩數值 (0.0 ~ 1.0)
    self.torque_filtered = 0.0

  @staticmethod
  def calc_v_by_curvature(curvature: float, max_lat_g: float = TARGET_LAT_G) -> float:
    """
    依據向心加速度物理公式換算目標車速：
      a_lat = v^2 / R = kappa * v^2
      => v = sqrt(a_lat / kappa) = sqrt((max_lat_g * g) / kappa)

    參數:
      curvature (float): 路徑曲率 (1/m)
      max_lat_g (float): 容許橫向加速度 G 值，預設 0.15G

    回傳:
      float: 物理速限 (m/s)
    """
    curv_abs = abs(curvature)
    if curv_abs < MIN_CURVATURE:
      return V_CRUISE_MAX

    a_lat_limit = max_lat_g * GRAVITY
    return math.sqrt(a_lat_limit / curv_abs)

  def _get_current_curvature(self, sm: messaging.SubMaster) -> float:
    """
    取得車輛當前瞬時控制曲率 (取兩大來源絕對值之最大值)。
    來源 1: controlsState.desiredCurvature
    來源 2: lateralManeuverPlan.desiredCurvature / modelV2.action.desiredCurvature
    """
    curv_source_1 = sm['controlsState'].desiredCurvature

    curv_source_2 = 0.0
    if sm.valid.get('lateralManeuverPlan', False):
      curv_source_2 = sm['lateralManeuverPlan'].desiredCurvature
    else:
      curv_source_2 = sm['modelV2'].action.desiredCurvature

    return max(abs(curv_source_1), abs(curv_source_2))

  def _eval_predictive_curve(self, sm: messaging.SubMaster, v_cruise: float) -> tuple[float, float, bool]:
    """
    遍歷視覺模型 (modelV2) 前方 33 個預測點，利用 200 格時空網格累積信心表驗證前方彎道，
    並在確認成立時提取全域最大曲率及其距離。

    回傳:
      tuple[float, float, bool]: (最大曲率點對應之目標車速 v_curve, 縱向距離 d, 是否確認轉彎成立)
    """
    # 【檢查是否有 33 點】若長度不足則略過預測計算，平緩衰減信心表
    if len(sm['modelV2'].position.x) < 33 or len(sm['modelV2'].position.y) < 33:
      self.confidence_table[:-1] = self.confidence_table[1:]
      self.confidence_table[-1] = 0.0
      return V_CRUISE_MAX, 0.0, False

    x_raw = np.array(sm['modelV2'].position.x)
    y_raw = np.array(sm['modelV2'].position.y)
    num_pts = len(x_raw)

    # 1. 將自車原點 (0, 0) 加入前端，構築 34 個座標點集計算 33 點 Menger 曲率
    pts = [(0.0, 0.0)] + list(zip(x_raw, y_raw))
    curvatures_33 = np.zeros(num_pts)
    for i in range(num_pts):
      if i == 0:
        k = menger_curvature(pts[0], pts[1], pts[2])
      elif i == num_pts - 1:
        k = menger_curvature(pts[num_pts - 2], pts[num_pts - 1], pts[num_pts])
      else:
        k = menger_curvature(pts[i], pts[i + 1], pts[i + 2])
      curvatures_33[i] = abs(k)

    # 2. 插值映射至 200 格等距時空網格 (0 ~ 10 秒)
    x_200 = np.interp(LINEAR_T, MODEL_T, x_raw)
    k_200 = np.interp(LINEAR_T, MODEL_T, curvatures_33)

    # 3. 200 格信心表時空物理平移 (向左平移 1 格，新的一格補 0.0)
    self.confidence_table[:-1] = self.confidence_table[1:]
    self.confidence_table[-1] = 0.0

    # 計算各網格槽位在 0.15G 下的速限
    v_200_limit = np.sqrt((TARGET_LAT_G * GRAVITY) / np.maximum(k_200, MIN_CURVATURE))

    # 判定各時空槽位是否存在降速需求 (曲率大於門檻且速限小於巡航速)
    curve_mask = (k_200 > MIN_CURVATURE) & (v_200_limit < v_cruise)

    # 滿足條件信心累積 (+0.05)，未滿足則衰減 (-0.01)
    self.confidence_table[curve_mask] += 0.05
    self.confidence_table[~curve_mask] -= 0.01
    self.confidence_table = np.clip(self.confidence_table, 0.0, 1.0)

    # 4. 判定轉彎是否成立 (信心度高於 60% 的有效槽位)
    valid_indices = np.where(self.confidence_table > CONF_THRESHOLD)[0]
    is_curve_confirmed = len(valid_indices) > 0

    if is_curve_confirmed:
      # 在所有通過信心驗證的網格中，尋找全域最大曲率 (最保守安全策略)
      max_valid_idx = valid_indices[np.argmax(k_200[valid_indices])]
      k_pred_max = float(k_200[max_valid_idx])
      d_pred_max = max(0.0, float(x_200[max_valid_idx]))

      v_curve_pred = self.calc_v_by_curvature(k_pred_max)
      return v_curve_pred, d_pred_max, True

    return V_CRUISE_MAX, 0.0, False

  def update_target(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float):
    """
    核心目標車速與加速度運算迴圈。
    """
    # -------------------------------------------------------------
    # 步驟 1: 計算「當前狀態」即時目標車速 (不受 modelV2 影響)
    # -------------------------------------------------------------
    curr_curvature = self._get_current_curvature(sm)
    v_current = self.calc_v_by_curvature(curr_curvature)

    # -------------------------------------------------------------
    # 步驟 2: 檢查 Model 存在並計算「前方預測彎道」動態減速
    # -------------------------------------------------------------
    # 【update_target 檢查 model 存在保護】若不存在或無效則略過預測計算，不 return
    has_model = ('modelV2' in sm and sm.valid.get('modelV2', False))
    if has_model:
      v_curve_pred, d_pred, is_curve_confirmed = self._eval_predictive_curve(sm, v_cruise)
    else:
      self.confidence_table[:-1] = self.confidence_table[1:]
      self.confidence_table[-1] = 0.0
      v_curve_pred, d_pred, is_curve_confirmed = V_CRUISE_MAX, 0.0, False

    v_predictive = V_CRUISE_MAX

    if is_curve_confirmed:
      # 只要判斷轉彎成立，就套用減速公式，無須再判斷距離與速度
      # 動態運動學公式：a_req = (v_end^2 - v_start^2) / (2 * d)
      d_calc = max(d_pred, 1.0)  # 分母保護避免除以零
      kinematic_a_req = (v_curve_pred**2 - v_ego**2) / (2.0 * d_calc)

      # 單步推進目標車速，並將加速度設定為求解出的動態加速度 (由 TargetsBase 進行 ACCEL_MIN/MAX 夾取)
      v_predictive = max(0.0, v_ego + kinematic_a_req * DT_MDL)
      self.a_target = kinematic_a_req
    else:
      self.a_target = a_ego

    # -------------------------------------------------------------
    # 步驟 3: 綜合幾何曲率速限 (取當前與預測之最小值)
    # -------------------------------------------------------------
    v_geom_target = min(v_current, v_predictive)

    # -------------------------------------------------------------
    # 步驟 4: 方向盤扭力一階低通濾波與線性插值安全縮放
    # -------------------------------------------------------------
    # 【扭力讀取保護】
    # torque_factor 預設為 1.0 (100% 車速無抑制)
    # 僅在 controlsState 為 torqueState 時才讀取扭矩與插值，非扭力狀態絕不強制停車
    torque_factor = 1.0
    if sm['controlsState'].lateralControlState.which() == 'torqueState':
      torque_raw = abs(sm['controlsState'].lateralControlState.torqueState.output)
      self.torque_filtered = self._torque_filter.update(min(1.0, torque_raw))
      # 0.6 ~ 0.9 對應係數 1.0 ~ 0.0 (>= 0.9 強制歸零以保護車輛不推頭)
      torque_factor = float(np.interp(self.torque_filtered, [0.6, 0.9], [1.0, 0.0]))
    else:
      # 非扭力狀態平緩重置濾波器
      self.torque_filtered = self._torque_filter.update(0.0)

    # -------------------------------------------------------------
    # 步驟 5: 結算最終目標車速與作動狀態
    # -------------------------------------------------------------
    self.v_target = v_geom_target * torque_factor

    # 判定介入狀態 (action)：當目標車速低於巡航設定速、轉彎成立、或扭力受限時標記介入
    self.action = bool((self.v_target < v_cruise) or is_curve_confirmed or (torque_factor < 1.0))

    # 呼叫父類別 TargetsBase 的仲裁邏輯 (包含 ACCEL_MIN/MAX 極限保護) 與系統日誌輸出
    return super().update_target(sm, v_ego, a_ego, v_cruise)
