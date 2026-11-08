"""
動態過彎速度控制器 (Dynamic Turn Speed Controller - DTSC)

本模組繼承自 TargetsBase，負責在車輛行駛與即將過彎時提供動態降速目標：
1. 依據固定向心 G 值 (0.15G) 物理公式，將路徑曲率換算為舒適過彎極限速度。
2. 結合當前控制曲率與前方視覺模型 (modelV2) 預測之 33 個點完整路徑曲率。
3. 採用全路徑「最大曲率 (最低速)」為最保守安全約束，並透過「連續 20 幀確認」機制徹底排除單格視覺雜訊誤判。
4. 採用到達目標距離時剛好達標的「動態運動學即時減速度」，無固定舒適減速度限制。
5. 監控 EPS 方向盤橫向轉向扭矩，經一階低通濾波後於 0.6 ~ 0.9 區間線性插值至 1.0 ~ 0.0，直接對目標速度進行安全縮放。
"""

import math
import numpy as np

from openpilot.cereal import messaging
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase


# ==========================================
# 全域物理常數與演算法參數定義
# ==========================================
TARGET_LAT_G = 0.15        # 目標舒適橫向向心 G 值 (0.15G)
GRAVITY = 9.81            # 重力加速度常數 (m/s^2)
MIN_CURVATURE = 1e-4      # 最小有效曲率門檻 (半徑大於 10,000m 視為純直線，避免除以零)

# 防誤判連續確認幀數門檻 (modelV2 頻率為 20Hz，20 幀約等於連續觀測 1.0 秒)
PERSISTENT_FRAME_THRESHOLD = 20


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

    # --- 防誤判計數器 ---
    self.curve_frame_counter = 0  # 連續偵測到彎道降速需求的幀數計數器

    # --- 方向盤橫向扭力濾波器 ---
    # 依使用者規格：FirstOrderFilter(0, 0.1, 1 / DL_ML)
    # x0=0 (初值), rc=0.1 (時間常數), dt=1/DL_ML (步長 0.05s)
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
      max_lat_g (float): 容許橫向加速度 G 值，預設 0.2G

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
    來源 2: lateralManeuverPlan.desiredCurvature (若無或未啟用則取 model action 或車輛當前曲率)
    """
    # 來源 1: 控制器當下目標曲率
    curv_source_1 = sm['controlsState'].desiredCurvature

    # 來源 2: 規劃器或模型 Action 曲率
    if sm.valid.get('lateralManeuverPlan', False):
      curv_source_2 = sm['lateralManeuverPlan'].desiredCurvature
    else:
      curv_source_2 = sm['modelV2'].action.desiredCurvature

    # 取絕對值最大者代表當前最大轉向需求
    return max(abs(curv_source_1), abs(curv_source_2))

  def _eval_predictive_curve(self, sm: messaging.SubMaster, v_cruise: float) -> tuple[float, float, bool]:
    """
    遍歷視覺模型 (modelV2) 前方 33 個預測點，找出路徑上的全域最大曲率及其對應距離，
    並執行連續 20 幀防誤判判定。

    回傳:
      tuple[float, float, bool]: (最大曲率點對應之目標車速 v_curve, 縱向距離 d, 是否通過20幀確認)
    """
    x_pts = list(sm['modelV2'].position.x)
    y_pts = list(sm['modelV2'].position.y)
    num_pts = len(x_pts)

    # 1. 將車身自車原點 (0, 0) 加入前端，構築 34 個座標點點集
    # pts[0] 為 (0, 0)，pts[1] 為模型第 0 點 ... pts[33] 為模型第 32 點
    pts = [(0.0, 0.0)] + list(zip(x_pts, y_pts))

    # 2. 遍歷全部 33 個模型點計算 Menger 離散曲率
    curvatures = []
    for i in range(num_pts):
      if i == 0:
        # 第 0 點 (起點)：結合自車原點 (0, 0)、第 0 點、第 1 點
        k = menger_curvature(pts[0], pts[1], pts[2])
      elif i == num_pts - 1:
        # 第 32 點 (終點)：邊界點，沿用倒數前三點幾何曲率 (30, 31, 32)
        k = menger_curvature(pts[num_pts - 2], pts[num_pts - 1], pts[num_pts])
      else:
        # 中間點 (1~31)：取相鄰前、中、後三點 (i-1, i, i+1)
        k = menger_curvature(pts[i], pts[i + 1], pts[i + 2])
      curvatures.append(abs(k))

    # 3. 找出全路徑之最大曲率點與其縱向距離 (最保守安全策略)
    max_idx = int(np.argmax(curvatures))
    k_pred_max = curvatures[max_idx]
    d_pred_max = max(0.0, float(x_pts[max_idx]))

    # 計算該最大曲率在 0.2G 下允許之過彎速度
    v_curve_pred = self.calc_v_by_curvature(k_pred_max)

    # 4. 20 幀防誤判判定邏輯 (以曲率絕對值為基準，完全不區分正負左右彎，通吃 S 彎)
    # 觸發條件：曲率大於直線雜訊門檻且目標速小於巡航定速，代表前方存在具體降速需求
    has_curve_demand = (k_pred_max > MIN_CURVATURE) and (v_curve_pred < v_cruise)

    if has_curve_demand:
      self.curve_frame_counter += 1
    else:
      # 一旦彎道需求消失 (如噪訊中斷或已回正)，計數器歸零
      self.curve_frame_counter = 0

    is_confirmed = (self.curve_frame_counter >= PERSISTENT_FRAME_THRESHOLD)

    return v_curve_pred, d_pred_max, is_confirmed

  def update_target(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float):
    """
    核心目標車速與加速度運算迴圈。
    """
    # -------------------------------------------------------------
    # 步驟 1: 計算「當前狀態」即時目標車速
    # -------------------------------------------------------------
    curr_curvature = self._get_current_curvature(sm)
    v_current = self.calc_v_by_curvature(curr_curvature)

    # -------------------------------------------------------------
    # 步驟 2: 計算「前方預測彎道」目標車速與動態運動學減速
    # -------------------------------------------------------------
    v_curve_pred, d_pred, is_curve_confirmed = self._eval_predictive_curve(sm, v_cruise)

    # 預設預測速度不限制
    v_predictive = V_CRUISE_MAX
    kinematic_a_req = 0.0

    if is_curve_confirmed:
      # 通過連續 20 幀驗證，前方彎道確認為真
      if d_pred > 1.0 and v_ego > v_curve_pred:
        # 動態運動學公式：v_end^2 - v_start^2 = 2 * a * d
        # 目標是在到達位置 d 時剛好完成減速至 v_curve_pred，故動態解算所需加速度 a_req：
        kinematic_a_req = (v_curve_pred**2 - v_ego**2) / (2.0 * d_pred)

        # 當前控制週期 (DT_MDL) 依據即時動態減速度所推進的平滑目標速度
        # 並以 v_curve_pred 作為下限保護
        v_predictive = max(v_curve_pred, v_ego + kinematic_a_req * DT_MDL)
      else:
        # 已達彎心或車速已低於/等於彎道速限，直接錨定為該彎道速限
        v_predictive = v_curve_pred
        kinematic_a_req = 0.0

    # -------------------------------------------------------------
    # 步驟 3: 綜合幾何曲率速限 (取當前與預測之最小值)
    # -------------------------------------------------------------
    v_geom_target = min(v_current, v_predictive)

    # -------------------------------------------------------------
    # 步驟 4: 方向盤扭力一階低通濾波與線性插值安全縮放
    # -------------------------------------------------------------
    # 取得 carOutput 當前輸出扭力絕對值
    torque_raw = 0.0
    if 'carOutput' in sm and hasattr(sm['carOutput'], 'actuatorsOutput'):
      torque_raw = abs(sm['carOutput'].actuatorsOutput.torque)

    # 送入一階低通濾波器
    self.torque_filtered = self._torque_filter.update(min(1.0, torque_raw))

    # 依使用者規格：0.6 ~ 0.9 對應係數 1.0 ~ 0.0
    # 當扭力 >= 0.9 時，係數降為 0.0 (以停車為最高安全原則，防止衝出車道)
    torque_factor = float(np.interp(self.torque_filtered, [0.6, 0.9], [1.0, 0.0]))

    # -------------------------------------------------------------
    # 步驟 5: 結算最終目標車速與規劃加速度
    # -------------------------------------------------------------
    # 直接乘上扭力安全係數
    self.v_target = v_geom_target * torque_factor

    # 加速度設定：若預測彎道正在執行減速且該減速度小於當前加速度，採用動態減速度作為起點
    if is_curve_confirmed and kinematic_a_req < 0.0:
      self.a_target = kinematic_a_req
    else:
      self.a_target = a_ego

    # 判定介入狀態 (action)：當目標車速顯著低於巡航設定速、扭力開始抑制、或彎道生效時即標記介入
    self.action = bool((self.v_target < (v_cruise - 0.2)) or (torque_factor < 1.0) or is_curve_confirmed)

    # 呼叫父類別 TargetsBase 的仲裁邏輯與系統日誌輸出
    return super().update_target(sm, v_ego, a_ego, v_cruise)
