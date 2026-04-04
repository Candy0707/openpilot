"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.accel_controller import AccelPersonalityController
from openpilot.sunnypilot.selfdrive.controls.lib.dynamic_personality.dynamic_follow import FollowDistanceController
from openpilot.sunnypilot.selfdrive.controls.lib.dynamic_turn_speed_controller.dynamic_turn_speed_controller import DynamicTurnSpeedController
from openpilot.sunnypilot.selfdrive.controls.lib.path_deviation_monitor.path_deviation_monitor import PathDeviationMonitor
from opendbc.car.interfaces import ACCEL_MIN

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.accel_controller = AccelPersonalityController()
    self.dynamic_follow = FollowDistanceController()
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()
    self.dtsc = DynamicTurnSpeedController(CP, mpc)
    self.pdm = PathDeviationMonitor(CP, mpc)

    self.output_v_target = 0.0
    self.output_a_target = 0.0

  @property
  def mlsim(self) -> bool:
    # If we don't have a generation set, we assume it's default model. Which as of today are mlsim.
    return bool(self.generation is None or self.generation >= 11)

  def get_mpc_mode(self) -> str | None:
    if not self.dec.active():
      return None

    return self.dec.mode()

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def get_accel_clip(self, v_ego: float) -> list[float] | None:
    if self.accel_controller.is_enabled():
      return [ACCEL_MIN, self.accel_controller.get_max_accel(v_ego)]
    return None

  def get_cruise_min_accel(self, v_ego: float) -> float | None:
    if self.accel_controller.is_enabled():
      return self.accel_controller.get_min_accel(v_ego)
    return None

  def get_t_follow(self, v_ego: float) -> float | None:
    if self.dynamic_follow.is_enabled():
      return self.dynamic_follow.get_follow_distance_multiplier(v_ego)
    return None

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(
      long_enabled,
      long_override,
      v_ego,
      a_ego,
      v_cruise_cluster,
      self.resolver.speed_limit,
      self.resolver.speed_limit_final_last,
      has_speed_limit,
      self.resolver.distance,
      self.events_sp,
    )

    self.dtsc.update_target(sm, v_ego, a_ego, v_cruise)
    self.pdm.update_target(sm, v_ego, a_ego, v_cruise)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
      LongitudinalPlanSource.dtsc: (self.dtsc.output_v_target, self.dtsc.output_a_target),
      LongitudinalPlanSource.pdm: (self.pdm.output_v_target, self.pdm.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def update_a_desired_trajectory(self, sm: messaging.SubMaster, a_desired_trajectory: list[float], v_ego: float, t_follow_override: float):
    """
    攔截 MPC 輸出的加速度軌跡，根據跟車距離百分比實作主動滑行 (0.0 m/s^2)
    參數設定：-0.4 (微煞車攔截下限) 與 -1.4 (相對速差安全門檻)
    """
    # 1. 取得雷達與前車資訊
    radar_state = sm['radarState']
    lead_one = radar_state.leadOne

    # 無前車則不介入，直接回傳原始軌跡
    if not lead_one.status:
      return a_desired_trajectory

    # 2. 提取物理數值
    d_rel = lead_one.dRel  # 與前車的實際距離 (m)
    v_rel = lead_one.vRel  # 相對速度 (m/s)

    # 3. 計算 MPC 的理想跟車目標距離
    # 確保 t_follow_override 有效 (若為 None 則給予預設值 1.45s)
    tf = t_follow_override if t_follow_override is not None else 1.45

    # 計算動態安全距離：基礎距離 + (自車速度 * 跟車秒數)
    # 加上 4.0m 為最低安全墊片，避免低速塞車時除以過小的數值
    target_dist = max(v_ego * tf, 4.0)

    # 4. 計算當前距離是理想距離的百分比
    dist_perc = d_rel / target_dist

    # 5. 滑行判斷參數設定
    COAST_MIN_ACCEL = -0.4       # 攔截微煞車的範圍 (-0.4 ~ 0.0)
    SAFE_V_REL_THRESHOLD = -1.4  # 相對速差門檻，防止前車急煞時還在滑行

    # 6. 核心判斷：距離百分比 > 80% 且速差安全
    # 只要符合這個條件，就算是插入車 (Cut-in) 也會因為距離大於 80% 緩衝而進行柔和滑行
    is_safe_to_coast = (dist_perc > 0.8) and (v_rel > SAFE_V_REL_THRESHOLD)

    # 7. 執行軌跡覆寫
    if is_safe_to_coast:
      # a_desired_trajectory 是一個包含未來多個時間點的陣列
      for i in range(len(a_desired_trajectory)):
        # 只有在 MPC 要求「輕微煞車」時才介入改為 0.0 (純滑行)
        if COAST_MIN_ACCEL <= a_desired_trajectory[i] < 0.0:
          a_desired_trajectory[i] = 0.0
        # 如果 MPC 已經要求大於 -0.4 的重煞 (例如 -0.5)，代表模型判斷有風險
        # 此時提早 break 迴圈，保留 MPC 原生的防撞重煞反應
        elif a_desired_trajectory[i] < COAST_MIN_ACCEL:
          break

    return a_desired_trajectory

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)
    self.accel_controller.update(sm)
    self.dynamic_follow.update(sm)
    self.dtsc.update(sm)
    self.pdm.update(sm)




  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # ==========================================
    # 優雅寫入 targets 列表邏輯
    # ==========================================

    # 1. 取得 Enum (動態適應 custom.capnp 的變更)
    source = LongitudinalPlanSource.schema.enumerants

    # 2. 動態初始化 targets 陣列長度
    targets_list = longitudinalPlanSP.init('targets', len(source))

    # 2. 自動動態對應：遍歷 Enum 中的每一個定義 (例如 'cruise', 'dtsc' 等)
    for name, enum_value in source.items():
      # 使用 getattr 動態從 self 抓取同名的控制器實例
      # 例如：當 name 為 'dtsc' 時，getattr(self, 'dtsc') 等同於呼叫 self.dtsc
      controller = getattr(self, name, None)

      # 確保該屬性存在，且具有 write_to_msg 方法 (即繼承自 TargetsBase)
      if controller is not None and hasattr(controller, 'write_to_msg'):
        idx = enum_value
        controller.write_to_msg(targets_list[idx])

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    longitudinalPlanSP.accelPersonality = int(self.accel_controller.get_accel_personality())

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
