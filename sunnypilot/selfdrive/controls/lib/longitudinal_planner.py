"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
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
from openpilot.sunnypilot.selfdrive.controls.lib.adaptive_coasting_manager import AdaptiveCoastingManager
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
    self.acm = AdaptiveCoastingManager()
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

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

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def update_a_desired_trajectory(self, sm: messaging.SubMaster, a_desired_trajectory: list[float], v_ego, t_follow) -> list[float]:
    """
    軌跡仲裁與最後防線：
    1. 收集並仲裁 (取最低)
    2. 執行強制安全限制 (0.0 限速)
    3. 執行物理與 TTC 防撞校驗
    """
    radarState = sm['radarState']
    lead = radarState.leadOne

    a_traj_np = np.array(a_desired_trajectory)

    # ---------------------------------------------------------
    # 階段 1：多模組仲裁 (取得最小加速度)
    # ---------------------------------------------------------
    constraints = [a_traj_np] # 初始為 MPC 軌跡

    # 取得 ACM 的建議限制
    acm_a = self.acm.update(sm, v_ego, t_follow)
    if acm_a is not None:
      constraints.append(acm_a)

    # 執行仲裁：取最保守 (最低) 的加速度
    final_trajectory = a_traj_np
    for limit in constraints[1:]:
      final_trajectory = np.minimum(final_trajectory, limit)

    # ---------------------------------------------------------
    # 階段 2：狀態限制 (強制 0.0)
    # ---------------------------------------------------------
    if lead.status:
      d_safe = max(4.0, (v_ego * t_follow) * 0.75)

      # 若進入跟車距離，且前車並未加速逃離 (v_rel < 0.5)
      if lead.dRel < d_safe and lead.vRel < 0.5:
        # 強制將加速部分 (大於 0 的數值) 削平至 0.0
        final_trajectory = np.minimum(final_trajectory, 0.0)

    # ---------------------------------------------------------
    # 階段 3：最終物理與時間防線 (熔斷機制)
    # ---------------------------------------------------------
    if lead.status and lead.vRel < -0.1: # 只在顯著接近時校驗
      # a. TTC 防線：低於 2.0s 立即回傳原始 MPC 軌跡
      ttc = lead.dRel / abs(lead.vRel)
      if ttc < 2.0:
        return a_traj_np.tolist()

      # b. 物理防線：計算所需的減速度
      s_stop = max(0.2, lead.dRel - 2.0)
      a_req_physics = -(lead.vRel**2) / (2 * s_stop)

      # 取軌跡中最溫和的煞車值 (max) 來跟物理要求對比
      min_planned_braking = np.max(final_trajectory)

      # 如果規劃的煞車力道大於物理要求 (例如規劃 -0.5, 但物理需要 -1.5)
      # 代表目前仲裁結果有碰撞風險，強制回傳原始 MPC
      if min_planned_braking > a_req_physics:
        return a_traj_np.tolist()

    # 安全通過所有校驗，回傳仲裁後的軌跡
    return final_trajectory.tolist()

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)
    self.accel_controller.update(sm)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

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
