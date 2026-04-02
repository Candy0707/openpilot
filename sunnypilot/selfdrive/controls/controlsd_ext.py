"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import cereal.messaging as messaging
from cereal import log, custom

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0

# 導入 HTD 模組
from openpilot.sunnypilot.selfdrive.controls.lib.human_turn_detection import HumanTurnDetection, HTDState


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()

    # --- 初始化 HTD ---
    self.htd = HumanTurnDetection()
    self.htd_state = HTDState.INACTIVE
    # ------------------

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP']
    self.pm_services_ext = ['carControlSP']

  def initialize_lateral_control(self, lac, CI, dt):
    # --- 攔截並注入 TSS2 動態熱備援控制器 (移除 try-except，錯誤直接報出) ---
    from opendbc.car.toyota.values import TSS2_CAR

    # 🌟 必須先用 .which() 判斷當前活躍的 Union 是不是 'torque'，避免 Cap'n Proto 底層報錯
    if self.CP.carFingerprint in TSS2_CAR and self.CP.lateralTuning.which() == 'torque':
      # 確認是 torque 後，才能安全讀取裡面的參數長度
      if len(self.CP.lateralTuning.torque.as_builder().to_dict()) > 0:
        # ⚠️ 請注意：這裡的 import 路徑必須完全正確。如果寫錯，開機就會直接報錯！
        from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_dynamic import LatControlDynamic
        return LatControlDynamic(self.CP, self.CP_SP, CI, dt)
    # ----------------------------------------

    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_versions = self.params.get("TorqueControlTune")
    if not enforce_torque_control:
      return lac

    if torque_versions == 0.0:  # v0
      return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
    else:
      return lac

  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["liveDelay"].lateralDelay)

      self._param_update_time = time.monotonic()

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    CS = sm['carState']
    _lat_active = False

    # 先判斷方向燈是否暫停橫向控制，或者依據 MADS 狀態決定 _lat_active
    if self.blinker_pause_lateral.update(CS):
      _lat_active = False
    else:
      ss_sp = sm['selfdriveStateSP']
      if ss_sp.mads.available:
        _lat_active = bool(ss_sp.mads.active)
      else:
        # MADS not available, use stock state to engage
        _lat_active = bool(sm['selfdriveState'].active)

    # --- 防呆開關判斷 (HTD) ---
    # 1. 不管開關有沒有開，永遠執行 update() 讓系統記錄扭力數據
    htd_allowed, self.htd_state = self.htd.update(
        _lat_active,
        CS.cruiseState.enabled,
        CS.steeringAngleDeg,
        CS.steeringTorque,
        CS.vEgo,
        CS.steeringPressed
    )

    # 2. 只有當車主在介面開啟 HTD 功能時，才真正允許 HTD 切斷自動轉向
    if self.htd._enabled:
        _lat_active = _lat_active and htd_allowed
    # -------------------------------------

    return _lat_active

  @staticmethod
  def get_lead_data(ld: log.RadarState.LeadData) -> dict:
    return {
      "dRel": ld.dRel,
      "yRel": ld.yRel,
      "vRel": ld.vRel,
      "aRel": ld.aRel,
      "vLead": ld.vLead,
      "dPath": ld.dPath,
      "vLat": ld.vLat,
      "vLeadK": ld.vLeadK,
      "aLeadK": ld.aLeadK,
      "fcw": ld.fcw,
      "status": ld.status,
      "aLeadTau": ld.aLeadTau,
      "modelProb": ld.modelProb,
      "radar": ld.radar,
      "radarTrackId": ld.radarTrackId,
    }

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    CC_SP.leadOne = self.get_lead_data(sm['radarState'].leadOne)
    CC_SP.leadTwo = self.get_lead_data(sm['radarState'].leadTwo)

    # MADS state
    CC_SP.mads = sm['selfdriveStateSP'].mads

    CC_SP.intelligentCruiseButtonManagement = sm['selfdriveStateSP'].intelligentCruiseButtonManagement

    return CC_SP

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)
