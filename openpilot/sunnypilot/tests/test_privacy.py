import pytest

from openpilot.sunnypilot import privacy
from openpilot.sunnypilot.privacy import PRIVACY_BLOCKED_PROCS, SUNNYLINK_BLOCKED_METHODS


class FakeParams:
  def __init__(self, value=None, raise_=False):
    self.value, self.raise_ = value, raise_

  def get_bool(self, key):
    if self.raise_:
      raise KeyError(key)
    return bool(self.value)


def test_privacy_mode_defaults_to_private_on_error():
  assert privacy.privacy_mode(FakeParams(raise_=True)) is True
  assert privacy.privacy_mode(FakeParams(True)) is True
  assert privacy.privacy_mode(FakeParams(False)) is False


def test_blocked_procs_exist():
  from openpilot.system.manager.process_config import managed_processes
  for name in ("manage_athenad", "uploader", "statsd", "statsd_sp"):
    assert name in PRIVACY_BLOCKED_PROCS
    assert name in managed_processes


def test_blocked_methods_cover_athena_data_access():
  for name in ("getMessage", "startStream", "uploadFilesToUrls", "uploadFileToUrl", "listDataDirectory", "startLocalProxy"):
    assert name in SUNNYLINK_BLOCKED_METHODS


def test_comma_api_blocked(mocker):
  from openpilot.common.api import comma_connect
  mocker.patch.object(comma_connect, "privacy_mode", return_value=True)
  base = mocker.patch("openpilot.common.api.base.BaseApi.api_get")
  with pytest.raises(ConnectionError):
    comma_connect.CommaConnectApi(None).api_get("v2/pilotauth/", method="POST")
  base.assert_not_called()

  mocker.patch.object(comma_connect, "privacy_mode", return_value=False)
  comma_connect.CommaConnectApi(None).api_get("v1/me")
  base.assert_called_once()


def test_onroad_latch(mocker):
  state = {"on": True}
  mocker.patch.object(privacy, "privacy_mode", side_effect=lambda params=None: state["on"])
  assert privacy.privacy_mode_onroad(False, None) is True   # offroad：即時
  assert privacy.privacy_mode_onroad(True, None) is True    # 發動：鎖定
  state["on"] = False
  assert privacy.privacy_mode_onroad(True, None) is True    # 行駛中切換：不變
  assert privacy.privacy_mode_onroad(False, None) is False  # 熄火：解除鎖定
  assert privacy.privacy_mode_onroad(True, None) is False


def test_dm_disabled_in_privacy_mode():
  # 隱私模式：不讀 driverStateV2，駕駛監控停用，長時間不碰方向盤也不發警示
  from types import SimpleNamespace
  from openpilot.common.realtime import DT_DMON
  from openpilot.selfdrive.monitoring.dmonitoringd import wheeltouch_step
  from openpilot.selfdrive.monitoring.policy import DriverMonitoring, MonitoringPolicy, AlertLevel
  from opendbc.car.structs import car

  DM = DriverMonitoring()
  DM._set_policy(MonitoringPolicy.wheeltouch)
  cs = SimpleNamespace(vEgo=25., gearShifter=car.CarState.GearShifter.drive, steeringPressed=False, gasPressed=False)
  sm = {"carState": cs, "selfdriveState": SimpleNamespace(enabled=True), "carControl": SimpleNamespace(latActive=True)}
  for _ in range(int(120 / DT_DMON)):  # 120 秒不碰方向盤
    wheeltouch_step(DM, sm)
    assert DM.alert_level == AlertLevel.none
  assert DM.awareness == 1.
  dat = DM.get_state_packet()
  assert dat.valid and dat.driverMonitoringState.activePolicy == MonitoringPolicy.wheeltouch
