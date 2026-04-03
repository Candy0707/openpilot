import pytest
import numpy as np
from unittest.mock import MagicMock, patch
import sys
import os
import importlib.util

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))

sys.modules["cereal"] = MagicMock()
sys.modules["cereal.messaging"] = MagicMock()
sys.modules["opendbc.car.common.conversions"] = MagicMock()
sys.modules["openpilot.selfdrive.car.cruise"] = MagicMock()
sys.modules["openpilot.common.realtime"] = MagicMock()

mock_targets_base_module = MagicMock()
sys.modules["dragonpilot.selfdrive.controls.lib.targetsbase"] = mock_targets_base_module

sys.modules["opendbc.car.common.conversions"].Conversions.MS_TO_KPH = 3.6
sys.modules["opendbc.car.common.conversions"].Conversions.KPH_TO_MS = 1 / 3.6
sys.modules["openpilot.selfdrive.car.cruise"].V_CRUISE_MAX = 40.0
sys.modules["openpilot.common.realtime"].DT_MDL = 0.05


class MockTargetsBase:
  def __init__(self, CP, mpc):
    self.params = MagicMock()
    self.action = False
    self.v_target = 0.0
    self.a_target = 0.0

  def update_target(self, sm, v_ego, a_ego, v_cruise):
    return False


mock_targets_base_module.TargetsBase = MockTargetsBase


def load_module_without_relative_imports(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  module = importlib.util.module_from_spec(spec)
  with open(path, 'r', encoding='utf-8') as f:
    source = f.read()
  source = source.replace('from dragonpilot.selfdrive.controls.lib.targetsbase', 'from dragonpilot.selfdrive.controls.lib.targetsbase')
  exec(source, module.__dict__)
  return module


module_path = os.path.join(parent_dir, 'path_deviation_monitor.py')
pdm_module = load_module_without_relative_imports("path_deviation_monitor", module_path)
PathDeviationMonitor = pdm_module.PathDeviationMonitor


def make_mock_sm():
  sm = {}
  car_state = MagicMock()
  car_state.leftBlinker = False
  car_state.rightBlinker = False
  car_state.steeringAngleDeg = 0.0  # 用於感測器備援測試
  car_state.yawRate = 0.0
  sm['carState'] = car_state

  model_v2 = MagicMock()
  model_v2.laneLineProbs = [0.0, 0.9, 0.9, 0.0]

  path_len = 33
  model_v2.position.x = np.linspace(0.0, 50.0, path_len)
  model_v2.position.y = np.zeros(path_len)

  class MockLaneLine:
    def __init__(self, y_val):
      self.y = np.full(path_len, y_val)

  model_v2.laneLines = {1: MockLaneLine(1.5), 2: MockLaneLine(-1.5)}
  sm['modelV2'] = model_v2
  return sm


@pytest.fixture
def monitor():
  CP = MagicMock()
  CP.steerRatio = 15.0
  CP.wheelbase = 2.7
  mpc = MagicMock()
  monitor = PathDeviationMonitor(CP, mpc)
  monitor.params.get_bool.return_value = True
  monitor.update_params()
  return monitor


def test_lookup_table_precision(monitor):
  test_cases = [
    (0.100, 1.00, "死區邊界"),
    (0.300, 0.75, "第一段插值"),
    (0.500, 0.50, "第一段關鍵點"),
    (0.800, 0.25, "第二段關鍵點"),
    (1.000, 0.00, "完全煞停"),
  ]
  for ratio, expected, desc in test_cases:
    assert monitor._get_speed_factor(ratio) == pytest.approx(expected, abs=0.001)


def test_ema_accumulation_and_activation(monitor):
  sm = make_mock_sm()
  sm['modelV2'].position.y[:] = 1.0
  for _ in range(15):
    monitor.update_target(sm, v_ego=30.0, a_ego=0.0, v_cruise=30.0)
  assert monitor.action is True


def test_rate_limiter(monitor):
  sm = make_mock_sm()
  sm['modelV2'].position.y[:] = 1.2
  monitor.update_target(sm, v_ego=30.0, a_ego=0.0, v_cruise=30.0)
  expected_v = 30.0 - monitor.MAX_DECEL_RATE

  for _ in range(5):
    monitor.update_target(sm, v_ego=30.0, a_ego=0.0, v_cruise=30.0)
  assert monitor.action is True
  assert monitor.v_target <= expected_v


def test_safety_override_detailed(monitor):
  sm = make_mock_sm()
  sm['modelV2'].position.y[:] = 1.0

  for _ in range(15):
    monitor.update_target(sm, v_ego=30.0, a_ego=0.0, v_cruise=30.0)
  assert monitor.action is True

  # 模擬駕駛打方向燈，強制作為閃避/變換車道意圖
  sm['carState'].leftBlinker = True
  monitor.update_target(sm, v_ego=30.0, a_ego=0.0, v_cruise=30.0)
  assert monitor.cooldown_timer == int(2.0 / 0.05)
  assert monitor.ratio_ema == 0.0

  # 模擬關閉方向燈，且車身回到車道中央
  sm['carState'].leftBlinker = False
  sm['modelV2'].position.y[:] = 0.0

  # [V8.4 關鍵修正驗證] 因為我們把加速恢復交給了原生 MPC，
  # 所以只要落差歸零，action 應該在下一幀「立即解除 (False)」，不再有平滑卡死的狀況。
  monitor.update_target(sm, v_ego=30.0, a_ego=0.0, v_cruise=30.0)

  assert monitor.action is False, "【防護未立即解除】 落差歸零後，應立即將控制權交還 MPC！"


def test_low_speed_lockout(monitor):
  sm = make_mock_sm()
  sm['modelV2'].position.y[:] = 1.0
  for _ in range(20):
    monitor.update_target(sm, v_ego=0.5, a_ego=0.0, v_cruise=30.0)
  assert monitor.action is False
