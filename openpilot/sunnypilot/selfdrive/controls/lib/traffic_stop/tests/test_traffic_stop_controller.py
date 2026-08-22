"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop.traffic_stop_controller import (
  TrafficStopController,
  TrafficStopState,
)


class _Obj:
  def __init__(self, **kw):
    self.__dict__.update(kw)


class _XYZ:
  def __init__(self, x, y):
    self.x = x
    self.y = y


N = 33
DT_MDL = 0.05


def _make_model(stop_dist_m: float, v0: float, v_final: float = 0.2):
  x = np.linspace(0, max(stop_dist_m * 1.1, 1.0), N)
  x[-2] = stop_dist_m
  x[-1] = stop_dist_m
  v = np.linspace(v0, v_final, N)
  y = np.zeros(N)
  return _Obj(position=_XYZ(x=x, y=y), velocity=_XYZ(x=v, y=None))


def _no_lead():
  return _Obj(leadOne=_Obj(present=False, dRel=1000.0))


def _car_state(**overrides):
  base = dict(gasPressed=False, brakePressed=False, leftBlinker=False, steeringAngleDeg=0.0)
  base.update(overrides)
  return _Obj(**base)


def _enabled_controller(distance_adjust_m: int = 0) -> TrafficStopController:
  """Build a controller with the UI master toggle force-enabled, bypassing the
  real Params poll (unit tests don't have a working params_pyx backend)."""
  ctrl = TrafficStopController()
  ctrl.params.put("TrafficStopEnabled", 1)
  ctrl.params.put("TrafficStopDistanceAdjust", distance_adjust_m)
  ctrl.frame = 0
  ctrl._poll_params()
  return ctrl


def test_disabled_by_default_never_stops():
  """TrafficStopEnabled defaults to False (params_keys.h default "0"); the
  feature must be an explicit opt-in from the UI toggle."""
  ctrl = TrafficStopController()
  assert ctrl.enabled is False
  car_state = _car_state()
  radar_state = _no_lead()
  for _ in range(50):
    res = ctrl.update(_make_model(stop_dist_m=30.0, v0=10.0), car_state, radar_state,
                       v_ego=10.0, a_ego=0.0, v_cruise=10.0)
  assert res.state == TrafficStopState.cruise
  assert res.stop_dist_m is None


def test_no_stop_when_model_stays_fast():
  ctrl = _enabled_controller()
  car_state = _car_state()
  radar_state = _no_lead()
  for _ in range(20):
    res = ctrl.update(_make_model(stop_dist_m=100.0, v0=10.0, v_final=10.0), car_state, radar_state,
                       v_ego=10.0, a_ego=0.0, v_cruise=10.0)
  assert res.state == TrafficStopState.cruise
  assert res.stop_dist_m is None


def test_full_stop_and_release_on_green():
  ctrl = _enabled_controller()
  car_state = _car_state()
  radar_state = _no_lead()

  v_ego = 10.0
  a_ego = 0.0
  v_cruise = 10.0
  world_stop_x = 60.0
  entered_stopping = False

  for _ in range(400):
    res = ctrl.update(_make_model(stop_dist_m=world_stop_x, v0=v_ego), car_state, radar_state, v_ego, a_ego, v_cruise)
    if res.state == TrafficStopState.stopping:
      entered_stopping = True
    if res.v_cruise_limited is not None:
      v_cruise = min(10.0, res.v_cruise_limited)
      a_ego = -1.5 if v_ego > v_cruise + 0.1 else 0.0
    else:
      a_ego = 0.0
    v_ego = max(0.0, v_ego + a_ego * DT_MDL)
    world_stop_x = max(0.0, world_stop_x - v_ego * DT_MDL)
    if res.state == TrafficStopState.stopped:
      break

  assert entered_stopping
  assert res.state == TrafficStopState.stopped
  assert v_ego < 0.5

  # simulate the light turning green: model no longer predicts a stop
  green_model = _make_model(stop_dist_m=100.0, v0=10.0, v_final=10.0)
  for _ in range(30):
    res = ctrl.update(green_model, car_state, radar_state, v_ego=0.0, a_ego=0.0, v_cruise=10.0)
  assert res.state == TrafficStopState.cruise
  assert res.stop_dist_m is None


def test_lead_car_defers_to_lead_following():
  ctrl = _enabled_controller()
  car_state = _car_state()
  # a real lead is present well before the (would-be) stop line
  radar_state = _Obj(leadOne=_Obj(present=True, dRel=15.0))

  for _ in range(20):
    res = ctrl.update(_make_model(stop_dist_m=60.0, v0=10.0), car_state, radar_state,
                       v_ego=10.0, a_ego=0.0, v_cruise=10.0)
  # never enters a signal stop while a closer real lead is already tracked
  assert res.state == TrafficStopState.cruise


def test_steering_angle_suppresses_new_stop_entry():
  ctrl = _enabled_controller()
  car_state = _car_state(steeringAngleDeg=90.0)  # mid-turn
  radar_state = _no_lead()

  for _ in range(20):
    res = ctrl.update(_make_model(stop_dist_m=15.0, v0=5.0), car_state, radar_state,
                       v_ego=5.0, a_ego=0.0, v_cruise=5.0)
  assert res.state == TrafficStopState.cruise


def test_distance_adjust_moves_obstacle_position():
  """+N should place the virtual obstacle further away (later stop),
  -N should pull it closer (earlier stop), relative to the unadjusted case."""
  car_state = _car_state()
  radar_state = _no_lead()

  def run(distance_adjust_m):
    ctrl = _enabled_controller(distance_adjust_m=distance_adjust_m)
    res = None
    for _ in range(60):
      res = ctrl.update(_make_model(stop_dist_m=30.0, v0=10.0), car_state, radar_state,
                         v_ego=10.0, a_ego=0.0, v_cruise=10.0)
    return res

  res_zero = run(0)
  res_plus = run(3)
  res_minus = run(-3)

  assert res_zero.stop_dist_m is not None
  assert res_plus.stop_dist_m == pytest.approx(res_zero.stop_dist_m + 3, abs=1e-6)
  assert res_minus.stop_dist_m == pytest.approx(res_zero.stop_dist_m - 3, abs=1e-6)


def test_distance_adjust_is_clamped_to_plus_minus_5m():
  ctrl = TrafficStopController()
  ctrl.frame = 0
  ctrl.params.put("TrafficStopDistanceAdjust", 999)
  ctrl._poll_params()
  assert ctrl.distance_adjust_m == 5

  ctrl.frame = 0
  ctrl.params.put("TrafficStopDistanceAdjust", -999)
  ctrl._poll_params()
  assert ctrl.distance_adjust_m == -5
