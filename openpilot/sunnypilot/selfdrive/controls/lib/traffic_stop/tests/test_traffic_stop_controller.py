"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop.traffic_stop_controller import (
  CAMERA_TO_FRONT_M,
  STARTING_SUPPRESS_FRAMES,
  TrafficState,
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
  -N should pull it closer (earlier stop), relative to the unadjusted case.

  NOTE: this only exercises TrafficStopController's own math. It could not
  by itself have caught a real bug that existed for a while: long_mpc.py
  used to re-apply a second, hardcoded +2.5m offset on top of whatever this
  controller returned (a leftover from before the UI slider existed), so
  the UI's effective range was silently -2.5..+7.5m instead of -5..+5m --
  setting the slider to its most negative value could never pull the
  obstacle back more than 2.5m. That duplicate offset has since been
  deleted entirely from long_mpc.py, so this controller's output is now the
  single source of truth end to end; the class of bug can no longer recur
  because the mechanism that caused it no longer exists, not merely because
  a test would catch it.
  """
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


def test_obstacle_releases_on_off_not_just_green():
  """Mirrors cp's post-state-machine override exactly:

    if trafficState in [off, green] or xState not in [e2eStop, e2eStopped]:
      stop_model_x = 1000.0

  The obstacle must be released the instant traffic_state reads `off` --
  not only on a confirmed `green` -- even though the internal `state`
  (our xState-equivalent) has not formally transitioned back to `cruise`.
  """
  ctrl = _enabled_controller()
  car_state = _car_state()
  radar_state = _no_lead()

  # Get into an active `stopping` state with a red light.
  res = None
  for _ in range(40):
    res = ctrl.update(_make_model(stop_dist_m=30.0, v0=10.0), car_state, radar_state,
                       v_ego=10.0, a_ego=0.0, v_cruise=10.0)
  assert res.state == TrafficStopState.stopping
  assert res.stop_dist_m is not None

  # Simulate a single-frame detector flicker to `off` (model neither
  # confidently predicts a stop nor confidently predicts green), by
  # monkeypatching the internal detector for exactly one update() call.
  real_check = ctrl._check_model_stopping

  def _force_off(*args, **kwargs):
    ctrl.traffic_state = TrafficState.off

  ctrl._check_model_stopping = _force_off
  try:
    res_flicker = ctrl.update(_make_model(stop_dist_m=30.0, v0=10.0), car_state, radar_state,
                               v_ego=10.0, a_ego=0.0, v_cruise=10.0)
  finally:
    ctrl._check_model_stopping = real_check

  # Obstacle must be released this frame even though `state` is still `stopping`.
  assert res_flicker.traffic_state == TrafficState.off
  assert res_flicker.state == TrafficStopState.stopping
  assert res_flicker.stop_dist_m is None

  # Once the detector reports red again next frame, braking resumes immediately.
  res_resumed = ctrl.update(_make_model(stop_dist_m=30.0, v0=10.0), car_state, radar_state,
                             v_ego=10.0, a_ego=0.0, v_cruise=10.0)
  assert res_resumed.state == TrafficStopState.stopping
  assert res_resumed.stop_dist_m is not None


def test_v_cruise_limited_never_exceeds_current_speed():
  """Guards against a surge-then-brake artifact: if the car arrives at this
  moment already going slower than the pure distance/comfort-brake formula
  would allow (e.g. DEC/e2e was decelerating for its own reasons right before
  the is_e2e bypass in longitudinal_planner.py hands control to us), the
  soft speed cap must never ask for more speed than the car currently has.
  """
  ctrl = _enabled_controller()
  car_state = _car_state()
  radar_state = _no_lead()

  v_ego_suppressed = 16.7  # ~60 km/h, artificially low vs. what the distance alone would allow

  res = None
  for _ in range(60):
    res = ctrl.update(_make_model(stop_dist_m=90.0, v0=v_ego_suppressed), car_state, radar_state,
                       v_ego=v_ego_suppressed, a_ego=0.0, v_cruise=27.78)
  assert res.state == TrafficStopState.stopping
  assert res.stop_dist_m is not None

  # Sanity check: the *unclamped* formula would indeed have exceeded v_ego here,
  # otherwise this test isn't actually exercising the clamp.
  unclamped = (2.0 * ctrl.comfort_brake * max(res.stop_dist_m - 1.0, 0.0)) ** 0.5
  assert unclamped > v_ego_suppressed, "test scenario doesn't exercise the clamp; adjust stop_dist_m/v_ego"

  assert res.v_cruise_limited == pytest.approx(v_ego_suppressed, abs=1e-6)


def test_gas_press_suppression_only_arms_when_overriding_an_active_stop():
  """Regression test: an earlier version of this port re-armed the 10s
  gas-press suppression counter (`_starting_suppress_count`) on ANY gas
  press, regardless of state -- diverging from cp, which only arms it inside
  the `e2eStop` branch's gasPressed override (i.e. only when the driver
  manually overrides an ALREADY active stop).

  The bug: routine manual driving (or just engaging cruise control shortly
  before a light, which typically follows releasing the gas pedal) kept
  re-arming a fresh 10-second suppression window during normal `cruise`
  state -- so a light reached within 10s of the driver's last gas release
  would silently never trigger a stop.
  """
  ctrl = _enabled_controller()
  radar_state = _no_lead()

  # Phase 1: manual driving, gas held down continuously, far from any light.
  for _ in range(100):
    car_state = _car_state(gasPressed=True)
    ctrl.update(_make_model(stop_dist_m=200.0, v0=15.0), car_state, radar_state,
                v_ego=15.0, a_ego=0.0, v_cruise=15.0)
  assert ctrl._starting_suppress_count == 0, \
    "gas press during normal cruise must never arm the suppression counter"

  # Phase 2: driver releases the gas / engages cruise right as a light is close.
  res = None
  for _ in range(60):
    car_state = _car_state(gasPressed=False)
    res = ctrl.update(_make_model(stop_dist_m=30.0, v0=15.0), car_state, radar_state,
                       v_ego=15.0, a_ego=0.0, v_cruise=15.0)

  assert res.state == TrafficStopState.stopping
  assert res.stop_dist_m is not None


def test_gas_press_suppression_still_arms_when_overriding_active_stop():
  """The suppression counter must still work for its actual intended purpose:
  if the driver manually overrides an ALREADY active stop by pressing gas,
  briefly suppress re-triggering a new stop for the same light."""
  ctrl = _enabled_controller()
  car_state = _car_state()
  radar_state = _no_lead()

  for _ in range(40):
    res = ctrl.update(_make_model(stop_dist_m=30.0, v0=10.0), car_state, radar_state,
                       v_ego=10.0, a_ego=0.0, v_cruise=10.0)
  assert res.state == TrafficStopState.stopping

  car_state_gas = _car_state(gasPressed=True)
  res = ctrl.update(_make_model(stop_dist_m=30.0, v0=10.0), car_state_gas, radar_state,
                     v_ego=10.0, a_ego=0.0, v_cruise=10.0)
  assert res.state == TrafficStopState.cruise
  assert ctrl._starting_suppress_count == pytest.approx(STARTING_SUPPRESS_FRAMES, abs=1)


def test_obstacle_position_corrected_from_device_frame_to_front_of_car():
  """modelV2.position.x is in DEVICE frame (camera/device mounting location
  as origin), not front-bumper frame -- see cereal/log.capnp. radard.py
  already corrects for this exact offset for real lead cars via
  RADAR_TO_CAMERA=1.52m (dRel = lead_msg.x[0] - RADAR_TO_CAMERA). Our virtual
  stop-line obstacle must apply the same correction, or it sits a fixed
  ~1.5m further from the car than intended -- independent of (and additive
  with) the UI's distance_adjust_m, which manifests as a constant overshoot
  no amount of UI adjustment can fully cancel out.
  """
  ctrl = _enabled_controller()
  car_state = _car_state()
  radar_state = _no_lead()

  # Use a distance safely outside the 50m get_virtual_traffic_stop_distance
  # fade zone so this test isolates just the frame correction, not that
  # separate (pre-existing, cp-original) distance-shaping formula.
  raw_device_frame_distance = 80.0
  ctrl.update(_make_model(stop_dist_m=raw_device_frame_distance, v0=0.0), car_state, radar_state,
              v_ego=0.0, a_ego=0.0, v_cruise=5.0)

  assert ctrl._stop_x_rl == pytest.approx(raw_device_frame_distance - CAMERA_TO_FRONT_M, abs=0.01)
