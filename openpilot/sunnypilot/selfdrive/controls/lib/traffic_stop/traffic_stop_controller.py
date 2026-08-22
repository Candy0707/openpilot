"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

TrafficStopController
======================

Modularized port of the "carrot" openpilot fork's normal-mode ("一般模式")
traffic-light / stop-sign stopping behavior (`selfdrive/carrot/carrot_functions.py`,
the `XState.e2ePrepare -> e2eStop -> e2eStopped` chain).

The core idea, unchanged from carrot:

  1. The driving model's own predicted trajectory (`modelV2.position`,
     `modelV2.velocity`) is used to infer that the model itself intends to
     decelerate to a stop within some distance -- this is treated as a
     "red light / stop sign detected" signal (`TrafficState`). No separate
     traffic-light vision classifier is required.
  2. Once a stop is detected and steering angle / lead-car checks allow it,
     a virtual, stationary obstacle is placed at (a speed-corrected version
     of) that stop-line distance.
  3. That virtual obstacle distance (`stop_dist_m`) is meant to be merged
     into the longitudinal MPC's existing obstacle list exactly like a
     lead car -- see `LongitudinalMpc.update(..., traffic_stop_obstacle=...)`
     in `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py`. The MPC
     itself does not need to know this obstacle came from a traffic light;
     it is simply the closest thing to brake for.
  4. The obstacle is released (stop_dist_m -> None) the instant
     `traffic_state` reads `off` OR `green` -- not only on a confirmed
     `green` -- even if the internal `state` (our xState-equivalent) has
     not yet formally transitioned back to `cruise`. This exactly mirrors
     cp's post-state-machine override (`if trafficState in [off, green] or
     xState not in [e2eStop, e2eStopped]: stop_model_x = 1000.0`), including
     the resulting frame-to-frame flicker if the detector itself flickers
     between `red` and `off`. This was verified against cp's source line by
     line and matched deliberately rather than smoothed out.

Deliberately NOT ported (out of scope for "一般模式"/general-mode stopping,
these belonged to carrot's broader navigation/UI stack and are not needed
for the core stop-at-red-light mechanism):
  - ATC / navigation-triggered stops, DrivingMode (Eco/Safe/Normal/High)
  - carrot_man / carrotNavi integration, soft-hold UI toggle
  - user_stop_distance manual override
  - trafficSignChanged/trafficStopping alert events (sunnypilot's cereal
    schema does not define these EventName entries; wire up your own UI
    hook against `TrafficStopController.state` / `.traffic_state` if desired)

UI-controllable parameters (see selfdrive/ui/sunnypilot/layouts/settings/cruise.py):
  - Params "TrafficStopEnabled" (bool): master on/off switch. Polled once a
    second (not every 20ms frame) like other sunnypilot toggles
    (see sunnypilot/selfdrive/controls/lib/targetsbase.py for the same pattern).
    When off, the controller resets to `cruise` and never emits an obstacle.
  - Params "TrafficStopDistanceAdjust" (int, meters, [-5, 5]): manual fine-tune
    applied on top of the automatic stop-line position. Positive values move
    the virtual obstacle further from the car (stop later / closer to the
    line); negative values pull it closer to the car (stop earlier / further
    back from the line).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop.traffic_stop import (
  get_traffic_stop_reference_speed,
  get_virtual_traffic_stop_distance,
  is_traffic_stop_entry_allowed,
)

# modelV2.position / .velocity / .orientation are TRAJECTORY_SIZE-point arrays
# (see selfdrive/modeld/constants.py ModelConstants.IDX_N). Index -2 is used
# (rather than the very last, noisier point) as the raw model stop-line guess,
# matching carrot's `x[31]` for a 33-point trajectory.
STOP_MODEL_IDX = -2

NO_STOP_DISTANCE_M = 1000.0  # "no active stop" sentinel, mirrors carrot's stop_model_x = 1000.0
DEFAULT_COMFORT_BRAKE = 2.5  # m/s^2, only used for the soft v_cruise cap below, independent of the MPC's own comfort brake
DEFAULT_STOP_DISTANCE_ADJUST = 2.5  # m, matches carrot's default TrafficStopDistanceAdjust
LEAD_CLOSE_TO_STOP_LINE_M = 2.0  # if a real lead is within this margin of the stop line, defer to lead-following
STOPPED_SPEED_MS = 0.3  # below this we consider the car fully halted at the line
STOPPING_HOLD_FRAMES = int(0.5 / DT_MDL)  # debounce frames before declaring "stopped"
STARTING_SUPPRESS_FRAMES = int(10.0 / DT_MDL)  # after a manual gas-pedal restart, ignore new red-light triggers briefly
PARAM_POLL_FRAMES = int(1.0 / DT_MDL)  # poll UI-controlled Params once a second, like TargetsBase.enable does
DISTANCE_ADJUST_MIN_M = -5
DISTANCE_ADJUST_MAX_M = 5


class TrafficState(IntEnum):
  off = 0
  red = 1
  green = 2


class TrafficStopState(IntEnum):
  cruise = 0      # no active signal-stop handling; MPC obstacle disabled
  stopping = 1    # actively braking for the virtual stop line
  stopped = 2     # fully halted at the line, waiting for green


class _MovingAverage:
  """Minimal median+mean smoothing filter, self-contained so this module
  does not need to depend on / modify the shared common/filter_simple.py."""

  def __init__(self, window_size: int):
    self.window_size = window_size
    self.values: deque[float] = deque(maxlen=window_size)

  def process(self, value: float, median: bool = False) -> float:
    self.values.append(value)
    if median:
      return float(np.median(self.values))
    return float(sum(self.values)) / len(self.values)


@dataclass
class TrafficStopResult:
  # Distance (m) to feed into LongitudinalMpc.update(traffic_stop_obstacle=...).
  # None when there is no active signal stop (equivalent to carrot's 1000.0 sentinel).
  stop_dist_m: float | None
  # Optional soft v_cruise cap (m/s) for a smoother early approach; None if not applicable.
  v_cruise_limited: float | None
  state: TrafficStopState
  traffic_state: TrafficState


class TrafficStopController:
  """
  Note: this controller returns the raw stop-line obstacle distance
  (equivalent to carrot's `carrot.stop_dist`). The final
  `get_traffic_stop_obstacle_distance()` correction (adding the
  configurable `distance_adjust` offset) is applied on the MPC side in
  `long_mpc.py`, exactly mirroring carrot's architecture where the
  controller and the MPC are separate layers.
  """

  def __init__(self, comfort_brake: float = DEFAULT_COMFORT_BRAKE):
    self.comfort_brake = comfort_brake
    self.params = Params()
    self.frame = 0
    self.enabled = self.params.get_bool("TrafficStopEnabled")
    self.distance_adjust_m = self._clamp_distance_adjust(self.params.get("TrafficStopDistanceAdjust", return_default=True))

    self._stop_x_filter_median = _MovingAverage(3)
    self._stop_x_filter_avg = _MovingAverage(15)
    self._model_v_filter = _MovingAverage(10)

    self._stop_x_rl: float | None = None  # rate-limited (only-approaching) model stop distance
    self.stop_sign_count = 0
    self.start_sign_count = 0
    self.traffic_state = TrafficState.off

    self.state = TrafficStopState.cruise
    self.actual_stop_distance = 0.0
    self.traffic_stop_reference_speed_kph: float | None = None
    self._stopping_hold_count = 0
    self._starting_suppress_count = 0

  def _update_stop_dist(self, stop_x_raw: float) -> float:
    stop_x = self._stop_x_filter_median.process(stop_x_raw, median=True)
    return self._stop_x_filter_avg.process(stop_x)

  @staticmethod
  def _clamp_distance_adjust(value) -> int:
    return int(np.clip(int(value), DISTANCE_ADJUST_MIN_M, DISTANCE_ADJUST_MAX_M))

  def _poll_params(self) -> None:
    """Re-read UI-controlled Params once a second, matching the existing
    sunnypilot convention (see TargetsBase.update_target) so we don't hit
    disk/IPC every 20 ms control cycle."""
    if self.frame % PARAM_POLL_FRAMES == 0:
      self.enabled = self.params.get_bool("TrafficStopEnabled")
      self.distance_adjust_m = self._clamp_distance_adjust(self.params.get("TrafficStopDistanceAdjust", return_default=True))
    self.frame += 1

  def _check_model_stopping(self, v_cruise: float, model_v_traj: np.ndarray, v_ego: float, a_ego: float,
                             model_x_end: float, model_y_traj: np.ndarray, d_rel: float) -> None:
    """Infer red/green purely from the driving model's own predicted trajectory."""
    v_ego_kph = v_ego * CV.MS_TO_KPH
    model_v = self._model_v_filter.process(float(model_v_traj[-1]))
    start_sign = model_v > 5.0 or model_v > (float(model_v_traj[0]) + 2)

    if v_ego_kph < 1.0:
      stop_sign = model_x_end < 20.0 and model_v < 10.0
    elif v_ego_kph < 82.0:
      stop_sign = (model_x_end < d_rel - 3.0 and
                   model_x_end < np.interp(model_v_traj[0] * 3.6, [60, 80], [120.0, 150]) and
                   ((model_v < 3.0) or (model_v < model_v_traj[0] * 0.7)) and
                   abs(float(model_y_traj[-1])) < 5.0)
      # Normal driving deceleration (e.g. camera-based speed limits) causes false positives.
      # If regen braking has already zeroed v_cruise, allow the signal to be detected anyway.
      if v_cruise != 0 and self.state == TrafficStopState.cruise and a_ego < -1.0:
        stop_sign = False
    else:
      stop_sign = False

    self.stop_sign_count = self.stop_sign_count + 1 if stop_sign else 0
    self.start_sign_count = self.start_sign_count + 1 if (start_sign and not stop_sign) else 0

    if self.stop_sign_count * DT_MDL > 0.0:
      self.traffic_state = TrafficState.red
    elif self.start_sign_count * DT_MDL > 0.2:
      self.traffic_state = TrafficState.green
    else:
      self.traffic_state = TrafficState.off

  def update(self, model_v2, car_state, radar_state, v_ego: float, a_ego: float, v_cruise: float) -> TrafficStopResult:
    """Run one control-loop cycle.

    model_v2:   sm['modelV2']
    car_state:  sm['carState']
    radar_state: sm['radarState']
    v_ego, a_ego: current ego speed (m/s) / accel (m/s^2)
    v_cruise:   current cruise target speed (m/s)
    """
    self._poll_params()

    if not self.enabled:
      # Master UI toggle is off: fully reset so a stale latch doesn't survive
      # the toggle being re-enabled mid-drive, and skip the (cheap but
      # unnecessary) model-stopping detection below.
      self.state = TrafficStopState.cruise
      self.traffic_state = TrafficState.off
      self.traffic_stop_reference_speed_kph = None
      self.actual_stop_distance = 0.0
      self._stop_x_rl = None
      return TrafficStopResult(stop_dist_m=None, v_cruise_limited=None,
                                state=self.state, traffic_state=self.traffic_state)

    v_ego_kph = v_ego * CV.MS_TO_KPH
    lead_detected = bool(radar_state.leadOne.present)
    d_rel = radar_state.leadOne.dRel if lead_detected else 1000.0

    x = model_v2.position.x
    y = model_v2.position.y
    v = model_v2.velocity.x

    stop_model_x_raw = self._update_stop_dist(float(x[STOP_MODEL_IDX]))

    if self._stop_x_rl is None:
      self._stop_x_rl = stop_model_x_raw
    else:
      max_close = v_ego * DT_MDL + 0.5
      if stop_model_x_raw > self._stop_x_rl:
        self._stop_x_rl = stop_model_x_raw
      else:
        self._stop_x_rl = max(self._stop_x_rl - max_close, stop_model_x_raw)
    stop_model_x_rl = self._stop_x_rl

    self._check_model_stopping(v_cruise, v, v_ego, a_ego, float(x[-1]), y, d_rel)

    if car_state.gasPressed:
      self._starting_suppress_count = STARTING_SUPPRESS_FRAMES
    else:
      self._starting_suppress_count = max(0, self._starting_suppress_count - 1)

    # --- state machine -----------------------------------------------------
    if self.state == TrafficStopState.stopped:
      if car_state.gasPressed:
        self.state = TrafficStopState.cruise
      elif lead_detected and (d_rel - stop_model_x_raw) < LEAD_CLOSE_TO_STOP_LINE_M:
        self.state = TrafficStopState.cruise  # defer to lead-following
      elif self._stopping_hold_count == 0 and self.traffic_state == TrafficState.green and not car_state.leftBlinker:
        self.state = TrafficStopState.cruise
      self._stopping_hold_count = max(0, self._stopping_hold_count - 1)

    elif self.state == TrafficStopState.stopping:
      self._stopping_hold_count = 0
      if car_state.gasPressed:
        self.state = TrafficStopState.cruise
        self._starting_suppress_count = STARTING_SUPPRESS_FRAMES
      elif lead_detected and (d_rel - stop_model_x_raw) < LEAD_CLOSE_TO_STOP_LINE_M:
        self.state = TrafficStopState.cruise
      elif self.traffic_state == TrafficState.green:
        self.state = TrafficStopState.cruise
      else:
        self.traffic_stop_reference_speed_kph = get_traffic_stop_reference_speed(
          v_ego_kph, self.traffic_stop_reference_speed_kph,
        )
        stop_dist = get_virtual_traffic_stop_distance(stop_model_x_rl, self.traffic_stop_reference_speed_kph)
        if stop_dist > 10.0:  # only refresh once far enough away for the estimate to be meaningful
          self.actual_stop_distance = stop_dist
        if v_ego < STOPPED_SPEED_MS:
          self._stopping_hold_count = STOPPING_HOLD_FRAMES
          self.state = TrafficStopState.stopped

    else:  # cruise: watch for a new red-light entry
      if lead_detected:
        pass  # never enter a signal-stop while already tracking a real lead
      elif (self.traffic_state == TrafficState.red
            and is_traffic_stop_entry_allowed(car_state.steeringAngleDeg)
            and self._starting_suppress_count == 0):
        self.state = TrafficStopState.stopping
        self.traffic_stop_reference_speed_kph = get_traffic_stop_reference_speed(v_ego_kph, None)
        self.actual_stop_distance = get_virtual_traffic_stop_distance(stop_model_x_rl, self.traffic_stop_reference_speed_kph)

    if self.state not in (TrafficStopState.stopping, TrafficStopState.stopped):
      self.traffic_stop_reference_speed_kph = None
      self._stop_x_rl = stop_model_x_raw  # don't let the rate-limiter carry stale state into the next stop

    # --- finalize distance ---------------------------------------------------
    # Faithfully mirror cp's post-state-machine override:
    #   if trafficState in [off, green] or xState not in [e2eStop, e2eStopped]:
    #     stop_model_x = 1000.0
    #   ...
    #   if stop_model_x == 1000.0: actual_stop_distance = 0.0
    #   elif actual_stop_distance > 0: stop_model_x = 0.0
    #
    # This means the obstacle is released the instant `traffic_state` reads
    # off OR green -- not only on a confirmed `green` -- even if the internal
    # `state` (our xState-equivalent) hasn't formally transitioned back to
    # `cruise` yet (e.g. state is still `stopping` while trafficState briefly
    # flickers to `off` between red-detection frames). cp accepts the
    # resulting frame-to-frame flicker in exchange for never holding the
    # brake on stale/uncertain detection; we replicate that behavior exactly
    # rather than smoothing it out, per explicit request to match the
    # validated cp implementation rather than diverge from it.
    release = (self.traffic_state in (TrafficState.off, TrafficState.green)
               or self.state not in (TrafficStopState.stopping, TrafficStopState.stopped))

    stop_model_x = NO_STOP_DISTANCE_M if release else 0.0

    self.actual_stop_distance = max(0.0, self.actual_stop_distance - v_ego * DT_MDL)

    if stop_model_x == NO_STOP_DISTANCE_M:
      self.actual_stop_distance = 0.0
      return TrafficStopResult(stop_dist_m=None, v_cruise_limited=None,
                                state=self.state, traffic_state=self.traffic_state)
    elif self.actual_stop_distance > 0:
      stop_model_x = 0.0  # no-op in this port (already 0.0 above); kept for structural fidelity with cp

    stop_dist = max(0.0, stop_model_x + self.actual_stop_distance)

    # Manual UI fine-tune: +N moves the virtual obstacle further away (stop
    # later / closer to the physical line), -N pulls it closer to the car
    # (stop earlier / further back from the line).
    stop_dist = max(0.0, stop_dist + self.distance_adjust_m)

    v_cruise_limited = None
    if stop_dist < 300.0:
      stop_dist_soft = max(stop_dist - 1.0, 0.0)
      v_cruise_limited = float(np.sqrt(max(0.0, 2.0 * self.comfort_brake * stop_dist_soft)))

    return TrafficStopResult(stop_dist_m=stop_dist, v_cruise_limited=v_cruise_limited,
                              state=self.state, traffic_state=self.traffic_state)
