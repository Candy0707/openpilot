#!/usr/bin/env python3
from cereal import messaging, car
from opendbc.car import structs

from openpilot.selfdrive.controls import radard
from openpilot.selfdrive.controls.radard import KalmanParams, Track, RadarD


class TrackSP(Track):
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    super().__init__(identifier, v_lead, kalman_params)


# ==============================================================================
# 【核心魔法：Monkey Patching】
# 強制將原廠 radard 模組裡的 Track 類別替換為我們的 TrackSP。
# 這樣一來，等一下 super().update 執行時，底層產生的軌跡就會是 TrackSP。
radard.Track = TrackSP
# ==============================================================================


class RadarDSP(RadarD):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    super().__init__(CP, CP_SP, delay)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    super().update(sm, rr)
