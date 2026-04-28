#!/usr/bin/env python3
import capnp
from typing import Any
from cereal import messaging, car
from opendbc.car import structs

# 1. 引入整個 radard 模組進行 Monkey Patch
from openpilot.selfdrive.controls import radard
# 2. 正常引入我們需要的元件與原始函數
from openpilot.selfdrive.controls.radard import (
    KalmanParams, Track, RadarD, match_vision_to_track,
    get_RadarState_from_vision, get_custom_yrel, RADAR_TO_CAMERA
)

# ==============================================================================
# 提早鎖定 (Early Lock) 擴充模組參數設定
# ==============================================================================
MATCH_D_REL_THRES = 1.5   # 縱向距離允許誤差 (公尺)
MATCH_Y_REL_THRES = 0.5   # 橫向距離允許誤差 (公尺)
MATCH_V_REL_THRES = 1.0   # 相對速度允許誤差 (m/s)
STRICT_MATCH_FRAMES = 20  # 連續滿足條件的所需幀數 (以 20Hz 計約 1 秒)
PROB_THRES_REDUCED = 0.3  # 雷達嚴格驗證通過後的放寬視覺信心度門檻


class TrackSP(Track):
  """
  繼承自原始 Track，加入 20 幀連續嚴格重合驗證的記憶能力。
  """
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    super().__init__(identifier, v_lead, kalman_params)
    # 使用字典分別為 leadOne(0) 與 leadTwo(1) 紀錄連續重合的幀數
    self.strict_match_cnt = {0: 0, 1: 0}

  def update_strict_match(self, lead_idx: int, offset_vision_dist: float, vision_y: float, vision_v: float, v_ego: float):
    err_d = abs(self.dRel - offset_vision_dist)
    err_y = abs(self.yRel - vision_y)
    err_v = abs((self.vRel + v_ego) - vision_v)

    # 嚴苛物理重合條件判斷
    if err_d < MATCH_D_REL_THRES and err_y < MATCH_Y_REL_THRES and err_v < MATCH_V_REL_THRES:
      self.strict_match_cnt[lead_idx] += 1
    else:
      self.strict_match_cnt[lead_idx] = 0


def get_lead_ext(v_ego: float, ready: bool, tracks: dict[int, TrackSP], lead_msg: capnp._DynamicStructReader,
                 model_v_ego: float, CP: structs.CarParams, CP_SP: structs.CarParamsSP, low_speed_override: bool = True) -> dict[str, Any]:
  """
  擴充的前車評估函數，完整保留原廠邏輯，僅在符合 20 幀嚴格驗證時，導入 0.3 的動態降信心度邏輯。
  """
  # 利用 low_speed_override 來區別是計算 leadOne (True) 還是 leadTwo (False)
  lead_idx = 0 if low_speed_override else 1

  offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
  vision_y = -lead_msg.y[0]
  vision_v = lead_msg.v[0]

  has_confident_radar = False

  # 1. 遍歷軌跡並更新嚴格重合計數
  if ready:
    for track in tracks.values():
      track.update_strict_match(lead_idx, offset_vision_dist, vision_y, vision_v, v_ego)
      if track.strict_match_cnt[lead_idx] >= STRICT_MATCH_FRAMES:
        has_confident_radar = True

  # 2. 動態信心門檻：若雷達 20 幀驗證過關，信心門檻降至 0.3，否則維持 0.5
  current_prob_thres = PROB_THRES_REDUCED if has_confident_radar else 0.5

  # ========================================================================
  # 下方為原廠 get_lead 判斷邏輯的完美移植，僅將硬編碼的 0.5 替換為 current_prob_thres
  # 如此便可完全保留原廠的防護與決策機制
  # ========================================================================

  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > current_prob_thres:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)
  elif (track is None) and ready and (lead_msg.prob > current_prob_thres):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


# ==============================================================================
# 【核心魔法：雙重 Monkey Patching】
# 1. 強制將原廠 radard 模組裡的 Track 類別替換為我們的 TrackSP。
radard.Track = TrackSP
# 2. 強制將原廠的 get_lead 判斷邏輯，替換為我們的 get_lead_ext。
radard.get_lead = get_lead_ext
# ==============================================================================


class RadarDSP(RadarD):
  """
  繼承自 RadarD，因為已經透過 Monkey Patching 替換了底層運作的類別與函數，
  所以這個類別可以保持極致的簡潔，一行原廠迴圈都不用改寫！
  """
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    super().__init__(CP, CP_SP, delay)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    # 呼叫 super().update 時，底層將會自動使用我們寫好的 TrackSP 與 get_lead_ext！
    super().update(sm, rr)