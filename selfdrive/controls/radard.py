#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any, Tuple

import capnp
from cereal import messaging, log, car, custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1  # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.0  # no stationary object flag below this speed

# ==========================================
# [自訂參數區]
# ==========================================
STATIONARY_MAX_DIST = 120.0  # 縮短靜止車最遠偵測距離，避免隧道微彎誤判
STATIONARY_MIN_PROB = 0.4  # 靜止車專屬最低信心度門檻 (固定)
BLIND_SPOT_PRIORITY_DIST = 23.0  # 低速盲區煞停「強制接管並鎖定」的距離 (公尺)
BLIND_SPOT_HYSTERESIS_DIST = 25.0  # 盲區煞停「解除鎖定」的退場距離 (公尺)
# ==========================================

RADAR_TO_CENTER = 2.7  # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float):
    assert dt > 0.01 and dt < 0.2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [
      0.12287673,
      0.14556536,
      0.16522756,
      0.18281627,
      0.1988689,
      0.21372394,
      0.22761098,
      0.24069424,
      0.253096,
      0.26491023,
      0.27621103,
      0.28705801,
      0.29750003,
      0.30757767,
      0.31732515,
      0.32677158,
      0.33594201,
      0.34485814,
      0.35353899,
      0.36200124,
    ]
    K1 = [
      0.29666309,
      0.29330885,
      0.29042818,
      0.28787125,
      0.28555364,
      0.28342219,
      0.28144091,
      0.27958406,
      0.27783249,
      0.27617149,
      0.27458948,
      0.27307714,
      0.27162685,
      0.27023228,
      0.26888809,
      0.26758976,
      0.26633338,
      0.26511557,
      0.26393339,
      0.26278425,
    ]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

    # --- 前車信心度分數 (抗抖動漏桶機制) ---
    self.is_stopped_car_count = 0
    self.selected_count = 0

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float):
    # relative values, copy
    self.dRel = d_rel  # LONG_DIST
    self.yRel = y_rel  # -LAT_DIST
    self.vRel = v_rel  # REL_SPEED
    self.vLead = v_lead
    self.measured = measured  # measured or estimate

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

    # 每幀自然漏水扣分
    self.is_stopped_car_count = max(0, self.is_stopped_car_count - 1)

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < BLIND_SPOT_HYSTERESIS_DIST)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > 0.9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x - mu) / b)


def match_vision_to_track(
  v_ego: float,
  lead: capnp._DynamicStructReader,
  tracks: dict[int, Track],
  path_x: list[float],
  path_y: list[float],
  lane_data: dict,
  current_prob_threshold: float,
):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # ==========================================
  # 目標分類與驗證邏輯
  # ==========================================

  # 1. 動態車條件：使用動態降階門檻
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist) * 0.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  is_dynamic_target = dist_sane and vel_sane and (lead.prob > current_prob_threshold)

  # 2. 靜止車強化邏輯 (車道寬度限制 + 距離加權信心度)
  model_x = track.dRel + RADAR_TO_CAMERA
  expected_yRel = -np.interp(model_x, path_x, path_y)

  # --- 橫向容錯計算 ---
  # 預設最大橫向容錯為 1.0m
  y_threshold = 1.0
  if lane_data['left_prob'] > 0.3 and lane_data['right_prob'] > 0.3 and len(lane_data['x']) > 0:
    left_y_at_d = np.interp(model_x, lane_data['x'], lane_data['left_y'])
    right_y_at_d = np.interp(model_x, lane_data['x'], lane_data['right_y'])
    lane_width = abs(left_y_at_d - right_y_at_d)

    # 取車道半寬的 50% 作為有效範圍 (保證只抓取正中央的車，不摸牆壁)
    dynamic_threshold = (lane_width / 2.0) * 0.50

    # 限制最高不超過 1.0，最低不小於 0.5
    y_threshold = max(0.5, min(dynamic_threshold, 1.0))

  y_sane_on_path = abs(track.yRel - expected_yRel) < y_threshold
  v_absolute = track.vRel + v_ego
  is_physically_stationary = abs(v_absolute) < 2.0

  # 靜止車不跟隨動態降階，永遠固定使用自訂門檻
  is_stationary_target = (
    (0.0 < track.dRel <= STATIONARY_MAX_DIST) and is_physically_stationary and dist_sane and y_sane_on_path and (lead.prob > STATIONARY_MIN_PROB)
  )

  is_valid_lead = is_dynamic_target or is_stationary_target

  if is_valid_lead:
    track.is_stopped_car_count = min(track.is_stopped_car_count + 6, 25)

  best_track = None

  if is_dynamic_target:
    best_track = track
  elif track.is_stopped_car_count >= 20:
    best_track = track

  for c in tracks.values():
    if best_track is not None and c is best_track:
      c.selected_count += 1
    else:
      c.selected_count = 0

  return best_track


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_custom_yrel(CP: structs.CarParams, CP_SP: structs.CarParamsSP, lead_dict: dict[str, Any], lead_msg: capnp._DynamicStructReader) -> dict[str, Any]:
  if CP.brand == "hyundai" and (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC or CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
    lead_dict['yRel'] = float(-lead_msg.y[0])

  return lead_dict


def get_lead(
  v_ego: float,
  ready: bool,
  tracks: dict[int, Track],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  path_x: list[float],
  path_y: list[float],
  lane_data: dict,
  CP: structs.CarParams,
  CP_SP: structs.CarParamsSP,
  low_speed_override: bool = True,
  is_locked: bool = False,
  current_prob_threshold: float = 0.5,
) -> Tuple[dict[str, Any], bool]:

  # --- Step 1: 取得視覺融合目標 ---
  gate_threshold = min(current_prob_threshold, STATIONARY_MIN_PROB)

  if len(tracks) > 0 and ready and lead_msg.prob > gate_threshold:
    best_valid_track = match_vision_to_track(v_ego, lead_msg, tracks, path_x, path_y, lane_data, current_prob_threshold)
  else:
    best_valid_track = None

  fused_lead_dict = {'status': False}
  if best_valid_track is not None:
    fused_lead_dict = best_valid_track.get_RadarState(lead_msg.prob)
    fused_lead_dict = get_custom_yrel(CP, CP_SP, fused_lead_dict, lead_msg)
  elif ready and (lead_msg.prob > 0.5):
    fused_lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)
    fused_lead_dict = get_custom_yrel(CP, CP_SP, fused_lead_dict, lead_msg)

  # --- Step 2: 盲區雷達強制接管與單向條件鎖定邏輯 ---
  lead_dict = fused_lead_dict
  new_locked_state = is_locked

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]

    if len(low_speed_tracks) > 0:
      closest_low_speed_track = min(low_speed_tracks, key=lambda c: c.dRel)
      blind_spot_dict = closest_low_speed_track.get_RadarState()

      if is_locked:
        if blind_spot_dict['dRel'] <= BLIND_SPOT_HYSTERESIS_DIST:
          lead_dict = blind_spot_dict
          new_locked_state = True
        else:
          new_locked_state = False

      else:
        if blind_spot_dict['dRel'] <= BLIND_SPOT_PRIORITY_DIST:
          lead_dict = blind_spot_dict
          new_locked_state = True
        else:
          if not fused_lead_dict['status'] or blind_spot_dict['dRel'] < fused_lead_dict.get('dRel', 1000.0):
            lead_dict = blind_spot_dict

    else:
      new_locked_state = False
  else:
    new_locked_state = False

  return lead_dict, new_locked_state


class RadarD:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, delay: float = 0.0):
    self.CP = CP
    self.CP_SP = CP_SP

    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL)) + 1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False
    self.lead_one_locked = False

    # ==========================================
    # 純視覺動態信心度參數狀態 (積分制漏桶)
    # ==========================================
    self.dynamic_prob_threshold = 0.5
    self.low_prob_score = 0

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9 * max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]
      v_lead = rpt[2] + self.v_ego_hist[0]

      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)

      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    # 擷取預測路徑
    if len(sm['modelV2'].position.x) > 0:
      path_x = list(sm['modelV2'].position.x)
      path_y = list(sm['modelV2'].position.y)
    else:
      path_x = [0.0, 100.0]
      path_y = [0.0, 0.0]

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego

    # 擷取車道線資訊供動態容錯使用
    ll_x, ll_y_left, ll_y_right = [], [], []
    ll_prob_left, ll_prob_right = 0.0, 0.0

    if len(sm['modelV2'].laneLines) == 4:
      ll_x = list(sm['modelV2'].laneLines[1].x)
      ll_y_left = list(sm['modelV2'].laneLines[1].y)
      ll_y_right = list(sm['modelV2'].laneLines[2].y)
      ll_prob_left = sm['modelV2'].laneLineProbs[1]
      ll_prob_right = sm['modelV2'].laneLineProbs[2]

    lane_data = {'x': ll_x, 'left_y': ll_y_left, 'right_y': ll_y_right, 'left_prob': ll_prob_left, 'right_prob': ll_prob_right}

    leads_v3 = sm['modelV2'].leadsV3

    # ==========================================
    # 動態信心度門檻調節邏輯 (彈性積分制漏桶演算法)
    # ==========================================
    if len(leads_v3) > 0:
      lead_prob = leads_v3[0].prob

      if 0.15 <= lead_prob < 0.5:
        # 疑似雨中水花遮擋，緩慢累積 (上限設為 60 分，約 3 秒)
        self.low_prob_score = min(self.low_prob_score + 1, 60)

      elif lead_prob >= 0.5:
        # 清楚看到真車，快速扣分，加速解除降階狀態
        self.low_prob_score = max(self.low_prob_score - 2, 0)

      else:
        # 小於 0.15 視為完全遮擋或空曠處雜訊，緩慢冷卻 (允許短暫斷訊)
        self.low_prob_score = max(self.low_prob_score - 1, 0)

      # --- 決定是否降階 ---
      if self.low_prob_score >= 50:  # 累積達到約 2.5 秒的不穩定狀態才降階
        self.dynamic_prob_threshold = 0.45
      elif self.low_prob_score == 0:  # 徹底冷卻後恢復原廠門檻
        self.dynamic_prob_threshold = 0.5

    # ==============================================================
    # [2026 最新融合邏輯] 多目標追蹤與防重複分配 (按信心度優先配對)
    # ==============================================================
    num_leads = len(leads_v3)

    if num_leads > 0:
      # 1. 根據模型輸出的視覺目標數量，動態配置 Cap'n Proto 陣列大小
      self.radar_state.init('leads', num_leads)

      # 2. 貪婪點池：建立可用雷達點，避免同一個硬體點被分配給多台車 (Double Assignment)
      available_tracks = self.tracks.copy()

      # 3. 建立配對優先權：算出機率由高到低的索引值
      # 確保最有自信的目標 (機率最高) 優先挑選最完美的雷達訊號
      sorted_indices = sorted(range(num_leads), key=lambda x: leads_v3[x].prob, reverse=True)

      # 暫存陣列，用來確保最後能依照模型原本的預期順序 (0, 1, 2) 輸出
      temp_leads = {}
      temp_locked_states = {}

      # 4. 依照「信心度由高到低」的順序執行融合配對
      for i in sorted_indices:
        # 降階門檻與低速盲區鎖定，永遠只綁定模型認為最重要的第 1 順位 (原始 i==0)
        current_prob = self.dynamic_prob_threshold if i == 0 else 0.5
        is_locked = self.lead_one_locked if i == 0 else False

        # 執行視覺與雷達融合 (修正：正確傳遞 lane_data 以及保留 CP, CP_SP)
        l_data, new_locked_state = get_lead(
          self.v_ego,
          self.ready,
          available_tracks,
          leads_v3[i],
          model_v_ego,
          path_x,
          path_y,
          lane_data,
          self.CP,
          self.CP_SP,
          low_speed_override=(i == 0),
          is_locked=is_locked,
          current_prob_threshold=current_prob,
        )

        # 防重複分配保護：配對成功即從池中剔除
        if l_data['status'] and l_data.get('radar') and l_data.get('radarTrackId') != -1:
          used_id = l_data['radarTrackId']
          if used_id in available_tracks:
            del available_tracks[used_id]

        # 寫入底層陣列並記錄到暫存區
        self.radar_state.leads[i] = l_data
        temp_leads[i] = l_data
        temp_locked_states[i] = new_locked_state

      # 5. 威脅篩選：把順序強制拉回原廠預期的 0, 1, 2 (配對時講實力，排隊時講規矩)
      valid_leads = []
      valid_locked_states = []

      for i in range(num_leads):
        # 信任神經網路的判斷：只要它認為是有效的目標，就納入跟車名單
        if temp_leads[i]['status']:
          valid_leads.append(temp_leads[i])
          valid_locked_states.append(temp_locked_states[i])

      # 6. 相容性指派：將過濾後的名單交給 OP 後端縱向控制系統 (Planner)
      if len(valid_leads) > 0:
        self.radar_state.leadOne = valid_leads[0]
        self.lead_one_locked = valid_locked_states[0]  # 同步第 1 順位的煞車鎖定狀態
      else:
        self.lead_one_locked = False  # 確保前方無車時完全解除鎖定

      if len(valid_leads) > 1:
        self.radar_state.leadTwo = valid_leads[1]

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # 修正：補回 CP_SP 的讀取，避免 RadarD 初始化失敗
  cloudlog.info("radard is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(Params().get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("radard got CarParamsSP")

  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  # 修正：完整傳入 CP 與 CP_SP
  RD = RadarD(CP, CP_SP, CP.radarDelay)

  while 1:
    sm.update()
    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
