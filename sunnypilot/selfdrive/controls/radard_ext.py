#!/usr/bin/env python3
import capnp
import numpy as np
from typing import Any
from cereal import messaging, car
from opendbc.car import structs

# ==============================================================================
# 1. 引入整個 radard 模組進行 Monkey Patch (動態替換)
# ==============================================================================
from openpilot.selfdrive.controls import radard

# 2. 正常引入我們需要的元件與原始函數
from openpilot.selfdrive.controls.radard import (
    KalmanParams, Track, RadarD, match_vision_to_track,
    get_RadarState_from_vision, get_custom_yrel, RADAR_TO_CAMERA
)

# 3. 引入 cloudlog 用於記錄我們自訂的提早鎖定事件
from openpilot.common.swaglog import cloudlog

# ==============================================================================
# 提早鎖定 (Early Lock) 擴充模組參數設定 - 終極純淨管線版
# ==============================================================================
# 📐 階段一與二：空間遲滯與物理打分邊界
LANE_WIDTH_FALLBACK = 1.5           # 預測車道基準單側半寬 (m)，無實體線時使用
LANE_HYSTERESIS_MARGIN = 0.5        # 邊界外的遲滯容錯預度 (m)，完美消滅邊緣抽搐閃爍
FUZZY_BOUNDS = [0.5, 1.5]           # 物理誤差 (m 或 m/s): 0.5 以內給滿分 1.0，大於 1.5 總分歸零

# 🧠 階段三：EMA 基礎學習率設定
ALPHA_BASE = 0.2                    # 常規上升學習率 (確保真車在 0.2~0.4 秒內穩定鎖定)
ALPHA_DOWN = 0.1                    # 常規下降與短路過濾時的衰減學習率 (極速踢出雜訊)

# 🔥 階段三：威脅乘數融合矩陣 (限制最高 1.2 倍，保留空間讓 EMA 平滑爬升)
BRAKE_THRES_RANGE = [-3.0, -1.2]    # 急煞觸發區間 (m/s²)
MULT_RANGE = [1.2, 1.0]             # 對應威脅倍率 (從最高 1.2 緩降至 1.0)
CUTIN_DIST_LIMIT = 40.0             # 評估切入威脅的最大縱向有效距離 (m)
DYNAMIC_SPEED_PCT = 0.2             # 動態相對速度閥值比例 (以當前車速的 20% 為基準)

# 🐢 階段四：慢速與靜止防護網 (基於電腦視覺物理極限的反轉門檻)
CAM_PROB_SPEED_RANGE = [10.0, 25.0] # 動態相機門檻車速區間 (約 36 ~ 90 km/h)
CAM_PROB_RANGE = [0.5, 0.3]         # 動態相機審查門檻 (市區低速要求 0.5 嚴格把關，高速放寬至 0.3)
STATIC_EMA_CAP = 0.6                # 目標未達審查門檻時，EMA 絕對不可超越的天花板

# 🎛️ 階段五：全局反向插值映射
EMA_VAL_RANGE = [0.4, 0.8]          # 本地 EMA 信心度 X 軸 (0.4 起跳，0.8 達峰值)
PROB_THRES_RANGE = [0.5, 0.3]       # 映射出對應的「視覺提早放行門檻」 Y 軸 (最高可強勢拉低至 0.3)


class TrackSP(Track):
  """
  繼承自原始 Track，加入非對稱 EMA 信心度、模糊邏輯打分與動態威脅乘數評估能力。
  採用高內聚力的物件導向設計，將複雜管線拆解為獨立方法，確保資料流純潔無狀態。
  """

  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    super().__init__(identifier, v_lead, kalman_params)
    # 針對 leadOne(0) 與 leadTwo(1) 紀錄跨幀狀態
    # 【核心機制：極速起跑點】初始信心度設定為生效門檻邊緣 0.4。
    # 完美利用數學斜率與實車卡鉗物理遲滯：確保新 ID (閃爍或切入) 能在第一幀就突破門檻發揮作用，
    # 且若為雜訊則會在一幀內跌出 0.4 門檻並迅速衰減死亡。完全拔除複雜的跨幀記憶體負擔。
    self.ema_confidence = {0: 0.4, 1: 0.4}
    self.is_out_of_lane = False

  def _check_spatial_boundaries(self, vision_y: float) -> bool:
    """
    【階段一：空間幾何絕對邊界審查】
    建立遲滯狀態機 (Hysteresis State Machine)，過濾車道外的無效雜訊。
    """
    # 視覺模型的 vision_y 本身即代表模型預測的最佳行駛路徑
    left_bound = vision_y + LANE_WIDTH_FALLBACK
    right_bound = vision_y - LANE_WIDTH_FALLBACK
    current_y = self.yRel

    if not self.is_out_of_lane:
      # 【正常鎖定狀態】：若橫向飄移超出「邊界 + 0.5m」容錯區，觸發斷鎖標記
      if current_y > (left_bound + LANE_HYSTERESIS_MARGIN) or current_y < (right_bound - LANE_HYSTERESIS_MARGIN):
        self.is_out_of_lane = True
    else:
      # 【出界斷鎖狀態】：目標必須乖乖完全縮回「原始邊界內」(不含 0.5m 容錯)，才准許解除標記
      if right_bound <= current_y <= left_bound:
        self.is_out_of_lane = False

    return not self.is_out_of_lane

  def _calculate_fuzzy_score(self, offset_vision_dist: float, vision_y: float, vision_v: float, v_ego: float) -> float:
    """
    【階段二：嚴格三維物理模糊打分】
    計算縱向、橫向與速度的物理誤差。遵守視覺主導底線：雷達與視覺必須在三維空間重合才准許打分。
    """
    err_d = abs(self.dRel - offset_vision_dist)
    err_y = abs(self.yRel - vision_y)
    err_v = abs((self.vRel + v_ego) - vision_v)

    # 統一採用 [0.5, 1.5] 緩衝斜坡進行反向插值 (誤差越小，分數越高)
    score_d = float(np.interp(err_d, FUZZY_BOUNDS, [1.0, 0.0]))
    score_y = float(np.interp(err_y, FUZZY_BOUNDS, [1.0, 0.0]))
    score_v = float(np.interp(err_v, FUZZY_BOUNDS, [1.0, 0.0]))

    return score_d * score_y * score_v

  def _calculate_threat_multipliers(self, v_ego: float) -> float:
    """
    【階段三：威脅乘數融合】
    根據前車極端動態 (急煞 / 逼近切入)，疊加威脅倍率，動態決定 EMA 上升神經反射速度。
    """
    # 1. 動態急煞倍率 (透過 aLeadK 判斷，最高 +20%)
    brake_mult = float(np.interp(self.aLeadK, BRAKE_THRES_RANGE, MULT_RANGE))

    # 2. 動態切入倍率 (判斷前提：目標在近距離且具備一定橫向偏移)
    cutin_mult = 1.0
    if self.dRel < CUTIN_DIST_LIMIT and abs(self.yRel) > 1.0:
      # 以當前車速的正負 20% 作為完美對稱的零/滿威脅閥值
      v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)
      cutin_mult = float(np.interp(self.vRel, [-v_limit, v_limit], MULT_RANGE))

    # 3. 終極相乘融合 (保留空間不爆衝，倍率最高封頂 1.0)
    final_alpha = ALPHA_BASE * brake_mult * cutin_mult
    return min(1.0, final_alpha)

  def _apply_slow_protection(self, v_ego: float, cam_prob: float, current_ema: float) -> float:
    """
    【階段四：慢速與靜止防護網】
    針對絕對車速低於 20% 的慢速目標，嚴格實施相機門檻審查，根除對靜止金屬物引發幽靈急煞的可能。
    """
    abs_v_lead = abs(self.vRel + v_ego)
    dynamic_v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)  # 絕對車速判定門檻 (保底 1.0 m/s)

    if abs_v_lead < dynamic_v_limit:
      # 視覺物理極限插值：市區低速要求嚴格(0.5)，高速公路相對寬鬆(0.3)
      dynamic_cam_prob_thres = float(np.interp(v_ego, CAM_PROB_SPEED_RANGE, CAM_PROB_RANGE))

      # 若相機未達標，強制壓制 EMA 天花板卡在 0.6，不准觸發提早鎖定
      if cam_prob < dynamic_cam_prob_thres:
        return min(current_ema, STATIC_EMA_CAP)

    return current_ema

  def process_track_logic(self, lead_idx: int, lead_msg: capnp._DynamicStructReader, v_ego: float):
    """
    【主控管線：執行完整的過濾、短路阻斷與打分迴圈】
    """
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    # 🌪️ 漏斗第一關：極速剪枝 (O(1) Fast Pruning)
    # 如果不是實體回波，或橫向粗略偏離預測車道中心超過 2.0m (1.5+0.5 容錯)
    # 直接捨棄衰減，釋放 90% CPU 算力！
    if not self.measured or abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN):
      self.ema_confidence[lead_idx] = ALPHA_DOWN * 0.0 + (1 - ALPHA_DOWN) * self.ema_confidence[lead_idx]
      return


    # 執行階段一：幾何邊界精細審查 (遲滯狀態機)
    is_valid_spatial = self._check_spatial_boundaries(vision_y)

    # 執行階段二：物理模糊打分 (確保三維重合度)
    fuzzy_score = self._calculate_fuzzy_score(offset_vision_dist, vision_y, vision_v, v_ego)

    # ⚡ 終極短路過濾機制 (Short-Circuit Fast Decay)
    # 若精細審查出界，或物理誤差過大 (非重合目標)，強制衰減踢出決策圈
    if not is_valid_spatial or fuzzy_score == 0.0:
      self.ema_confidence[lead_idx] = ALPHA_DOWN * 0.0 + (1 - ALPHA_DOWN) * self.ema_confidence[lead_idx]
      return

    # 執行階段三：高價值目標，精算威脅乘數學習率
    final_alpha_up = self._calculate_threat_multipliers(v_ego)

    # 執行階段四：EMA 狀態平滑更新
    target_ema = fuzzy_score
    alpha = final_alpha_up if fuzzy_score > 0.5 else ALPHA_DOWN
    new_ema = alpha * target_ema + (1 - alpha) * self.ema_confidence[lead_idx]

    # 執行階段四附屬：套用市區慢速靜止防護封頂
    new_ema = self._apply_slow_protection(v_ego, lead_msg.prob, new_ema)

    # 紀錄最終結果
    self.ema_confidence[lead_idx] = new_ema


def get_lead_ext(
  v_ego: float,
  ready: bool,
  tracks: dict[int, TrackSP],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  CP: structs.CarParams,
  CP_SP: structs.CarParamsSP,
  low_speed_override: bool = True,
) -> dict[str, Any]:
  """
  擴充的前車評估函數：導入多階段過濾、反向動態門檻，並提供乾淨的候選名單移交給原廠邏輯。
  絕對遵守「不竄改 dRel 等原始物理數據」的鐵律，將極限減速計算權力 100% 交還給底層控制器。
  """
  lead_idx = 0 if low_speed_override else 1
  max_ema_confidence = 0.0

  if ready:
    # 1. 遍歷所有軌跡，送入管線處理更新狀態
    for track in tracks.values():
      track.process_track_logic(lead_idx, lead_msg, v_ego)

  # ==============================================================================
  # 🛡️ 終極防護網：洗乾淨名單
  # 只將「在車道內」且「未被短路衰減完全歸零」的優良目標挑選出來
  # ==============================================================================
  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane and v.ema_confidence[lead_idx] > 0.0}

  # ==============================================================================
  # 【修補漏洞】：從「本車道內」的合法目標中，提取全場最高 EMA
  # 絕對不拿隔壁車道的超高分來降本車道的門檻，完美防堵幽靈急煞！
  # ==============================================================================
  if len(valid_tracks) > 0:
    max_ema_confidence = max(track.ema_confidence[lead_idx] for track in valid_tracks.values())

  # ==============================================================================
  # 【階段五：全局門檻映射】
  # 使用反向線性插值，將全場最高的 EMA 信心度無縫映射為視覺門檻。
  # EMA 越高，原廠視覺把關門檻就被我們降得越低。若 EMA <= 0.4，則死守原廠 0.5 門檻。
  # ==============================================================================
  current_prob_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))

  # ========================================================================
  # 下方為原廠 get_lead 判斷邏輯移植
  # 將死板的 0.5 替換為 dynamic current_prob_thres，並餵給它乾淨的 valid_tracks
  # ========================================================================

  # Determine leads, this is where the essential logic happens
  if len(valid_tracks) > 0 and ready and lead_msg.prob > current_prob_thres:
    track = match_vision_to_track(v_ego, lead_msg, valid_tracks)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    # 取得原始物理數據，絕不進行竄改覆寫
    lead_dict = track.get_RadarState(lead_msg.prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)

    # 記錄由我們客製邏輯所觸發的提早鎖定事件，方便日後從 LOG 追蹤效能
    if current_prob_thres < 0.5 and (0.5 >= lead_msg.prob > current_prob_thres):
      cloudlog.debug(
        f"[RadarDSP_EarlyLock] 提早鎖定成功！目標 {lead_idx} | "
        f"相機機率: {lead_msg.prob:.2f} (動態門檻: {current_prob_thres:.2f}) | "
        f"最高 EMA: {max_ema_confidence:.2f}"
      )

  elif (track is None) and ready and (lead_msg.prob > current_prob_thres):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  # ==============================================================================
  # 🛡️ 原廠底線救援 (Low Speed Override Fallback)
  # 極低速 (< 14.4 km/h) 專屬防護：就算視覺瞎掉，雷達只要確認正前方極近距離有慢速目標，
  # 就強制寫入最終輸出，完美防止市區危險蠕行，達成優雅兜底！
  # ==============================================================================
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
# 1. 強制將原廠 radard 模組裡的 Track 類別替換為我們寫好的 TrackSP。
radard.Track = TrackSP
# 2. 強制將原廠的 get_lead 判斷邏輯，替換為我們帶有多層防護的 get_lead_ext。
radard.get_lead = get_lead_ext
# ==============================================================================


class RadarDSP(RadarD):
  """
  繼承自 RadarD。
  由於已透過 Monkey Patching 從源頭替換了底層運作的類別與判斷函數，
  這個類別保持極致的簡潔，一行原廠運算迴圈都不必改寫，完美相容未來原廠更新！
  """

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    super().__init__(CP, CP_SP, delay)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    # 呼叫 super().update 時，底層將自動使用我們寫好的 TrackSP 與 get_lead_ext 進行運作
    super().update(sm, rr)
