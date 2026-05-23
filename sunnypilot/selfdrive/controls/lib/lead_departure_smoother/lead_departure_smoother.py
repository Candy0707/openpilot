import numpy as np
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN

class LeadDepartureSmoother(TargetsBase):
  """
  LeadDepartureSmoother - 前車駛離與跟車加加速度平滑控制器
  Copyright (c) 2026 DragonPilot Contributors.

  ====================================================================================
  [系統架構與物理控制原理 - 教科書級說明]
  ====================================================================================
  本模組的核心目標：「全面防禦前車切出、駛離或加速時，系統因為瞬間產生巨大速度落差，
  而命令縱向 MPC 進行 100% 滿載輸出的突兀暴衝與貼背感。」

  為了將控制邏輯精簡到極致，本模組放棄了繁瑣的狀態機跳變與車距過濾，回歸純粹的數值追隨控制：

  1. 【全域動態目標速度鎖定 (Dynamic Target Anchoring)】
     - 每一幀 (Frame) 直接讀取經過濾波與追蹤的優質前車訊號 (`leadOne`)。
     - 當前方有車時：目標速度直接鎖定前車速度並補上固定的速度餘裕（+1.0 m/s），確保自車
       具備充足的追隨渴望，絕不因為穩態誤差而被前車越拉越遠。
     - 當前方淨空時：目標速度直接無縫切換為駕駛設定的巡航車速 (`v_cruise`)。

  2. 【加加速度限制器控制核心 (Jerk-Limited Rate Controller)】
     - 當目標速度因為前方淨空或前車加速而產生斷崖式跳變時，後端完全交由物理 Jerk 限制器處理。
     - 透過精準控制每個模型運算幀 (DT_MDL = 0.05s) 內的加速度變化率 (Jerk)：
       a_req = (raw_v_target - v_target) / DT_MDL
       jerk = (a_req - a_target) / DT_MDL
       jerk_clipped = np.clip(jerk, -MAX_JERK, MAX_JERK)
     - 經由一階與二階物理積分，將原始的階躍速度指令（Step Input）熨平成優雅絲滑的
       S 型加速曲線（S-curve），達成真正的量產級舒適體感。
  ====================================================================================
  """

  # ==========================================
  # 物理常數與舒適性邊界設定
  # ==========================================
  MAX_JERK = 1.2  # 絲滑體感核心：最大允許的加加速度 (m/s^3)。鉗制此值可控制加速度增長率，消除突兀暴衝
  V_OFFSET = 1.0  # 前車速度餘裕 (m/s)，約等於 3.6 km/h。給予適度溢價，防止跟車距離越拉越遠

  # 縱向安全加速度邊界控制
  ACCEL_MAX_LIMIT = ACCEL_MAX  # 限制理想目標加速度上限 (m/s^2)
  ACCEL_MIN_LIMIT = ACCEL_MIN  # 限制理想目標減速度下限 (m/s^2)

  def __init__(self, CP, mpc):
    super().__init__(CP, mpc)

    # 智慧型日誌控制
    self.debug_log = True
    self.log_counter = 0

  def update_params(self):
    """
    【參數更新覆寫】
    從系統內存參數讀取開關狀態。若參數尚不存在，則預設維持啟動 (True) 狀態，確保即裝即用。
    """
    param_val = self.params.get_bool(self.classname)
    self.enable = param_val if param_val is not None else True

  def update_target(self, sm, v_ego, a_ego, v_cruise):
    """
    核心目標車速與 Jerk 平滑控制迴圈
    """
    # 提取雷達狀態與第一順位主前車 (leadOne)
    radar_state = sm['radarState']
    lead_one = radar_state.leadOne

    # 【動態熱啟動防護】
    # 若當下系統處於未介入狀態，內部的物理追蹤暫存器必須全時與實車當前狀態 (v_ego, a_ego) 保持同步。
    # 這能確保當前車駛離、系統觸發介入的瞬間，Jerk 限制器能夠從「當下最真實的物理起點」開始平滑過渡。
    if not self.action:
      self.v_target = v_ego
      self.a_target = a_ego

    # ===========================================================
    # 步驟 1: 全域目標速度判定 (Raw Target Calculation)
    # ===========================================================
    if lead_one.status:
      # 前方有車：目標車速直接錨定前車速度，並加上舒適跟車餘裕 (V_OFFSET)
      raw_v_target = lead_one.vLead + self.V_OFFSET
    else:
      # 前方淨空：目標車速直接對齊駕駛設定的巡航最高車速
      raw_v_target = v_cruise

    # 安全防禦牆：無論前車速度如何波動，計算出的原始目標速度絕對不得超越駕駛設定的巡航上限
    raw_v_target = min(raw_v_target, v_cruise)

    # ===========================================================
    # 步驟 2: 判定是否啟動控制權介入 (Action Decision)
    # ===========================================================
    # 觸發條件極致精簡：
    # 1. 當前車限速小於巡航速度時（前方有慢車，需壓制速度維持跟車）。
    # 2. 當系統正在執行 Jerk 平滑回升，且內部目標速度尚未完全追上巡航設定值時（過渡期防護）。
    self.action = (raw_v_target < v_cruise) or (self.v_target < v_cruise - 0.05)

    # ===========================================================
    # 步驟 3: Jerk 限制器與雙階物理積分器 (Jerk-Limited Rate Control)
    # ===========================================================
    if self.action:
      # A. 逆推當前幀所需的理想目標加速度 (要求在單幀時間 DT_MDL 內消除與原始目標的速度差)
      a_req = (raw_v_target - self.v_target) / DT_MDL

      # B. 將理想加速度鉗制在車輛縱向物理安全範圍內
      a_req = np.clip(a_req, self.ACCEL_MIN_LIMIT, self.ACCEL_MAX_LIMIT)

      # C. 計算該加速度與上一幀實際規劃加速度的變更率 (即真實加加速度 Jerk)
      jerk = (a_req - self.a_target) / DT_MDL

      # D. 【動態金箍咒】強制將變化率鉗制在舒適加加速度 MAX_JERK 範圍內
      # 當前車突然消失時，jerk 會是極大的正值。透過 clip 壓制它，加速度就只能以穩健、緩慢的斜率爬升
      jerk_clipped = np.clip(jerk, -self.MAX_JERK, self.MAX_JERK)

      # E. 一階物理積分：更新內部規劃加速度
      self.a_target += jerk_clipped * DT_MDL

      # F. 二階物理積分：由平滑後的加速度，積分推導出最終絲滑的目標速度指令
      self.v_target += self.a_target * DT_MDL

      # 雙重防線：確保積分運算不產生幾何發散，死死鉗制在合理車速區間內
      self.v_target = max(0.0, min(self.v_target, v_cruise))
    else:
      # 前方完全淨空且已圓滿回升至巡航定速，釋放控制權，回歸系統預設
      self.v_target = v_cruise
      self.a_target = a_ego

    # ===========================================================
    # 步驟 4: 狀態更新、輸出與 Log 記錄
    # ===========================================================
    if self.debug_log:
      self._print_log(lead_one.status, raw_v_target, lead_one.vLead)

    # 呼叫基底類別 (TargetsBase) 的核心仲裁邏輯，自動完成與 V_CRUISE_MAX 的安全裁切並寫入最終輸出
    return super().update_target(sm, v_ego, a_ego, v_cruise)

  def _print_log(self, lead_status, raw_v_target, v_lead):
    """
    優雅的除錯日誌輸出機制
    介入時每幀即時列印以利精準分析；巡航或跟車平穩時每 60 幀 (約 1 秒) 輸出一次心跳包，拒絕洗版。
    """
    self.log_counter += 1
    if self.action or self.log_counter >= 60:
      state_str = "🛑 [平滑介入中]" if self.action else "✅ [穩態巡航/跟車]"
      lead_str = f"有車 (前車速:{v_lead * 3.6:.1f}km/h)" if lead_status else "無車 (前方淨空)"

      log_msg = (
        f"[LDS V1.0.0] {state_str} 前方狀態:{lead_str} | "
        f"原始目標速:{raw_v_target * 3.6:.1f}km/h | 核心輸出速:{self.v_target * 3.6:.1f}km/h | "
        f"規劃加速度:{self.a_target:.3f}m/s²"
      )
      print(log_msg)
      cloudlog.debug(log_msg)
      self.log_counter = 0