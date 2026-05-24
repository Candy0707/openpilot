import numpy as np
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase

class LeadDepartureSmoother(TargetsBase):
  """
  LeadDepartureSmoother - 前車駛離與跟車加加速度平滑控制器 (極簡純粹版)
  Copyright (c) 2026 DragonPilot Contributors.

  ====================================================================================
  [系統架構與物理控制原理 - 教科書級說明]
  ====================================================================================
  本模組的核心目標：「全面防禦前車切出、駛離或加速時，系統因為瞬間產生巨大速度落差，
  而命令縱向 MPC 進行 100% 滿載輸出的突兀暴衝與貼背感。」

  本模組貫徹極簡控制哲學，不區分前車加速或減速，全權交由單一加加速度限制器進行全時平滑：

  1. 【全域動態目標速度鎖定 (Dynamic Target Anchoring)】
     - 每一幀 (Frame) 直接讀取優質前車訊號 (`leadOne`)。
     - 當前方有車時：輸出速度 = 前車速度 + 1.0 m/s，最大不可超過巡航車速 v_cruise。
     - 當前方淨空時：輸出速度 = 巡航車速 v_cruise。

  2. 【加加速度限制器控制核心 (Jerk-Limited Rate Controller)】
     - 剩下的完全交由 Jerk 限制器進行平滑處理。
     - 透過限制每個模型運算幀 (DT_MDL = 0.05s) 內的加速度變化率 (Jerk)，配合實車當前 a_ego，
       自動將任何階躍的速度指令熨平成完美的 S 型平滑加速曲線，達到舒適回速的效果。

  3. 【MPC 控制權解放 (MPC Accel Handover)】
     - 本模組只控制車速 (v_target)，不控制加速度。全時直接回傳實車當前的 a_ego。
     - 讓車速指令優雅交給下游 MPC 進行最終的縱向動態優化解算。
  ====================================================================================
  """

  # ==========================================
  # 物理常數與舒適性邊界設定
  # ==========================================
  MAX_JERK = 1.2  # 絲滑體感核心：最大允許的加加速度 (m/s^3)。控制速度增長率，消除突兀暴衝
  V_OFFSET = 1.0  # 前車速度餘裕 (m/s)，約等於 3.6 km/h。防止系統追車追不到且距離越拉越遠

  def __init__(self, CP, mpc):
    super().__init__(CP, mpc)

    # 智慧型日誌控制
    self.debug_log = True
    self.log_counter = 0

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
      # 規則 1：有車，輸出速度 = 前車速度 + 1 m/s
      raw_v_target = lead_one.vLead + self.V_OFFSET
    else:
      # 規則 2：沒車，輸出速度 = v_cruise
      raw_v_target = v_cruise

    # 最大不可超過 v_cruise
    raw_v_target = min(raw_v_target, v_cruise)

    # ===========================================================
    # 步驟 2: 判定是否啟動控制權介入 (Action Decision)
    # ===========================================================
    if self.action:
      self.action = (raw_v_target < v_cruise) or (self.v_target < v_cruise - 0.05)
    else:
      self.action = (raw_v_target < v_cruise - 0.05)

    # ===========================================================
    # 步驟 3: 萬流歸宗 ── 純粹的 Jerk 平滑控制器
    # ===========================================================
    if self.action:
      # A. 逆推當前幀所需的理想目標加速度
      a_req = (raw_v_target - self.v_target) / DT_MDL

      # B. 將理想加速度限制在車輛縱向物理安全範圍內 (2.0 至 -3.5 m/s²)
      a_req = np.clip(a_req, ACCEL_MIN, ACCEL_MAX)

      # C. 以實車當前加速度 a_ego 為基準，計算變更率 (Jerk)
      jerk = (a_req - a_ego) / DT_MDL

      # D. 強制限制加加速度，不分加速減速，全由 MAX_JERK 一網打盡，自然生成 S 曲線
      jerk_clipped = np.clip(jerk, -self.MAX_JERK, self.MAX_JERK)

      # E. 透過 Jerk 限制後的增量更新目標車速 (我們只控制車速，不控制加速度)
      v_acc = a_ego + jerk_clipped * DT_MDL
      self.v_target += v_acc * DT_MDL

      # 直接回傳當前實車加速度，交由 MPC 算車速
      self.a_target = a_ego

      # 雙重防線：限制在合理車速區間內
      self.v_target = max(0.0, min(self.v_target, v_cruise))
    else:
      # 規則 3：歸還控制權時必須強制重置內部目標車速為 V_CRUISE_MAX
      self.v_target = V_CRUISE_MAX
      self.a_target = a_ego

    # ===========================================================
    # 步驟 4: 狀態更新與 Log 記錄
    # ===========================================================
    if self.debug_log:
      self._print_log(lead_one.status, raw_v_target, lead_one.vLead)

    return super().update_target(sm, v_ego, a_ego, v_cruise)

  def _print_log(self, lead_status, raw_v_target, v_lead):
    """
    優雅的除錯日誌輸出機制
    """
    self.log_counter += 1
    if self.action or self.log_counter >= 60:
      state_str = "🛑 [平滑介入中]" if self.action else "✅ [穩態巡航/跟車]"
      lead_str = f"有車 (前車速:{v_lead * CV.MS_TO_KPH :.1f}km/h)" if lead_status else "無車 (前方淨空)"

      log_msg = (
        f"[LDS V1.2.0] {state_str} 前方狀態:{lead_str} | "
        f"原始目標速:{raw_v_target * CV.MS_TO_KPH:.1f}km/h | 核心輸出速:{self.v_target * CV.MS_TO_KPH:.1f}km/h | "
        f"實車當前加速度:{self.a_target:.3f}m/s²"
      )
      print(log_msg)
      cloudlog.debug(log_msg)
      self.log_counter = 0