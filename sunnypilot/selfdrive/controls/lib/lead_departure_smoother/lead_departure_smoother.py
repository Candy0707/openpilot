import numpy as np
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase

class LeadDepartureSmoother(TargetsBase):
  """
  LeadDepartureSmoother - 前車駛離與跟車雙階 EMA 車速平滑濾波控制器
  Copyright (c) 2026 DragonPilot Contributors.

  ====================================================================================
  [控制學革命：純車速域雙階級聯低通濾波原理 - 教科書級說明]
  ====================================================================================
  本模組的核心目標：「全面防禦前車切出、駛離或加速時，系統因為瞬間產生巨大速度落差，
  而命令縱向 MPC 進行 100% 滿載輸出的突兀暴衝與貼背感，實現順順追車、絕不點煞車的絲滑體感。」

  本模組徹底顛覆傳統繁瑣的「二階物理積分限制（逆推 a_req -> 鉗制 Jerk -> 積分速度）」，
  將控制維度完美降維至純粹的「訊號濾波學」，從根本上免除了感測器雜訊與狀態死鎖的硬體盲點：

  1. 【全域動態目標速度鎖定 (Dynamic Target Anchoring)】
     - 每一幀直接讀取優質前車訊號 (`leadOne`)。
     - 結合相對速度阻尼項（隨 vRel 增大而變大，接近時自動收回），提供完美的接近緩和體感。

  2. 【雙階級聯 EMA (Nested Exponential Moving Average)】
     - 如果只用單階 EMA，在目標速跳階時第一幀的推力會最大，體感會略顯突兀。
     - 本模組將兩個一階低通濾波器進行級聯（串聯），在控制工程中這等同於「二階臨界阻尼濾波器」。
     - 完全不需要計算任何加速度與變化率，系統會自發性地在物理世界中揉捏出最優雅的 S 型加速曲線。

  3. 【控制權解放與零發散安全】
     - 本模組只管控車速指令 v_target，加速度 a_target 直接回傳實車當前的 a_ego 歸還給 MPC。
     - EMA 的數學特性為內插，輸出值永遠被夾在實車速與巡航速的安全廊道內，絕不發散、無蓄能暴衝風險。
  ====================================================================================
  """

  # ==============================================================================
  # 全域調整參數宣告區 (類別常數) —— 實車測試微調看這裡！
  # ==============================================================================
  # 1. 相對速度動態餘裕參數 (控制追車下拉力與接近阻尼)
  V_OFFSET_BASE = 1.0  # 保底速度餘裕 (m/s)，約 3.6 km/h。防止 MPC 產生穩態誤差而被前車越拉越遠
  V_OFFSET_GAIN = 0.20 # 相對速度增益係數。前車加速拉開時，依此係數放大餘裕提供強大拉力；
                       # 快速接近前車時，相對速度變小，餘裕自動收回，主動提前鬆油門，消滅加速後點煞車的震盪。

  # 2. 自車速動態靈敏度參數 (控制 EMA 濾波時間常數 Alpha)
  # 起步低速時 Alpha 大（時間常數短，動態響應極快，跨越傳動死區，起步輕快有勁）
  # 高速巡航時 Alpha 小（時間常數長，提供極致的定速濾震能力，保障高速乘客舒適度）
  ALPHA_BP = [0.0, 5.0, 15.0]     # 自車速度中斷點 (m/s)，分別對應 0, 18, 54 km/h
  ALPHA_V  = [0.12, 0.06, 0.025]  # 各車速中斷點對應的 Alpha 權重值

  def __init__(self, CP, mpc):
    super().__init__(CP, mpc)

    # 雙階級聯低通濾波器的內部第一階核心緩衝狀態暫存器
    self.v_filter = 0.0

    # 智慧型日誌控制
    self.debug_log = True
    self.log_counter = 0

  def update_target(self, sm, v_ego, a_ego, v_cruise):
    """
    核心目標車速純 EMA 級聯濾波控制迴圈
    """
    # 提取雷達狀態與第一順位主前車 (leadOne)
    radar_state = sm['radarState']
    lead_one = radar_state.leadOne

    # 【動態全時熱啟動防護】
    # 若當下系統處於未介入狀態，內部的雙階低通狀態必須與實車當前最真實的速度 (v_ego) 保持 100% 同步。
    # 這能確保當前方前車突然駛離、系統觸發介入的瞬間，濾波器能從當下最精準的物理起點優雅出發。
    if not self.action:
      self.v_target = v_ego
      self.v_filter = v_ego

    # ===========================================================
    # 步驟 1: 結合相對速度之全域目標速度判定 (Raw Target Calculation)
    # ===========================================================
    if lead_one.status:
      # 導入智慧相對速度餘裕：餘裕 = 保底值 + 增益 * max(0.0, 前車相對速度)
      v_offset_dynamic = self.V_OFFSET_BASE + self.V_OFFSET_GAIN * max(0.0, float(lead_one.vRel))

      # 規則 1：有車，輸出速度 = 前車速度 + 動態相對速度餘裕
      raw_v_target = lead_one.vLead + v_offset_dynamic
    else:
      # 規則 2：沒車，輸出速度 = v_cruise
      raw_v_target = v_cruise

    # 最大安全屏障：無論相對速度如何拉大，計算出的原始目標速度絕對不得超越駕駛設定的巡航上限
    raw_v_target = min(raw_v_target, v_cruise)

    # ===========================================================
    # 步驟 2: 判定是否啟動控制權介入 (Action Decision) —— 狀態機維持不變
    # ===========================================================
    if self.action:
      self.action = (raw_v_target < v_cruise) or (self.v_target < v_cruise - 0.05)
    else:
      self.action = (raw_v_target < v_cruise - 0.05)

    # ===========================================================
    # 步驟 3: 萬流歸宗 ── 雙階級聯 EMA 車速濾波核心迴圈 (去加速度化)
    # ===========================================================
    if self.action:
      # A. 依據當前自車速度，動態查表獲取最適合當前情境的 Alpha 權重
      alpha = float(np.interp(v_ego, self.ALPHA_BP, self.ALPHA_V))

      # B. 第一階低通濾波：熨平跳階原始目標速度指令的斷崖突波
      self.v_filter = alpha * raw_v_target + (1.0 - alpha) * self.v_filter

      # C. 第二階低通級聯濾波：由第一階狀態再度濾波，自發性在時間軸上勾勒出完美的 S 加速曲線
      self.v_target = alpha * self.v_filter + (1.0 - alpha) * self.v_target

      # D. 依據要求：我們完全不控制、不修改加速度，直接回傳當前實車加速度 a_ego 給上層 Planner 餵給 MPC
      self.a_target = a_ego

      # 安全廊道雙重限制：確保最終指令死鎖在合理物理區間內
      self.v_target = max(0.0, min(self.v_target, v_cruise))
    else:
      # 規則 3：歸還控制權時必須強制將內部目標車速與第一階核心緩衝同步重置為 V_CRUISE_MAX
      self.v_target = V_CRUISE_MAX
      self.v_filter = v_ego
      self.a_target = a_ego

    # ===========================================================
    # 步驟 4: 狀態更新與 Log 記錄
    # ===========================================================
    if self.debug_log:
      self._print_log(lead_one.status, raw_v_target, lead_one.vLead, lead_one.vRel)

    return super().update_target(sm, v_ego, a_ego, v_cruise)

  def _print_log(self, lead_status, raw_v_target, v_lead, v_rel):
    """
    優雅的除錯日誌輸出機制
    介入時每幀即時列印以利精準分析；巡航或跟車平穩時每 60 幀 (約 1 秒) 輸出一次心跳包，拒絕洗版。
    """
    self.log_counter += 1
    if self.action or self.log_counter >= 60:
      state_str = "🛑 [EMA平滑中]" if self.action else "✅ [穩態巡航/跟車]"
      lead_str = f"有車 (前車速:{v_lead * CV.MS_TO_KPH :.1f}km/h, 相對速:{v_rel * CV.MS_TO_KPH :+.1f}km/h)" if lead_status else "無車 (前方淨空)"

      log_msg = (
        f"[LDS V1.4.0 純EMA版] {state_str} 前方狀態:{lead_str} | "
        f"原始目標速:{raw_v_target * CV.MS_TO_KPH:.1f}km/h | 核心輸出速:{self.v_target * CV.MS_TO_KPH:.1f}km/h | "
        f"一階緩衝速:{self.v_filter * CV.MS_TO_KPH:.1f}km/h | 實車當前加速度:{self.a_target:.3f}m/s²"
      )
      print(log_msg)
      cloudlog.debug(log_msg)
      self.log_counter = 0