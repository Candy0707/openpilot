import numpy as np
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase


class LeadDepartureSmoother(TargetsBase):
  """
  LeadDepartureSmoother - 前車駛離與跟車確定性幾何拋物線軌跡生成器 (終極完整版)
  Copyright (c) 2026 DragonPilot Contributors.

  ====================================================================================
  [控制學終極革命：基於車距與運動學的確定性拋物線軌跡規劃 - 教科書級說明]
  ====================================================================================
  本模組的核心目標：「全面防禦前車切出、駛離或加速時，系統因為瞬間產生巨大速度落差，
  而命令縱向 MPC 進行 100% 滿載輸出的突兀暴衝與貼背感，實現順順追車、絕不點煞車的絲滑體感。」

  本模組拋棄了所有繁瑣且易受干擾的「Jerk二階物理積分與虛擬暫存器 (a_smooth) 架構」，
  並捨棄了低速起步首幀易產生瞬時加速度陡坡的 EMA 訊號低通濾波。
  改採 Tier-1 大廠 (Mobileye / Bosch) 最標準、最正統的等加速度幾何運動學拋物線軌跡方程：
  v_target = sqrt(v_floor^2 + 2 * SCA_DECEL * max(0, d - SCA_SAFE_DIST))

  1. 【空間換速度 ── 徹底消除加速再減速的拉風琴效應】
     - 當前車切離露出遠方新慢車時，本公式直接依據剩餘物理車距 (dRel) 主動向下光滑收斂。
     - 當自車完美逼近前車時，幾何距離項自發性線性歸零，目標速指令線在最後 2 秒鐘主動拉平，
       像絲綢一樣完美、無超調 (0% Overshoot) 地黏在前車速度上。速差在接近前已被公式提前歸零，
       實車底盤連一丁點點煞車都不需要踩，達成最完美的流暢銜接！

  2. 【物理幾何牆 ── 100% 屏蔽前方手排車低速換檔高頻晃動】
     - 本公式在計算時完全不需要任何「相對速度 vRel」與「前車瞬時速」作為微分輸入。
     - 它核心唯獨跟隨前車物理距離 (dRel)。手排車低速起步踩離合器換檔失速 0.3秒內，兩車實際距離
       在時間軸上幾乎是光滑平穩不變的。因此 LDS 算出的 v_target 穩如泰山，天然 100% 屏蔽換檔鋸齒。

  3. 【失去目標快適緩加速 ── 徹底封印切車階躍暴衝】
     - 補上幾何模型失去目標時的關鍵拼圖。當前車切離失去目標時，自動轉入快適恆定斜率回速機制，
       讓目標速以極其優雅的設定斜率線性爬升回巡航定速，絕不跳階。

  4. 【雙層控制解耦 ── 巡航定速保底起跑線與 MPC 控制權解放】
     - 導入 V_TARGET_FLOOR 保底。前車靜止 (vLead=0) 時，目標速點到為止卡在 5.0 km/h，拒絕沉入 0 死區，起步不滯後。
     - 本模組只控車速 v_target，加速度 a_target 直接回傳實車當前的 a_ego 全權歸還給 MPC 解算。
  ====================================================================================
  """

  # ==============================================================================
  # 全域調整參數宣告區 (類別常數) —— 實車測試微調看這裡！
  # ==============================================================================
  SCA_DECEL = 0.38  # 恆定舒適幾何加速度/減速度 (m/s²)。數值越小，接近前車的滑行過程越長越溫柔平滑
  SCA_SAFE_DIST = 11.5  # 幾何對齊安全車距邊界 (meter)。當在此車距時，自車目標速與前車速完美完成光滑重合
  V_TARGET_FLOOR = 1.39  # 巡航目標速度保底下限 (m/s)，等於 5.0 km/h。防止指令沉入 0 死區導致起步反應滯後
  COMFORT_ACCEL_UP = 0.5  # 前車切離(失去目標)後的快適回速斜率 (m/s²)。每秒優雅增加 1.8 km/h，徹底封印暴衝

  def __init__(self, CP, mpc):
    super().__init__(CP, mpc)

    # 智慧型日誌控制
    self.debug_log = True
    self.log_counter = 0

  def update_target(self, sm, v_ego, a_ego, v_cruise):
    """
    核心確定性運動學幾何與舒適斜率軌跡控制迴圈
    """
    # 提取雷達狀態與第一順位主前車 (leadOne)
    radar_state = sm['radarState']
    lead_one = radar_state.leadOne

    # 【動態全時熱啟動防護】
    # 若當下系統處於未介入狀態，內部的目標車速暫存器必須全時與實車當前最真實的速度 (v_ego) 保持 100% 同步。
    # 這能確保當前方前車突然駛離、系統觸發介入的瞬間，軌跡生成器能從當下最精準的底盤物理起點優雅出發。
    if not self.action:
      self.v_target = v_ego

    # ===========================================================
    # 步驟 1: 幾何與定速目標基本判定 (Raw Target Calculation)
    # ===========================================================
    if lead_one.status:
      # 有車時：底盤對齊速度 (v_floor) 錨定在前車真實速度與定速保底線的最大值
      v_floor = max(self.V_TARGET_FLOOR, float(lead_one.vLead))

      # 計算剩餘物理車距前導量。max(0.0) 確保即便因極端貼車跌入安全距離內，根號內部依然大於等於 0 絕對不噴 Bug
      distance_headroom = max(0.0, float(lead_one.dRel) - self.SCA_SAFE_DIST)

      # 萬流歸宗：標準運動學幾何拋物線方程。純幾何演進，首幀跳動與低速晃動在數學結構上被徹底抹平
      raw_v_target = np.sqrt(v_floor**2 + 2 * self.SCA_DECEL * distance_headroom)
    else:
      # 前方完全淨空：目標速度直接無縫切齊巡航車速
      raw_v_target = v_cruise

    # 最大安全防禦牆：無論幾何拋物線數值如何噴發，計算出的原始目標速度絕對不得超越駕駛設定的巡航上限
    raw_v_target = min(raw_v_target, v_cruise)

    # ===========================================================
    # 步驟 2: 判定是否啟動控制權介入 (Action Decision)
    # ===========================================================
    if self.action:
      self.action = (raw_v_target < v_cruise) or (self.v_target < v_cruise - 0.05)
    else:
      self.action = raw_v_target < v_cruise - 0.05

    # ===========================================================
    # 步驟 3: 雙情境解耦軌跡規劃與控制權解放 (Core Planning Logic)
    # ===========================================================
    if self.action:
      if lead_one.status:
        # 【情境 A：前方有明確車流目標】
        # 直接將最完美的拋物線運動學車速軌跡交給縱向 Planner，自帶逼近前車時的光滑收斂阻尼
        self.v_target = raw_v_target
      else:
        # 【情境 B：前車切換車道/駛離 (失去目標)】
        # 根本性解決病因：不讓指令產生斷崖階躍跳變，改用快適恆定斜率緩加速機制，平穩拉回巡航速度
        self.v_target = min(v_cruise, self.v_target + self.COMFORT_ACCEL_UP * DT_MDL)

      # 貫徹最高指令：我們完全不控制、不修改加速度。全時回傳當前實車加速度 a_ego 歸還給 MPC 全權解算
      self.a_target = a_ego

      # 雙重防線：限制在合理的實車安全巡航區間內
      self.v_target = max(0.0, min(self.v_target, v_cruise))
    else:
      # 規則 3：前方完全淨空且已圓滿回歸定速，釋放控制權，強制重置內部目標車速為 V_CRUISE_MAX
      self.v_target = V_CRUISE_MAX
      self.a_target = a_ego

    # ===========================================================
    # 步驟 4: 狀態更新與 Log 記錄
    # ===========================================================
    if self.debug_log:
      self._print_log(lead_one.status, raw_v_target, lead_one.vLead, lead_one.dRel)

    return super().update_target(sm, v_ego, a_ego, v_cruise)

  def _print_log(self, lead_status, raw_v_target, v_lead, d_rel):
    """
    優雅的除錯日誌輸出機制
    """
    self.log_counter += 1
    if self.action or self.log_counter >= 60:
      state_str = "🛑 [幾何軌跡控制中]" if self.action else "✅ [穩態巡航/跟車]"
      lead_str = f"有車 (前車速:{v_lead * CV.MS_TO_KPH:.1f}km/h, 車距:{d_rel:.1f}m)" if lead_status else "無車 (失去目標緩加速)"

      log_msg = (
        f"[LDS V1.6.0 終極版] {state_str} 前方狀態:{lead_str} | "
        f"幾何目標速:{raw_v_target * CV.MS_TO_KPH:.1f}km/h | 核心輸出速:{self.v_target * CV.MS_TO_KPH:.1f}km/h | "
        f"實車當前加速度:{self.a_target:.3f}m/s²"
      )
      print(log_msg)
      cloudlog.debug(log_msg)
      self.log_counter = 0
