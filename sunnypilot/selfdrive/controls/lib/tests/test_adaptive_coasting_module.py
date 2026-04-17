"""
Test Suite for Adaptive Coasting Module (ACM) - Real World Edition
(全場景實車路況模擬測試：已校準物理動態參數，避開極端保命攔截)
"""

import pytest
import math
import numpy as np

# ⚠️ 注意：請確認此 Import 路徑符合您目前的專案結構！
from sunnypilot.selfdrive.controls.lib.adaptive_coasting_module import AdaptiveCoastingModule, STOP_DISTANCE


class MockLead:
  def __init__(self, status=False, dRel=0.0, vRel=0.0):
    self.status = status
    self.dRel = dRel
    self.vRel = vRel


class MockRadarState:
  def __init__(self, lead):
    self.leadOne = lead


class MockSubMaster(dict):
  def __init__(self, radar_state):
    super().__init__()
    self['radarState'] = radar_state
    self.valid = {'radarState': True}


class TestAdaptiveCoastingModule:
  @pytest.fixture
  def acm(self):
    """初始化 ACM 狀態機"""
    return AdaptiveCoastingModule()

  def _create_sm(self, status, d_rel, v_rel):
    """產生模擬的雷達 SubMaster 資料"""
    lead = MockLead(status=status, dRel=d_rel, vRel=v_rel)
    radar_state = MockRadarState(lead)
    return MockSubMaster(radar_state)

  def _calc_drel(self, v_ego, t_follow, target_percent):
    """
    真實物理距離反推器：給定車速、秒數與預期的百分比，反推雷達應回報的 d_rel
    公式：dynamic_actual = target_percent * dynamic_target
         d_rel = dynamic_actual + STOP_DISTANCE
    """
    dynamic_target = max(v_ego, 2.0) * t_follow
    dynamic_actual = dynamic_target * target_percent
    return dynamic_actual + STOP_DISTANCE

  # ==========================================================
  # [場景 1: 無車狀態] - E2E 視覺模型神經質微煞車抹平
  # 真實路況：高速巡航時，純視覺模型常會給出 -0.2 ~ -0.3 的幽靈微煞車
  # ==========================================================
  def test_no_lead_smooth_nervous_braking(self, acm):
    v_ego, t_follow = 25.0, 1.5  # 90 km/h
    sm = self._create_sm(status=False, d_rel=0.0, v_rel=0.0)

    # 模擬真實軌跡：包含神經質微煞車 (-0.2), 輕微加速 (0.1), 與真正需要煞車的重煞 (-0.8)
    a_desired = [-0.2, -0.3, 0.1, -0.8] + [0.0] * 29
    result = acm.update(sm, a_desired, v_ego, t_follow)

    # 驗證：-0.2 與 -0.3 被抹平為 0.0，但 0.1 與 -0.8(超出抹平極限) 必須原封不動放行！
    assert result[0] == 0.0, "錯誤：微煞車未被抹平"
    assert result[1] == 0.0, "錯誤：微煞車未被抹平"
    assert result[2] == 0.1, "錯誤：無車時的微加速不應被攔截"
    assert result[3] == -0.8, "錯誤：真實重煞車被錯誤抹平，非常危險！"

  # ==========================================================
  # [場景 2: 區域 A (單純滑行)] - 遠距離提前收油門
  # 真實路況：距離 90%，前車微煞車，系統應提早進入 0.0 滑行狀態
  # ==========================================================
  def test_zone_a_pure_coasting(self, acm):
    v_ego, t_follow = 20.0, 1.5  # 72 km/h
    d_rel = self._calc_drel(v_ego, t_follow, 0.90)  # 90% 距離
    v_rel = -1.0  # 正在緩慢接近前車
    sm = self._create_sm(status=True, d_rel=d_rel, v_rel=v_rel)

    # 原廠想要給出輕微的煞車 (-0.5) 或微補油門 (0.2)
    a_desired = [-0.5, 0.2] * 16 + [-0.5]
    result = acm.update(sm, a_desired, v_ego, t_follow)

    # 驗證：在區域 A，所有數值都應該被死鎖在 0.0，實現完美滑行
    for val in result:
      assert val == 0.0, f"錯誤：區域 A 未執行絕對滑行，得到了 {val}"
    assert acm.acm_active is True

  # ==========================================================
  # [場景 3: 區域 B (平滑退讓)] - TTA 動態減速運算
  # 真實路況：距離 80%，前車明顯減速，系統應開始計算 TTA 並平滑介入
  # ==========================================================
  def test_zone_b_smooth_yield(self, acm):
    v_ego, t_follow = 15.0, 1.5
    d_rel = self._calc_drel(v_ego, t_follow, 0.80)  # 80% 距離 (落在 75~85 區間)

    # 🌟 修正點：使用溫和的逼近速度 -0.5 m/s。
    # (避免 raw_a_calc 小於 -1.2 m/s² 而錯誤觸發「TTA極限保命退場」)
    v_rel = -0.5
    sm = self._create_sm(status=True, d_rel=d_rel, v_rel=v_rel)

    a_desired = [0.0] * 33
    result = acm.update(sm, a_desired, v_ego, t_follow)

    # 驗證物理限制：ACM 給出的退讓力道必須被限制在 MIN_RECOVERY_ACCEL (-1.0) 與 0.0 之間
    for val in result:
      assert -1.0 <= val <= 0.0, f"錯誤：區域 B 輸出的加速度 {val} 超出安全邊界"
      assert val < 0.0, "錯誤：面對明顯負速差，ACM 沒有發出煞車指令"

  # ==========================================================
  # [場景 4: 區域 C (危急防護) & 線性插值平順煞停 (Chauffeur Stop)]
  # 真實路況：車距跌破 75%，極低速準備煞停。驗證 np.interp 動態天花板！
  # ==========================================================
  def test_zone_c_chauffeur_stop_interp(self, acm):
    t_follow = 1.5

    # 模擬原廠 MPC 為了防點頭，發出激進的 +0.5 放卡鉗指令
    a_desired = [0.5] * 33

    # --- 測試 4-1: 完全靜止 (v_ego = 0.0) ---
    # 預期：interp 應該給出最大天花板 +0.25
    sm1 = self._create_sm(True, STOP_DISTANCE + 0.1, -0.1)  # 距離極近，區域 C
    res1 = acm.update(sm1, a_desired, v_ego=0.0, t_follow_override=t_follow)
    assert res1[0] == pytest.approx(0.25, 0.01), "錯誤：靜止時未正確發揮 +0.25 防點頭保護"

    # --- 測試 4-2: 半速煞停中 (v_ego = 0.75 m/s) ---
    # 預期：interp 在 0.0 到 1.5 之間取半，天花板應為 +0.125
    sm2 = self._create_sm(True, STOP_DISTANCE + 0.5, -0.5)
    res2 = acm.update(sm2, a_desired, v_ego=0.75, t_follow_override=t_follow)
    assert res2[0] == pytest.approx(0.125, 0.01), "錯誤：線性插值未精準運作於中間速度"

    # --- 測試 4-3: 中高速 (v_ego = 1.6 m/s) ---
    # 預期：超過 1.5 m/s，天花板應死鎖在 0.0 (絕對防暴衝)
    # 🌟 修正點：使用溫和的 v_rel = -0.2 m/s 避免觸發 TTA 保命防護。
    sm3 = self._create_sm(True, STOP_DISTANCE + 1.0, -0.2)
    res3 = acm.update(sm3, a_desired, v_ego=1.6, t_follow_override=t_follow)
    assert res3[0] == 0.0, "錯誤：中高速時天花板未鎖死 0.0，有暴衝風險"

  # ==========================================================
  # [場景 5: 多重極限保命退場] - TTC 與 TTA 防護
  # 真實路況：前車急煞到靜止 (鬼切或靜止車輛)
  # ==========================================================
  def test_extreme_fallback_protections(self, acm):
    v_ego, t_follow = 30.0, 1.5

    # 模擬 TTC 過短 (高速逼近靜止車)
    d_rel = 15.0  # 距離極近
    v_rel = -15.0  # 相對速度極大
    sm = self._create_sm(True, d_rel, v_rel)

    # 原廠發出重煞 -2.5
    a_desired = [-2.5] * 33
    result = acm.update(sm, a_desired, v_ego, t_follow)

    # 驗證：ACM 必須立刻交還控制權，原封不動輸出 -2.5
    assert result == a_desired, "錯誤：遭遇極端 TTC 時，ACM 未能及時交還保命控制權"
    assert acm.acm_active is False

  # ==========================================================
  # [場景 6: 加速意圖鎖定] - 確保起步與塞車跟車不遲滯
  # 真實路況：塞車時前車起步駛離，系統必須 100% 信任原廠的油門指令
  # ==========================================================
  def test_acceleration_intent_override(self, acm):
    v_ego, t_follow = 5.0, 1.5
    d_rel = self._calc_drel(v_ego, t_follow, 0.70)  # 70% 距離 (落在區域 C 內)
    v_rel = 2.0  # 前車正在遠離
    sm = self._create_sm(status=True, d_rel=d_rel, v_rel=v_rel)

    # 原廠 MPC 決定大腳油門跟上 (+1.5)
    # 確保滿足 INTENT_LOOKAHEAD (前 6 個點有 3 個大於 0.05)
    a_desired = [1.5] * 33
    result = acm.update(sm, a_desired, v_ego, t_follow)

    # 驗證：雖然處於區域 C，且車速低於 1.5，但因為有加速意圖，
    # 系統必須無視所有的 0.0 或 0.25 限制，直接放出 +1.5！
    assert acm.intent_accelerating is True, "錯誤：未能正確捕捉起步加速意圖"
    assert result == a_desired, "錯誤：起步意圖被錯誤壓制，會導致嚴重起步遲滯"

  # ==========================================================
  # [場景 7: 狀態機遲滯 (Hysteresis) 連續幀測試]
  # 真實路況：在 95% 邊界反覆橫跳，ACM 應保持穩定不閃爍
  # ==========================================================
  def test_state_machine_hysteresis(self, acm):
    v_ego, t_follow = 20.0, 1.5
    a_desired = [-0.1] * 33

    # 幀 1：距離 98% (大於啟動線 95%) -> ACM 應該保持未啟動
    sm1 = self._create_sm(True, self._calc_drel(v_ego, t_follow, 0.98), -0.1)
    res1 = acm.update(sm1, a_desired, v_ego, t_follow)
    assert acm.acm_active is False
    assert res1 == a_desired  # 未啟動，原樣輸出

    # 幀 2：距離 92% (跨越 95% 啟動線) -> ACM 啟動，進入區域 A 滑行
    sm2 = self._create_sm(True, self._calc_drel(v_ego, t_follow, 0.92), -0.1)
    res2 = acm.update(sm2, a_desired, v_ego, t_follow)
    assert acm.acm_active is True
    assert res2[0] == 0.0  # 執行區域 A 0.0 滑行

    # 幀 3：距離回到 98% (進入遲滯區 95%~100%) -> ACM 必須「維持」啟動狀態，防止頓挫！
    sm3 = self._create_sm(True, self._calc_drel(v_ego, t_follow, 0.98), 0.1)
    res3 = acm.update(sm3, a_desired, v_ego, t_follow)
    assert acm.acm_active is True  # 狀態完美鎖定
    assert res3[0] == 0.0
