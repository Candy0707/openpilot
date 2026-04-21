"""
Test Suite for Adaptive Coasting Module (ACM) - Flagship Edition
(支援 EMA 雙重濾波、前車鎖定防閃爍、40% 極限信任區)
"""

import pytest
import math
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
        """反推真實物理距離"""
        dynamic_target = max(v_ego * t_follow, 1.0)
        dynamic_actual = dynamic_target * target_percent
        return dynamic_actual + STOP_DISTANCE

    # ==========================================================
    # [場景 1: 無車狀態] - E2E 視覺模型神經質微煞車抹平
    # ==========================================================
    def test_no_lead_smooth_nervous_braking(self, acm):
        v_ego, t_follow = 25.0, 1.5
        sm = self._create_sm(status=False, d_rel=0.0, v_rel=0.0)

        a_desired = [-0.2, -0.3, 0.1, -0.8] + [0.0] * 29
        result = acm.update(sm, a_desired, v_ego, t_follow)

        assert result[0] == 0.0, "錯誤：微煞車未被抹平"
        assert result[1] == 0.0, "錯誤：微煞車未被抹平"
        assert result[2] == 0.1, "錯誤：無車時的微加速不應被攔截"
        assert result[3] == -0.8, "錯誤：真實重煞車被錯誤抹平，非常危險！"

    # ==========================================================
    # [場景 2: 區域 C 與區域 D] - 驗證 40% 極限信任防護
    # ==========================================================
    def test_zone_c_and_d_mpc_trust(self, acm):
        v_ego, t_follow = 5.0, 1.5

        # 原廠想要給出 +0.5 的加速 (防點頭或跟車起步)
        a_desired = [0.5] * 33

        # --- 測試 2-1: 區域 C (40% ~ 75%) ---
        # 預期：不允許加速，限制在 0.0
        d_rel_c = self._calc_drel(v_ego, t_follow, 0.60) # 距離 60%
        sm_c = self._create_sm(True, d_rel_c, -0.1)

        # 餵入 5 幀以消除 EMA 濾波器的初始延遲
        for _ in range(5):
            acm.update(sm_c, a_desired, v_ego, t_follow)
        res_c = acm.update(sm_c, a_desired, v_ego, t_follow)

        assert res_c[0] == 0.0, f"錯誤：區域 C 未發揮 min(a, 0.0) 限制，得到了 {res_c[0]}"

        # --- 測試 2-2: 區域 D (< 40%) ---
        # 預期：完全信任原廠 MPC，直接放行 +0.5
        d_rel_d = self._calc_drel(v_ego, t_follow, 0.30) # 距離 30%
        sm_d = self._create_sm(True, d_rel_d, -0.1)

        for _ in range(5):
            acm.update(sm_d, a_desired, v_ego, t_follow)
        res_d = acm.update(sm_d, a_desired, v_ego, t_follow)

        assert res_d[0] == 0.5, "錯誤：區域 D 未能完全放行原廠 MPC 指令"

    # ==========================================================
    # [場景 3: 前車鎖定防閃爍 (Lead Lost Lock)] - 驗證 5 幀保護
    # ==========================================================
    def test_lead_lost_lock_logic(self, acm):
        v_ego, t_follow = 20.0, 1.5
        d_rel = self._calc_drel(v_ego, t_follow, 0.80)

        # 1. 建立有車鎖定
        sm_lead = self._create_sm(True, d_rel, -0.1)
        acm.update(sm_lead, [0.0]*33, v_ego, t_follow)
        assert acm.has_lead_locked is True, "錯誤：無法建立前車鎖定"

        # 2. 模擬雷達瞬間閃爍丟失前車
        sm_lost = self._create_sm(False, 0.0, 0.0)

        # 前 4 幀，系統應該因為鎖定保護，繼續認為「有車」
        for _ in range(4):
            acm.update(sm_lost, [0.0]*33, v_ego, t_follow)
            assert acm.has_lead_locked is True, "錯誤：防閃爍提早失效"
            assert acm.lead_lost_counter > 0

        # 第 5 幀，超時，系統應判定真正無車並解開鎖定
        acm.update(sm_lost, [0.0]*33, v_ego, t_follow)
        assert acm.has_lead_locked is False, "錯誤：經過 5 幀仍未解除鎖定"

    # ==========================================================
    # [場景 4: 狀態機遲滯 (Hysteresis) 與 EMA 濾波器驗證]
    # ==========================================================
    def test_state_machine_hysteresis_with_ema(self, acm):
        v_ego, t_follow = 20.0, 1.5
        a_desired = [-0.1] * 33

        # 幀 1：距離 98% -> ACM 應該保持未啟動
        sm1 = self._create_sm(True, self._calc_drel(v_ego, t_follow, 0.98), -0.1)
        acm.update(sm1, a_desired, v_ego, t_follow)
        assert acm.acm_active is False

        # 幀 2~6：距離降到 92%
        # 因為有 EMA 濾波器 (Alpha=0.2)，我們需要給予連續幾幀讓系統物理認知跟上
        sm2 = self._create_sm(True, self._calc_drel(v_ego, t_follow, 0.92), -0.1)
        for _ in range(5):
            acm.update(sm2, a_desired, v_ego, t_follow)

        assert acm.acm_active is True, "錯誤：EMA 濾波收斂後仍未正確啟動 ACM"

        # 幀 7~11：距離回到 98% (進入遲滯區 95%~100%)
        # ACM 必須「維持」啟動狀態
        sm3 = self._create_sm(True, self._calc_drel(v_ego, t_follow, 0.98), 0.1)
        for _ in range(5):
            acm.update(sm3, a_desired, v_ego, t_follow)

        assert acm.acm_active is True, "錯誤：遲滯區未能成功鎖定狀態"

    # ==========================================================
    # [場景 5: 多重極限保命退場] - 驗證 Raw Data 零延遲
    # ==========================================================
    def test_extreme_fallback_protections(self, acm):
        v_ego, t_follow = 30.0, 1.5

        # 高速逼近靜止車 (極端 TTC)
        d_rel = 15.0
        v_rel = -15.0
        sm = self._create_sm(True, d_rel, v_rel)

        a_desired = [-2.5] * 33
        result = acm.update(sm, a_desired, v_ego, t_follow)

        # 驗證：即使只有 1 幀 (濾波器還沒降下來)，底層 TTC 依然透過 Raw Data 精準觸發退出！
        assert result == a_desired, "錯誤：遭遇極端 TTC 時，未能以 Raw Data 零延遲交還保命控制權"
        assert acm.acm_active is False