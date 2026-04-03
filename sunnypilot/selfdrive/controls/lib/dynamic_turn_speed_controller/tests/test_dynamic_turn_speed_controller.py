"""
Test Suite for Dynamic Turn Speed Controller (DTSC) - v8.7 God-Tier Edition
(內含 12 大極端環境測試與精準中文錯誤標籤)
"""

import pytest
import numpy as np
from unittest.mock import MagicMock

from dragonpilot.selfdrive.controls.lib.dynamic_turn_speed_controller.dynamic_turn_speed_controller import DynamicTurnSpeedController
from opendbc.car import structs

class MockSubMaster(dict):
    def __init__(self):
        super().__init__()
        self.valid = {}

class TestDTSC_Universal:

    @pytest.fixture
    def dtsc(self):
        CP = structs.CarParams()
        CP.openpilotLongitudinalControl = True
        controller = DynamicTurnSpeedController(CP, mpc=None)
        controller.enable = True
        controller.available = True
        return controller

    def _create_sm(self, v_ego, yaw_rates, pred_y_offset=None, steer_saturated=False, steering_pressed=False, steer_rate_deg=0.0):
        sm = MockSubMaster()
        class ModelV2:
            class Position:
                x = np.linspace(0, 100, 33).tolist()
                y = pred_y_offset if pred_y_offset else [0.0] * 33
            class OrientationRate:
                z = yaw_rates
            position = Position()
            orientationRate = OrientationRate()
        sm['modelV2'] = ModelV2()
        sm.valid['modelV2'] = True

        class CarState:
            yawRate = yaw_rates[0] if yaw_rates else 0.0
            steeringPressed = steering_pressed
            steeringRateDeg = steer_rate_deg
        sm['carState'] = CarState()
        sm.valid['carState'] = True

        class LateralControlState:
            def which(self): return 'pid'
            class PID:
                saturated = steer_saturated
            pid = PID()
        class ControlsState:
            lateralControlState = LateralControlState()
        sm['controlsState'] = ControlsState()
        sm.valid['controlsState'] = True
        return sm


    def test_low_speed_creeping(self, dtsc):
        v_ego = 2.5
        v_cruise = 10.0
        yaw_rates = [0.5] * 33
        sm = self._create_sm(v_ego, yaw_rates)

        out_v = v_ego
        for _ in range(20):
            out_v, _ = dtsc.update_target(sm, v_ego, 0.0, v_cruise)

        assert bool(dtsc.action) is False, "[測試 9: 低速蠕行] 錯誤：車速低於 3.0m/s 時系統未能強制休眠"
        assert float(out_v) >= v_cruise, "[測試 9: 低速蠕行] 錯誤：休眠時目標車速未能維持在原廠巡航上限"

    def test_single_frame_noise(self, dtsc):
        v_ego = 30.0
        v_cruise = v_ego
        yaw_noise = [0.0]*10 + [0.9]*5 + [0.0]*18
        sm_noise = self._create_sm(v_ego, yaw_noise)
        sm_straight = self._create_sm(v_ego, [0.0]*33)

        dtsc.update_target(sm_noise, v_ego, 0.0, v_cruise)
        for _ in range(19):
            dtsc.update_target(sm_straight, v_ego, 0.0, v_cruise)

        assert bool(dtsc.action) is False, "[測試 10: 單幀突發雜訊] 錯誤：時間畫布未能完美吸收單幀雜訊，引發了幽靈煞車"

    def test_curvature_energy_clear(self, dtsc):
        v_ego = 20.0
        v_cruise = v_ego
        sm_curve = self._create_sm(v_ego, [0.3]*33)
        for _ in range(20):
            dtsc.update_target(sm_curve, v_ego, 0.0, v_cruise)

        assert bool(dtsc.action) is True, "[測試 11: 能量濾波清除] 錯誤：前半段的正常彎道未能成功觸發降速"

        sm_straight = self._create_sm(v_ego, [0.0]*33)
        for _ in range(10):
            dtsc.update_target(sm_straight, v_ego, 0.0, v_cruise)

        assert bool(dtsc.action) is False, "[測試 11: 能量濾波清除] 錯誤：駛出彎道後，能量濾波器未能快速清除殘留的煞車狀態"

    def test_turn_entry_prediction_catalyst(self, dtsc):
        v_ego = 25.0
        v_cruise = v_ego
        yaw_rates = np.linspace(0.0, 0.6, 33).tolist()
        sm = self._create_sm(v_ego, yaw_rates)

        current_v = v_ego
        for _ in range(12):
            out_v, _ = dtsc.update_target(sm, current_v, 0.0, v_cruise)
            current_v = min(float(out_v), v_ego) if dtsc.action else v_ego

        assert bool(dtsc.action) is True, "[測試 12: 入彎預判催化] 錯誤：曲率陡升時，雙倍積分催化失效，未能提早觸發減速"

    def test_90_degree_sharp_turn(self, dtsc):
        v_ego = 50.0 / 3.6
        v_cruise = v_ego
        yaw_rates = [0.0] * 33
        for i in range(8, 18): yaw_rates[i] = 0.8

        sm = self._create_sm(v_ego, yaw_rates)
        current_v = v_ego
        for _ in range(20):
            out_v, _ = dtsc.update_target(sm, current_v, 0.0, v_cruise)
            current_v = min(float(out_v), v_ego) if dtsc.action else v_ego

        assert bool(dtsc.action) is True, "[測試 1: 直角死角彎] 錯誤：連續確認 20 幀後系統未能觸發作動"
        assert float(current_v) < v_cruise * 0.85, "[測試 1: 直角死角彎] 錯誤：Preview Braking 未能迫使車輛進行明顯減速"

    def test_gentle_highway_curve(self, dtsc):
        v_ego = 110.0 / 3.6
        v_cruise = v_ego
        yaw_rates = [0.0] * 33
        for i in range(12, 33): yaw_rates[i] = 0.076

        sm = self._create_sm(v_ego, yaw_rates)
        current_v = v_ego
        for _ in range(20):
            out_v, _ = dtsc.update_target(sm, current_v, 0.0, v_cruise)
            current_v = min(float(out_v), v_ego) if dtsc.action else v_ego

        assert bool(dtsc.action) is True, "[測試 2: 高速稍大彎] 錯誤：曲率已達作動門檻，但系統未能介入"
        assert float(current_v) < v_cruise, "[測試 2: 高速稍大彎] 錯誤：目標車速未能低於當前巡航車速"

    def test_interchange_s_curve(self, dtsc):
        v_ego = 80.0 / 3.6
        v_cruise = v_ego
        yaw_rates = [0.0] * 33
        for i in range(0, 11): yaw_rates[i] = 0.15
        for i in range(18, 30): yaw_rates[i] = -0.45

        sm = self._create_sm(v_ego, yaw_rates)
        current_v = v_ego
        for _ in range(20):
            out_v, _ = dtsc.update_target(sm, current_v, 0.0, v_cruise)
            current_v = min(float(out_v), v_ego) if dtsc.action else v_ego

        assert bool(dtsc.action) is True, "[測試 3: 交流道 S 彎] 錯誤：系統未能成功偵測到前方的連續反向彎道"
        assert float(current_v) < v_cruise * 0.85, "[測試 3: 交流道 S 彎] 錯誤：未能成功觸發 8 折重心轉移懲罰"

    def test_trajectory_deviation_penalty(self, dtsc):
        v_ego = 60.0 / 3.6
        v_cruise = v_ego
        yaw_rates = [0.4] * 33
        pred_y = np.linspace(0.0, 2.0, 33).tolist()

        sm = self._create_sm(v_ego, yaw_rates, pred_y_offset=pred_y)
        current_v = v_ego
        for _ in range(20):
            out_v, _ = dtsc.update_target(sm, current_v, 0.0, v_cruise)
            current_v = min(float(out_v), v_ego) if dtsc.action else v_ego

        assert bool(dtsc.action) is True, "[測試 4: 推頭軌跡偏移] 錯誤：發生嚴重推頭時系統未能介入"
        assert float(current_v) < v_cruise * 0.5, "[測試 4: 推頭軌跡偏移] 錯誤：真實推頭發生時，未能觸發大幅度的極限重煞"

    def test_city_lane_change(self, dtsc):
        v_ego = 50.0 / 3.6
        v_cruise = v_ego
        yaw_rates = [0.0] * 33
        for i in range(3, 9): yaw_rates[i] = 0.06
        for i in range(9, 15): yaw_rates[i] = -0.06

        pred_y = [0.0] * 3
        pred_y.extend(np.linspace(0.0, 3.5, 30).tolist())
        sm = self._create_sm(v_ego, yaw_rates, pred_y_offset=pred_y)
        current_v = v_ego
        for _ in range(20):
            out_v, _ = dtsc.update_target(sm, current_v, 0.0, v_cruise)
            current_v = min(float(out_v), v_ego) if dtsc.action else v_ego

        assert bool(dtsc.action) is False, "[測試 5: 市區平順變道] 錯誤：平順的變換車道被誤判為急彎，觸發了幽靈煞車"

    def test_eps_panic_rescue(self, dtsc):
        v_ego = 50.0 / 3.6
        v_cruise = v_ego
        yaw_rates = [0.6] * 33
        sm = self._create_sm(v_ego, yaw_rates, steer_saturated=True, steering_pressed=True, steer_rate_deg=90.0)
        out_v, _ = dtsc.update_target(sm, v_ego, 0.0, v_cruise)

        assert bool(dtsc.action) is True, "[測試 6: EPS 恐慌救車] 錯誤：駕駛急拉方向盤且 EPS 滿載時，保母機制未能瞬間觸發"
        assert float(out_v) < v_cruise * 0.9, "[測試 6: EPS 恐慌救車] 錯誤：保母機制未能對目標車速進行明顯打折"

    def test_long_straight_highway(self, dtsc):
        v_ego = 100.0 / 3.6
        v_cruise = v_ego
        yaw_rates = [0.0] * 33
        sm = self._create_sm(v_ego, yaw_rates)
        for _ in range(20):
            out_v, _ = dtsc.update_target(sm, v_ego, 0.0, v_cruise)

        assert bool(dtsc.action) is False, "[測試 7: 大直線巡航] 錯誤：系統在大直線上發生了不該有的降速介入"
        assert float(out_v) >= v_cruise, "[測試 7: 大直線巡航] 錯誤：大直線目標車速低於巡航車速"

    def test_slight_highway_curve(self, dtsc):
        v_ego = 110.0 / 3.6
        v_cruise = v_ego
        yaw_rates = [0.045] * 33
        sm = self._create_sm(v_ego, yaw_rates)
        for _ in range(20):
            out_v, _ = dtsc.update_target(sm, v_ego, 0.0, v_cruise)

        assert bool(dtsc.action) is False, "[測試 8: 高速微彎防護] 錯誤：曲率未達危險門檻的高速微彎，系統不應過度敏感而觸發"
        assert float(out_v) >= v_cruise, "[測試 8: 高速微彎防護] 錯誤：高速微彎不應干擾原本的巡航車速"