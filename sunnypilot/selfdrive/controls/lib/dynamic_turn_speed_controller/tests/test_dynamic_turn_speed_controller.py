"""
Test Suite for Dynamic Turn Speed Controller (DTSC) - Final Pro Edition
(通用型模擬：精準驗證 4 階段狀態機、G 力強制介入與 20 幀出彎防護)
"""

import pytest
import numpy as np

from sunnypilot.selfdrive.controls.lib.dynamic_turn_speed_controller.dynamic_turn_speed_controller import DynamicTurnSpeedController
from opendbc.car import structs


# ==========================================================
# [通用型模擬環境建構]
# ==========================================================
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
    # 初始化信心度，避免未定義錯誤
    controller.confidence_table = np.zeros(200)
    return controller

  def _create_sm(self, v_ego, ai_yaw_rates, real_curvature=0.0, ai_pitch=None):
    """
    通用型狀態機模擬器 (Simulator)
    可獨立設定「AI 預測視野 (ai_yaw_rates)」與「實車底盤狀態 (real_curvature)」
    """
    sm = MockSubMaster()

    # 模擬 AI 模型預測 (ModelV2)
    class ModelV2:
      class Position:
        x = np.linspace(0, 100, 33).tolist()

      class Velocity:
        x = [v_ego] * 33

      class OrientationRate:
        z = ai_yaw_rates

      class Orientation:
        y = ai_pitch if ai_pitch else [0.0] * 33

      position = Position()
      velocity = Velocity()
      orientationRate = OrientationRate()
      orientation = Orientation()

    sm['modelV2'] = ModelV2()
    sm.valid['modelV2'] = True

    # 模擬實車底盤回饋 (ControlsState)
    class ControlsState:
      curvature = real_curvature

    sm['controlsState'] = ControlsState()
    sm.valid['controlsState'] = True

    return sm

  # ==========================================================
  # [測試 1] 第一階段：AI 視覺前饋啟動 (遠距預判)
  # ==========================================================
  def test_stage1_ai_activation(self, dtsc):
    v_ego = 30.0  # 108 km/h
    v_cruise = v_ego
    # AI 看到遠方有連續彎道 (曲率高)
    ai_yaw_rates = [0.0] * 10 + [0.3] * 23
    sm = self._create_sm(v_ego, ai_yaw_rates, real_curvature=0.0)

    # 模擬 15 幀 (0.75秒) 讓信心度累積過 60%
    for _ in range(15):
      dtsc.update_target(sm, v_ego, 0.0, v_cruise)

    assert bool(dtsc.action) is True, "[測試 1] 錯誤：AI 連續看見彎道，但系統未能成功啟用介入"

  # ==========================================================
  # [測試 2] 第一階段：實車 G 力強制介入 (AI 盲點防護)
  # ==========================================================
  def test_stage1_g_force_override(self, dtsc):
    v_ego = 20.0  # 72 km/h
    v_cruise = v_ego
    # AI 瞎掉，沒看到彎道
    ai_yaw_rates = [0.0] * 33

    # 但實車方向盤已經打下去，算出 G 力 = 20^2 * 0.005 = 2.0G (遠超舒適極限 1.6G)
    real_curvature = 0.005
    sm = self._create_sm(v_ego, ai_yaw_rates, real_curvature=real_curvature)

    # G 力介入是瞬間的，不需累積信心，1 幀就該作動
    dtsc.update_target(sm, v_ego, 0.0, v_cruise)

    assert bool(dtsc.action) is True, "[測試 2] 錯誤：實體 G 力已爆表，保命機制未能瞬間強制介入！"

  # ==========================================================
  # [測試 3] 第三階段：預先減速實作 (遠距 TTA)
  # ==========================================================
  def test_stage3_pre_deceleration(self, dtsc):
    v_ego = 25.0
    v_cruise = v_ego
    # 彎道在較遠處 (index 15 以後)
    ai_yaw_rates = [0.0] * 15 + [0.4] * 18
    sm = self._create_sm(v_ego, ai_yaw_rates, real_curvature=0.0)

    # 累積信心並啟動
    for _ in range(15):
      dtsc.update_target(sm, v_ego, 0.0, v_cruise)

    assert bool(dtsc.action) is True
    # 預先減速階段，不應該出現正加速度 (補油)
    assert dtsc.a_target <= 0.0, "[測試 3] 錯誤：在遠距 TTA 預先減速階段，系統產生了違規的加速(補油)指令"

  # ==========================================================
  # [測試 4] 第三階段：彎中比例控制實作 (G-Control)
  # ==========================================================
  def test_stage3_proportional_control(self, dtsc):
    v_ego = 15.0  # 54 km/h
    v_cruise = v_ego
    # 強制進入彎中動態 (G力介入)
    real_curvature = 0.008  # G = 15^2 * 0.008 = 1.8G (大於舒適極限約 1.5G)
    sm = self._create_sm(v_ego, ai_yaw_rates=[0.0] * 33, real_curvature=real_curvature)

    # 跑 10 幀讓平滑器充滿
    for _ in range(10):
      dtsc.update_target(sm, v_ego, 0.0, v_cruise)

    # 應該產生明顯的煞車力道 (比例控制發威)
    assert dtsc.a_target < -0.5, "[測試 4] 錯誤：在彎中動態階段，比例控制未能給予足夠的煞車力道"

  # ==========================================================
  # [測試 5] 第四階段：20 幀出彎防護計時器 (S彎防震盪)
  # ==========================================================
  def test_stage4_exit_frame_protection(self, dtsc):
    v_ego = 20.0
    v_cruise = v_ego

    # 步驟 A：進入彎道並將信心度「灌滿」
    sm_curve = self._create_sm(v_ego, [0.3]*33, real_curvature=0.004)
    for _ in range(40): # 確保信心度達到 1.0 (需要 20 幀以上)
      dtsc.update_target(sm_curve, v_ego, 0.0, v_cruise)
    assert bool(dtsc.action) is True

    # 步驟 B：模擬 S 彎中繼點或駛出彎道 (實體方向盤打平，且 AI 前方無彎道)
    sm_straight = self._create_sm(v_ego, [0.0]*33, real_curvature=0.0)

    # 1. 信心度從 1.0 放電掉到 0.61 需要 39 幀。
    # 這 39 幀內，is_curve_ahead 仍為 True，防護計時器尚未啟動。
    for _ in range(39):
      dtsc.update_target(sm_straight, v_ego, 0.0, v_cruise)

    assert bool(dtsc.action) is True, "錯誤：信心度尚未低於門檻，不應提早解除"

    # 2. 第 40 幀開始，信心度降至 0.60，不滿足 >0.60 門檻。
    # exit_condition_raw 成立，20 幀防護計時器開始運作！(此時 count = 1)
    dtsc.update_target(sm_straight, v_ego, 0.0, v_cruise)

    # 3. 我們再精準地跑 18 幀 (加上前一幀，此時 count = 19)。
    # 因為未滿 20 幀防護，系統必須還是介入狀態！
    for _ in range(18):
      dtsc.update_target(sm_straight, v_ego, 0.0, v_cruise)

    assert bool(dtsc.action) is True, "[測試 5] 錯誤：出彎防護未滿 20 幀，系統卻提早解除了介入！(S彎致命傷)"

    # 4. 再跑 1 幀 (此時 count 剛好滿 20 幀)
    dtsc.update_target(sm_straight, v_ego, 0.0, v_cruise)
    assert bool(dtsc.action) is False, "[測試 5] 錯誤：出彎已滿 20 幀且條件安全，系統未能正常退出解除"

  # ==========================================================
  # [測試 6] 邊界條件：單幀雜訊過濾
  # ==========================================================
  def test_single_frame_noise_rejection(self, dtsc):
    v_ego = 30.0
    v_cruise = v_ego
    sm_straight = self._create_sm(v_ego, [0.0] * 33, real_curvature=0.0)
    sm_noise = self._create_sm(v_ego, [0.9] * 33, real_curvature=0.0)

    # 平穩行駛
    for _ in range(10):
      dtsc.update_target(sm_straight, v_ego, 0.0, v_cruise)

    # 突發 2 幀極度危險的 AI 雜訊
    dtsc.update_target(sm_noise, v_ego, 0.0, v_cruise)
    dtsc.update_target(sm_noise, v_ego, 0.0, v_cruise)

    # 信心度只會 +0.10，未達 0.60 門檻
    assert bool(dtsc.action) is False, "[測試 6] 錯誤：時間畫布濾波失效，系統被單幀雜訊欺騙引發幽靈煞車"

  # ==========================================================
  # [測試 7] 邊界條件：低速靜止防頓挫
  # ==========================================================
  def test_low_speed_ignore(self, dtsc):
    v_ego = 0.05  # 車輛幾乎靜止
    v_cruise = 10.0
    # 在靜止時狂打方向盤 (產生極大曲率)
    sm_turn = self._create_sm(v_ego, [0.5] * 33, real_curvature=0.05)

    for _ in range(20):
      dtsc.update_target(sm_turn, v_ego, 0.0, v_cruise)

    assert bool(dtsc.action) is False, "[測試 7] 錯誤：車輛於靜止/極低速蠕行時，打方向盤依然引發系統介入"
