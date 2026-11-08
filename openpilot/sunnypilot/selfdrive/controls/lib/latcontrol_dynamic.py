"""
TSS2 動態橫向控制器（LTA 角度 / LKA 扭矩 熱備援切換）

路徑：openpilot/sunnypilot/selfdrive/controls/lib/latcontrol_dynamic.py
由 controlsd_ext.initialize_lateral_control() 對 TSS2_CAR + torque tune 建立。

v2（2026-11-09，依 openpilot-test rlog 49/64/65/66 修正）
  1. 扭矩控制器改依 sunnypilot 原本的選擇邏輯建立（預設 TorqueV0），不再固定用 upstream v1。
     原因：v1 的摩擦項輸入為 error + 0.3 × desired_lateral_jerk，jerk 來自模型 20 Hz 階梯曲率的微分，
     使扭矩指令逐幀跳動（66 段平均 24 units/幀，p90 64），超過 Toyota STEER_DELTA_UP=15，
     38–62% 的幀卡在速率限制 → 追蹤增益 0.4–0.9 → 市區 5–9 秒週期蛇行。V0 的摩擦項只看 error。
     選擇規則與 controlsd_ext 相同：
       EnforceTorqueControl = False               → V0（sunnypilot 預設，FIXME-SP 註解）
       EnforceTorqueControl = True, Tune == 0.0   → V0
       EnforceTorqueControl = True, Tune != 0.0   → v1（upstream LatControlTorque）
  2. torque → angle 切換新增「角度目標與實際角度差」門檻（SWITCH_ANGLE_ERR_MAX_DEG）。
     原因：66 段 57.68 s 切入 LTA 時目標 −3.50°、實際 −1.52°，駕駛扭矩衝到 166 觸發 wind-down，
     車子在 2 秒內偏到車道中心左側 1.25 m。
  3. angle → torque 切換後 STEER_LIMITED_MASK_FRAMES 幀內忽略 steer_limited_by_safety。
     原因：controlsd.publish() 以 CP.steerControlType（torque）計算 steer_limited_by_safety，
     LTA 期間 carOutput.torque 恆為 0，該值幾乎恆為 True；切回扭矩後頭幾幀會誤凍結積分。

與 cp（commaai/openpilot）的刻意分歧：
  - cp 只對帶 ToyotaFlags.ANGLE_CONTROL 的車用 LTA；本控制器讓 TSS2 在 > 22 m/s 用 LTA、< 16 m/s 用扭矩。
  - 對外介面（extension / update_torque_parameters / use_angle / reset / update 回傳格式）與 v1 版相同，
    controlsd.py、controlsd_ext.py 不需修改。
"""
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0

# 切換車速遲滯（m/s）：與 v1 版相同
ANGLE_ENGAGE_SPEED = 22.0      # 79 km/h 以上切 LTA
ANGLE_RELEASE_SPEED = 16.0     # 58 km/h 以下切回扭矩

# 「直行安全狀態」：與 v1 版相同
SWITCH_MAX_STEER_ANGLE_DEG = 10.0
SWITCH_MAX_STEER_RATE_DEG = 5.0

# v2 新增：torque → angle 時，LTA 角度目標與實際方向盤角度的最大允許差（deg）。
# 角度目標含 vehicleParameters.angleOffsetDeg，與 carState.steeringAngleDeg 同一座標（同 latcontrol_angle 飽和判斷）。
SWITCH_ANGLE_ERR_MAX_DEG = 1.0

# v2 新增：angle → torque 後忽略 steer_limited_by_safety 的幀數（100 Hz，0.1 秒）
STEER_LIMITED_MASK_FRAMES = 10


def _build_torque_controller(CP, CP_SP, CI, dt):
  params = Params()
  enforce_torque_control = params.get_bool("EnforceTorqueControl")
  torque_version = params.get("TorqueControlTune")

  use_v1 = enforce_torque_control and torque_version is not None and float(torque_version) != 0.0
  if use_v1:
    cloudlog.info("LatControlDynamic: torque controller = v1 (EnforceTorqueControl, TorqueControlTune != 0)")
    return LatControlTorque(CP, CP_SP, CI, dt)

  cloudlog.info("LatControlDynamic: torque controller = V0 (sunnypilot default)")
  return LatControlTorqueV0(CP, CP_SP, CI, dt)


class LatControlDynamic(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    # 同時初始化兩個控制器
    self.angle_ctrl = LatControlAngle(CP, CP_SP, CI, dt)
    self.torque_ctrl = _build_torque_controller(CP, CP_SP, CI, dt)

    # 預設使用 CP 讀出來的設定值
    self.use_angle = (CP.steerControlType == structs.CarParams.SteerControlType.angle)
    self._steer_limited_mask = 0

  # ==========================================
  # Sunnypilot Torque 專屬代理轉發 (Proxy)
  # ==========================================
  @property
  def extension(self):
    # 當外層要求 extension 時，直接把 torque_ctrl 的 extension 交出去（V0 / v1 都有）
    return self.torque_ctrl.extension

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, frictionCoefficient):
    # 將即時參數更新指令轉發給內部的 torque_ctrl
    self.torque_ctrl.update_torque_parameters(latAccelFactor, latAccelOffset, frictionCoefficient)
  # ==========================================

  def _reset_torque(self):
    self.torque_ctrl.reset()
    if hasattr(self.torque_ctrl, 'pid'):
      self.torque_ctrl.pid.reset()  # 徹底清除積分

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # 1. Angle 控制器永遠先運算：切換判斷需要本幀的角度目標（幾何計算，無副作用風險）
    _, a_steer, a_log = self.angle_ctrl.update(active, CS, VM, params, steer_limited_by_safety, desired_curvature,
                                               calibrated_pose, curvature_limited, lat_delay)

    # 2. 判斷主控權與遲滯區間，並且鎖死過彎時的切換
    is_safe_to_switch = abs(CS.steeringAngleDeg) < SWITCH_MAX_STEER_ANGLE_DEG and abs(CS.steeringRateDeg) < SWITCH_MAX_STEER_RATE_DEG
    angle_target_close = abs(a_steer - CS.steeringAngleDeg) < SWITCH_ANGLE_ERR_MAX_DEG

    if CS.vEgo > ANGLE_ENGAGE_SPEED and not self.use_angle and is_safe_to_switch and angle_target_close:
      self.use_angle = True
      self.angle_ctrl.reset()  # 確保角度控制器狀態乾淨

    elif CS.vEgo < ANGLE_RELEASE_SPEED and self.use_angle and is_safe_to_switch:
      self.use_angle = False
      self._reset_torque()
      self._steer_limited_mask = STEER_LIMITED_MASK_FRAMES

    # 3. Torque 控制器永遠運算（熱備援）
    if self.use_angle:
      torque_is_frozen = True
    else:
      if self._steer_limited_mask > 0:
        self._steer_limited_mask -= 1
        torque_is_frozen = False   # 上一段 LTA 的 steer_limited_by_safety 無效（見檔頭 v2-3）
      else:
        torque_is_frozen = steer_limited_by_safety
    t_steer, _, t_log = self.torque_ctrl.update(active, CS, VM, params, torque_is_frozen, desired_curvature,
                                                calibrated_pose, curvature_limited, lat_delay)

    # 4. 雙輸出合併：回傳 (扭矩輸出, 角度輸出, 當前主控的 Log)
    if self.use_angle:
      return t_steer, a_steer, a_log
    return t_steer, a_steer, t_log

  def reset(self):
    super().reset()
    self.angle_ctrl.reset()
    self._reset_torque()
    self._steer_limited_mask = 0
