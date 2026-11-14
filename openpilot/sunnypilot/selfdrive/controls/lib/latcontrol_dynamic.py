from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque as LatControlTorqueV1
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


class LatControlDynamic(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.params = Params()

    # 1. 初始化角度控制器
    self.angle_ctrl = LatControlAngle(CP, CP_SP, CI, dt)

    # 2. 讀取 sunnypilot UI 的 Torque 版本設定 (與 controlsd_ext.py 邏輯同步)
    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_version = self.params.get("TorqueControlTune")

    # 為了相容 bytes / str / float 型態，統一轉為 float 判斷
    try:
      is_v1 = (float(torque_version) == 1.0) if torque_version is not None else False
    except (ValueError, TypeError):
      is_v1 = False

    # 只有在「開啟 EnforceTorqueControl」且「UI 明確選擇 v1 (1.0)」時才用 V1
    # 其他所有情況（包含未設定、預設狀態、或選 v0）一律預設使用最穩定的 V0！
    if enforce_torque_control and is_v1:
      self.torque_ctrl = LatControlTorqueV1(CP, CP_SP, CI, dt)
    else:
      self.torque_ctrl = LatControlTorqueV0(CP, CP_SP, CI, dt)

    # 預設使用 CP 讀出來的設定值
    self.use_angle = (CP.steerControlType == structs.CarParams.SteerControlType.angle)

  # ==========================================
  # 🌟 新增：Sunnypilot Torque 專屬代理轉發 (Proxy)
  # ==========================================
  @property
  def extension(self):
    # 當外層要求 extension 時，直接把 torque_ctrl 的 extension 交出去
    return self.torque_ctrl.extension

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, frictionCoefficient):
    # 將即時參數更新指令轉發給內部的 torque_ctrl
    self.torque_ctrl.update_torque_parameters(latAccelFactor, latAccelOffset, frictionCoefficient)
  # ==========================================

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # 定義「安全直行狀態」：方向盤打角小於 10 度，且轉動速率小於 5 度/秒
    is_safe_to_switch = abs(CS.steeringAngleDeg) < 10.0 and abs(CS.steeringRateDeg) < 5.0

    # 1. 判斷主控權與遲滯區間，並且鎖死過彎時的切換
    if CS.vEgo > 22.0 and not self.use_angle and is_safe_to_switch:
      self.use_angle = True
      self.angle_ctrl.reset()  # 確保角度控制器狀態乾淨

    elif CS.vEgo < 16.0 and self.use_angle and is_safe_to_switch:
      self.use_angle = False
      self.torque_ctrl.reset() # 確保扭矩控制器狀態乾淨
      if hasattr(self.torque_ctrl, 'pid'):
        self.torque_ctrl.pid.reset() # 徹底清除積分

    # 2. Angle 控制器永遠運算 (幾何計算，無風險)
    _, a_steer, a_log = self.angle_ctrl.update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay)

    # 3. Torque 控制器永遠運算 (熱備援)
    torque_is_frozen = True if self.use_angle else steer_limited_by_safety
    t_steer, _, t_log = self.torque_ctrl.update(active, CS, VM, params, torque_is_frozen, desired_curvature, calibrated_pose, curvature_limited, lat_delay)

    # 4. 雙輸出合併：回傳 (扭矩輸出, 角度輸出, 當前主控的Log)
    if self.use_angle:
      return t_steer, a_steer, a_log
    else:
      return t_steer, a_steer, t_log

  def reset(self):
    super().reset()
    self.angle_ctrl.reset()
    self.torque_ctrl.reset()
    if hasattr(self.torque_ctrl, 'pid'):
      self.torque_ctrl.pid.reset()
