from cereal import car
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque

class LatControlDynamic(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    # 同時初始化兩個控制器
    self.angle_ctrl = LatControlAngle(CP, CP_SP, CI, dt)
    self.torque_ctrl = LatControlTorque(CP, CP_SP, CI, dt)

    # 預設使用 CP (CarParams) 讀出來的設定值
    self.use_angle = (CP.steerControlType == car.CarParams.SteerControlType.angle)

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

    # 2. Angle 控制器永遠運算 (幾何計算，無風險)
    _, a_steer, a_log = self.angle_ctrl.update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay)

    # 3. Torque 控制器永遠運算 (熱備援)
    # 關鍵防護：如果當前是 Angle 主控，強制觸發 steer_limited_by_safety 來凍結 Torque 的 PID 積分
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