import pyray as rl

from collections import deque
import time

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state

HISTORY_SEC = 10.0  # ← 可調整的時間窗（秒）
MAX_POINTS = 2000  # 保險用（避免爆記憶體）


class TrendRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.last_a_ego = 0.0
    self.last_accel_cmd = 0.0
    self.last_accel_out = 0.0
    self.ts = deque(maxlen=MAX_POINTS)
    self.a_ego = deque(maxlen=MAX_POINTS)
    self.accel_cmd = deque(maxlen=MAX_POINTS)
    self.accel_out = deque(maxlen=MAX_POINTS)

  def _update_state(self) -> None:
    sm = ui_state.sm
    # 更新 latest value（有更新才換）
    if sm.updated['carState']:
      self.last_a_ego = sm['carState'].aEgo

    if sm.updated['carControl']:
      self.last_accel_cmd = sm['carControl'].actuators.accel

    if sm.updated['carOutput']:
      self.last_accel_out = sm['carOutput'].actuatorsOutput.accel

    now = time.monotonic()
    # 同步 append
    self.ts.append(now)
    self.a_ego.append(self.last_a_ego)
    self.accel_cmd.append(self.last_accel_cmd)
    self.accel_out.append(self.last_accel_out)
    self.trim_history(now)

  def trim_history(self, now):
    while self.ts and (now - self.ts[0]) > HISTORY_SEC:
      self.ts.popleft()
      self.a_ego.popleft()
      self.accel_cmd.popleft()
      self.accel_out.popleft()

  def _render(self, rect: rl.Rectangle) -> None:
    # === 計算受限畫布 ===
    is_wide = rect.width >= 2000

    draw_w = min(rect.width, 600) if is_wide else min(rect.width, 460)
    draw_h = min(rect.height, 210)

    draw_x = int(rect.x + 260)
    draw_y = int(rect.y + 40)

    # 保證不超出父 rect
    draw_w = min(draw_w, rect.width - (draw_x - rect.x))
    draw_h = min(draw_h, rect.height - (draw_y - rect.y))

    draw_rect = rl.Rectangle(draw_x, draw_y, draw_w, draw_h)

    # === 背景 + 外框 ===
    rl.draw_rectangle(draw_x, draw_y, draw_w, draw_h, rl.Color(30, 30, 30, 180))
    rl.draw_rectangle_lines(draw_x, draw_y, draw_w, draw_h, rl.WHITE)

    # === 參考線 (0 accel baseline) ===
    v_min = -3.0
    v_max = 3.0
    y_zero = int(draw_y + draw_h - (0 - v_min) / (v_max - v_min) * draw_h)
    rl.draw_line(draw_x, y_zero, draw_x + draw_w, y_zero, rl.WHITE)

    # === 畫線工具函式 ===
    def map_x(t):
      if len(self.ts) < 2:
        return draw_x
      t0 = self.ts[0]
      t1 = self.ts[-1]
      return int(draw_x + (t - t0) / (t1 - t0) * draw_w)

    def map_y(v):
      v = max(v_min, min(v_max, v))
      return int(draw_y + draw_h - (v - v_min) / (v_max - v_min) * draw_h)

    def draw_series(values, color):
      prev_x = None
      prev_y = None
      for t, v in zip(self.ts, values):
        x = map_x(t)
        y = map_y(v)

        # clip 到 draw_rect 內
        x = max(draw_x, min(draw_x + draw_w, x))
        y = max(draw_y, min(draw_y + draw_h, y))

        if prev_x is not None:
          rl.draw_line(prev_x, prev_y, x, y, color)

        prev_x = x
        prev_y = y

    # === 畫三條線 ===
    draw_series(self.accel_cmd, rl.YELLOW)  # 目標加速度
    draw_series(self.accel_out, rl.BLUE)  # 實際輸出
    draw_series(self.a_ego, rl.GREEN)  # 實際加速度
