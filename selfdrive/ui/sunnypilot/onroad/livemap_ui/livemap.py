import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state


class LiveMapRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)

    self.speedLimitValid = bool
    self.speedLimit = 0.0

    self.speedLimitAheadValid = bool
    self.speedLimitAhead = 0.0
    self.speedLimitAheadDistance = 0.0

    self.roadName = None

  def _update_state(self) -> None:
    sm = ui_state.sm
    is_metric = ui_state.is_metric
    if sm.updated['liveMapDataSP']:
      self.speedLimitValid = sm['liveMapDataSP'].speedLimitValid
      self.speedLimit = sm['liveMapDataSP'].speedLimit * CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH

      self.speedLimitAheadValid = sm['liveMapDataSP'].speedLimitAheadValid
      self.speedLimitAhead = sm['liveMapDataSP'].speedLimitAhead
      self.speedLimitAheadDistance = sm['liveMapDataSP'].speedLimitAheadDistance

      self.roadName = sm['liveMapDataSP'].roadName

  def _render(self, rect: rl.Rectangle) -> None:
    if ui_state.road_name:
      self._draw_road_name(rect)
    self._draw_speed_limit(rect)

  def _draw_road_name(self, rect: rl.Rectangle):
    if not self.roadName:
      return

    text = self.roadName
    font_size = 36

    # 螢幕中心
    center_x = rect.x + rect.width / 2

    # 量文字寬（用 ex，比 measure_text 準）
    measure_text = measure_text_cached(self._font_bold, text, font_size)
    text_w = measure_text.x
    text_h = measure_text.y

    bg_w = text_w * 1.05
    bg_h = text_h * 1.05
    bg_x = text_x = center_x - bg_w / 2
    bg_y = rect.y

    # === 背景矩形 ===
    bg_rect = rl.Rectangle(bg_x, bg_y, bg_w, bg_h)

    rl.draw_rectangle_rounded(bg_rect, 0.25, 8, rl.Color(0, 0, 0, 160))

    # === 文字（只求視覺置中） ===
    text_x = center_x - text_w / 2
    text_y = bg_rect.y

    rl.draw_text_ex(self._font_bold, text, rl.Vector2(text_x, text_y), font_size, 0, rl.Color(255, 255, 255, 255))

  def _draw_speed_limit(self, rect: rl.Rectangle):
    """
    在 HUD 上繪製速限圓圈 + 調試矩形
    rect : 參考矩形
    diameter : 圓圈直徑
    outer_thickness : 外圈厚度
    """

    # 圓圈中心（固定）
    center_x = rect.x + 2 * 72
    center_y = rect.y + 2 * 180
    radius = 102

    # 顏色設定
    if self.speedLimitValid:
      outer_color = rl.RED
      inner_color = rl.WHITE
      text_color = rl.BLACK
    else:
      outer_color = rl.Color(128, 128, 128, 128)
      inner_color = rl.Color(200, 200, 200, 128)
      text_color = rl.Color(100, 100, 100, 128)

    # ----------------------
    # 畫外圈（模擬厚度）
    # ----------------------
    for t in range(20):
      rl.draw_circle_lines(int(center_x), int(center_y), int(radius - t), outer_color)

    # ----------------------
    # 畫內圈
    # ----------------------
    rl.draw_circle(int(center_x), int(center_y), int(radius - 20), inner_color)

    # ----------------------
    # 畫文字置中
    # ----------------------
    speed_text = str(int(self.speedLimit))
    font_size = 80
    measure_text = measure_text_cached(self._font_bold, speed_text, font_size)
    text_width = measure_text.x
    text_height = measure_text.y

    text_x = center_x - text_width / 2
    text_y = center_y - text_height / 2

    rl.draw_text_ex(self._font_bold, speed_text, rl.Vector2(text_x, text_y), font_size, 0, text_color)
