import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state

from cereal import custom

MapState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState
VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState


class SmartCruiseControlRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self.frame = 0
    self._blink_on = True

    # === Vision SCC 初始值 ===
    self.smartCruiseControlVisionEnabled = False
    self.smartCruiseControlVisionActive = False
    self.smartCruiseControlVisionState = VisionState.disabled

    # === Map SCC 初始值 ===
    self.smartCruiseControlMapEnabled = False
    self.smartCruiseControlMapActive = False
    self.smartCruiseControlMapState = MapState.disabled

  def _update_state(self) -> None:
    sm = ui_state.sm

    self.lp_sp = sm['longitudinalPlanSP']
    if sm.updated['longitudinalPlanSP']:
      self.lp_sp = sm['longitudinalPlanSP']
      # Vision SCC
      self.smartCruiseControlVisionEnabled = self.lp_sp.smartCruiseControl.vision.enabled
      self.smartCruiseControlVisionActive = self.lp_sp.smartCruiseControl.vision.active
      self.smartCruiseControlVisionState = self.lp_sp.smartCruiseControl.vision.state
      # Map SCC
      self.smartCruiseControlMapEnabled = self.lp_sp.smartCruiseControl.map.enabled
      self.smartCruiseControlMapActive = self.lp_sp.smartCruiseControl.map.active
      self.smartCruiseControlMapState = self.lp_sp.smartCruiseControl.map.state

  def _render(self, rect: rl.Rectangle) -> None:
    # 每 frame 控制閃爍
    blink_rate = 15
    if self.frame % blink_rate == 0:
      self._blink_on = not self._blink_on

    # 畫面位置設定
    offset_x = 300  # 左邊距
    offset_y = 100
    center_x = offset_x + rect.x + rect.width / 2
    y = offset_y + rect.y
    spacing_y = 60
    font_size = 24

    # RGBA 顏色宣告
    color_disabled = rl.Color(100, 100, 100, 200)  # 暗灰
    color_enabled = rl.Color(0, 150, 0, 200)  # 暗綠
    color_active = rl.Color(200, 180, 0, 200)  # 暗黃，不刺眼

    # SCC-V 顏色判斷（三元運算子）
    color_v = (
      color_active if self.smartCruiseControlVisionActive and self._blink_on else color_enabled if self.smartCruiseControlVisionEnabled else color_disabled
    )

    # SCC-M 顏色判斷（三元運算子）
    color_m = color_active if self.smartCruiseControlMapActive and self._blink_on else color_enabled if self.smartCruiseControlMapEnabled else color_disabled

    # 畫 SCC-M
    self.draw_scc_text_aligned("SCC-M", str(self.smartCruiseControlMapState), center_x, y, font_size, color_m)

    # 畫 SCC-V
    y += spacing_y
    self.draw_scc_text_aligned("SCC-V", str(self.smartCruiseControlVisionState), center_x, y, font_size, color_v)

    # 更新 frame
    self.frame += 1

  def draw_scc_text_aligned(self, label: str, state: str, center_x: float, center_y: float, font_size: int, bg_color: rl.Color, padding: float = 6):
    """
    畫背景矩形 + 文字
    標籤靠左，狀態靠右
    """
    # label + 冒號
    text_label = f"{label}："
    text_state = f"A{state}"
    # 量文字寬度
    measure_label = measure_text_cached(self._font_bold, text_label, font_size)
    measure_state = measure_text_cached(self._font_bold, text_state, font_size)

    # 矩形寬度自動依文字總長度放大
    bg_w = measure_label.x + measure_state.x + padding * 4
    bg_h = max(measure_label.y, measure_state.y) + padding * 2
    bg_x = center_x - bg_w / 2
    bg_y = center_y - bg_h / 2

    # 畫背景圓角矩形
    bg_rect = rl.Rectangle(bg_x, bg_y, bg_w, bg_h)
    rl.draw_rectangle_rounded(bg_rect, 0.25, 8, bg_color)

    # label 靠左
    label_x = bg_x + padding
    label_y = bg_y + (bg_h - measure_label.y) / 2
    rl.draw_text_ex(self._font_bold, text_label, rl.Vector2(label_x, label_y), font_size, 0, rl.Color(255, 255, 255, 255))

    # state 靠右
    state_x = bg_x + bg_w - measure_state.x - padding
    state_y = bg_y + (bg_h - measure_state.y) / 2
    rl.draw_text_ex(self._font_bold, text_state, rl.Vector2(state_x, state_y), font_size, 0, rl.Color(255, 255, 255, 255))
