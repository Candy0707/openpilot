# __init__.py

import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached


class LongitudinalPlanRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self.source_text = "N/A"
    self.source_color = rl.WHITE
    self.label_size = 60

  def update(self) -> None:
    sm = ui_state.sm
    # 確保你的 submaster 有訂閱 longitudinalPlanSP 或 longitudinalPlan
    if sm.updated['longitudinalPlanSP']:
      plan = sm['longitudinalPlanSP']

      state_val = plan.adaptiveCoastingModule.state
      leadDist = plan.adaptiveCoastingModule.leadDist
      targetDist = plan.adaptiveCoastingModule.targetDist
      distPercent = plan.adaptiveCoastingModule.distPercent

      # 使用我們定義的 Data 類別轉換
      self.state_text = f"state: {state_val}"
      self.leadDist_text = f"leadDist: {leadDist:.1f}"
      self.targetDist_text = f"targetDist: {targetDist:.1f}"
      self.distPercent_text = f"distPercent: {distPercent:.2f}"
      self.source_text = f"{self.state_text} | {self.distPercent_text}"

  def _render(self, rect: rl.Rectangle) -> None:
    # 設定顯示位置（跟隨傳入的動態 rect 區塊）
    text_size = measure_text_cached(self._font_bold, self.source_text, self.label_size, 0)

    # 預留邊距
    margin = 20

    # 【關鍵修正】加上 rect.x 與 rect.y 作為起點基準
    x = int(rect.x + (rect.width / 2) - (text_size.x / 2))
    y = int(rect.y + margin)

    # 畫一個半透明背景框，增加閱讀性
    bg_rect = rl.Rectangle(x - 10, y - 5, text_size.x + 20, text_size.y + 10)
    rl.draw_rectangle_rounded(bg_rect, 0.3, 8, rl.Color(0, 0, 0, 150))

    # 繪製文字
    rl.draw_text_ex(self._font_bold, self.source_text, rl.Vector2(x, y), self.label_size, 0, self.source_color)
