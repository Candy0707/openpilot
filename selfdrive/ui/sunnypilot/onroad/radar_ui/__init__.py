import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.radar_ui.elements import RadarData

class RadarModelWrapper:
    def __init__(self, model_renderer):
        self._model = model_renderer

    def map_to_screen(self, dRel, yRel, z=0.0):
        screen_pt = self._model._map_to_screen(dRel, yRel, z)
        return screen_pt

    def map(self, x, y, z=0.0):
       offset = self._model.path_offset_z
       return self._model._map_to_screen(x, -y, z + offset)

class RadarUiRenderer(Widget):
  RADAR_UI_OFF = 0
  RADAR_UI_POINT = 1
  RADAR_UI_POINTINFO = 2

  def __init__(self, model_renderer):
    super().__init__()
    self.is_metric = False
    self.radar_ui_mode = 0
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)

    self.model = RadarModelWrapper(model_renderer)
    self.radar = RadarData()

  def _update_state(self) -> None:
    self.radar_ui_mode = ui_state.radar_ui
    self.is_metric = ui_state.is_metric

    sm = ui_state.sm

    if sm.updated['liveTracks']:
      radarpoint = sm['liveTracks'].points
      self.radar.update(radarpoint)


  def _render(self, rect: rl.Rectangle) -> None:
    # Draw OFF
    if self.radar_ui_mode == 0:
      return

    # Draw Points
    if self.radar_ui_mode == 1:
      self._draw_radar_ui_point(rect)

    # Draw PointInfo
    if self.radar_ui_mode == 2:
      self._draw_radar_ui_point(rect)
      self._draw_radar_ui_info(rect)

  def _draw_radar_ui_point(self, rect: rl.Rectangle) -> None:
    if not self.radar.Points:
        return

    for point in self.radar.Points:
      # 取得螢幕座標
      screen_pt = self.model.map(point.dRel, point.yRel)
      x = int(screen_pt[0])
      y = int(screen_pt[1])

      rl.draw_circle(x, y, 3, point.color)

  def _draw_radar_ui_info(self, rect: rl.Rectangle) -> None:
    if not self.radar.Points:
        return

    # 單位與轉換
    unit = "km/h" if self.is_metric else "mph"
    conversion = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    for point in self.radar.Points:
      # 取得螢幕座標
      screen_pt = self.model.map(point.dRel, point.yRel)
      x = int(screen_pt[0])
      y = int(screen_pt[1])

      # 顯示資訊：ID, dRel, yRel, vRel
      text = (
        f"ID:{point.trackId} " +
        f"d:{point.dRel:.1f} " +
        f"y:{point.yRel:.1f} " +
        f"v:{point.vRel * conversion:.1f} {unit}"
      )


      # draw_font_ex 需要傳入 font、文字、位置、大小、顏色
      rl.draw_font_ex(
        self._font_bold,  # font
        text,
        x + 5,  # pos_x
        y - 5,  # pos_y
        12,     # font_size
        rl.WHITE  # color
      )

