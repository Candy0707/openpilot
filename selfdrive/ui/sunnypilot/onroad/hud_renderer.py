"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.radar_ui import RadarUiRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.trend_ui import TrendRenderer


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.developer_ui = DeveloperUiRenderer()
    self.trend_ui = TrendRenderer()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.developer_ui.render(rect)
    self.RadarUiRenderer.render(rect)
    self.trend_ui.render(rect)

  def set_model_renderer(self, model_renderer):
    self.model_renderer = model_renderer
    self.RadarUiRenderer = RadarUiRenderer(model_renderer)


