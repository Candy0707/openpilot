"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import pyray as rl
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.chevron_metrics import ChevronMetrics
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath


class ModelRendererSP(ModelRenderer):
  def __init__(self):
    super().__init__()
    self.rainbow_path = RainbowPath()
    self.chevron_metrics = ChevronMetrics()

    self.leftBlindspot = False
    self.rightBlindspot = False

  def _render(self, rect: rl.Rectangle):
    super()._render(rect)
    sm = ui_state.sm
    carState = sm['carState']

    self.leftBlindspot = carState.leftBlindspot
    self.rightBlindspot = carState.rightBlindspot

  def get_lane_line_color(self, line) -> rl.Color:
      alpha = np.clip(self._lane_line_probs[line], 0.0, 0.7)
      color = rl.Color(255, 255, 255, int(alpha * 255))

      if (self.leftBlindspot and line == 1) or \
         (self.rightBlindspot and line == 2):
         color = rl.Color(255, 165, 0, 255)  # 橘色，alpha=255 不透明

      return color


