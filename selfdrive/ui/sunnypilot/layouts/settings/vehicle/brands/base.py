"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import abc
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.ui_state import ui_state

from cereal import custom
from opendbc.car import structs

class BrandSettings(abc.ABC):
  def __init__(self):
    self.items = []
    self.ui_state = ui_state
    self.sm = self.ui_state.sm
    self.params = ui_state.params
    self.engaged = ui_state.engaged
    self.cloudlog = cloudlog
    self.CP: structs.carParams = self.sm['carParams']
    self.CP_SP: custom.CarParamsSP = self.sm['carParamsSP']

  def update_state(self) -> None:
    self.CP = self.sm['carParams']
    self.CP_SP = self.sm['carParamsSP']

  @abc.abstractmethod
  def update_settings(self) -> None:
    """Update the settings based on the current vehicle brand."""
