"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.common.params import Params
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item

DESCRIPTIONS = {
  "DeveloperUI": tr_noop("Developer UI"),
  "RadarInfoUI": tr_noop("Radar Info UI"),
  "ChevronUI": tr_noop("Chevron UI"),
}


class VisualsLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    self._DeveloperUI_Setting = multiple_button_item(
      lambda: tr("Developer UI"),
      lambda: tr(DESCRIPTIONS["DeveloperUI"]),
      buttons=[lambda: tr("OFF"), lambda: tr("RIGHT"), lambda: tr("BOTTOM"), lambda: tr("BOTH")],
      button_width=240,
      callback=self._set_developer_ui,
      selected_index=self._params.get("DevUIInfo", return_default=True),
      icon="speed_limit.png",
    )
    self._RadarInfoUI_Setting = multiple_button_item(
      lambda: tr("Radar Info UI"),
      lambda: tr(DESCRIPTIONS["RadarInfoUI"]),
      buttons=[lambda: tr("OFF"), lambda: tr("Point"), lambda: tr("Info")],
      button_width=240,
      callback=self._set_radarinfo_ui,
      selected_index=self._params.get("RadarUIInfo", return_default=True),
    )
    self._ChevronUI_Setting = multiple_button_item(
      lambda: tr("Chevron UI"),
      lambda: tr(DESCRIPTIONS["ChevronUI"]),
      buttons=[lambda: tr("OFF"), lambda: tr("DISTANCE_ONLY"), lambda: tr("SPEED_ONLY"), lambda: tr("TTC_ONLY"), lambda: tr("ALL")],
      button_width=240,
      callback=self._set_chevron_ui,
      selected_index=self._params.get("ChevronInfo", return_default=True),
    )
    items = [self._DeveloperUI_Setting, self._RadarInfoUI_Setting, self._ChevronUI_Setting]
    return items

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()

  def _set_developer_ui(self, button_index: int):
    self._params.put("DevUIInfo", button_index)

  def _set_radarinfo_ui(self, button_index: int):
    self._params.put("RadarUIInfo", button_index)

  def _set_chevron_ui(self, button_index: int):
    self._params.put("ChevronInfo", button_index)
