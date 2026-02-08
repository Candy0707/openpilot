"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp


class CruiseLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    self._accelpersonalitycontroller_toggle = toggle_item_sp(
      title=tr("Enable AccelPersonalityController"),
      description=tr(""),
      param="AccelPersonalityEnabled",
      callback=self._accelpersonalitycontroller_toggle_callback,
    )

    items = [
      self._accelpersonalitycontroller_toggle,
    ]
    return items

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()

  def _accelpersonalitycontroller_toggle_callback(self, state: bool):
    self._params.put_bool("AccelPersonalityEnabled", state)
