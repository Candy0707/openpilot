"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.mici.layouts.settings.developer import DeveloperLayoutMici
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl
from openpilot.system.ui.lib.multilang import tr

OPVIEW_DESCRIPTION = ("Streams the road camera and driving data to the opview App (Android) on the same network " +
                      "(e.g. your phone hotspot). Also lets opview read and set the vehicle while offroad.")


class DeveloperLayoutMiciSP(DeveloperLayoutMici):
  def __init__(self):
    super().__init__()

    # opview：與 tici 開發者頁的「啟用 opview」同一個參數（OpviewEnabled）
    # 標題由 BigButton 顯示時再翻譯（_tr），這裡傳英文 msgid；說明在建立時翻譯
    self._opview_toggle = BigParamControl("enable opview", "OpviewEnabled", description=tr(OPVIEW_DESCRIPTION))
    self._scroller.add_widget(self._opview_toggle)

    # sunnylink / SSH 從外部改參數時，回到此頁會同步開關狀態
    self._refresh_toggles = (*self._refresh_toggles, ("OpviewEnabled", self._opview_toggle))
