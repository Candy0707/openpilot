from __future__ import annotations

import os

REMOTE_UI = os.getenv("REMOTE_UI") == "1"
_hooks_installed = False


def install_remote_hooks() -> None:
  """
  在 UI 啟動最前期（匯入 ui_state 與各設定子頁之前）動態替換原生底層模組：
  - openpilot.common.params.Params -> 遠端快取＋WebRTC 同步版 Params
  - openpilot.cereal.messaging.SubMaster -> WebRTC Capnp 二進位版 SubMaster
  - openpilot.cereal.messaging.PubMaster -> 遠端安全 No-op 版 PubMaster

  透過此統一入口，全專案數十個上游檔案無需逐一加上 if REMOTE_UI 分流即可自動生效。
  """
  global _hooks_installed
  if _hooks_installed or not REMOTE_UI:
    return

  import openpilot.cereal.messaging as _native_messaging
  import openpilot.common.params as _native_params
  from openpilot.selfdrive.ui.remote.cereal import messaging as _remote_messaging
  from openpilot.selfdrive.ui.remote.params import Params as _RemoteParams

  _native_params.Params = _RemoteParams  # type: ignore[misc]
  _native_messaging.SubMaster = _remote_messaging.SubMaster  # type: ignore[misc]
  _native_messaging.PubMaster = _remote_messaging.PubMaster  # type: ignore[misc]

  _hooks_installed = True
