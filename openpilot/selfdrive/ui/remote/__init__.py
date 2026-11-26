from __future__ import annotations

import json
import os
import sys
import time
import types
import urllib.error
import urllib.request

from openpilot.selfdrive.ui.remote.webrtc_client import RemoteWebRTCClient

REMOTE_UI = os.getenv("REMOTE_UI") == "1"
_hooks_installed = False
_connection_ready = False


def wait_for_remote_service(
  host: str,
  port: int,
  poll_interval: float = 1.0,
  timeout: float | None = None,
) -> bool:
  """
  持續探測遠端 webrtcd_ext 的 /schema 端點直到可連通為止。
  在正式模式（timeout=None）下未連上服務前完全阻塞不放行，避免無資料來源時往下開窗。
  """
  schema_url = f"http://{host}:{port}/schema"
  last_error: str | None = None
  deadline = (time.monotonic() + timeout) if timeout is not None else None

  while True:
    try:
      req = urllib.request.Request(schema_url, method="GET")
      with urllib.request.urlopen(req, timeout=2.0) as resp:
        if resp.status == 200:
          # 確認回傳為合法 JSON，代表信令服務已完整就緒
          json.loads(resp.read().decode("utf-8"))
          print(f"[RemoteUI] 已確認遠端服務就緒 ({schema_url})")
          return True
    except urllib.error.URLError as e:
      err_msg = str(e.reason if getattr(e, "reason", None) else e)
    except Exception as e:
      err_msg = str(e)

    # 僅在錯誤訊息變更時輸出提示，避免重試迴圈洗版
    if err_msg != last_error:
      print(f"[RemoteUI] 等待遠端服務 ({schema_url}) 中：{err_msg}，持續重試不往下開窗...")
      last_error = err_msg

    if deadline is not None and time.monotonic() >= deadline:
      return False

    time.sleep(poll_interval)


def _install_cereal_shim_if_needed(_remote_messaging: types.ModuleType) -> None:
  """
  掛載 openpilot.cereal.messaging：
  - 若目前環境（如 WSL / Linux）已存在完整 openpilot.cereal，直接替換其 messaging 子模組，
    保留原生 openpilot.cereal.log 列舉常數。
  - 若在無 pycapnp 的純跨平台環境（Windows / Android / iOS），自動建立輕量套件殼層。
  """
  sys.modules["openpilot.cereal.messaging"] = _remote_messaging
  try:
    import openpilot.cereal as _cereal_pkg
    _cereal_pkg.messaging = _remote_messaging  # type: ignore[attr-defined]
  except Exception:
    cereal_pkg = types.ModuleType("openpilot.cereal")
    cereal_pkg.messaging = _remote_messaging  # type: ignore[attr-defined]
    sys.modules["openpilot.cereal"] = cereal_pkg


def install_remote_hooks(wait_service: bool = True) -> None:
  """
  在 UI 啟動最前期（匯入 ui_state 與各設定子頁之前）透過 sys.modules 動態注入純 Python 遠端模組：
  - openpilot.common.params -> 遠端真實快取版 Params（跳過 libparams_c.so 載入）
  - openpilot.cereal.messaging -> 遠端原生 JSON 版 SubMaster / PubMaster（跳過 msgq / pycapnp）

  注入完成後預設阻塞等待遠端 /schema 就緒，連不上不放行往下開窗。
  """
  global _hooks_installed
  if _hooks_installed or not REMOTE_UI:
    return

  from openpilot.selfdrive.ui.remote import params as _remote_params
  from openpilot.selfdrive.ui.remote.cereal import messaging as _remote_messaging

  # 直接注入 sys.modules，確保後續所有 from openpilot.common.params import ... 零 C 擴充依賴
  sys.modules["openpilot.common.params"] = _remote_params
  _install_cereal_shim_if_needed(_remote_messaging)

  _hooks_installed = True

  if wait_service:
    host = os.getenv("REMOTE_HOST", "127.0.0.1")
    port = int(os.getenv("REMOTE_PORT", "5001"))
    wait_for_remote_service(host=host, port=port)


def connect_remote_ui(
  required_services: list[str] | None = None,
  poll_interval: float = 0.2,
  timeout: float | None = None,
) -> RemoteWebRTCClient:
  """
  在 ui_state（SubMaster）完成所有服務註冊後呼叫：
  於 __init__ 內部統一啟動連線，並阻塞等待三段式開窗閘門（WebRTC 連通＋真實 Params 快照＋
  核心服務真實資料）全部就緒後才回傳 RemoteWebRTCClient 實例供外部使用。
  """
  global _connection_ready
  client = RemoteWebRTCClient.get_instance()
  if not REMOTE_UI:
    return client

  if not _connection_ready:
    ready = client.wait_until_ready(
      required_services=required_services or ["deviceState"],
      poll_interval=poll_interval,
      timeout=timeout,
    )
    if ready:
      _connection_ready = True

  return client


def get_remote_client() -> RemoteWebRTCClient:
  """直接從 openpilot.selfdrive.ui.remote 取出共用的 RemoteWebRTCClient 單例。"""
  return RemoteWebRTCClient.get_instance()


__all__ = [
  "REMOTE_UI",
  "RemoteWebRTCClient",
  "connect_remote_ui",
  "get_remote_client",
  "install_remote_hooks",
  "wait_for_remote_service",
]