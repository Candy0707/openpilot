from __future__ import annotations

import atexit
import contextlib
import json
import os
import signal
import sys
import time
import types
from typing import Any
import urllib.error
import urllib.request

from openpilot.selfdrive.ui.remote.webrtc_client import RemoteWebRTCClient

REMOTE_UI = os.getenv("REMOTE_UI") == "1"
_hooks_installed = False
_connection_ready = False
_gui_hooks_installed = False
_offroad_sync_installed = False


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

  try:
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
  except KeyboardInterrupt:
    print("\n[RemoteUI] 收到中斷訊號，取消等待遠端服務並結束")
    sys.exit(130)


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


def _install_visionipc_shim(_remote_visionipc: types.ModuleType) -> None:
  """
  掛載 msgq.visionipc（以及無 C 擴充環境下的 openpilot.cereal.visionipc 退回層）：
  - 直接替換 msgq.visionipc，使上游 CameraView 無需修改即可讀取遠端內建 FFmpeg NV12 影格。
  - 若目前環境無法匯入原生 openpilot.cereal.visionipc，同步提供純 Python VisionStreamType。
  """
  sys.modules["msgq.visionipc"] = _remote_visionipc
  try:
    import msgq as _msgq_pkg
    _msgq_pkg.visionipc = _remote_visionipc  # type: ignore[attr-defined]
  except Exception:
    msgq_pkg = types.ModuleType("msgq")
    msgq_pkg.visionipc = _remote_visionipc  # type: ignore[attr-defined]
    sys.modules["msgq"] = msgq_pkg

  try:
    import openpilot.cereal.visionipc  # noqa: F401
  except Exception:
    visionipc_enum_mod = types.ModuleType("openpilot.cereal.visionipc")
    visionipc_enum_mod.VisionStreamType = _remote_visionipc.VisionStreamType  # type: ignore[attr-defined]
    sys.modules["openpilot.cereal.visionipc"] = visionipc_enum_mod
    cereal_pkg = sys.modules.get("openpilot.cereal")
    if cereal_pkg is not None:
      cereal_pkg.visionipc = visionipc_enum_mod  # type: ignore[attr-defined]


def _install_wifi_manager_shim(_remote_wifi_manager: types.ModuleType) -> None:
  """
  掛載 openpilot.system.ui.lib.wifi_manager 安全空實作替身：
  - 阻止遠端 UI 連線本機 D-Bus（jeepney / NetworkManager）與啟動背景掃描執行緒，
    徹底消除關閉視窗時的 RouterClosed 與 Bad file descriptor 例外，並支援非 Linux 跨平台執行。
  - 僅在父套件 openpilot.system.ui.lib 已載入時同步掛載屬性，避免過早觸發 raylib / GUI 初始化。
  """
  sys.modules["openpilot.system.ui.lib.wifi_manager"] = _remote_wifi_manager
  ui_lib_pkg = sys.modules.get("openpilot.system.ui.lib")
  if ui_lib_pkg is not None:
    ui_lib_pkg.wifi_manager = _remote_wifi_manager  # type: ignore[attr-defined]


def _install_offroad_video_sync(client: RemoteWebRTCClient) -> None:
  """
  掛載 ui_state 的 Onroad / Offroad 切換回呼：
  - Offroad 時自動停用視訊串流與視訊解碼器，立即釋放環形緩衝區記憶體並節省頻寬。
  - 進入 Onroad 時自動啟用視訊串流並向車機請求 IDR 關鍵幀。
  """
  global _offroad_sync_installed
  if _offroad_sync_installed:
    return

  ui_state_mod = sys.modules.get("openpilot.selfdrive.ui.ui_state")
  if ui_state_mod is None:
    return

  ui_state = getattr(ui_state_mod, "ui_state", None)
  if ui_state is None:
    return

  def _on_offroad_transition() -> None:
    client.set_video_enabled(ui_state.is_onroad())

  ui_state.add_offroad_transition_callback(_on_offroad_transition)
  # 同步目前狀態（若啟動當下已在 Onroad 則自動開啟）
  client.set_video_enabled(ui_state.is_onroad())
  _offroad_sync_installed = True


def _install_gui_shutdown_hooks(client: RemoteWebRTCClient) -> None:
  """
  在不修改 application.py 與 ui.py 的前提下，修復遠端 UI 模式的 Ctrl+C 退出死當問題：
  1. 包裝 gui_app.close()：確保先安全停止 RemoteWebRTCClient（關閉 libdatachannel 回呼與視訊
     解碼器），再於主執行緒執行原生 OpenGL / raylib 卸載，並具備冪等防重入保護。
  2. 包裝 gui_app.init_window()：保留原生初始化，並在完成後將 SIGINT 覆寫為兩段式安全處理器：
     - 第一次 Ctrl+C：僅呼叫 gui_app.request_close()，讓主迴圈安全跑完當幀 rl.end_drawing()
       後自然退出並走 atexit 清理，避免在繪圖週期內重入 rl.close_window() 導致死鎖。
     - 第二次 Ctrl+C：若底層仍阻塞於 raylib / WSLg C 層，立即以 os._exit(130) 強制結束。
  """
  global _gui_hooks_installed
  if _gui_hooks_installed:
    return

  # 即使在 init_window 前退出，atexit 亦會確保關閉背景 WebRTC 執行緒與解碼器
  def _stop_client_safe() -> None:
    with contextlib.suppress(Exception):
      client.stop()

  atexit.register(_stop_client_safe)

  app_mod = sys.modules.get("openpilot.system.ui.lib.application")
  if app_mod is None:
    _gui_hooks_installed = True
    return

  gui_app = getattr(app_mod, "gui_app", None)
  if gui_app is None:
    _gui_hooks_installed = True
    return

  orig_close = gui_app.close
  orig_init_window = gui_app.init_window
  close_state = {"done": False}
  sigint_state = {"count": 0}

  def _remote_safe_close() -> None:
    if close_state["done"]:
      return
    close_state["done"] = True
    _stop_client_safe()
    orig_close()

  def _remote_sigint_handler(sig: int, frame: Any) -> None:
    sigint_state["count"] += 1
    if sigint_state["count"] == 1:
      print("\n[RemoteUI] 收到中斷訊號，正在安全關閉視窗與遠端連線...（若未回應再按一次 Ctrl+C 強制結束）")
      gui_app.request_close()
    else:
      print("\n[RemoteUI] 強制結束遠端 UI 行程")
      os._exit(130)

  def _remote_init_window(*args: Any, **kwargs: Any) -> Any:
    # orig_init_window 內部的 atexit.register(self.close) 會自動註冊 _remote_safe_close
    res = orig_init_window(*args, **kwargs)
    # 覆寫原生直接於 signal handler 呼叫 self.close() 的 SIGINT 處理器，消除繪圖重入死鎖
    signal.signal(signal.SIGINT, _remote_sigint_handler)
    return res

  gui_app.close = _remote_safe_close
  gui_app.init_window = _remote_init_window
  _gui_hooks_installed = True


def install_remote_hooks(wait_service: bool = True) -> None:
  """
  在 UI 啟動最前期（匯入 ui_state 與各設定子頁之前）透過 sys.modules 動態注入純 Python 遠端模組：
  - openpilot.common.params -> 遠端真實快取版 Params（跳過 libparams_c.so 載入）
  - openpilot.cereal.messaging -> 遠端原生 JSON 版 SubMaster / PubMaster（跳過 msgq / pycapnp）
  - msgq.visionipc -> 遠端內建 FFmpeg NV12 版 VisionIpcClient / VisionBuf（讓上游 CameraView 零修改運作）
  - openpilot.system.ui.lib.wifi_manager -> 遠端安全空實作替身 WifiManager（跳過本機 D-Bus 與 jeepney）

  注入完成後預設阻塞等待遠端 /schema 就緒，連不上不放行往下開窗。
  """
  global _hooks_installed
  if _hooks_installed or not REMOTE_UI:
    return

  from openpilot.selfdrive.ui.remote import params as _remote_params
  from openpilot.selfdrive.ui.remote import visionipc as _remote_visionipc
  from openpilot.selfdrive.ui.remote import wifi_manager as _remote_wifi_manager
  from openpilot.selfdrive.ui.remote.cereal import messaging as _remote_messaging

  # 直接注入 sys.modules，確保後續所有 from openpilot.common.params / msgq.visionipc / wifi_manager import ... 零平台相依
  sys.modules["openpilot.common.params"] = _remote_params
  _install_cereal_shim_if_needed(_remote_messaging)
  _install_visionipc_shim(_remote_visionipc)
  _install_wifi_manager_shim(_remote_wifi_manager)

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
  於內部統一啟動連線、阻塞等待三段式開窗閘門（WebRTC 連通＋真實 Params 快照＋
  核心服務真實資料）全部就緒，並自動掛載 Onroad 視訊開關同步與安全退出保護。
  """
  global _connection_ready
  client = RemoteWebRTCClient.get_instance()
  if not REMOTE_UI:
    return client

  if not _connection_ready:
    try:
      ready = client.wait_until_ready(
        required_services=required_services or ["deviceState"],
        poll_interval=poll_interval,
        timeout=timeout,
      )
    except KeyboardInterrupt:
      print("\n[RemoteUI] 收到中斷訊號，停止連線並結束")
      client.stop()
      sys.exit(130)
    if ready:
      _connection_ready = True

  _install_offroad_video_sync(client)
  _install_gui_shutdown_hooks(client)
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
