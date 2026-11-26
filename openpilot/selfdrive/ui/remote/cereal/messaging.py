from __future__ import annotations

import threading
import time
from typing import Any

# 完整匯出原生 messaging 模組的工具函式（如 new_message、log_from_bytes 等）
from openpilot.cereal import log, messaging as _native_messaging
from openpilot.cereal.messaging import *  # noqa: F403
from openpilot.selfdrive.ui.remote.webrtc_client import RemoteWebRTCClient


class SubMaster(_native_messaging.SubMaster):
  """
  遠端版 SubMaster（繼承自原生 openpilot.cereal.messaging.SubMaster）。

  設計說明：
  1. 透過繼承原生類別，完整滿足 isinstance(sm, messaging.SubMaster) 檢查，並沿用原生
     的 __getitem__、all_alive、all_valid、all_checks 等純 Python 介面。
  2. 刻意不呼叫 super().__init__()，避免在無本機 msgq 的遠端裝置上開啟本機 SubSocket。
  3. 透過 RemoteWebRTCClient 動態訂閱 services，背景執行緒收到原始 Capnp 二進位封包後
     直接反序列化並寫入暫存緩衝區；主執行緒呼叫 update() 時以微秒級鎖定完成同步。
  """

  def __init__(
    self,
    services: list[str],
    poll: str | None = None,
    ignore_alive: list[str] | None = None,
    ignore_avg_freq: list[str] | None = None,
    ignore_valid: list[str] | None = None,
    frequency: float | None = None,
  ):
    self.services = list(services)
    self.frame: int = -1
    self.ignore_alive = set(ignore_alive or [])
    self.ignore_avg_freq = set(ignore_avg_freq or [])
    self.ignore_valid = set(ignore_valid or [])

    # 與原生 SubMaster 一致的狀態追蹤字典（使用 dict.fromkeys 符合 Ruff C420）
    self.updated: dict[str, bool] = dict.fromkeys(self.services, False)
    self.seen: dict[str, bool] = dict.fromkeys(self.services, False)
    self.alive: dict[str, bool] = dict.fromkeys(self.services, False)
    self.valid: dict[str, bool] = dict.fromkeys(self.services, True)
    self.recv_time: dict[str, float] = dict.fromkeys(self.services, 0.0)
    self.recv_frame: dict[str, int] = dict.fromkeys(self.services, 0)
    self.logMonoTime: dict[str, int] = dict.fromkeys(self.services, 0)
    self.sock: dict[str, Any] = dict.fromkeys(self.services, None)

    # 初始化各 service 的預設空 Capnp 訊息，確保尚未收到遠端封包前 UI 讀取不報錯
    self.data: dict[str, Any] = {}
    for s in self.services:
      msg = log.Event.new_message()
      try:
        # 一般 Struct 或 Void 型態 union 欄位可直接無參數初始化
        msg.init(s)
      except Exception:
        # 清單型態 union 欄位（如 onroadEvents、pandaStates）必須指定初始長度 0
        msg.init(s, 0)
      self.data[s] = getattr(msg.as_reader(), s)

    # 執行緒安全暫存區：背景執行緒寫入，主執行緒在 update() 取走
    # 格式：{service: (reader_data, logMonoTime, valid, recv_mono)}
    self._staging_lock = threading.Lock()
    self._staging_msgs: dict[str, tuple[Any, int, bool, float]] = {}

    # 判定 service 存活的超時閾值（秒）
    self._alive_timeout = 2.0

    # 綁定全域 WebRTC 客戶端、完成動態訂閱註冊後再啟動連線（確保首輪握手帶入完整清單）
    self._client = RemoteWebRTCClient.get_instance()
    self._client.register_services(self.services)
    self._client.add_capnp_callback(self._on_capnp_bytes)
    self._client.start()

  def _on_capnp_bytes(self, raw_bytes: bytes) -> None:
    """
    [背景執行緒] 收到 C4 推播的原始 Capnp 二進位封包時觸發。
    直接反序列化並僅保留每個 service 最新一筆資料（自動覆蓋過期舊幀）。
    """
    try:
      event = log.Event.from_bytes(raw_bytes)
      service = event.which()
      if service not in self.data:
        return

      service_data = getattr(event, service)
      now = time.monotonic()

      with self._staging_lock:
        self._staging_msgs[service] = (service_data, event.logMonoTime, event.valid, now)
    except Exception:
      # 忽略單一損毀或 schema 不相容封包，避免影響其餘服務
      pass

  def update(self, timeout: int = 100) -> None:
    """
    [主執行緒] 將背景收到的最新遠端狀態同步至 self.data。
    註：timeout 參數為相容原生介面保留，遠端模式下採非阻塞讀取以維持 60 FPS。
    """
    self.frame += 1
    now = time.monotonic()

    # 重置本幀 updated 旗標
    for s in self.services:
      self.updated[s] = False

    # 快速交換暫存區，將鎖定時間壓縮至微秒級
    with self._staging_lock:
      incoming = self._staging_msgs
      self._staging_msgs = {}

    # 更新本幀收到的新資料
    for s, (service_data, log_mono_time, valid, recv_t) in incoming.items():
      self.data[s] = service_data
      self.logMonoTime[s] = log_mono_time
      self.valid[s] = valid
      self.recv_time[s] = recv_t
      self.recv_frame[s] = self.frame
      self.updated[s] = True
      self.seen[s] = True

    # 更新各 service 的 alive 狀態
    for s in self.services:
      if s in self.ignore_alive:
        self.alive[s] = True
      elif self.seen[s] and (now - self.recv_time[s]) < self._alive_timeout:
        self.alive[s] = True
      else:
        self.alive[s] = False


class PubMaster(_native_messaging.PubMaster):
  """
  遠端版 PubMaster（繼承自原生 openpilot.cereal.messaging.PubMaster）。

  設計說明：
  ui.py 會建立 PubMaster(['uiDebug']) 並每幀發送效能統計。
  遠端裝置無本機 msgq 環境，因此覆寫 __init__ 與 send 為安全的 No-op 空操作，
  讓上層 ui.py 程式碼完全不需加上額外的 if 判斷。
  """

  def __init__(self, services: list[str]):
    self.sock: dict[str, Any] = dict.fromkeys(services, None)

  def send(self, s: str, dat: bytes | Any) -> None:
    """遠端模式下略過本機 IPC 發送。"""
    return

  def wait_for_readers_to_update(self, s: str, timeout: int, dt: float = 0.05) -> bool:
    """遠端模式下直接回傳 True，避免阻塞。"""
    return True

  def all_readers_updated(self, s: str) -> bool:
    """遠端模式下直接回傳 True。"""
    return True
