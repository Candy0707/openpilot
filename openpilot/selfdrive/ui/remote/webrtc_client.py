from __future__ import annotations

import asyncio
from collections.abc import Callable
import contextlib
import json
import os
import queue
import threading
import time
from typing import Any
from urllib.parse import urlencode
import urllib.request

from openpilot.selfdrive.ui.remote.video_decoder import (
  DecodedFrame,
  DecoderFactory,
  H264RtpUnpacker,
  RemoteVideoPipeline,
  VideoDecoderBackend,
  create_default_video_decoder,
)

# 型別別名：服務資料回呼 (service_name, data_dict, log_mono_time, valid)
ServiceCallback = Callable[[str, Any, int, bool], None]
ControlCallback = Callable[[dict[str, Any]], None]


class RemoteWebRTCClient:
  """
  遠端 UI 專用的 WebRTC 客戶端單例（完全沿用原生 webrtcd JSON 協議，零 pycapnp / 零 PyAV 依賴）。

  架構說明：
  1. 主執行緒由 pyray (ui.py) 獨佔執行 60 FPS 渲染迴圈。
  2. 背景執行緒職責切分（徹底消除 GIL 與 libdatachannel 互鎖）：
     - RemoteWebRTCThread：運行 asyncio 事件迴圈處理信令、連線與自動退避重連。
     - RemoteControlSenderThread：所有上行控制指令統一走非阻塞 HTTP POST /api/control，
       讓 WebRTC DataChannel 專職單向高速下行推播，徹底根除 nanobind ch.send() 死鎖。
     - RemoteVideoPipeline（於 video_decoder.py）：獨立處理 RTP 拆包與內建 FFmpeg NV12 解碼。
  """

  _instance: RemoteWebRTCClient | None = None
  _instance_lock = threading.Lock()
  # 使用 staticmethod 包裝一般工廠函式，避免透過 self._decoder_factory 存取時
  # 觸發 Python 描述符綁定將 self 誤塞為第一個位置參數
  _decoder_factory: DecoderFactory = staticmethod(create_default_video_decoder)  # type: ignore[assignment]

  # 原生系統控制訊息 type，不應視為 cereal 服務封包分派給 SubMaster
  _SYSTEM_MSG_TYPES = frozenset({
    "clockSync",
    "disconnect",
    "livestreamCameraSwitch",
    "livestreamSettings",
    "livestreamVideoEnable",
    "enableTimingSei",
  })

  @classmethod
  def get_instance(cls) -> RemoteWebRTCClient:
    """取得全域唯一的 WebRTC 客戶端實例，確保 SubMaster、Params 與 Camera 共用同一條連線。"""
    with cls._instance_lock:
      if cls._instance is None:
        host = os.getenv("REMOTE_HOST", "127.0.0.1")
        port = int(os.getenv("REMOTE_PORT", "5001"))
        cls._instance = cls(host=host, port=port)
      return cls._instance

  @classmethod
  def reset_instance(cls) -> None:
    """重設單例（供測試或切換目標主機時使用）。"""
    with cls._instance_lock:
      if cls._instance is not None:
        cls._instance.stop()
        cls._instance = None

  @classmethod
  def set_video_decoder_factory(cls, factory: DecoderFactory) -> None:
    """允許行動端 App 框架於啟動前注入原生硬體解碼器（MediaCodec / VideoToolbox）。"""
    # 同樣以 staticmethod 包裝，確保外部注入一般函式或類別時皆不會被綁定 self
    cls._decoder_factory = staticmethod(factory)  # type: ignore[assignment]
    if cls._instance is not None:
      cls._instance._video.set_decoder_factory(factory)

  def __init__(self, host: str, port: int = 5001):
    self.host = host
    self.port = port
    self.base_url = f"http://{self.host}:{self.port}"
    self.signaling_url = f"{self.base_url}/stream"
    self.control_url = f"{self.base_url}/api/control"

    # 連線與三段式就緒閘門狀態
    self._running = False
    self._connected = False
    self._connected_event = threading.Event()
    self._params_ready_event = threading.Event()
    self._data_lock = threading.Lock()
    self._received_services: set[str] = set()
    self._data_updated_cond = threading.Condition(self._data_lock)

    self._loop: asyncio.AbstractEventLoop | None = None
    self._stop_event: asyncio.Event | None = None
    self._thread: threading.Thread | None = None
    self._stream: Any = None

    # 獨立 HTTP POST 上行控制執行緒與佇列（避開 libdatachannel DataChannel.send 互鎖）
    self._control_queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=256)
    self._control_thread: threading.Thread | None = None

    # 動態訂閱的 cereal 服務集合（由 RemoteSubMaster 註冊）
    self._services_lock = threading.Lock()
    self._subscribed_services: set[str] = set()

    # 回呼函式清單
    self._service_callbacks: list[ServiceCallback] = []
    self._control_callbacks: list[ControlCallback] = []

    # 視訊拆包與解碼獨立管線（預設於 Offroad 關閉，進入 Onroad 才啟用）
    self._video = RemoteVideoPipeline(decoder_factory=self._decoder_factory)
    self._current_camera = "road"
    self._video_track: Any = None
    self._video_track_attached = False

  # =========================================================================
  # 外部註冊與控制介面（由主執行緒安全呼叫）
  # =========================================================================

  def register_services(self, services: list[str]) -> None:
    """
    註冊需要動態訂閱的 cereal service 清單。
    若在已連線狀態下追加新服務，立即透過非阻塞 HTTP 控制端點發送 subscribe_services 補訂。
    """
    with self._services_lock:
      new_services = sorted(set(services) - self._subscribed_services)
      self._subscribed_services.update(services)

    if new_services and self._connected:
      self.send_control_command({
        "cmd": "subscribe_services",
        "services": new_services,
      })

  def add_service_callback(self, callback: ServiceCallback) -> None:
    """註冊原生 JSON cereal 封包接收回呼（通常由 RemoteSubMaster 註冊）。"""
    self._service_callbacks.append(callback)

  def add_control_callback(self, callback: ControlCallback) -> None:
    """註冊 JSON 控制訊息回呼（通常由 RemoteParams 註冊）。"""
    self._control_callbacks.append(callback)

  def fetch_enums(self, services: list[str], timeout: float = 3.0) -> dict[str, Any]:
    """向 C4 GET /api/enums 端點拉取指定服務的真實 Cap'n Proto 列舉編號對照表。"""
    if not services:
      return {}
    query = urlencode({"services": ",".join(services)})
    url = f"{self.base_url}/api/enums?{query}"
    try:
      req = urllib.request.Request(url, method="GET")
      with urllib.request.urlopen(req, timeout=timeout) as resp:
        if resp.status == 200:
          data = json.loads(resp.read().decode("utf-8"))
          return data if isinstance(data, dict) else {}
    except Exception:
      pass
    return {}

  def fetch_schema(self, services: list[str], timeout: float = 3.0) -> dict[str, Any]:
    """向 C4 原生 GET /schema 端點拉取指定服務的真實結構定義（用於建立未收包前的零值骨架）。"""
    if not services:
      return {}
    query = urlencode({"services": ",".join(services)})
    url = f"{self.base_url}/schema?{query}"
    try:
      req = urllib.request.Request(url, method="GET")
      with urllib.request.urlopen(req, timeout=timeout) as resp:
        if resp.status == 200:
          data = json.loads(resp.read().decode("utf-8"))
          return data if isinstance(data, dict) else {}
    except Exception:
      pass
    return {}

  def start(self) -> None:
    """啟動連線監管、上行控制與獨立視訊解碼背景執行緒。"""
    if self._running:
      return
    self._running = True

    self._control_thread = threading.Thread(
      target=self._control_sender_loop,
      name="RemoteControlSenderThread",
      daemon=True,
    )
    self._control_thread.start()

    self._video.start()

    self._thread = threading.Thread(
      target=self._run_event_loop,
      name="RemoteWebRTCThread",
      daemon=True,
    )
    self._thread.start()

  def stop(self) -> None:
    """停止所有背景執行緒並優雅關閉 WebRTC 連線與解碼管線（可安全重複呼叫）。"""
    self._running = False
    self._connected = False
    self._connected_event.clear()
    self._params_ready_event.clear()
    with self._data_updated_cond:
      self._data_updated_cond.notify_all()

    self._drain_control_queue()
    with contextlib.suppress(queue.Full):
      self._control_queue.put_nowait(None)

    self._video.stop()

    loop = self._loop
    if loop is not None and not loop.is_closed():
      if self._stop_event is not None:
        with contextlib.suppress(Exception):
          loop.call_soon_threadsafe(self._stop_event.set)
      if self._stream is not None:
        with contextlib.suppress(Exception):
          asyncio.run_coroutine_threadsafe(self._stream.stop(), loop)

    cur = threading.current_thread()
    for t in (self._control_thread, self._thread):
      if t is not None and t.is_alive() and t is not cur:
        t.join(timeout=2.0)

  def _drain_control_queue(self) -> None:
    """非阻塞排空控制指令佇列。"""
    while True:
      try:
        self._control_queue.get_nowait()
      except queue.Empty:
        break

  def wait_until_ready(
    self,
    required_services: list[str] | None = None,
    poll_interval: float = 0.2,
    timeout: float | None = None,
  ) -> bool:
    """
    三段式開窗閘門（未達成前持續阻塞，不往下開窗）：
    1. WebRTC PeerConnection 與 DataChannel 已連通。
    2. 已收到 C4 回傳的第一份真實 all_params 快照。
    3. 已收到 required_services（預設 ['deviceState']）的第一筆真實資料。

    若 timeout 為 None（正式 UI 模式），會無限等待直到三段條件全數滿足；
    若指定 timeout（例如測試模式），逾時回傳 False。
    """
    req_services = list(required_services) if required_services is not None else ["deviceState"]
    if req_services:
      self.register_services(req_services)

    if not self._running:
      self.start()

    deadline = (time.monotonic() + timeout) if timeout is not None else None
    last_stage: str | None = None

    while self._running:
      # 檢查目前停在哪一個階段
      if not self._connected_event.is_set():
        stage = f"等待 WebRTC 通道連線 ({self.host}:{self.port})"
      elif not self._params_ready_event.is_set():
        stage = "已連線，等待 C4 同步真實 Params 快照"
      else:
        with self._data_lock:
          missing = [s for s in req_services if s not in self._received_services]
        if not missing:
          print(f"[RemoteWebRTC] 連線、真實 Params 與核心資料 ({', '.join(req_services)}) 皆已就緒，放行開窗")
          return True
        stage = f"已同步 Params，等待核心服務真實資料 ({', '.join(missing)})"

      if stage != last_stage:
        print(f"[RemoteWebRTC] {stage}...")
        last_stage = stage

      wait_time = poll_interval
      if deadline is not None:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
          return False
        wait_time = min(wait_time, remaining)

      with self._data_updated_cond:
        self._data_updated_cond.wait(timeout=wait_time)

    return False

  @property
  def is_connected(self) -> bool:
    """回傳目前 WebRTC 是否處於已連線狀態。"""
    return self._connected

  @property
  def video_enabled(self) -> bool:
    """回傳目前是否已開啟道路視訊串流（Onroad 狀態）。"""
    return self._video.enabled

  @property
  def current_camera(self) -> str:
    """回傳目前請求串流中的遠端鏡頭名稱（road / wideRoad / driver）。"""
    return self._current_camera

  def set_video_enabled(self, enabled: bool) -> None:
    """
    切換道路視訊串流開關（配合進 Onroad 再顯示道路串流的設計）：
    - enabled=True：送出原生 livestreamVideoEnable(True) 並啟動解碼管線。
    - enabled=False：送出原生 livestreamVideoEnable(False)、停止解碼器並清空舊畫面。
    """
    enabled = bool(enabled)
    changed = self._video.set_enabled(enabled)
    if enabled and self._connected and self._video_track is not None:
      self._video.ensure_started_if_enabled()

    if self._connected and changed:
      self.send_control_command({
        "type": "livestreamVideoEnable",
        "data": {"enabled": enabled},
      })

  def switch_camera(self, camera: str) -> None:
    """透過原生 livestreamCameraSwitch 指令切換遠端鏡頭（road / wideRoad / driver）。"""
    if camera == self._current_camera:
      return
    self._current_camera = camera
    self._video.reset_stream()

    if self._connected:
      self.send_control_command({
        "type": "livestreamCameraSwitch",
        "data": {"camera": camera},
      })

  def get_latest_frame(self) -> tuple[DecodedFrame | None, int, float]:
    """
    取得最新一張解碼後的 NV12 道路影像幀（供 RemoteVisionIpcClient 轉交 CameraView 更新 Texture）。
    回傳值：(decoded_frame, frame_id, timestamp)
    """
    return self._video.get_latest_frame()

  def send_control_command(self, payload: dict[str, Any]) -> bool:
    """
    執行緒安全地將控制指令放入獨立 HTTP POST 發送佇列（POST /api/control）：
    - 不呼叫 DataChannel.send()，徹底避免 nanobind / libdatachannel 的 GIL 互鎖死鎖。
    - 絕不阻塞主執行緒；若尚未連線則回傳 False。
    """
    if not self._running or not self._connected:
      return False

    try:
      self._control_queue.put_nowait(payload)
      return True
    except queue.Full:
      with contextlib.suppress(queue.Empty):
        self._control_queue.get_nowait()
      with contextlib.suppress(queue.Full):
        self._control_queue.put_nowait(payload)
      return True

  # =========================================================================
  # 獨立 HTTP POST 控制發送執行緒
  # =========================================================================

  def _post_control(self, payload: dict[str, Any], timeout: float = 2.0) -> bool:
    """透過 HTTP POST /api/control 送出單筆控制指令。"""
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
      self.control_url,
      data=data,
      headers={"Content-Type": "application/json"},
      method="POST",
    )
    try:
      with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status == 200
    except Exception:
      return False

  def _control_sender_loop(self) -> None:
    """背景執行緒：循序以 HTTP POST 發送上行控制指令，與 WebRTC DataChannel 完全解耦。"""
    while self._running:
      try:
        payload = self._control_queue.get(timeout=0.5)
      except queue.Empty:
        continue
      if payload is None:
        break
      if not self._running or not self._connected:
        continue
      self._post_control(payload)

  # =========================================================================
  # 背景 asyncio 連線與斷線重連迴圈
  # =========================================================================

  def _run_event_loop(self) -> None:
    """背景執行緒入口：建立獨立的 asyncio Event Loop 與停止事件。"""
    self._loop = asyncio.new_event_loop()
    asyncio.set_event_loop(self._loop)
    self._stop_event = asyncio.Event()
    try:
      self._loop.run_until_complete(self._connection_supervisor())
    finally:
      self._loop.close()

  async def _connection_supervisor(self) -> None:
    """連線監管器：負責建立 WebRTC 連線，並在斷線時自動退避重連。"""
    retry_delay = 1.0
    while self._running:
      try:
        await self._connect_once()
        retry_delay = 1.0  # 連線成功過則重置退避時間
      except Exception as e:
        print(f"[RemoteWebRTC] 連線異常 ({self.signaling_url}): {e}")
      finally:
        self._connected = False
        self._connected_event.clear()
        self._video_track = None
        self._video_track_attached = False
        self._video.stop_decoder(clear_frame=True)
        if self._stream is not None:
          with contextlib.suppress(Exception):
            await self._stream.stop()
          self._stream = None

      if self._running:
        print(f"[RemoteWebRTC] {retry_delay:.1f} 秒後嘗試重新連線...")
        await asyncio.sleep(retry_delay)
        retry_delay = min(retry_delay * 1.5, 5.0)

  @staticmethod
  def _post_signaling(url: str, payload: dict[str, Any]) -> dict[str, Any]:
    """
    使用 Python 內建 urllib.request 發送 HTTP POST 信令請求。
    透過 asyncio.to_thread() 呼叫以避免阻塞事件迴圈。
    """
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
      url,
      data=data,
      headers={"Content-Type": "application/json"},
      method="POST",
    )
    with urllib.request.urlopen(req, timeout=5.0) as resp:
      if resp.status != 200:
        raise RuntimeError(f"webrtcd_ext 信令回應錯誤 HTTP {resp.status}")
      return json.loads(resp.read().decode("utf-8"))

  def _on_messaging_data(self, message: bytes | str) -> None:
    """
    單一 JSON messaging_channel 接收分流器：
    - 帶有 "cmd" 欄位：控制與 Params 同步訊息 → 分派給 RemoteParams，並於收到 all_params 時點亮閘門。
    - 帶有 "type" 與 "data" 欄位（原生 webrtcd 格式）：cereal 服務封包 → 分派給 RemoteSubMaster。
    """
    try:
      if isinstance(message, bytes):
        text = message.decode("utf-8", errors="ignore")
      elif isinstance(message, str):
        text = message
      else:
        return
      data = json.loads(text)
    except (json.JSONDecodeError, UnicodeError):
      return

    if not isinstance(data, dict):
      return

    # 1. 控制與 Params 同步訊息
    if "cmd" in data:
      for cb in self._control_callbacks:
        cb(data)
      if data.get("cmd") == "all_params":
        self._params_ready_event.set()
        with self._data_updated_cond:
          self._data_updated_cond.notify_all()
      return

    # 2. 原生 webrtcd cereal 服務 JSON 封包
    msg_type = data.get("type")
    if isinstance(msg_type, str) and msg_type not in self._SYSTEM_MSG_TYPES and "data" in data:
      payload = data["data"]
      log_mono_time = int(data.get("logMonoTime", 0))
      valid = bool(data.get("valid", True))

      for cb in self._service_callbacks:
        cb(msg_type, payload, log_mono_time, valid)

      with self._data_updated_cond:
        if msg_type not in self._received_services:
          self._received_services.add(msg_type)
        self._data_updated_cond.notify_all()

  def _attach_video_track(self, track: Any) -> None:
    """將 incoming video track 接收回呼直接串接至 RemoteVideoPipeline 的非阻塞入列函式。"""
    self._video_track = track
    if self._video_track_attached:
      return

    track.on_message(self._video.on_rtp_packet)
    self._video_track_attached = True
    self._video.ensure_started_if_enabled()

  async def _connect_once(self) -> None:
    """使用原生 teleoprtc WebRTCOfferBuilder 執行單次連線並維持直到中斷。"""
    from teleoprtc.builder import WebRTCOfferBuilder
    from teleoprtc.stream import RTCSessionDescription, StreamingOffer

    assert self._stop_event is not None

    with self._services_lock:
      services_list = sorted(self._subscribed_services)

    async def signaling_provider(offer: StreamingOffer) -> RTCSessionDescription:
      """將 teleoprtc 產生的 StreamingOffer 透過 HTTP POST 送往 C4 的 webrtcd_ext。"""
      request_body = {
        "sdp": offer.sdp,
        "cameras": list(offer.video) if offer.video else [self._current_camera],
        "enabled": True,
        "bridge_services_in": [],
        "bridge_services_out": services_list,
        "remote_ui": True,                      # 啟用不逾時與獨立影像開關模式
        "video_enabled": self._video.enabled,   # Offroad 預設 False，進 Onroad 再啟動影像
        "max_hz": 25.0,                         # 逐服務節流上限 25 Hz，平衡流暢度與區網頻寬
      }
      answer_data = await asyncio.to_thread(self._post_signaling, self.signaling_url, request_body)
      return RTCSessionDescription(
        sdp=answer_data["sdp"],
        type=answer_data["type"],
      )

    # 1. 建立 teleoprtc Offer 串流（宣告道路鏡頭軌道 + 雙向 messaging 通道）
    builder = WebRTCOfferBuilder(signaling_provider)
    builder.offer_to_receive_video_stream(self._current_camera)
    builder.add_messaging()
    stream = builder.stream()
    self._stream = stream

    # 2. 在連線建立前先掛載訊息處理器，避免漏接初期封包
    stream.set_message_handler(self._on_messaging_data)

    # 3. 啟動信令交換並等待連線完成
    await stream.start()
    await asyncio.wait_for(stream.wait_for_connection(), timeout=15.0)
    self._connected = True
    self._connected_event.set()
    with self._data_updated_cond:
      self._data_updated_cond.notify_all()
    print(f"[RemoteWebRTC] 已連線至 C4 ({self.host}:{self.port})")

    # 4. 連線成功後透過 HTTP POST /api/control 請求同步初始 Params 快照與補訂服務
    #    （全程不呼叫 DataChannel.send()，避免與下行推播搶鎖）
    await asyncio.to_thread(self._post_control, {"cmd": "get_all_params"})
    with self._services_lock:
      extra_services = sorted(self._subscribed_services - set(services_list))
    if extra_services:
      await asyncio.to_thread(
        self._post_control,
        {"cmd": "subscribe_services", "services": extra_services},
      )

    # 5. 掛載道路鏡頭軌道（解碼器會等進入 Onroad 開啟視訊後才真正啟動）
    if stream.has_incoming_video_track(self._current_camera):
      video_track = stream.get_incoming_video_track(self._current_camera)
      self._attach_video_track(video_track)

    # 6. 以 asyncio.Event 等待斷線或外部停止訊號
    disconnect_task = asyncio.create_task(stream.wait_for_disconnection())
    stop_task = asyncio.create_task(self._stop_event.wait())
    _, pending = await asyncio.wait(
      {disconnect_task, stop_task},
      return_when=asyncio.FIRST_COMPLETED,
    )
    for task in pending:
      task.cancel()


__all__ = [
  "ControlCallback",
  "DecodedFrame",
  "DecoderFactory",
  "H264RtpUnpacker",
  "RemoteWebRTCClient",
  "ServiceCallback",
  "VideoDecoderBackend",
]
