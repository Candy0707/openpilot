from __future__ import annotations

from abc import ABC, abstractmethod
import asyncio
from collections.abc import Callable
import contextlib
import json
import os
import shutil
import subprocess
import threading
import time
from typing import Any
from urllib.parse import urlencode
import urllib.request

import numpy as np


class VideoDecoderBackend(ABC):
  """
  跨平台 H.264 視訊解碼器抽象介面：
  - 桌面端（Linux / WSL / Windows / macOS）：預設使用 FFmpegSubprocessDecoder。
  - 行動端（Android / iOS App 框架）：於啟動 ui.py 前注入原生 MediaCodec / VideoToolbox 解碼器。
  """

  @abstractmethod
  def feed(self, nal_data: bytes) -> None:
    """餵入重組後的 H.264 NAL Unit 資料。"""

  @abstractmethod
  def stop(self) -> None:
    """停止解碼器並釋放資源。"""


class FFmpegSubprocessDecoder(VideoDecoderBackend):
  """桌面端預設 H.264 -> RGB24 低延遲解碼管線（零 PyAV 依賴，直接呼叫系統 ffmpeg/ffprobe）。"""

  def __init__(self, on_frame: Callable[[np.ndarray], None]):
    self._on_frame = on_frame
    self._running = True
    self._proc: subprocess.Popen[bytes] | None = None
    self._lock = threading.Lock()
    self._probe_buf = bytearray()
    self._reader_thread: threading.Thread | None = None

  @staticmethod
  def _probe_h264_dimensions(h264_sample: bytes) -> tuple[int, int] | None:
    """
    使用系統 ffprobe 從累積的 H.264 樣本資料（含 SPS/PPS）解析實際視訊解析度。
    若尚未收到 SPS 標頭則回傳 None 繼續累積。
    """
    ffprobe_bin = shutil.which("ffprobe")
    if not ffprobe_bin:
      # 若無 ffprobe，退回 comma 3X road 串流標準預設解析度
      return (1928, 1208)

    cmd = [
      ffprobe_bin,
      "-v", "error",
      "-f", "h264",
      "-select_streams", "v:0",
      "-show_entries", "stream=width,height",
      "-of", "json",
      "-",
    ]
    try:
      res = subprocess.run(cmd, input=h264_sample, capture_output=True, timeout=2.0)
      if res.returncode != 0:
        return None
      info = json.loads(res.stdout.decode("utf-8", errors="ignore"))
      streams = info.get("streams", [])
      if streams and streams[0].get("width") and streams[0].get("height"):
        return int(streams[0]["width"]), int(streams[0]["height"])
    except Exception:
      pass
    return None

  def _start_ffmpeg(self, width: int, height: int) -> None:
    """啟動低延遲 ffmpeg 子程序，從 stdin 接收 H.264 並於 stdout 輸出 raw RGB24 畫面。"""
    ffmpeg_bin = shutil.which("ffmpeg")
    if not ffmpeg_bin:
      print("[RemoteWebRTC] 找不到 ffmpeg 執行檔，無法解碼視訊串流")
      return

    cmd = [
      ffmpeg_bin,
      "-loglevel", "error",
      "-fflags", "nobuffer",
      "-flags", "low_delay",
      "-f", "h264",
      "-i", "pipe:0",
      "-pix_fmt", "rgb24",
      "-f", "rawvideo",
      "pipe:1",
    ]
    proc = subprocess.Popen(
      cmd,
      stdin=subprocess.PIPE,
      stdout=subprocess.PIPE,
      stderr=subprocess.DEVNULL,
      bufsize=0,
    )
    self._proc = proc

    def _read_stdout_frames() -> None:
      """背景執行緒：持續從 ffmpeg stdout 讀取完整 RGB24 畫面幀並回報上層。"""
      frame_bytes = width * height * 3
      stdout = proc.stdout
      if stdout is None:
        return

      while self._running and proc.poll() is None:
        try:
          raw = stdout.read(frame_bytes)
          if not raw or len(raw) < frame_bytes:
            break
          rgb_array = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 3)).copy()
          self._on_frame(rgb_array)
        except Exception:
          break

    self._reader_thread = threading.Thread(
      target=_read_stdout_frames,
      name="RemoteFFmpegReader",
      daemon=True,
    )
    self._reader_thread.start()

  def feed(self, nal_data: bytes) -> None:
    if not self._running or not nal_data:
      return

    with self._lock:
      # 階段一：尚未啟動 ffmpeg 前，先累積初期資料探測實際解析度（避免硬編碼寬高造成畫面錯位）
      if self._proc is None:
        self._probe_buf.extend(nal_data)
        # 限制探測緩衝上限 512 KB，防止未收到 SPS 時無限膨脹
        if len(self._probe_buf) > 512 * 1024:
          del self._probe_buf[: -256 * 1024]

        dims = self._probe_h264_dimensions(bytes(self._probe_buf))
        if dims is None:
          return

        width, height = dims
        self._start_ffmpeg(width, height)
        if self._proc is None or self._proc.stdin is None:
          return

        # 將探測期間累積的完整初始序列（含 SPS/PPS/IDR）一併寫入 ffmpeg
        initial_payload = bytes(self._probe_buf)
        self._probe_buf.clear()
        try:
          self._proc.stdin.write(initial_payload)
          self._proc.stdin.flush()
        except Exception:
          self._proc = None
        return

      # 階段二：持續將後續 H.264 NAL Unit 寫入 ffmpeg stdin
      proc = self._proc
      if proc is not None and proc.stdin is not None and proc.poll() is None:
        try:
          proc.stdin.write(nal_data)
          proc.stdin.flush()
        except Exception:
          self._proc = None

  def stop(self) -> None:
    """安全關閉並清理 ffmpeg 子程序與探測緩衝區。"""
    self._running = False
    with self._lock:
      proc = self._proc
      self._proc = None
      self._probe_buf.clear()

    if proc is not None:
      with contextlib.suppress(Exception):
        if proc.stdin:
          proc.stdin.close()
        proc.terminate()
        proc.wait(timeout=1.0)
      if proc.poll() is None:
        with contextlib.suppress(Exception):
          proc.kill()


# 型別別名：服務資料回呼 (service_name, data_dict, log_mono_time, valid)
ServiceCallback = Callable[[str, Any, int, bool], None]
ControlCallback = Callable[[dict[str, Any]], None]
DecoderFactory = Callable[[Callable[[np.ndarray], None]], VideoDecoderBackend]


class RemoteWebRTCClient:
  """
  遠端 UI 專用的 WebRTC 客戶端單例（完全沿用原生 webrtcd JSON 協議，零 pycapnp 依賴）。

  架構說明：
  1. 主執行緒由 pyray (ui.py) 獨佔執行 60 FPS 渲染迴圈。
  2. 本類別啟動獨立的 daemon 背景執行緒運行 asyncio 事件迴圈，處理所有網路 I/O。
  3. 透過原生 teleoprtc 與 C4 的 webrtcd_ext 建立連線，並分流處理影像與 JSON 訊息：
     - Video Track ("road")：預設於 Offroad 關閉，進入 Onroad 後才開啟並交由解碼後端輸出 RGB24。
     - messaging_channel (JSON)：統一承載原生 cereal 服務資料（{"type", "logMonoTime", "valid", "data"}）
       與雙向控制指令（Params 初始快照／讀寫、動態補訂服務、視訊與鏡頭切換）。
  """

  _instance: RemoteWebRTCClient | None = None
  _instance_lock = threading.Lock()
  _decoder_factory: DecoderFactory = FFmpegSubprocessDecoder

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
    cls._decoder_factory = factory

  def __init__(self, host: str, port: int = 5001):
    self.host = host
    self.port = port
    self.base_url = f"http://{self.host}:{self.port}"
    self.signaling_url = f"{self.base_url}/stream"

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

    # 動態訂閱的 cereal 服務集合（由 RemoteSubMaster 註冊）
    self._services_lock = threading.Lock()
    self._subscribed_services: set[str] = set()

    # 回呼函式清單
    self._service_callbacks: list[ServiceCallback] = []
    self._control_callbacks: list[ControlCallback] = []

    # 影像雙緩衝區與 Onroad 串流開關（預設關閉，進入 Onroad 才啟用）
    self._frame_lock = threading.Lock()
    self._latest_frame: np.ndarray | None = None
    self._frame_id: int = 0
    self._frame_timestamp: float = 0.0
    self._video_enabled = False
    self._current_camera = "road"
    self._decoder: VideoDecoderBackend | None = None
    self._decoder_lock = threading.Lock()
    self._video_track: Any = None
    self._video_track_attached = False

  # =========================================================================
  # 外部註冊與控制介面（由主執行緒安全呼叫）
  # =========================================================================

  def register_services(self, services: list[str]) -> None:
    """
    註冊需要動態訂閱的 cereal service 清單。
    若在已連線狀態下追加新服務，立即透過 DataChannel 發送 subscribe_services 補訂。
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
    """啟動背景執行緒並開始連線至 C4。"""
    if self._running:
      return
    self._running = True
    self._thread = threading.Thread(
      target=self._run_event_loop,
      name="RemoteWebRTCThread",
      daemon=True,
    )
    self._thread.start()

  def stop(self) -> None:
    """停止背景執行緒並優雅關閉 WebRTC 連線與解碼器。"""
    self._running = False
    self._connected = False
    self._connected_event.clear()
    self._params_ready_event.clear()
    with self._data_updated_cond:
      self._data_updated_cond.notify_all()

    self._stop_video_decoder(clear_frame=True)
    if self._loop:
      if self._stop_event:
        self._loop.call_soon_threadsafe(self._stop_event.set)
      if self._stream is not None:
        with contextlib.suppress(Exception):
          asyncio.run_coroutine_threadsafe(self._stream.stop(), self._loop)
    if self._thread and self._thread.is_alive():
      self._thread.join(timeout=2.0)

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
    return self._video_enabled

  def set_video_enabled(self, enabled: bool) -> None:
    """
    切換道路視訊串流開關（配合進 Onroad 再顯示道路串流的設計）：
    - enabled=True：送出原生 livestreamVideoEnable(True) 並啟動解碼器。
    - enabled=False：送出原生 livestreamVideoEnable(False)、停止解碼器並清空舊畫面。
    """
    enabled = bool(enabled)
    changed = (self._video_enabled != enabled)
    self._video_enabled = enabled

    if not enabled:
      self._stop_video_decoder(clear_frame=True)
    elif self._connected and self._video_track is not None:
      self._ensure_video_decoder_started()

    if self._connected and changed:
      self.send_control_command({
        "type": "livestreamVideoEnable",
        "data": {"enabled": enabled},
      })
      if enabled and self._video_track is not None:
        with contextlib.suppress(Exception):
          self._video_track.request_keyframe()

  def switch_camera(self, camera: str) -> None:
    """透過原生 livestreamCameraSwitch 指令切換遠端鏡頭（road / wideRoad / driver）。"""
    if camera == self._current_camera:
      return
    self._current_camera = camera
    if self._connected:
      self.send_control_command({
        "type": "livestreamCameraSwitch",
        "data": {"camera": camera},
      })

  def get_latest_frame(self) -> tuple[np.ndarray | None, int, float]:
    """
    取得最新一張解碼後的道路影像幀（供 CameraView 更新 pyray Texture）。
    回傳值：(rgb24_ndarray, frame_id, timestamp)
    """
    with self._frame_lock:
      return self._latest_frame, self._frame_id, self._frame_timestamp

  def send_control_command(self, payload: dict[str, Any]) -> bool:
    """
    執行緒安全地透過 teleoprtc messaging_channel 送出 JSON 指令給 C4。
    若尚未連線則回傳 False，絕不阻塞主執行緒。
    """
    if not self._loop or not self._connected or self._stream is None:
      return False

    message = json.dumps(payload)

    def _send() -> None:
      if self._stream is not None and self._stream.has_messaging_channel():
        ch = self._stream.get_messaging_channel()
        if ch.is_open():
          ch.send(message)

    self._loop.call_soon_threadsafe(_send)
    return True

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
        self._stop_video_decoder(clear_frame=True)
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

  # =========================================================================
  # 視訊軌道與可抽換解碼後端管理
  # =========================================================================

  def _on_decoded_frame(self, rgb_array: np.ndarray) -> None:
    """解碼後端回呼：將最新 RGB24 畫面幀寫入執行緒安全雙緩衝區。"""
    if not self._video_enabled:
      return
    now = time.monotonic()
    with self._frame_lock:
      self._latest_frame = rgb_array
      self._frame_id += 1
      self._frame_timestamp = now

  def _ensure_video_decoder_started(self) -> None:
    """確保目前存在運作中的解碼後端實例。"""
    with self._decoder_lock:
      if self._decoder is None:
        self._decoder = self._decoder_factory(self._on_decoded_frame)

  def _stop_video_decoder(self, clear_frame: bool = False) -> None:
    """停止解碼後端，並視需要清除舊畫面（例如離開 Onroad 時）。"""
    with self._decoder_lock:
      decoder = self._decoder
      self._decoder = None
    if decoder is not None:
      with contextlib.suppress(Exception):
        decoder.stop()

    if clear_frame:
      with self._frame_lock:
        self._latest_frame = None
        self._frame_id = 0
        self._frame_timestamp = 0.0

  def _attach_video_track(self, track: Any) -> None:
    """
    將 libdatachannel 的 H264RtpDepacketizer 掛載至 incoming video track；
    僅在 _video_enabled 為 True（Onroad 狀態）時才將 NAL Unit 餵入解碼器。
    """
    from libdatachannel import H264RtpDepacketizer, NalUnit

    self._video_track = track
    if self._video_track_attached:
      return

    depacketizer = H264RtpDepacketizer(NalUnit.Separator.LongStartSequence)
    track.set_media_handler(depacketizer)

    def on_h264_frame(data: bytes | str) -> None:
      if not self._running or not self._video_enabled or not isinstance(data, bytes) or not data:
        return
      self._ensure_video_decoder_started()
      with self._decoder_lock:
        decoder = self._decoder
      if decoder is not None:
        decoder.feed(data)

    track.on_message(on_h264_frame)
    self._video_track_attached = True

    if self._video_enabled:
      self._ensure_video_decoder_started()
      with contextlib.suppress(Exception):
        track.request_keyframe()

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
        "remote_ui": True,                 # 啟用不逾時與獨立影像開關模式
        "video_enabled": self._video_enabled,  # Offroad 預設 False，進 Onroad 再啟動影像
        "max_hz": 25.0,                    # 逐服務節流上限 25 Hz，平衡流暢度與區網頻寬
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

    # 4. 連線成功後立即請求同步初始 Params 快照，並補訂連線期間新增的服務
    if stream.has_messaging_channel():
      ch = stream.get_messaging_channel()
      if ch.is_open():
        ch.send(json.dumps({"cmd": "get_all_params"}))
        with self._services_lock:
          extra_services = sorted(self._subscribed_services - set(services_list))
        if extra_services:
          ch.send(json.dumps({"cmd": "subscribe_services", "services": extra_services}))

    # 5. 掛載道路鏡頭軌道（解碼器會等 _video_enabled=True 即進入 Onroad 才真正啟動）
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