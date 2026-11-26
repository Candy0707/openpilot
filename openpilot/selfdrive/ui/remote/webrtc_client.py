from __future__ import annotations

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
import urllib.request

from libdatachannel import H264RtpDepacketizer, NalUnit
import numpy as np
from teleoprtc.builder import WebRTCOfferBuilder
from teleoprtc.stream import RTCSessionDescription, StreamingOffer


class RemoteWebRTCClient:
  """
  遠端 UI 專用的 WebRTC 客戶端單例（基於 openpilot 內建 teleoprtc 與 libdatachannel）。

  架構說明：
  1. 主執行緒由 pyray (ui.py) 獨佔執行 60 FPS 渲染迴圈。
  2. 本類別啟動獨立的 daemon 背景執行緒運行 asyncio 事件迴圈，處理所有網路 I/O。
  3. 透過 teleoprtc 與 C4 的 webrtcd_ext 建立連線，並分流處理影像與控制資料：
     - Video Track ("road")：透過 libdatachannel H264RtpDepacketizer 重組 H.264 封包，
       並交由系統內建 ffmpeg 低延遲子程序管線解碼為 RGB24 寫入執行緒安全緩衝區。
     - messaging_channel (bytes)：接收 C4 推播的原始 Cap'n Proto 二進位封包（交給 RemoteSubMaster）。
     - messaging_channel (str)：雙向 JSON 控制通道，負責 Params 初始同步與遠端讀寫（交給 RemoteParams）。
  """

  _instance: RemoteWebRTCClient | None = None
  _instance_lock = threading.Lock()

  @classmethod
  def get_instance(cls) -> RemoteWebRTCClient:
    """取得全域唯一的 WebRTC 客戶端實例，確保 SubMaster、Params 與 Camera 共用同一條連線。"""
    with cls._instance_lock:
      if cls._instance is None:
        host = os.getenv("REMOTE_HOST", "127.0.0.1")
        port = int(os.getenv("REMOTE_PORT", "5001"))
        cls._instance = cls(host=host, port=port)
      return cls._instance

  def __init__(self, host: str, port: int = 5001):
    self.host = host
    self.port = port
    self.signaling_url = f"http://{self.host}:{self.port}/stream"

    # 連線狀態與執行緒控制
    self._running = False
    self._connected = False
    self._loop: asyncio.AbstractEventLoop | None = None
    self._stop_event: asyncio.Event | None = None
    self._thread: threading.Thread | None = None
    self._stream: Any = None

    # 動態訂閱的 cereal 服務集合（由 RemoteSubMaster 註冊）
    self._services_lock = threading.Lock()
    self._subscribed_services: set[str] = set()

    # 回呼函式清單
    self._capnp_callbacks: list[Callable[[bytes], None]] = []
    self._control_callbacks: list[Callable[[dict[str, Any]], None]] = []

    # 影像雙緩衝區（儲存最新解碼的 RGB24 numpy 陣列與幀編號）
    self._frame_lock = threading.Lock()
    self._latest_frame: np.ndarray | None = None
    self._frame_id: int = 0
    self._frame_timestamp: float = 0.0

    # FFmpeg H.264 解碼子程序與同步控制
    self._ffmpeg_proc: subprocess.Popen[bytes] | None = None
    self._ffmpeg_lock = threading.Lock()
    self._h264_probe_buf = bytearray()
    self._video_reader_thread: threading.Thread | None = None

  # =========================================================================
  # 外部註冊與控制介面（由主執行緒安全呼叫）
  # =========================================================================

  def register_services(self, services: list[str]) -> None:
    """註冊需要動態訂閱的 cereal service 清單。"""
    with self._services_lock:
      self._subscribed_services.update(services)

  def add_capnp_callback(self, callback: Callable[[bytes], None]) -> None:
    """註冊 Capnp 二進位封包接收回呼（通常由 RemoteSubMaster 註冊）。"""
    self._capnp_callbacks.append(callback)

  def add_control_callback(self, callback: Callable[[dict[str, Any]], None]) -> None:
    """註冊 JSON 控制訊息回呼（通常由 RemoteParams 註冊）。"""
    self._control_callbacks.append(callback)

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
    """停止背景執行緒並優雅關閉 WebRTC 連線與 FFmpeg 解碼管線。"""
    self._running = False
    self._stop_ffmpeg_decoder()
    if self._loop:
      if self._stop_event:
        self._loop.call_soon_threadsafe(self._stop_event.set)
      if self._stream is not None:
        asyncio.run_coroutine_threadsafe(self._stream.stop(), self._loop)
    if self._thread and self._thread.is_alive():
      self._thread.join(timeout=2.0)

  @property
  def is_connected(self) -> bool:
    """回傳目前 WebRTC 是否處於已連線狀態。"""
    return self._connected

  def get_latest_frame(self) -> tuple[np.ndarray | None, int, float]:
    """
    取得最新一張解碼後的道路影像幀（供 CameraView 更新 pyray Texture）。
    回傳值：(rgb24_ndarray, frame_id, timestamp)
    """
    with self._frame_lock:
      return self._latest_frame, self._frame_id, self._frame_timestamp

  def send_control_command(self, payload: dict[str, Any]) -> bool:
    """
    執行緒安全地透過 teleoprtc messaging_channel 送出 JSON 指令給 C4（例如 put_param）。
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
        self._stop_ffmpeg_decoder()
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
    單一 messaging_channel 接收分流器：
    - bytes：C4 推播的原始 Capnp 二進位封包 → 分派給 RemoteSubMaster。
    - str：JSON 控制與 Params 同步訊息 → 分派給 RemoteParams。
    """
    if isinstance(message, bytes):
      for cb in self._capnp_callbacks:
        cb(message)
      return

    if isinstance(message, str):
      try:
        data = json.loads(message)
        if isinstance(data, dict):
          for cb in self._control_callbacks:
            cb(data)
      except json.JSONDecodeError:
        pass

  # =========================================================================
  # FFmpeg 子程序 H.264 -> RGB24 低延遲解碼管線
  # =========================================================================

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

  def _start_ffmpeg_decoder(self, width: int, height: int) -> None:
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
    self._ffmpeg_proc = proc

    def _read_stdout_frames() -> None:
      """背景執行緒：持續從 ffmpeg stdout 讀取完整 RGB24 畫面幀並更新雙緩衝區。"""
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
          now = time.monotonic()
          with self._frame_lock:
            self._latest_frame = rgb_array
            self._frame_id += 1
            self._frame_timestamp = now
        except Exception:
          break

    self._video_reader_thread = threading.Thread(
      target=_read_stdout_frames,
      name="RemoteFFmpegReader",
      daemon=True,
    )
    self._video_reader_thread.start()

  def _stop_ffmpeg_decoder(self) -> None:
    """安全關閉並清理 ffmpeg 子程序與探測緩衝區。"""
    with self._ffmpeg_lock:
      proc = self._ffmpeg_proc
      self._ffmpeg_proc = None
      self._h264_probe_buf.clear()

    if proc is not None:
      with contextlib.suppress(Exception):
        if proc.stdin:
          proc.stdin.close()
        proc.terminate()
        proc.wait(timeout=1.0)
      if proc.poll() is None:
        with contextlib.suppress(Exception):
          proc.kill()

  def _attach_video_decoder(self, track: Any) -> None:
    """
    將 libdatachannel 的 H264RtpDepacketizer 掛載至 incoming video track，
    並將重組後的 H.264 NAL Unit 餵入 ffmpeg 低延遲解碼管線。
    """
    self._stop_ffmpeg_decoder()

    # 掛載 C++ RTP 解封裝器，將 RTP 封包自動重組為帶起始碼的完整 H.264 NAL Unit
    depacketizer = H264RtpDepacketizer(NalUnit.Separator.LongStartSequence)
    track.set_media_handler(depacketizer)

    def on_h264_frame(data: bytes | str) -> None:
      if not self._running or not isinstance(data, bytes) or not data:
        return

      with self._ffmpeg_lock:
        # 階段一：尚未啟動 ffmpeg 前，先累積初期資料探測實際解析度（避免硬編碼寬高造成畫面錯位）
        if self._ffmpeg_proc is None:
          self._h264_probe_buf.extend(data)
          # 限制探測緩衝上限 512 KB，防止未收到 SPS 時無限膨脹
          if len(self._h264_probe_buf) > 512 * 1024:
            del self._h264_probe_buf[: -256 * 1024]

          dims = self._probe_h264_dimensions(bytes(self._h264_probe_buf))
          if dims is None:
            return

          width, height = dims
          self._start_ffmpeg_decoder(width, height)
          if self._ffmpeg_proc is None or self._ffmpeg_proc.stdin is None:
            return

          # 將探測期間累積的完整初始序列（含 SPS/PPS/IDR）一併寫入 ffmpeg
          initial_payload = bytes(self._h264_probe_buf)
          self._h264_probe_buf.clear()
          try:
            self._ffmpeg_proc.stdin.write(initial_payload)
            self._ffmpeg_proc.stdin.flush()
          except Exception:
            self._ffmpeg_proc = None
          return

        # 階段二：持續將後續 H.264 NAL Unit 寫入 ffmpeg stdin
        proc = self._ffmpeg_proc
        if proc is not None and proc.stdin is not None and proc.poll() is None:
          try:
            proc.stdin.write(data)
            proc.stdin.flush()
          except Exception:
            self._ffmpeg_proc = None

    track.on_message(on_h264_frame)
    # 主動請求 C4 送出關鍵幀（IDR），縮短首張畫面出現的等待時間
    with contextlib.suppress(Exception):
      track.request_keyframe()

  async def _connect_once(self) -> None:
    """使用 teleoprtc WebRTCOfferBuilder 執行單次連線並維持直到中斷。"""
    assert self._stop_event is not None

    with self._services_lock:
      services_list = sorted(self._subscribed_services)

    async def signaling_provider(offer: StreamingOffer) -> RTCSessionDescription:
      """將 teleoprtc 產生的 StreamingOffer 透過 HTTP POST 送往 C4 的 webrtcd_ext。"""
      request_body = {
        "sdp": offer.sdp,
        "cameras": list(offer.video) if offer.video else ["road"],
        "enabled": True,
        "bridge_services_in": [],
        "bridge_services_out": services_list,
        "raw_capnp": True,  # 啟用 webrtcd_ext 的原始 Capnp 二進位推播模式
        "max_hz": 25.0,     # 節流上限 25 Hz，平衡流暢度與區網頻寬
      }
      answer_data = await asyncio.to_thread(self._post_signaling, self.signaling_url, request_body)
      return RTCSessionDescription(
        sdp=answer_data["sdp"],
        type=answer_data["type"],
      )

    # 1. 建立 teleoprtc Offer 串流（請求 road 鏡頭 + 雙向 messaging 通道）
    builder = WebRTCOfferBuilder(signaling_provider)
    builder.offer_to_receive_video_stream("road")
    builder.add_messaging()
    stream = builder.stream()
    self._stream = stream

    # 2. 在連線建立前先掛載訊息處理器，避免漏接初期封包
    stream.set_message_handler(self._on_messaging_data)

    # 3. 啟動信令交換並等待連線完成
    await stream.start()
    await asyncio.wait_for(stream.wait_for_connection(), timeout=15.0)
    self._connected = True
    print(f"[RemoteWebRTC] 已連線至 C4 ({self.host}:{self.port})")

    # 4. 連線成功後立即請求同步初始 Params 快照
    if stream.has_messaging_channel():
      ch = stream.get_messaging_channel()
      if ch.is_open():
        ch.send(json.dumps({"cmd": "get_all_params"}))

    # 5. 掛載道路鏡頭 H.264 解碼器（方法名稱為 get_incoming_video_track）
    if stream.has_incoming_video_track("road"):
      video_track = stream.get_incoming_video_track("road")
      self._attach_video_decoder(video_track)

    # 6. 以 asyncio.Event 等待斷線或外部停止訊號（Ruff ASYNC110）
    disconnect_task = asyncio.create_task(stream.wait_for_disconnection())
    stop_task = asyncio.create_task(self._stop_event.wait())
    _, pending = await asyncio.wait(
      {disconnect_task, stop_task},
      return_when=asyncio.FIRST_COMPLETED,
    )
    for task in pending:
      task.cancel()
