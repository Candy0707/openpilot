from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
import contextlib
import errno
import json
import os
import platform
import queue
import shutil
import subprocess
import threading
import time
from typing import Any, NamedTuple

import numpy as np


class DecodedFrame(NamedTuple):
  """
  解碼完成的零拷貝 NV12 視訊影格容器（完全對齊上游 CameraView 所需欄位）：
  - data：一維 C-contiguous uint8 緩衝區（前段為 Y 平面，後段為交錯 UV 平面）。
  - width / height：影像實際像素寬高。
  - stride：單列步距（緊密排列時等於 width）。
  - uv_offset：UV 平面於 data 內的位元組偏移量（等於 stride * height）。
  """

  data: np.ndarray
  width: int
  height: int
  stride: int
  uv_offset: int


class H264RtpUnpacker:
  """
  純 Python RFC 6184 H.264 RTP 拆包與 Access Unit 重組器（零外部依賴）：
  - 解決 libdatachannel on_message 直接送出原始 RTP / RTCP 封包導致解碼器讀不到 Annex-B 的問題。
  - 自動過濾 RTCP 封包（PT 192-223），解析 RTP 標頭、CSRC、擴充標頭與填充位元組。
  - 支援 Single NAL（1-23）、STAP-A（24）與 FU-A（28）分片重組，輸出標準 Annex-B（00 00 00 01）。
  - 依 RTP Timestamp 與 Marker Bit（M=1）將同一影格的多個 NAL（例如 SPS + PPS + IDR）
    合併為單一完整 Access Unit 後再吐出，消除額外解碼延遲。
  - 若輸入資料本身已是 Annex-B 起始碼開頭，直接透傳不重複處理。
  """

  _START_CODE = b"\x00\x00\x00\x01"

  def __init__(self) -> None:
    self._fua_buf = bytearray()
    self._fua_active = False
    self._au_buf = bytearray()
    self._au_ts: int | None = None

  def reset(self) -> None:
    """清空尚未重組完成的 FU-A 分片與 Access Unit 緩衝（切換鏡頭或斷線重連時呼叫）。"""
    self._fua_buf.clear()
    self._fua_active = False
    self._au_buf.clear()
    self._au_ts = None

  def feed(self, packet: bytes) -> list[bytes]:
    """餵入單一封包（原始 RTP 或已拆包 Annex-B），回傳零至多個完整 Annex-B Access Unit。"""
    if not packet:
      return []

    # 若底層已自行完成拆包（已是 Annex-B 起始碼），直接透傳
    if packet.startswith(self._START_CODE) or packet.startswith(b"\x00\x00\x01"):
      return [packet]

    n = len(packet)
    if n < 12:
      return []

    # 檢查 RTP 版本必須為 2（bit 7-6 == 0b10）
    b0 = packet[0]
    if (b0 >> 6) != 2:
      return []

    # 過濾 RTCP 封包（依 RFC 5761，RTCP 類型落在 192-223，常見 SR=200、RR=201、SDES=202）
    b1 = packet[1]
    if 192 <= b1 <= 223:
      return []

    marker = bool(b1 & 0x80)
    rtp_ts = (packet[4] << 24) | (packet[5] << 16) | (packet[6] << 8) | packet[7]

    # 計算 RTP 固定標頭（12B）＋ CSRC 列表（cc * 4B）
    cc = b0 & 0x0F
    offset = 12 + cc * 4
    if offset >= n:
      return []

    # 略過 RTP 擴充標頭（Extension Header）
    if b0 & 0x10:
      if offset + 4 > n:
        return []
      ext_words = (packet[offset + 2] << 8) | packet[offset + 3]
      offset += 4 + ext_words * 4
      if offset >= n:
        return []

    # 扣除尾端填充位元組（Padding）
    end = n
    if b0 & 0x20:
      pad_len = packet[-1]
      if pad_len == 0 or offset + pad_len >= end:
        return []
      end -= pad_len

    payload = packet[offset:end]
    if not payload:
      return []

    completed: list[bytes] = []

    # 若 RTP Timestamp 切換但上一幀漏接 Marker 封包，先吐出上一幀並重置殘缺 FU-A
    if self._au_ts is not None and rtp_ts != self._au_ts:
      if self._au_buf:
        completed.append(bytes(self._au_buf))
        self._au_buf.clear()
      self._fua_buf.clear()
      self._fua_active = False
    self._au_ts = rtp_ts

    nal_hdr = payload[0]
    nal_type = nal_hdr & 0x1F

    # 1. Single NAL Unit（單一完整 NAL：SPS=7、PPS=8、IDR=5、Non-IDR=1 等）
    if 1 <= nal_type <= 23:
      self._fua_buf.clear()
      self._fua_active = False
      self._au_buf.extend(self._START_CODE)
      self._au_buf.extend(payload)

    # 2. STAP-A（單一封包內聚合多個小型 NAL，常見於 SPS + PPS 合併傳送）
    elif nal_type == 24:
      self._fua_buf.clear()
      self._fua_active = False
      pos = 1
      plen = len(payload)
      while pos + 2 <= plen:
        nal_size = (payload[pos] << 8) | payload[pos + 1]
        pos += 2
        if nal_size == 0 or pos + nal_size > plen:
          break
        self._au_buf.extend(self._START_CODE)
        self._au_buf.extend(payload[pos : pos + nal_size])
        pos += nal_size

    # 3. FU-A（大型影格如 IDR / P-frame 跨多包 RTP 分片重組）
    elif nal_type == 28 and len(payload) >= 2:
      fu_hdr = payload[1]
      start_bit = bool(fu_hdr & 0x80)
      end_bit = bool(fu_hdr & 0x40)
      orig_nal_type = fu_hdr & 0x1F
      reconstructed_hdr = (nal_hdr & 0xE0) | orig_nal_type

      if start_bit:
        self._fua_buf.clear()
        self._fua_buf.extend(self._START_CODE)
        self._fua_buf.append(reconstructed_hdr)
        self._fua_buf.extend(payload[2:])
        self._fua_active = True
      elif self._fua_active:
        self._fua_buf.extend(payload[2:])

      if end_bit and self._fua_active:
        self._au_buf.extend(self._fua_buf)
        self._fua_buf.clear()
        self._fua_active = False

    # 收到本影格最後一包 RTP（Marker Bit = 1），立刻吐出完整 Access Unit
    if marker and self._au_buf:
      completed.append(bytes(self._au_buf))
      self._au_buf.clear()

    return completed


def _extract_from_sps(data: bytes) -> bytes | None:
  """
  掃描 Annex-B 起始碼（00 00 00 01 或 00 00 01），尋找第一個 SPS（NAL type 7）。
  串流中途加入時在收到 SPS 前略過殘缺 P-frame，避免花屏與解碼器報錯。
  """
  n = len(data)
  i = 0
  while i < n - 3:
    start_len = 0
    if data[i] == 0 and data[i + 1] == 0:
      if data[i + 2] == 1:
        start_len = 3
      elif i + 3 < n and data[i + 2] == 0 and data[i + 3] == 1:
        start_len = 4

    if start_len > 0:
      hdr_idx = i + start_len
      if hdr_idx < n and (data[hdr_idx] & 0x1F) == 7:  # SPS
        return data[i:]
      i = hdr_idx + 1
    else:
      i += 1
  return None


def _contains_vcl_nal(data: bytes) -> bool:
  """檢查 Annex-B 緩衝內是否包含影像切片（VCL NAL：Non-IDR=1 或 IDR=5）。"""
  n = len(data)
  i = 0
  while i < n - 3:
    start_len = 0
    if data[i] == 0 and data[i + 1] == 0:
      if data[i + 2] == 1:
        start_len = 3
      elif i + 3 < n and data[i + 2] == 0 and data[i + 3] == 1:
        start_len = 4

    if start_len > 0:
      hdr_idx = i + start_len
      if hdr_idx < n and (data[hdr_idx] & 0x1F) in (1, 5):
        return True
      i = hdr_idx + 1
    else:
      i += 1
  return False


class VideoDecoderBackend(ABC):
  """
  跨平台 H.264 視訊解碼器抽象介面：
  - Linux / WSL2 / C3X / C4：優先使用 BuiltinFFmpegDecoder（直連內建 comma-deps-ffmpeg，零 PyAV、零編譯）。
  - Windows / macOS / 無 .so 環境：自動退回 SubprocessFFmpegNV12Decoder（呼叫系統 ffmpeg 輸出 NV12）。
  - Android / iOS App 框架：於啟動 ui.py 前透過 set_video_decoder_factory() 注入原生 MediaCodec / VideoToolbox。
  """

  @abstractmethod
  def feed(self, nal_data: bytes) -> None:
    """餵入重組後的 H.264 Annex-B 資料。"""

  @abstractmethod
  def reset(self) -> None:
    """重設解碼狀態（切換鏡頭或重新要求關鍵幀時呼叫）。"""

  @abstractmethod
  def stop(self) -> None:
    """停止解碼器並釋放資源。"""


class BuiltinFFmpegDecoder(VideoDecoderBackend):
  """
  複用專案內建 openpilot/tools/camerastream/ffmpeg_decoder.py 的純 Python + ctypes 解碼管線：
  1. 零 PyAV 依賴、零 C++ 編譯，直接載入環境內建 comma-deps-ffmpeg（libavcodec / libswscale）。
  2. 全自動硬體解碼偵測（不依賴任何環境變數）：
     - C3X / C4（高通 845 Venus VPU）與 ARM64 裝置：優先嘗試 h264_v4l2m2m，失敗退回 h264 軟解。
     - x86_64 Linux / WSL2：依序嘗試 h264_v4l2m2m -> h264_cuvid（NVIDIA 硬解）-> h264 軟解。
     - 執行期防呆降級：若硬體解碼器開啟成功但於首幀解碼拋錯，自動即時切換回 CPU h264 軟解並重試。
  3. 原生 NV12 直拷優化：當硬體解碼器（h264_cuvid / h264_v4l2m2m）輸出格式本身即為 NV12 時，
     直接依 linesize 拷貝 Y 與 UV 平面至環形緩衝，繞開 sws_scale(NV12->NV12) 遺失 UV 平面造成偏綠的問題，
     並同時省去多餘的色彩轉換開銷；非 NV12（如 yuv420p）則維持走 sws_scale 轉成緊密 NV12。
  4. 預先配置 3 槽環形 NV12 緩衝區（1928x1208 單幀僅約 3.5 MB），確保主執行緒 pyray GPU 上傳與背景解碼無競爭。
  """

  _RING_SIZE = 3
  _MAX_PENDING_PREFIX_BYTES = 65536

  @staticmethod
  def _detect_candidate_codecs() -> tuple[str, ...]:
    """依目前硬體平台自動決定解碼器嘗試順序（硬解優先，最終保證退回 CPU h264）。"""
    is_comma_hw = os.path.isfile("/AGNOS")
    machine = platform.machine().lower()
    if is_comma_hw or machine in ("aarch64", "arm64"):
      return ("h264_v4l2m2m", "h264")
    return ("h264_v4l2m2m", "h264_cuvid", "h264")

  def __init__(self, on_frame: Callable[[DecodedFrame], None]):
    # 惰性匯入內建 ffmpeg_decoder：非 Linux 平台（無 .so）時由外層工廠自動退回跨平台備援
    from openpilot.tools.camerastream.ffmpeg_decoder import (
      AV_PIX_FMT_NV12,
      Decoder,
      FFmpegError,
      _avcodec,
      _check,
      _swscale,
    )

    self._decoder_cls = Decoder
    self._ffmpeg_error_cls = FFmpegError
    self._nv12_pix_fmt = AV_PIX_FMT_NV12
    self._avcodec = _avcodec
    self._swscale = _swscale
    self._check = _check

    self._on_frame = on_frame
    self._running = True
    self._lock = threading.Lock()

    self._decoder: Any = None
    self.codec_name: str = ""
    self._seen_sps = False
    self._has_decoded_frame = False
    self._pending_prefix = bytearray()

    # 預先配置的 3 槽環形 NV12 緩衝區，避免與主執行緒紋理上傳競爭及每幀配置碎片
    self._ring_bufs: list[np.ndarray] = []
    self._ring_idx: int = 0
    self._buf_shape: tuple[int, int] = (0, 0)

    self._open_auto_decoder()

  def _open_auto_decoder(self) -> None:
    """依平台候選順序自動開啟第一個可用的解碼器。"""
    last_err: Exception | None = None
    for name in self._detect_candidate_codecs():
      try:
        dec = self._decoder_cls(name)
        self._decoder = dec
        self.codec_name = name
        print(f"[RemoteWebRTC] 內建 FFmpeg 解碼器已自動啟用：{name}")
        return
      except Exception as e:
        last_err = e
        continue
    raise RuntimeError(f"無法初始化任何內建 FFmpeg H.264 解碼器：{last_err}")

  def _fallback_to_software(self, reason: Exception) -> bool:
    """當硬體解碼器於執行期無法解碼目前串流時，自動無縫降級回 CPU h264 軟解。"""
    if self.codec_name == "h264":
      return False
    prev_name = self.codec_name
    if self._decoder is not None:
      with contextlib.suppress(Exception):
        self._decoder.close()
      self._decoder = None
    try:
      self._decoder = self._decoder_cls("h264")
      self.codec_name = "h264"
      print(f"[RemoteWebRTC] 硬體解碼器 {prev_name} 執行期不支援目前串流 ({reason})，已自動降級為 CPU h264")
      return True
    except Exception:
      return False

  def _ensure_ring_buffers(self, width: int, height: int) -> None:
    """僅於首幀或解析度變更時配置 3 槽固定 NV12 緩衝區，之後全程原地復用。"""
    if self._buf_shape == (width, height) and len(self._ring_bufs) == self._RING_SIZE:
      return
    total_bytes = width * height * 3 // 2
    self._ring_bufs = [np.empty(total_bytes, dtype=np.uint8) for _ in range(self._RING_SIZE)]
    self._ring_idx = 0
    self._buf_shape = (width, height)

  def _copy_to_ring(self, nv12_buf: np.ndarray, width: int, height: int) -> DecodedFrame:
    """將 ffmpeg_decoder 的復用 NV12 緩衝複製至環形槽位並包裝成 DecodedFrame。"""
    self._ensure_ring_buffers(width, height)
    dst = self._ring_bufs[self._ring_idx]
    self._ring_idx = (self._ring_idx + 1) % self._RING_SIZE
    np.copyto(dst, nv12_buf)
    return DecodedFrame(
      data=dst,
      width=width,
      height=height,
      stride=width,
      uv_offset=width * height,
    )

  def _pack_nv12_frame_to_ring(self, frame: Any, width: int, height: int) -> DecodedFrame | None:
    """
    針對原生已為 NV12 格式的 AVFrame（例如 h264_cuvid、h264_v4l2m2m）：
    直接依 linesize 剔除列尾對齊填充，將 Y 與交錯 UV 平面拷貝至環形緩衝區，
    徹底避開 sws_scale(NV12 -> NV12) 未寫入 UV 平面導致畫面偏綠的問題。
    """
    ls0 = int(frame.linesize[0])
    ls1 = int(frame.linesize[1])
    if not frame.data[0] or not frame.data[1] or ls0 < width or ls1 < width:
      return None

    self._ensure_ring_buffers(width, height)
    dst = self._ring_bufs[self._ring_idx]
    self._ring_idx = (self._ring_idx + 1) % self._RING_SIZE

    y_size = width * height
    uv_h = height // 2

    # 拷貝 Y 平面（若 linesize == width 直接整塊拷貝，否則逐列切除填充位元組）
    if ls0 == width:
      y_src = np.ctypeslib.as_array(frame.data[0], shape=(y_size,))
      np.copyto(dst[:y_size], y_src)
    else:
      y_src = np.ctypeslib.as_array(frame.data[0], shape=(height, ls0))[:, :width]
      np.copyto(dst[:y_size].reshape(height, width), y_src)

    # 拷貝交錯 UV 平面（高度為 height // 2，每列寬度同樣為 width 位元組）
    if ls1 == width:
      uv_src = np.ctypeslib.as_array(frame.data[1], shape=(uv_h * width,))
      np.copyto(dst[y_size:], uv_src)
    else:
      uv_src = np.ctypeslib.as_array(frame.data[1], shape=(uv_h, ls1))[:, :width]
      np.copyto(dst[y_size:].reshape(uv_h, width), uv_src)

    return DecodedFrame(
      data=dst,
      width=width,
      height=height,
      stride=width,
      uv_offset=y_size,
    )

  def _decode_payload_locked(self, payload: bytes) -> list[DecodedFrame]:
    """在持有解碼鎖狀態下送出單筆完整 Access Unit 並取出所有已就緒 NV12 影格。"""
    dec = self._decoder
    dec._ensure_open()
    if not payload:
      return []

    dec._prepare_packet(payload)
    res = self._avcodec.avcodec_send_packet(dec._context, dec._packet)
    # 封包緩衝由 Decoder 管理，送出後立即清空借用指標，避免釋放階段誤觸
    dec._packet.contents.data = None
    dec._packet.contents.size = 0
    self._check(res, "send packet to decoder")

    frames: list[DecodedFrame] = []
    while self._running:
      rc = self._avcodec.avcodec_receive_frame(dec._context, dec._frame)
      if rc == -errno.EAGAIN:
        break
      self._check(rc, "receive decoded frame")

      try:
        frame = dec._frame.contents
        w = int(frame.width)
        h = int(frame.height)
        fmt = int(frame.format)
        if w <= 0 or h <= 0 or (w & 1) or (h & 1):
          continue

        # 優先直拷原生 NV12 平面；其餘像素格式（如軟體解碼 yuv420p）才交由 sws_scale 轉成 NV12
        packed: DecodedFrame | None = None
        if fmt == self._nv12_pix_fmt:
          packed = self._pack_nv12_frame_to_ring(frame, w, h)

        if packed is None:
          dec._prepare_output(frame)
          rows = self._swscale.sws_scale(
            dec._sws_context,
            frame.data,
            frame.linesize,
            0,
            h,
            dec._dst_data,
            dec._dst_linesize,
          )
          if rows != h:
            raise self._ffmpeg_error_cls(f"convert decoded frame: produced {rows} of {h} rows")
          packed = self._copy_to_ring(dec._output, w, h)

        frames.append(packed)
        self._has_decoded_frame = True
      finally:
        self._avcodec.av_frame_unref(dec._frame)

    return frames

  def feed(self, nal_data: bytes) -> None:
    if not self._running or not nal_data:
      return

    decoded_frames: list[DecodedFrame] = []
    with self._lock:
      if not self._running or self._decoder is None:
        return

      # 尚未收到 SPS 前先等待第一個 SPS，避免對殘缺 GOP 解碼報錯或花屏
      if not self._seen_sps:
        extracted = _extract_from_sps(nal_data)
        if extracted is None:
          return
        self._seen_sps = True
        nal_data = extracted

      # 若此段僅含 SPS / PPS / SEI 等非影像切片標頭，先暫存待與後續 Slice 合併送出
      if not _contains_vcl_nal(nal_data):
        if len(self._pending_prefix) + len(nal_data) <= self._MAX_PENDING_PREFIX_BYTES:
          self._pending_prefix.extend(nal_data)
        return

      if self._pending_prefix:
        payload = bytes(self._pending_prefix) + nal_data
        self._pending_prefix.clear()
      else:
        payload = nal_data

      try:
        decoded_frames = self._decode_payload_locked(payload)
      except self._ffmpeg_error_cls as e:
        # 若硬體解碼器尚未成功解出任何影格即報錯，立即降級回 CPU h264 並以同筆關鍵幀重試
        if not self._has_decoded_frame and self._fallback_to_software(e):
          with contextlib.suppress(Exception):
            decoded_frames = self._decode_payload_locked(payload)
        else:
          # 網路掉包造成單一 P-frame 損毀時略過，等下一個關鍵幀恢復
          return
      except Exception:
        return

    # 於釋放解碼鎖後再回報上層，避免與主執行緒產生鎖順序競爭
    for frame in decoded_frames:
      if not self._running:
        break
      self._on_frame(frame)

  def reset(self) -> None:
    """重設解碼器與等待 SPS 狀態，迎接新鏡頭的 SPS/PPS/IDR 序列。"""
    with self._lock:
      if not self._running:
        return
      self._pending_prefix.clear()
      self._seen_sps = False
      if self._decoder is not None:
        with contextlib.suppress(Exception):
          self._decoder.reset()

  def stop(self) -> None:
    """安全關閉解碼器並立即釋放環形緩衝區記憶體（離開 Onroad 時歸還 RAM）。"""
    self._running = False
    with self._lock:
      decoder = self._decoder
      self._decoder = None
      self._seen_sps = False
      self._has_decoded_frame = False
      self._pending_prefix.clear()
      self._ring_bufs.clear()
      self._buf_shape = (0, 0)
      self._ring_idx = 0

    if decoder is not None:
      with contextlib.suppress(Exception):
        decoder.close()


class SubprocessFFmpegNV12Decoder(VideoDecoderBackend):
  """
  跨平台備援 H.264 -> NV12 解碼管線（僅於 Windows / macOS 或無法載入內建 .so 時啟用）：
  - 直接要求系統 ffmpeg 輸出 raw NV12（相較 RGB24 頻寬與記憶體皆減半），
    並寫入 3 槽環形緩衝區直通上游 CameraView 原生 NV12 GPU Shader。
  """

  _RING_SIZE = 3

  def __init__(self, on_frame: Callable[[DecodedFrame], None]):
    self._on_frame = on_frame
    self._running = True
    self._proc: subprocess.Popen[bytes] | None = None
    self._lock = threading.Lock()
    self._probe_buf = bytearray()
    self._seen_sps = False
    self._reader_thread: threading.Thread | None = None
    self._ring_bufs: list[np.ndarray] = []
    self._ring_idx: int = 0

  @staticmethod
  def _probe_h264_dimensions(h264_sample: bytes) -> tuple[int, int] | None:
    """使用系統 ffprobe 從累積的 H.264 樣本（含 SPS/PPS）解析實際視訊解析度。"""
    ffprobe_bin = shutil.which("ffprobe")
    if not ffprobe_bin:
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
    """啟動低延遲 ffmpeg 子程序，從 stdin 接收 H.264 並於 stdout 直接輸出 raw NV12。"""
    ffmpeg_bin = shutil.which("ffmpeg")
    if not ffmpeg_bin:
      print("[RemoteWebRTC] 找不到系統 ffmpeg 執行檔，無法解碼視訊串流")
      return

    cmd = [
      ffmpeg_bin,
      "-loglevel", "error",
      "-fflags", "nobuffer",
      "-flags", "low_delay",
      "-threads", "1",
      "-f", "h264",
      "-i", "pipe:0",
      "-pix_fmt", "nv12",
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
    frame_bytes = width * height * 3 // 2
    self._ring_bufs = [np.empty(frame_bytes, dtype=np.uint8) for _ in range(self._RING_SIZE)]
    self._ring_idx = 0
    print(f"[RemoteWebRTC] 跨平台備援 FFmpeg NV12 解碼器已啟動 ({width}x{height})")

    def _read_stdout_frames() -> None:
      """背景執行緒：持續從 ffmpeg stdout 讀取完整 NV12 影格寫入環形槽位。"""
      stdout = proc.stdout
      if stdout is None:
        return
      uv_offset = width * height

      while self._running and proc.poll() is None:
        try:
          raw = stdout.read(frame_bytes)
          if not raw or len(raw) < frame_bytes:
            break
          dst = self._ring_bufs[self._ring_idx]
          self._ring_idx = (self._ring_idx + 1) % self._RING_SIZE
          dst[:] = np.frombuffer(raw, dtype=np.uint8)
          self._on_frame(DecodedFrame(
            data=dst,
            width=width,
            height=height,
            stride=width,
            uv_offset=uv_offset,
          ))
        except Exception:
          break

    self._reader_thread = threading.Thread(
      target=_read_stdout_frames,
      name="RemoteFFmpegNV12Reader",
      daemon=True,
    )
    self._reader_thread.start()

  def feed(self, nal_data: bytes) -> None:
    if not self._running or not nal_data:
      return

    with self._lock:
      if not self._seen_sps:
        extracted = _extract_from_sps(nal_data)
        if extracted is None:
          return
        self._seen_sps = True
        nal_data = extracted

      if self._proc is None:
        self._probe_buf.extend(nal_data)
        if len(self._probe_buf) > 512 * 1024:
          del self._probe_buf[: -256 * 1024]

        dims = self._probe_h264_dimensions(bytes(self._probe_buf))
        if dims is None:
          return

        width, height = dims
        self._start_ffmpeg(width, height)
        if self._proc is None or self._proc.stdin is None:
          return

        initial_payload = bytes(self._probe_buf)
        self._probe_buf.clear()
        try:
          self._proc.stdin.write(initial_payload)
          self._proc.stdin.flush()
        except Exception:
          self._proc = None
        return

      proc = self._proc
      if proc is not None and proc.stdin is not None and proc.poll() is None:
        try:
          proc.stdin.write(nal_data)
          proc.stdin.flush()
        except Exception:
          self._proc = None

  def reset(self) -> None:
    """切換鏡頭時重啟子程序以重新探測新鏡頭解析度。"""
    with self._lock:
      proc = self._proc
      self._proc = None
      self._probe_buf.clear()
      self._seen_sps = False
    if proc is not None:
      with contextlib.suppress(Exception):
        if proc.stdin:
          proc.stdin.close()
        proc.terminate()
        proc.wait(timeout=0.5)
      if proc.poll() is None:
        with contextlib.suppress(Exception):
          proc.kill()

  def stop(self) -> None:
    """安全關閉並清理 ffmpeg 子程序與環形緩衝區。"""
    self._running = False
    self.reset()
    with self._lock:
      self._ring_bufs.clear()
      self._ring_idx = 0


DecoderFactory = Callable[[Callable[[DecodedFrame], None]], VideoDecoderBackend]


def create_default_video_decoder(on_frame: Callable[[DecodedFrame], None]) -> VideoDecoderBackend:
  """
  預設解碼器工廠：
  1. 優先使用專案內建 comma-deps-ffmpeg（BuiltinFFmpegDecoder，自動偵測硬體解碼）。
  2. 若目前為 Windows / macOS 等無法載入 Linux .so 的跨平台環境，自動退回 SubprocessFFmpegNV12Decoder。
  """
  try:
    return BuiltinFFmpegDecoder(on_frame)
  except Exception as e:
    print(f"[RemoteWebRTC] 內建 FFmpeg 動態庫不可用 ({e})，自動退回系統 ffmpeg NV12 備援解碼器")
    return SubprocessFFmpegNV12Decoder(on_frame)


class RemoteVideoPipeline:
  """
  遠端視訊接收與解碼獨立管線（自 RemoteWebRTCClient 抽離以保持模組職責單一）：
  1. 接收來自 libdatachannel 視訊軌道的原始封包，以微秒級非阻塞方式推入有界佇列，
     絕不在 C++ 網路回呼執行緒內做拆包或解碼。
  2. 獨立背景執行緒 RemoteVideoDecodeThread 循序執行 RFC 6184 RTP 拆包與 NV12 解碼。
  3. 維護執行緒安全的最新影格雙緩衝（get_latest_frame），供 RemoteVisionIpcClient 讀取。
  """

  def __init__(self, decoder_factory: DecoderFactory = create_default_video_decoder):
    self._decoder_factory: DecoderFactory = decoder_factory
    self._running = False
    self._enabled = False

    self._video_queue: queue.Queue[bytes | None] = queue.Queue(maxsize=256)
    self._decode_thread: threading.Thread | None = None
    self._rtp_unpacker = H264RtpUnpacker()

    self._decoder_lock = threading.Lock()
    self._decoder: VideoDecoderBackend | None = None
    self._decoder_init_failed = False

    self._frame_lock = threading.Lock()
    self._latest_frame: DecodedFrame | None = None
    self._frame_id: int = 0
    self._frame_timestamp: float = 0.0

  @property
  def enabled(self) -> bool:
    """回傳目前是否啟用視訊串流解碼（Onroad 狀態）。"""
    return self._enabled

  def set_decoder_factory(self, factory: DecoderFactory) -> None:
    """替換解碼器工廠（供行動端注入原生硬解或測試替換用）。"""
    self._decoder_factory = factory
    self._decoder_init_failed = False

  def start(self) -> None:
    """啟動獨立視訊解碼背景執行緒。"""
    if self._running:
      return
    self._running = True
    self._decoder_init_failed = False
    self._decode_thread = threading.Thread(
      target=self._decode_loop,
      name="RemoteVideoDecodeThread",
      daemon=True,
    )
    self._decode_thread.start()

  def stop(self) -> None:
    """停止背景解碼執行緒並釋放解碼器與影格快取。"""
    self._running = False
    self._drain_queue()
    with contextlib.suppress(queue.Full):
      self._video_queue.put_nowait(None)
    self.stop_decoder(clear_frame=True)
    if self._decode_thread is not None and self._decode_thread.is_alive():
      if self._decode_thread is not threading.current_thread():
        self._decode_thread.join(timeout=2.0)

  def set_enabled(self, enabled: bool) -> bool:
    """
    切換視訊解碼開關，回傳狀態是否實際變更：
    - enabled=False：立即停止解碼器並清空舊畫面釋放記憶體。
    - enabled=True：保留開關狀態，待串流封包抵達或顯式呼叫 ensure_started_if_enabled() 時啟動解碼器。
    """
    enabled = bool(enabled)
    changed = (self._enabled != enabled)
    self._enabled = enabled
    if not enabled:
      self.stop_decoder(clear_frame=True)
    return changed

  def ensure_started_if_enabled(self) -> None:
    """若目前已啟用視訊串流，確保解碼後端已初始化就緒。"""
    if not self._running or not self._enabled or self._decoder_init_failed:
      return
    with self._decoder_lock:
      if self._decoder is None:
        try:
          self._decoder = self._decoder_factory(self._on_decoded_frame)
        except Exception as e:
          self._decoder_init_failed = True
          print(f"[RemoteWebRTC] 無法啟動視訊解碼後端：{e}")

  def stop_decoder(self, clear_frame: bool = False) -> None:
    """停止解碼後端、清空待處理 RTP 佇列與拆包緩衝，並視需要清除最新畫面。"""
    self._drain_queue()
    self._rtp_unpacker.reset()
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

  def reset_stream(self) -> None:
    """切換鏡頭時排空舊封包並重設拆包與解碼器，等待新鏡頭的 SPS/PPS/IDR 避免混淆。"""
    self._drain_queue()
    self._rtp_unpacker.reset()
    with self._decoder_lock:
      decoder = self._decoder
    if decoder is not None:
      with contextlib.suppress(Exception):
        decoder.reset()

  def on_rtp_packet(self, data: bytes | str) -> None:
    """
    libdatachannel 視訊軌道接收回呼：
    僅做非阻塞入列，立刻返回以釋放 C++ 網路執行緒。
    """
    if not self._running or not self._enabled or not isinstance(data, bytes) or not data:
      return
    try:
      self._video_queue.put_nowait(data)
    except queue.Full:
      # 佇列滿載時丟棄最舊封包並納入新封包，維持低延遲且絕不阻塞 C++ 執行緒
      with contextlib.suppress(queue.Empty):
        self._video_queue.get_nowait()
      with contextlib.suppress(queue.Full):
        self._video_queue.put_nowait(data)

  def get_latest_frame(self) -> tuple[DecodedFrame | None, int, float]:
    """
    取得最新一張解碼後的 NV12 道路影像幀（供 RemoteVisionIpcClient 轉交 CameraView 更新 Texture）。
    回傳值：(decoded_frame, frame_id, timestamp)
    """
    with self._frame_lock:
      return self._latest_frame, self._frame_id, self._frame_timestamp

  def _on_decoded_frame(self, frame: DecodedFrame | np.ndarray) -> None:
    """解碼後端回呼：將最新解碼影格寫入執行緒安全快取。"""
    if not self._running or not self._enabled:
      return
    now = time.monotonic()
    with self._frame_lock:
      self._latest_frame = frame  # type: ignore[assignment]
      self._frame_id += 1
      self._frame_timestamp = now

  def _drain_queue(self) -> None:
    """非阻塞排空待解碼封包佇列。"""
    while True:
      try:
        self._video_queue.get_nowait()
      except queue.Empty:
        break

  def _decode_loop(self) -> None:
    """獨立背景執行緒：循序消費 RTP 封包、執行 RFC 6184 拆包並送入解碼後端。"""
    while self._running:
      try:
        packet = self._video_queue.get(timeout=0.2)
      except queue.Empty:
        continue
      if packet is None:
        break
      if not self._running or not self._enabled:
        continue

      access_units = self._rtp_unpacker.feed(packet)
      if not access_units:
        continue

      self.ensure_started_if_enabled()
      with self._decoder_lock:
        decoder = self._decoder
      if decoder is not None:
        for au in access_units:
          if not self._running or not self._enabled:
            break
          decoder.feed(au)


__all__ = [
  "BuiltinFFmpegDecoder",
  "DecodedFrame",
  "DecoderFactory",
  "H264RtpUnpacker",
  "RemoteVideoPipeline",
  "SubprocessFFmpegNV12Decoder",
  "VideoDecoderBackend",
  "create_default_video_decoder",
]
