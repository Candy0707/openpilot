from __future__ import annotations

from enum import IntEnum
from typing import Any

import numpy as np

from openpilot.selfdrive.ui.remote.webrtc_client import DecodedFrame, RemoteWebRTCClient

# 優先沿用原生 openpilot.cereal.visionipc.VisionStreamType；
# 若於無 C 擴充的純跨平台環境（Windows / Android / iOS）則退回純 Python 對齊列舉
try:
  from openpilot.cereal.visionipc import VisionStreamType
except Exception:
  class VisionStreamType(IntEnum):  # type: ignore[no-redef]
    """純 Python 對齊版 VisionStreamType（零 msgq C 擴充依賴）。"""

    VISION_STREAM_ROAD = 0
    VISION_STREAM_NARROW_ROAD = 0
    VISION_STREAM_DRIVER = 1
    VISION_STREAM_CABIN = 1
    VISION_STREAM_WIDE_ROAD = 2
    VISION_STREAM_MAP = 3


def _build_stream_camera_map() -> dict[Any, str]:
  """建立 VisionStreamType -> webrtcd 鏡頭名稱對應表（同時相容新舊版列舉命名）。"""
  mapping: dict[Any, str] = {}
  candidates = (
    ("VISION_STREAM_NARROW_ROAD", "road"),
    ("VISION_STREAM_ROAD", "road"),
    ("VISION_STREAM_WIDE_ROAD", "wideRoad"),
    ("VISION_STREAM_CABIN", "driver"),
    ("VISION_STREAM_DRIVER", "driver"),
  )
  for attr_name, cam_name in candidates:
    enum_val = getattr(VisionStreamType, attr_name, None)
    if enum_val is not None:
      mapping[enum_val] = cam_name
  return mapping


_STREAM_TO_CAMERA: dict[Any, str] = _build_stream_camera_map()


class VisionBuf:
  """
  遠端版 VisionBuf（純 Python 實作，透過 sys.modules 取代原生 msgq.visionipc.VisionBuf）。

  完整提供上游 CameraView._render / _render_textures 所需的屬性：
  - data：一維 C-contiguous NV12 uint8 ndarray，支援 [:uv_offset] 與 [uv_offset:] 零拷貝切片。
  - width / height / stride / uv_offset：對應實際解碼解析度。
  - idx / frame_id / timestamp_sof / timestamp_eof / fd：對齊原生結構屬性。
  """

  __slots__ = (
    "data",
    "width",
    "height",
    "stride",
    "uv_offset",
    "idx",
    "fd",
    "frame_id",
    "timestamp_sof",
    "timestamp_eof",
  )

  def __init__(self, frame: DecodedFrame, frame_id: int, timestamp: float):
    self.data: np.ndarray = frame.data
    self.width: int = frame.width
    self.height: int = frame.height
    self.stride: int = frame.stride
    self.uv_offset: int = frame.uv_offset
    self.idx: int = frame_id
    self.fd: int = 0
    self.frame_id: int = frame_id
    ts_ns = int(timestamp * 1e9)
    self.timestamp_sof: int = ts_ns
    self.timestamp_eof: int = ts_ns


class VisionIpcClient:
  """
  遠端版 VisionIpcClient（純 Python 實作，透過 sys.modules 取代原生 msgq.visionipc.VisionIpcClient）。

  設計原則：
  1. 讓上游 openpilot/selfdrive/ui/onroad/cameraview.py 維持 100% 原生程式碼不需任何修改。
  2. 直接讀取 RemoteWebRTCClient 的 PyAV NV12 零拷貝影格，交由 CameraView 原生 GPU Shader 轉色。
  3. 完整支援 CameraView.switch_stream() 切換鏡頭與解析度變動時自動重連重建紋理。
  """

  def __init__(
    self,
    name: str,
    stream_type: VisionStreamType,
    conflate: bool = False,
    bus: Any = None,
  ):
    self._name = name
    self._stream_type = stream_type
    self._conflate = conflate
    self._client = RemoteWebRTCClient.get_instance()
    self._camera_name = _STREAM_TO_CAMERA.get(stream_type, "road")

    # 連線與解析度狀態（對齊上游 CameraView 存取的屬性名稱）
    self._connected: bool = False
    self.num_buffers: int = 0
    self.width: int = 0
    self.height: int = 0
    self.stride: int = 0

    # 影格去重與鏡頭切換基準追蹤
    self._last_frame_id: int = -1
    self._switch_requested: bool = False
    self._switch_baseline_frame_id: int = -1

  def is_connected(self) -> bool:
    """回傳目前遠端串流與影像通道是否處於可用狀態。"""
    if not self._connected:
      return False
    if not self._client.is_connected or not self._client.video_enabled:
      self._connected = False
      return False
    return True

  def connect(self, block: bool = False) -> bool:
    """
    嘗試綁定遠端 WebRTC 視訊串流並取得實際寬高：
    - 若目前鏡頭與本 client 請求不同，先發出 livestreamCameraSwitch 並等待新鏡頭首幀到達。
    - 收到真實解碼影格後記錄 width / height / stride 並回傳 True，供 CameraView 建立對應紋理。
    """
    if not self._client.is_connected:
      return False

    # CameraView._render() 正在主動請求畫面（例如進 Onroad 或開啟車內鏡頭預覽），確保視訊串流已開啟
    if not self._client.video_enabled:
      self._client.set_video_enabled(True)

    # 若目標鏡頭與目前串流鏡頭不同，發送切換請求並記下切換前的 frame_id 基準
    if self._client.current_camera != self._camera_name:
      _, cur_id, _ = self._client.get_latest_frame()
      self._switch_baseline_frame_id = cur_id
      self._switch_requested = True
      self._client.switch_camera(self._camera_name)
      return False

    frame, frame_id, _ = self._client.get_latest_frame()
    if frame is None:
      return False

    # 剛送出鏡頭切換時，等待切換後的新影格才回報已連線，避免沿用舊鏡頭尺寸建立紋理
    if self._switch_requested and frame_id <= self._switch_baseline_frame_id:
      return False

    self._switch_requested = False
    self.width = frame.width
    self.height = frame.height
    self.stride = frame.stride
    self.num_buffers = 3
    self._connected = True
    # 刻意保留 _last_frame_id 為 -1，確保緊接著呼叫的 recv() 能立刻拿到首張影格
    self._last_frame_id = -1
    return True

  def recv(self, timeout_ms: int = 0) -> VisionBuf | None:
    """
    非阻塞取得最新一張 NV12 影像（對齊上游 CameraView 呼叫 recv(timeout_ms=0) 的語意）：
    - 若連線中斷、視訊關閉或解析度改變，標記為斷線並回傳 None，觸發上游自動清空或重建紋理。
    - 若尚無新影格則回傳 None，避免 60 FPS 主迴圈重複上傳相同紋理至 GPU。
    """
    if not self.is_connected():
      return None

    frame, frame_id, ts = self._client.get_latest_frame()
    if frame is None:
      self._connected = False
      return None

    # 解析度變動時回報斷線，讓 CameraView._ensure_connection() 自動重新執行 _initialize_textures()
    if frame.width != self.width or frame.height != self.height or frame.stride != self.stride:
      self._connected = False
      return None

    if frame_id == self._last_frame_id:
      return None

    self._last_frame_id = frame_id
    return VisionBuf(frame=frame, frame_id=frame_id, timestamp=ts)

  @classmethod
  def available_streams(cls, name: str, block: bool = False) -> list[VisionStreamType]:
    """回傳遠端 webrtcd 支援的鏡頭串流清單（同時相容類別方法與實例方法呼叫）。"""
    streams: list[VisionStreamType] = []
    seen_values: set[int] = set()
    for stream_type in _STREAM_TO_CAMERA:
      val = int(stream_type)
      if val not in seen_values:
        streams.append(stream_type)
        seen_values.add(val)
    return streams


__all__ = [
  "VisionBuf",
  "VisionIpcClient",
  "VisionStreamType",
]