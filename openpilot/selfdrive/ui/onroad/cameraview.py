import platform
import numpy as np
import pyray as rl

from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient, VisionBuf
from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.egl import init_egl, create_egl_image, destroy_egl_image, bind_egl_image_to_texture, EGLImage
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.remote import REMOTE_UI

if REMOTE_UI:
  from openpilot.selfdrive.ui.remote.webrtc_client import RemoteWebRTCClient

CONNECTION_RETRY_INTERVAL = 0.2  # seconds between connection attempts

VERSION = """
#version 300 es
precision mediump float;
"""
if platform.system() == "Darwin":
  VERSION = """
    #version 330 core
  """


VERTEX_SHADER = VERSION + """
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in vec4 vertexColor;
uniform mat4 mvp;
out vec2 fragTexCoord;
out vec4 fragColor;
void main() {
  fragTexCoord = vertexTexCoord;
  fragColor = vertexColor;
  gl_Position = mvp * vec4(vertexPosition, 1.0);
}
"""

# Choose fragment shader based on platform capabilities
if COMMA_HARDWARE:
  FRAME_FRAGMENT_SHADER = """
    #version 300 es
    #extension GL_OES_EGL_image_external_essl3 : enable
    precision mediump float;
    in vec2 fragTexCoord;
    uniform samplerExternalOES texture0;
    out vec4 fragColor;
    void main() {
      vec4 color = texture(texture0, fragTexCoord);
      fragColor = vec4(pow(color.rgb, vec3(1.0/1.28)), color.a);
    }
    """
else:
  FRAME_FRAGMENT_SHADER = VERSION + """
    in vec2 fragTexCoord;
    uniform sampler2D texture0;
    uniform sampler2D texture1;
    out vec4 fragColor;
    void main() {
      float y = texture(texture0, fragTexCoord).r;
      vec2 uv = texture(texture1, fragTexCoord).ra - 0.5;
      fragColor = vec4(y + 1.402*uv.y, y - 0.344*uv.x - 0.714*uv.y, y + 1.772*uv.x, 1.0);
    }
    """


class CameraView(Widget):
  def __init__(self, name: str, stream_type: VisionStreamType):
    super().__init__()
    self._name = name
    self._stream_type: VisionStreamType = stream_type
    self.available_streams: list[VisionStreamType] = []

    # Target stream for switching
    self._target_client: VisionIpcClient | None = None
    self._target_stream_type: VisionStreamType | None = None
    self._switching: bool = False

    self._texture_needs_update = True
    self.last_connection_attempt: float = 0.0

    self.frame: VisionBuf | None = None
    self.texture_y: rl.Texture | None = None
    self.texture_uv: rl.Texture | None = None

    # EGL resources
    self.egl_images: dict[int, EGLImage] = {}
    self.egl_texture: rl.Texture | None = None

    self._placeholder_color: rl.Color | None = None

    # 遠端 WebRTC 影像資源（僅在 REMOTE_UI=1 模式下啟用）
    self._remote_client = RemoteWebRTCClient.get_instance() if REMOTE_UI else None
    self._remote_texture: rl.Texture | None = None
    self._remote_frame_id: int = -1
    self._remote_size: tuple[int, int] = (0, 0)

    if REMOTE_UI:
      # 遠端模式下略過本機 VisionIPC、NV12 Shader 與 EGL 初始化，直接改由 WebRTC 提供 RGB24 影像
      self.client: VisionIpcClient | None = None
      self.shader: rl.Shader | None = None
      self._texture1_loc: int = -1
      return

    # Primary stream
    self.client = VisionIpcClient(name, stream_type, conflate=True)
    self.shader = rl.load_shader_from_memory(VERTEX_SHADER, FRAME_FRAGMENT_SHADER)
    self._texture1_loc = rl.get_shader_location(self.shader, "texture1") if not COMMA_HARDWARE else -1

    # Initialize EGL for zero-copy rendering on COMMA_HARDWARE
    if COMMA_HARDWARE:
      if not init_egl():
        raise RuntimeError("Failed to initialize EGL")

      # Create a 1x1 pixel placeholder texture for EGL image binding
      temp_image = rl.gen_image_color(1, 1, rl.BLACK)
      self.egl_texture = rl.load_texture_from_image(temp_image)
      rl.unload_image(temp_image)

    ui_state.add_offroad_transition_callback(self._offroad_transition)

  def _offroad_transition(self):
    if REMOTE_UI:
      return

    # Reconnect if not first time going onroad
    if ui_state.is_onroad() and self.frame is not None:
      # Prevent old frames from showing when going onroad. Qt has a separate thread
      # which drains the VisionIpcClient SubSocket for us. Re-connecting is not enough
      # and only clears internal buffers, not the message queue.
      self.frame = None
      self.available_streams.clear()
      if self.client:
        del self.client
      self.client = VisionIpcClient(self._name, self._stream_type, conflate=True)

  def _set_placeholder_color(self, color: rl.Color):
    """Set a placeholder color to be drawn when no frame is available."""
    self._placeholder_color = color

  def switch_stream(self, stream_type: VisionStreamType) -> None:
    if self._stream_type == stream_type:
      return

    # 遠端模式下不建立本機 VisionIpcClient，僅更新目前串流型別紀錄
    if REMOTE_UI:
      self._stream_type = stream_type
      return

    if self._switching and self._target_stream_type == stream_type:
      return

    cloudlog.debug(f'Preparing switch from {self._stream_type} to {stream_type}')

    if self._target_client:
      del self._target_client

    self._target_stream_type = stream_type
    self._target_client = VisionIpcClient(self._name, stream_type, conflate=True)
    self._switching = True

  @property
  def stream_type(self) -> VisionStreamType:
    return self._stream_type

  def close(self) -> None:
    self._clear_textures()

    # Clean up EGL texture
    if COMMA_HARDWARE and self.egl_texture:
      rl.unload_texture(self.egl_texture)
      self.egl_texture = None

    # Clean up shader
    if self.shader and self.shader.id:
      rl.unload_shader(self.shader)

    self.frame = None
    self.available_streams.clear()
    self.client = None

  def __del__(self):
    self.close()

  def _calc_frame_matrix(self, rect: rl.Rectangle) -> np.ndarray:
    if not self.frame:
      return np.eye(3)

    # Calculate aspect ratios
    widget_aspect_ratio = rect.width / rect.height
    frame_aspect_ratio = self.frame.width / self.frame.height

    # Calculate scaling factors to maintain aspect ratio
    zx = min(frame_aspect_ratio / widget_aspect_ratio, 1.0)
    zy = min(widget_aspect_ratio / frame_aspect_ratio, 1.0)

    return np.array([
      [zx, 0.0, 0.0],
      [0.0, zy, 0.0],
      [0.0, 0.0, 1.0]
    ])

  def _render(self, rect: rl.Rectangle):
    # 遠端模式：改走 WebRTC RGB24 影像渲染流程
    if REMOTE_UI:
      self._render_remote(rect)
      return

    if self._switching:
      self._handle_switch()

    if not self._ensure_connection():
      self._draw_placeholder(rect)
      return

    # Try to get a new buffer without blocking
    buffer = self.client.recv(timeout_ms=0)
    if buffer:
      self._texture_needs_update = True
      self.frame = buffer
    elif not self.client.is_connected():
      # ensure we clear the displayed frame when the connection is lost
      self.frame = None

    if not self.frame:
      self._draw_placeholder(rect)
      return

    transform = self._calc_frame_matrix(rect)
    src_rect = rl.Rectangle(0, 0, float(self.frame.width), float(self.frame.height))
    # Flip cabin camera horizontally
    if self._stream_type == VisionStreamType.VISION_STREAM_CABIN:
      src_rect.width = -src_rect.width

    # Calculate scale
    scale_x = rect.width * transform[0, 0]  # zx
    scale_y = rect.height * transform[1, 1]  # zy

    # Calculate base position (centered)
    x_offset = rect.x + (rect.width - scale_x) / 2
    y_offset = rect.y + (rect.height - scale_y) / 2

    x_offset += transform[0, 2] * rect.width / 2
    y_offset += transform[1, 2] * rect.height / 2

    dst_rect = rl.Rectangle(x_offset, y_offset, scale_x, scale_y)

    # Render with appropriate method
    if COMMA_HARDWARE:
      self._render_egl(src_rect, dst_rect)
    else:
      self._render_textures(src_rect, dst_rect)

  def _render_remote(self, rect: rl.Rectangle) -> None:
    """
    遠端模式專用渲染函式：
    從 RemoteWebRTCClient 讀取最新解碼的 RGB24 影像幀，更新至 GPU Texture 並依原生寬高比置中繪製。
    """
    if self._remote_client is None:
      self._draw_placeholder(rect)
      return

    frame, frame_id, _ = self._remote_client.get_latest_frame()
    if frame is None:
      self._draw_placeholder(rect)
      return

    h, w = frame.shape[:2]

    # 首次收到影像或解析度變更時，建立對應尺寸的 RGB24 Texture
    if self._remote_texture is None or self._remote_size != (w, h):
      if self._remote_texture and self._remote_texture.id:
        rl.unload_texture(self._remote_texture)
      self._remote_texture = rl.load_texture_from_image(
        rl.Image(None, w, h, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8)
      )
      self._remote_size = (w, h)
      self._remote_frame_id = -1

    # 僅在收到新幀時才上傳至 GPU，避免 60 FPS 迴圈重複拷貝同一張畫面
    if frame_id != self._remote_frame_id:
      contig = np.ascontiguousarray(frame)
      rl.update_texture(self._remote_texture, rl.ffi.cast("void *", rl.ffi.from_buffer(contig)))
      self._remote_frame_id = frame_id

    # 沿用原生寬高比縮放與置中計算，確保上層車道線與 HUD 精準對齊
    widget_aspect_ratio = rect.width / rect.height
    frame_aspect_ratio = w / h
    zx = min(frame_aspect_ratio / widget_aspect_ratio, 1.0)
    zy = min(widget_aspect_ratio / frame_aspect_ratio, 1.0)

    scale_x = rect.width * zx
    scale_y = rect.height * zy
    x_offset = rect.x + (rect.width - scale_x) / 2
    y_offset = rect.y + (rect.height - scale_y) / 2

    src_rect = rl.Rectangle(0, 0, float(w), float(h))
    if self._stream_type == VisionStreamType.VISION_STREAM_CABIN:
      src_rect.width = -src_rect.width
    dst_rect = rl.Rectangle(x_offset, y_offset, scale_x, scale_y)

    # RGB24 已為標準色彩格式，直接繪製無需套用 NV12 Shader
    rl.draw_texture_pro(self._remote_texture, src_rect, dst_rect, rl.Vector2(0, 0), 0.0, rl.WHITE)

  def _draw_placeholder(self, rect: rl.Rectangle):
    if self._placeholder_color:
      rl.draw_rectangle_rec(rect, self._placeholder_color)

  def _render_egl(self, src_rect: rl.Rectangle, dst_rect: rl.Rectangle) -> None:
    """Render using EGL for direct buffer access"""
    if self.frame is None or self.egl_texture is None:
      return

    idx = self.frame.idx
    egl_image = self.egl_images.get(idx)

    # Create EGL image if needed
    if egl_image is None:
      egl_image = create_egl_image(self.frame.width, self.frame.height, self.frame.stride, self.frame.fd, self.frame.uv_offset)
      if egl_image:
        self.egl_images[idx] = egl_image
      else:
        return

    # Update texture dimensions to match current frame
    self.egl_texture.width = self.frame.width
    self.egl_texture.height = self.frame.height

    # Bind the EGL image to our texture
    bind_egl_image_to_texture(self.egl_texture.id, egl_image)

    # Render with shader
    rl.begin_shader_mode(self.shader)
    rl.draw_texture_pro(self.egl_texture, src_rect, dst_rect, rl.Vector2(0, 0), 0.0, rl.WHITE)
    rl.end_shader_mode()

  def _render_textures(self, src_rect: rl.Rectangle, dst_rect: rl.Rectangle) -> None:
    """Render using texture copies"""
    if not self.texture_y or not self.texture_uv or self.frame is None:
      return

    # Update textures with new frame data
    if self._texture_needs_update:
      y_data = self.frame.data[: self.frame.uv_offset]
      uv_data = self.frame.data[self.frame.uv_offset:]

      rl.update_texture(self.texture_y, rl.ffi.cast("void *", rl.ffi.from_buffer(y_data)))
      rl.update_texture(self.texture_uv, rl.ffi.cast("void *", rl.ffi.from_buffer(uv_data)))
      self._texture_needs_update = False

    # Render with shader
    rl.begin_shader_mode(self.shader)
    rl.set_shader_value_texture(self.shader, self._texture1_loc, self.texture_uv)
    rl.draw_texture_pro(self.texture_y, src_rect, dst_rect, rl.Vector2(0, 0), 0.0, rl.WHITE)
    rl.end_shader_mode()

  def _ensure_connection(self) -> bool:
    if not self.client.is_connected():
      self.frame = None
      self.available_streams.clear()

      # Throttle connection attempts
      current_time = rl.get_time()
      if current_time - self.last_connection_attempt < CONNECTION_RETRY_INTERVAL:
        return False
      self.last_connection_attempt = current_time

      if not self.client.connect(False) or not self.client.num_buffers:
        return False

      cloudlog.debug(f"Connected to {self._name} stream: {self._stream_type}, buffers: {self.client.num_buffers}")
      self._initialize_textures()
      self.available_streams = self.client.available_streams(self._name, block=False)

    return True

  def _handle_switch(self) -> None:
    """Check if target stream is ready and switch immediately."""
    if not self._target_client or not self._switching:
      return

    # Try to connect target if needed
    if not self._target_client.is_connected():
      if not self._target_client.connect(False) or not self._target_client.num_buffers:
        return

      cloudlog.debug(f"Target stream connected: {self._target_stream_type}")

    # Check if target has frames ready
    target_frame = self._target_client.recv(timeout_ms=0)
    if target_frame:
      self.frame = target_frame  # Update current frame to target frame
      self._complete_switch()

  def _complete_switch(self) -> None:
    """Instantly switch to target stream."""
    cloudlog.debug(f"Switching to {self._target_stream_type}")
    # Clean up current resources
    if self.client:
      del self.client

    # Switch to target
    assert self._target_client is not None and self._target_stream_type is not None
    self.client = self._target_client
    self._stream_type = self._target_stream_type
    self._texture_needs_update = True

    # Reset state
    self._target_client = None
    self._target_stream_type = None
    self._switching = False

    # Initialize textures for new stream
    self._initialize_textures()

  def _initialize_textures(self):
    self._clear_textures()
    if not COMMA_HARDWARE:
      self.texture_y = rl.load_texture_from_image(rl.Image(None, int(self.client.stride),
        int(self.client.height), 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_GRAYSCALE))
      self.texture_uv = rl.load_texture_from_image(rl.Image(None, int(self.client.stride // 2),
        int(self.client.height // 2), 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_GRAY_ALPHA))

  def _clear_textures(self):
    # 釋放遠端 WebRTC RGB24 Texture
    if self._remote_texture and self._remote_texture.id:
      rl.unload_texture(self._remote_texture)
      self._remote_texture = None
      self._remote_size = (0, 0)

    if self.texture_y and self.texture_y.id:
      rl.unload_texture(self.texture_y)
      self.texture_y = None

    if self.texture_uv and self.texture_uv.id:
      rl.unload_texture(self.texture_uv)
      self.texture_uv = None

    # Clean up EGL resources
    if COMMA_HARDWARE:
      for data in self.egl_images.values():
        destroy_egl_image(data)
      self.egl_images = {}


if __name__ == "__main__":
  gui_app.init_window("camera view")
  road = CameraView("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD)
  for _ in gui_app.render():
    road.render(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
