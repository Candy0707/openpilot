#!/usr/bin/env python3

from abc import abstractmethod
from collections.abc import Callable
import os
import socket
import time
import capnp
import argparse
import asyncio
import contextlib
import json
import uuid
import logging
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
from typing import Any

from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.system.webrtc.schema import generate_field
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.cereal import messaging, log
from openpilot.cereal.services import SERVICE_LIST

SESSION_TIMEOUT_SECONDS = 300

# ---------------------------------------------------------------------------
# opview 適配（dev20261022）
# opview 用 "initCamera" 送請求，以此判定為 opview session；comma connect（athena 經 localhost、
# 用 "cameras"）維持 upstream 行為不變。
# ---------------------------------------------------------------------------
OPVIEW_PARAM = "dp_dev_opview"

# opview 仍用舊 service 名稱；upstream 0.11 已改名，schema 內欄位相同，只需要改名
SERVICE_ALIASES = {
  "liveCalibration": "extrinsicsCalibration",
  "roadCameraState": "narrowRoadCameraState",
}

# 防止資料延遲累積（僅 opview session）
MIN_SEND_INTERVAL = 0.03          # 每個 service 最多約 33 Hz
MAX_BUFFERED_BYTES = 256 * 1024   # data channel 送出佇列上限（所有 session）

# 本分支 custom.Tdx（trafficStatus/roadEvent）→ opview（dplcc 格式 nextSpeed/congestion/events）
TDX_STATUS_TO_OPVIEW = {
  "freeFlow": "smooth",
  "heavyTraffic": "slow",
  "severeCongestion": "congested",
}
TDX_EVENT_PREFIXES = (("目前:", True), ("前方:", False))


def _r3(values) -> list[float]:
  return [round(float(v), 3) for v in values]


def _xyz(p) -> dict[str, list[float]]:
  return {"x": _r3(p.x), "y": _r3(p.y), "z": _r3(p.z)}


def compact_model_v2(m) -> dict[str, Any]:
  """只送 opview 用到的 modelV2 欄位（約 31 KB → 5.6 KB）。opview 要用新欄位時必須同步修改這裡。"""
  return {
    "frameId": m.frameId,
    "position": _xyz(m.position),
    "laneLines": [_xyz(line) for line in m.laneLines],
    "laneLineProbs": _r3(m.laneLineProbs),
    "roadEdges": [_xyz(edge) for edge in m.roadEdges],
    "roadEdgeStds": _r3(m.roadEdgeStds),
    "acceleration": {"x": _r3(m.acceleration.x)},
  }


def tdx_to_opview(t) -> dict[str, Any]:
  ts, ev = t.trafficStatus, t.roadEvent
  events = []
  desc = ev.description or ""
  if ev.isActive and desc:
    is_current = False
    for prefix, current in TDX_EVENT_PREFIXES:
      if desc.startswith(prefix):
        desc, is_current = desc[len(prefix):], current
        break
    events = [{"description": d.strip(), "isCurrentSection": is_current} for d in desc.split("/") if d.strip()]
  return {
    "speed": int(ts.speed),
    "nextSpeed": int(ts.nextSpeed),
    "congestion": TDX_STATUS_TO_OPVIEW.get(str(ts.status), "unknown"),
    "events": events,
  }


def _buffered_amount(channel) -> int:
  try:
    amount = channel.buffered_amount()
    return amount if isinstance(amount, int) else 0
  except Exception:
    return 0


def _is_unspecified_or_loopback(ip: str | None) -> bool:
  return not ip or ip == "0.0.0.0" or ip.startswith("127.") or ip == "::1"


# ice candidate parser for logging
def _ice_candidates(sdp: str) -> list[str]:
  return [line.removeprefix("a=") for line in sdp.splitlines() if line.startswith("a=candidate:")]

# socket trick: route lookup for 8.8.8.8 (nothing is sent or actually connected to)
# return the source interfaces IP which is the default interface of the device
def _default_route_ip() -> str | None:
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    s.connect(("8.8.8.8", 53))  # selects a route, sends nothing
    return s.getsockname()[0]
  except OSError:
    return None
  finally:
    s.close()

class AsyncTaskRunner:
  def __init__(self):
    self.task = None
    self.logger = logging.getLogger("webrtcd")

  def start(self):
    assert self.task is None
    self.task = asyncio.create_task(self.run())

  async def stop(self):
    if self.task is None:
      return
    task = self.task
    self.task = None
    if task.done():
      return
    task.cancel()
    with contextlib.suppress(asyncio.CancelledError):
      await task

  @abstractmethod
  async def run(self):
    pass


class CerealOutgoingMessageProxy(AsyncTaskRunner):
  def __init__(self, services: list[str], enabled: bool = True, opview: bool = False):
    super().__init__()
    self.opview = opview
    # opview：舊 service 名稱 → 本分支名稱；送出時 "type" 改回 opview 請求的名稱
    self.out_name: dict[str, str] = {}
    resolved = []
    for name in services:
      real = name
      if opview and name not in SERVICE_LIST and SERVICE_ALIASES.get(name) in SERVICE_LIST:
        real = SERVICE_ALIASES[name]
      self.out_name[real] = name
      resolved.append(real)
    self.services = resolved
    self.sm = messaging.SubMaster(self.services)
    self.channels = []
    self._enabled = enabled
    self._last_sent: dict[str, float] = {}
    self._congested_since: float | None = None
    self._dropped = 0

  def add_channel(self, channel):
    self.channels.append(channel)

  def enable(self, enable: bool):
    self._enabled = enable

  def to_json(self, msg_content: Any):
    if isinstance(msg_content, capnp._DynamicStructReader):
      msg_dict = msg_content.to_dict()
    elif isinstance(msg_content, capnp._DynamicListReader):
      msg_dict = [self.to_json(msg) for msg in msg_content]
    elif isinstance(msg_content, bytes):
      msg_dict = msg_content.decode()
    else:
      msg_dict = msg_content

    return msg_dict

  def _serialize(self, service: str) -> Any:
    msg = self.sm[service]
    if self.opview:
      try:
        if service == "modelV2":
          return compact_model_v2(msg)
        if service == "tdx" and hasattr(msg, "trafficStatus"):
          return tdx_to_opview(msg)
      except Exception:
        self.logger.exception("opview serialize failure (%s), falling back to to_dict", service)
    return self.to_json(msg)

  def _check_congestion(self, open_channels: list) -> bool:
    # 送出佇列超過上限時這一輪整批不送，下一輪送的一定是最新資料
    congested = bool(open_channels) and all(_buffered_amount(ch) > MAX_BUFFERED_BYTES for ch in open_channels)
    now = time.monotonic()
    if congested and self._congested_since is None:
      self._congested_since = now
      cloudlog.warning("webrtcd: data channel congested")
    elif not congested and self._congested_since is not None:
      cloudlog.warning("webrtcd: data channel recovered after %.1fs, dropped %d messages", now - self._congested_since, self._dropped)
      self._congested_since = None
      self._dropped = 0
    return congested

  def update(self):
    # this is blocking in async context...
    self.sm.update(0)
    updated_services = [s for s, updated in self.sm.updated.items() if updated]
    if not updated_services:
      return

    open_channels = [ch for ch in self.channels if ch.is_open()]
    if self._check_congestion(open_channels):
      self._dropped += len(updated_services)
      return

    now = time.monotonic()
    for service in updated_services:
      if self.opview:
        if now - self._last_sent.get(service, 0.0) < MIN_SEND_INTERVAL:
          continue
        self._last_sent[service] = now
      msg_dict = self._serialize(service)
      mono_time, valid = self.sm.logMonoTime[service], self.sm.valid[service]
      outgoing_msg = {"type": self.out_name.get(service, service), "logMonoTime": mono_time, "valid": valid, "data": msg_dict}
      encoded_msg = json.dumps(outgoing_msg).encode()
      for channel in open_channels:
        channel.send(encoded_msg)

  async def run(self):
    while True:
      if not self._enabled:
        await asyncio.sleep(0.01)
        continue
      try:
        self.update()
      except Exception:
        self.logger.exception("Cereal outgoing proxy failure")
      await asyncio.sleep(0.01)


class CerealIncomingMessageProxy:
  def __init__(self, pm: messaging.PubMaster):
    self.pm = pm

  def send(self, message: bytes):
    msg_json = json.loads(message)
    msg_type, msg_data = msg_json["type"], msg_json["data"]
    size = None
    if not isinstance(msg_data, dict):
      size = len(msg_data)

    msg = messaging.new_message(msg_type, size=size)
    setattr(msg, msg_type, msg_data)
    self.pm.send(msg_type, msg)


class DynamicPubMaster(messaging.PubMaster):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.lock = asyncio.Lock()

  async def add_services_if_needed(self, services):
    async with self.lock:
      for service in services:
        if service not in self.sock:
          self.sock[service] = messaging.pub_sock(service)


class LivestreamBitrateController(AsyncTaskRunner):
  bitrates = [500_000, 1_500_000, int(os.environ.get("STREAM_BITRATE", 5_000_000))]
  label_to_bitrate = { "high": bitrates[2], "med": bitrates[1], "low": bitrates[0]}
  sample_interval = 0.2
  high_level = 0.1 # drop immediately
  med_level = 0.05 # drop after # of samples
  low_level = 0 # raise after # of samples
  down_samples = 5
  param_name = "LivestreamEncoderBitrate"

  def __init__(self, get_stats: Callable[[], dict[str, Any]], params: Params, enabled: bool = True):
    super().__init__()
    self.get_stats = get_stats
    self.params = params

    self.level = 2
    self._publish(self.bitrates[self.level])
    self.prev_stats: tuple[Any, ...] | None = None
    self.counter = 0
    self.up_samples = 5 # 1s
    self._auto = True
    self._enabled = enabled

  def enable(self, enable: bool):
    self._enabled = enable

  async def run(self):
    while True:
      await asyncio.sleep(self.sample_interval)
      if not self._enabled:
        continue
      if not self._auto:
        continue

      loss_rate = self._sample()
      if loss_rate is None:
        continue
      if loss_rate >= self.med_level and self.level > 0:
        self.counter += 1
        if self.counter >= self.down_samples or loss_rate >= self.high_level:
          self.level -= 1
          self.up_samples *= 2 # exponential backoff before raising again
          self.counter = 0
          self._publish(self.bitrates[self.level])
      elif loss_rate <= self.low_level and self.level < len(self.bitrates) - 1:
        self.counter -= 1
        if -self.counter >= self.up_samples:
          self.level += 1
          self.counter = 0
          self._publish(self.bitrates[self.level])

  def _sample(self) -> float | None:
    report = next(iter(self.get_stats().values()), None)
    if report is None:
      return None

    current = (report.ssrc, report.fraction_lost, report.packets_lost, report.highest_seq_no, report.jitter, report.lsr, report.dlsr)
    if self.prev_stats == current:
      return None
    self.prev_stats = current

    loss_rate = report.fraction_lost / 256
    return loss_rate

  def _publish(self, bitrate: float):
    self.params.put(self.param_name, bitrate)

  def set_quality(self, quality):
    if quality in self.label_to_bitrate:
      self._publish(self.label_to_bitrate[quality])
      self._auto = False
    elif quality == "auto":
      self._auto = True


class StreamSession:
  shared_pub_master = DynamicPubMaster([])

  def __init__(self, body: StreamRequestBody, opview: bool = False, local_ip: str | None = None):
    from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
    from teleoprtc.builder import WebRTCAnswerBuilder

    self.identifier = str(uuid.uuid4())
    self.params = Params()
    self.opview = opview
    # opview 走區網：ICE 綁在手機實際連進來的那張網卡（例如 comma 開熱點時是 wlan，不是預設路由的 LTE）
    bind_address = local_ip if (opview and not _is_unspecified_or_loopback(local_ip)) else _default_route_ip()
    builder = WebRTCAnswerBuilder(body.sdp, bind_address=bind_address)

    self.enabled = body.enabled
    self.video_tracks = []
    for camera in body.cameras:
      track = LiveStreamVideoStreamTrack(camera, self.enabled)
      self.video_tracks.append(track)
      builder.add_video_stream(camera, track)
    self.stream = builder.stream()

    self.is_body = "testJoystick" in body.bridge_services_in

    self.incoming_bridge: CerealIncomingMessageProxy | None = None
    self.incoming_bridge_services = body.bridge_services_in
    self.outgoing_bridge: CerealOutgoingMessageProxy | None = None
    self.bitrate_controller: LivestreamBitrateController | None = None
    if len(body.bridge_services_in) > 0:
      self.incoming_bridge = CerealIncomingMessageProxy(self.shared_pub_master)
    if len(body.bridge_services_out) > 0:
      self.outgoing_bridge = CerealOutgoingMessageProxy(body.bridge_services_out, self.enabled, opview=opview)
    self.bitrate_controller = LivestreamBitrateController(self.stream.get_receiver_report_stats, self.params, self.enabled)

    self.run_task: asyncio.Task | None = None
    self._cleanup_lock = asyncio.Lock()
    self._cleanup_done = False
    self.logger = logging.getLogger("webrtcd")
    cloudlog.warning(
      "New stream session (%s), opview %s, bind %s, video cameras %s, video enabled %s, incoming services %s, outgoing services %s",
      self.identifier, opview, bind_address, [t.id for t in self.video_tracks], body.enabled, body.bridge_services_in, body.bridge_services_out,
    )

  def send_control(self, msg_type: str, data: Any) -> None:
    try:
      self.stream.get_messaging_channel().send(json.dumps({"type": msg_type, "data": data}))
    except Exception:
      pass

  def start(self):
    self.run_task = asyncio.create_task(self.run())

  async def stop(self):
    if self.run_task is not None and not self.run_task.done() and self.run_task is not asyncio.current_task():
      self.run_task.cancel()
      with contextlib.suppress(asyncio.CancelledError):
        await self.run_task
    self.run_task = None
    await self.post_run_cleanup()

  async def get_answer(self):
    return await self.stream.start()

  def message_handler(self, message: bytes):
    try:
      payload = json.loads(message) if isinstance(message, (bytes, str)) else None
      if isinstance(payload, dict):
        msg_type = payload.get("type")

        match msg_type:
          case "livestreamCameraSwitch":
            # only needed for 1 track stream
            if len(self.video_tracks) == 1:
              self.video_tracks[0].switch_camera(payload["data"]["camera"])
          case "livestreamSettings":
            if self.bitrate_controller is not None:
              self.bitrate_controller.set_quality(payload["data"]["quality"])
          case "livestreamVideoEnable":
            enabled = payload["data"]["enabled"]
            self.enabled = enabled
            for track in self.video_tracks:
              track.enable(enabled)
            if self.outgoing_bridge is not None:
              self.outgoing_bridge.enable(enabled)
            if self.bitrate_controller is not None:
              self.bitrate_controller.enable(enabled)
            if not enabled:
              self.params.put("LivestreamRequestKeyframe", True)
          case "clockSync":
            pong = json.dumps({"type": "clockSync", "data": {
              "action": "pong", "browserSendTime": payload["data"]["browserSendTime"], "deviceTime": time.time() * 1000, # noqa: TID251
            }})
            self.stream.get_messaging_channel().send(pong)
          case "enableTimingSei":
            for track in self.video_tracks:
              track.timing_sei_enabled = bool(payload["data"]["enabled"])
          case _:
            if msg_type not in self.incoming_bridge_services:
              return
            if self.incoming_bridge is not None:
              self.incoming_bridge.send(message)
    except Exception:
      self.logger.exception("Cereal incoming proxy failure")

  async def run_normal_session(self):
    if self.opview:
      # opview 是整趟行程的常駐畫面，不套用 comma connect 的 300 秒逾時
      await self.stream.wait_for_disconnection()
      return
    try:
      await asyncio.wait_for(self.stream.wait_for_disconnection(), timeout=SESSION_TIMEOUT_SECONDS)
    except TimeoutError:
      self.logger.warning("Stream session (%s) timed out after %d s", self.identifier, SESSION_TIMEOUT_SECONDS)
      try:
        self.stream.get_messaging_channel().send(json.dumps({"type": "disconnect", "data": "Session timed out"}))
      except Exception:
        pass

  async def run_body_session(self):
    await self.stream.wait_for_disconnection()

  async def run(self):
    try:
      self.params.put("LivestreamRequestKeyframe", True)

      # avoid datachannel race by adding messange_handler immediately
      self.stream.set_message_handler(self.message_handler)

      await asyncio.wait_for(self.stream.wait_for_connection(), timeout=15)
      if self.stream.has_messaging_channel():
        if self.incoming_bridge is not None:
          await self.shared_pub_master.add_services_if_needed(self.incoming_bridge_services)
        if self.outgoing_bridge is not None:
          channel = self.stream.get_messaging_channel()
          self.outgoing_bridge.add_channel(channel)
          self.outgoing_bridge.start()
      if self.bitrate_controller is not None:
        self.bitrate_controller.start()

      with cloudlog.ctx(session_id=self.identifier):
        cloudlog.warning("webrtcd.session.connected")
      if self.is_body:
        await self.run_body_session()
      else:
        await self.run_normal_session()
      with cloudlog.ctx(session_id=self.identifier):
        cloudlog.warning("webrtcd.session.ended")
    except Exception:
      self.logger.exception("Stream session failure")
      with cloudlog.ctx(session_id=self.identifier):
        cloudlog.exception("webrtcd.session.exception")
    finally:
      await self.post_run_cleanup()

  async def post_run_cleanup(self):
    async with self._cleanup_lock:
      if self._cleanup_done:
        return
      self._cleanup_done = True
      self.params.put("LivestreamRequestKeyframe", False)
      if self.bitrate_controller is not None:
        await self.bitrate_controller.stop()
      if self.outgoing_bridge is not None:
        await self.outgoing_bridge.stop()
      for track in self.video_tracks:
        track.stop()
      self.video_tracks.clear()
      await self.stream.stop()


class ServerState:
  def __init__(self):
    self.streams: dict[str, StreamSession] = {}
    self.stream_lock = asyncio.Lock()
    self.teardown: asyncio.TimerHandle | None = None


# if nothing connects for 5 seconds, tear down livestreaming processes
def schedule_teardown(state: ServerState):
  if state.teardown is not None:
    state.teardown.cancel()

  def clear():
    if not state.streams:
      Params().put_bool("IsLiveStreaming", False)

  state.teardown = asyncio.get_running_loop().call_later(5.0, clear)


def _json_response(obj: Any, status: int = 200) -> tuple[int, bytes, str]:
  return (status, json.dumps(obj).encode(), "application/json; charset=utf-8")


def _text_response(text: str, status: int = 200) -> tuple[int, bytes, str]:
  return (status, text.encode(), "text/plain; charset=utf-8")


def parse_stream_request(raw: dict[str, Any]) -> tuple[StreamRequestBody, bool]:
  """回傳 (body, is_opview)。opview 格式：{"sdp", "initCamera", "bridge_services_in", "bridge_services_out"}，
  沒有 "cameras"／"enabled"；upstream（athena）格式維持原樣。"""
  raw = dict(raw)
  init_camera = raw.pop("initCamera", None)
  is_opview = init_camera is not None
  if is_opview:
    raw["cameras"] = [init_camera]
    raw.setdefault("enabled", True)
  return StreamRequestBody(**raw), is_opview


async def handle_get_stream(state: ServerState, raw_body: bytes, content_type: str,
                            local_ip: str | None = None) -> tuple[int, bytes, str]:
  if content_type != "application/json":
    return _json_response({"error": "unsupported media type"}, status=415)

  stream_dict = state.streams
  body, is_opview = parse_stream_request(json.loads(raw_body))

  async with state.stream_lock:
    # don't remove existing connection on prewarm request
    enabled = any(s.run_task and not s.run_task.done() and s.enabled for s in stream_dict.values())
    if enabled and not body.enabled:
      return _json_response({"error": "busy", "message": "someone else is connected."})

    for sid, s in list(stream_dict.items()):
      if s.run_task and not s.run_task.done():
        if s.opview:
          # opview 收到 connectionReplaced 會停止自動重連，避免兩支手機互搶
          s.send_control("connectionReplaced", "Another device has connected, closing this session.")
        else:
          s.send_control("disconnect", "Another device has connected, closing this session.")
      await s.stop()
      stream_dict.pop(sid, None)

    session = StreamSession(body, opview=is_opview, local_ip=local_ip)
    stream_dict[session.identifier] = session
    try:
      answer = await asyncio.wait_for(session.get_answer(), timeout=30)
      cloudlog.event(
        "webrtcd.session.ice_candidates",
        session_id=session.identifier,
        offer_candidates=_ice_candidates(body.sdp),
        answer_candidates=_ice_candidates(answer.sdp),
      )
    except TimeoutError:
      await session.stop()
      stream_dict.pop(session.identifier, None)
      logging.getLogger("webrtcd").exception("Timed out creating stream answer")
      with cloudlog.ctx(session_id=session.identifier):
        cloudlog.warning("webrtcd.session.answer_timeout")
      raise
    except Exception:
      await session.stop()
      stream_dict.pop(session.identifier, None)
      logging.getLogger("webrtcd").exception("Failed to create stream answer")
      with cloudlog.ctx(session_id=session.identifier):
        cloudlog.exception("webrtcd.session.answer_exception")
      raise
    session.start()

    def remove_finished_session(_: asyncio.Task) -> None:
      stream_dict.pop(session.identifier, None)
      schedule_teardown(state)

    session.run_task.add_done_callback(remove_finished_session)

  return _json_response({"sdp": answer.sdp, "type": answer.type})


async def handle_get_schema(state: ServerState, services_param: str) -> tuple[int, bytes, str]:
  services = services_param.split(",")
  services = [s for s in services if s]
  assert all(s in log.Event.schema.fields and not s.endswith("DEPRECATED") for s in services), "Invalid service name"
  schema_dict = {s: generate_field(log.Event.schema.fields[s]) for s in services}
  return _json_response(schema_dict)


async def handle_post_notify(state: ServerState, payload: Any) -> tuple[int, bytes, str]:
  for session in list(state.streams.values()):
    try:
      ch = session.stream.get_messaging_channel()
      ch.send(json.dumps(payload))
    except Exception:
      continue

  return _text_response("OK")


async def on_shutdown(state: ServerState):
  for session in list(state.streams.values()):
    try:
      ch = session.stream.get_messaging_channel()
      ch.send(json.dumps({"type": "disconnect", "data": "device streaming has been stopped."}))
    except Exception:
      pass
    await session.stop()
  state.streams.clear()


class WebrtcdHandler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  # path -> allowed methods (aiohttp registered POST /stream, POST /notify, GET /schema + its auto HEAD)
  _routes = {
    "/schema": ("GET", "HEAD"),
    "/stream": ("POST",),
    "/notify": ("POST",),
  }

  def _send(self, status: int, body: bytes, content_type: str) -> None:
    self.send_response(status)
    self.send_header("Content-Type", content_type)
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    if self.command != "HEAD":
      self.wfile.write(body)

  def _read_body(self) -> bytes:
    length = int(self.headers.get("Content-Length", 0))
    return self.rfile.read(length) if length else b""

  def _run(self, coro) -> tuple[int, bytes, str]:
    return asyncio.run_coroutine_threadsafe(coro, self.server.loop).result()

  def _dispatch_request(self) -> None:
    parsed = urlparse(self.path)
    allowed = self._routes.get(parsed.path)

    try:
      client_ip = self.client_address[0] if self.client_address else None
      if not _is_unspecified_or_loopback(client_ip) and not self.server.params.get_bool(OPVIEW_PARAM):
        result = _json_response({"error": "forbidden", "message": f"remote access requires {OPVIEW_PARAM}"}, status=403)
      elif allowed is None:
        result = _json_response({"error": "not found"}, status=404)
      elif self.command not in allowed:
        result = _json_response({"error": "method not allowed"}, status=405)
      elif parsed.path == "/schema":
        services = parse_qs(parsed.query).get("services", [""])[0]
        result = self._run(handle_get_schema(self.server.state, services))
      elif parsed.path == "/stream":
        try:
          local_ip = self.connection.getsockname()[0]
        except Exception:
          local_ip = None
        result = self._run(handle_get_stream(self.server.state, self._read_body(), self.headers.get_content_type(), local_ip))
      else:  # /notify
        try:
          payload = json.loads(self._read_body())
        except Exception:
          result = _json_response({"error": "bad request"}, status=400)
        else:
          result = self._run(handle_post_notify(self.server.state, payload))
    except Exception as e:
      logging.getLogger("webrtcd").exception("Unhandled error handling %s", self.path)
      result = _json_response({"error": "exception", "message": f"{type(e).__name__}: {e}"}, status=500)

    self._send(*result)

  def do_GET(self) -> None:
    self._dispatch_request()

  def do_HEAD(self) -> None:
    self._dispatch_request()

  def do_POST(self) -> None:
    self._dispatch_request()

  def do_PUT(self) -> None:
    self._dispatch_request()

  def do_DELETE(self) -> None:
    self._dispatch_request()

  def do_PATCH(self) -> None:
    self._dispatch_request()

  def do_OPTIONS(self) -> None:
    self._dispatch_request()

  def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override
    # silence default access logging; errors are logged explicitly in _dispatch_request
    pass


class WebrtcdHTTPServer(ThreadingHTTPServer):
  daemon_threads = True
  allow_reuse_address = True
  state: ServerState
  loop: asyncio.AbstractEventLoop
  params: Params


async def _shutdown(server: WebrtcdHTTPServer, state: ServerState, loop: asyncio.AbstractEventLoop) -> None:
  # stop accepting new HTTP connections (blocks until serve_forever returns, so
  # run it off the loop) then tear down active stream sessions.
  await loop.run_in_executor(None, server.shutdown)
  await on_shutdown(state)
  loop.stop()


def prewarm_stream_session_imports() -> None:
  from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
  from teleoprtc.builder import WebRTCAnswerBuilder
  assert LiveStreamVideoStreamTrack
  assert WebRTCAnswerBuilder


def webrtcd_thread(host: str, port: int):
  logging.basicConfig(level=logging.INFO, handlers=[logging.StreamHandler()])
  prewarm_start = time.monotonic()
  prewarm_stream_session_imports()
  prewarm_end = time.monotonic()
  logging.getLogger("webrtcd").info(f"webrtc prewarm finished in {(prewarm_end - prewarm_start) * 1000} ms")

  loop = asyncio.new_event_loop()
  asyncio.set_event_loop(loop)
  state = ServerState()

  server = WebrtcdHTTPServer((host, port), WebrtcdHandler)
  server.state = state
  server.loop = loop
  server.params = Params()

  # serve HTTP on a daemon thread so the asyncio loop can own the main thread
  http_thread = threading.Thread(target=server.serve_forever, name="webrtcd-http", daemon=True)
  http_thread.start()

  shutting_down = False
  shutdown_task = None

  def request_shutdown() -> None:
    nonlocal shutting_down, shutdown_task
    if shutting_down:
      return
    shutting_down = True
    shutdown_task = loop.create_task(_shutdown(server, state, loop))

  for sig in (signal.SIGINT, signal.SIGTERM):
    loop.add_signal_handler(sig, request_shutdown)

  try:
    loop.run_forever()
  finally:
    server.server_close()
    loop.close()


def main():
  parser = argparse.ArgumentParser(description="WebRTC daemon")
  parser.add_argument("--host", type=str, default=None, help="Host to listen on")
  parser.add_argument("--port", type=int, default=5001, help="Port to listen on")
  args = parser.parse_args()

  # opview 從區網連 :5001，所以預設監聽所有網卡；未開啟 dp_dev_opview 時，
  # 非 localhost 的請求會在 _dispatch_request 被擋下（等同 upstream 只聽 127.0.0.1）。
  # 每次請求才檢查 param，避免 webrtcd 先因 comma connect 啟動、上路後 opview 卻連不進來。
  host = args.host if args.host is not None else "0.0.0.0"

  webrtcd_thread(host, args.port)


if __name__=="__main__":
  main()
