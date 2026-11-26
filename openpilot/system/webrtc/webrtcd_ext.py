#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import signal
import threading
import time
from typing import Any
from datetime import datetime
from urllib.parse import urlparse

from openpilot.cereal import messaging
from openpilot.common.params import ParamKeyType
from openpilot.common.swaglog import cloudlog
from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.system.webrtc.webrtcd import (
  AsyncTaskRunner,
  CerealOutgoingMessageProxy,
  ServerState,
  StreamSession,
  WebrtcdHandler,
  WebrtcdHTTPServer,
  _ice_candidates,
  _json_response,
  _shutdown,
  prewarm_stream_session_imports,
  schedule_teardown,
)


class RawCapnpOutgoingProxy(CerealOutgoingMessageProxy):
  """
  原始 Capnp 二進位推播代理（繼承自 CerealOutgoingMessageProxy）。

  設計目的：
  1. 取代上游的 SubMaster + to_json() 流程，直接透過 messaging.sub_sock 讀取
     msgq 內的原始 Capnp 二進位封包（bytes）並送入 DataChannel，將 C4 的 CPU 負擔降至最低。
  2. 內建逐服務降頻節流（預設 25 Hz），避免 100 Hz 的 carState / controlsState 塞爆區網 Wi-Fi。
  """

  def __init__(self, services: list[str], enabled: bool = True, max_hz: float = 25.0):
    # 直接初始化最底層 AsyncTaskRunner，刻意跳過父類建立 SubMaster 以免重複反序列化
    AsyncTaskRunner.__init__(self)
    self.services = list(services)
    self.channels: list[Any] = []
    self._enabled = enabled

    # 建立非阻塞且自動合併舊幀（conflate=True）的原始 msgq socket
    self.socks = {s: messaging.sub_sock(s, conflate=True) for s in self.services}

    # 節流控制：記錄每個 service 上次推播的單調時間
    self._min_interval = 1.0 / max(max_hz, 1.0)
    self._last_sent: dict[str, float] = dict.fromkeys(self.services, 0.0)

  def update(self) -> None:
    """非阻塞輪詢所有已訂閱的 socket，達節流間隔時將最新原始 bytes 推播至所有開啟中的通道。"""
    now = time.monotonic()
    open_channels = [ch for ch in self.channels if ch.is_open()]
    if not open_channels:
      return

    for service, sock in self.socks.items():
      if (now - self._last_sent[service]) < self._min_interval:
        continue

      raw_bytes = sock.receive(non_blocking=True)
      if raw_bytes is None:
        continue

      self._last_sent[service] = now
      for channel in open_channels:
        channel.send(raw_bytes)


class ExtendedStreamSession(StreamSession):
  """
  擴充版 WebRTC 串流工作階段（繼承自 StreamSession）。

  擴充功能：
  1. 支援 raw_capnp 模式：將 outgoing_bridge 替換為 RawCapnpOutgoingProxy。
  2. 擴充 message_handler：在單一 messaging_channel 上攔截並處理 Params 同步與讀寫指令，
     其餘原生指令（clockSync、livestreamCameraSwitch 等）完整交回父類處理。
  """

  def __init__(self, body: StreamRequestBody, raw_capnp: bool = False, max_hz: float = 25.0):
    super().__init__(body)
    self.raw_capnp = raw_capnp

    # 若客戶端請求原始 Capnp 二進位模式，替換掉上游預設的 JSON 推播代理
    if self.raw_capnp and len(body.bridge_services_out) > 0:
      self.outgoing_bridge = RawCapnpOutgoingProxy(
        body.bridge_services_out,
        enabled=self.enabled,
        max_hz=max_hz,
      )

  def _dump_utf8_params(self) -> tuple[dict[str, Any], dict[str, int]]:
    snapshot: dict[str, Any] = {}
    types: dict[str, int] = {}
    for raw_key in self.params.all_keys():
      key = raw_key.decode("utf-8", errors="ignore") if isinstance(raw_key, bytes) else str(raw_key)
      try:
        t = self.params.get_type(key)
        types[key] = int(t)  # 即使目前無值，也讓遠端知道型別

        val = self.params.get(key)
        if val is None or isinstance(val, bytes):
          continue
        val = self._to_jsonable(val)
        if len(json.dumps(val, ensure_ascii=False)) <= 4096:
          snapshot[key] = val
      except (TypeError, ValueError, UnicodeError):
        continue
      except Exception:
        self.logger.exception("Failed to serialize Params key %s", key)
        continue
    return snapshot, types

  def message_handler(self, message: bytes | str) -> None:
    """
    攔截遠端大螢幕送來的 Params 控制指令；非自訂指令則原封不動交回上游 StreamSession。
    """
    try:
      payload = json.loads(message) if isinstance(message, (bytes, str)) else None
      if isinstance(payload, dict):
        # 同時相容 cmd（RemoteParams 預設）與 type（webrtcd 原生慣例）
        cmd = payload.get("cmd") or payload.get("type")

        if cmd == "get_all_params":
          snapshot, types = self._dump_utf8_params()
          resp = json.dumps({"cmd": "all_params", "params": snapshot, "types": types})
          self.stream.get_messaging_channel().send(resp)
          return

        if cmd == "put_param":
          key = payload.get("key")
          value = payload.get("value")
          block = bool(payload.get("block", False))

          if not isinstance(key, str) or not key:
            raise ValueError("put_param requires a non-empty string key")

          try:
            coerced = self._coerce_for_put(key, value)
            self.params.put(key, coerced, block=block)
            ack_value = self._to_jsonable(coerced)
          except Exception:
            # 寫入失敗：回傳 C4 實際值，讓遠端樂觀快取回滾
            self.logger.exception("put_param failed for key %s", key)
            try:
              actual = self._to_jsonable(self.params.get(key))
              if isinstance(actual, bytes):
                actual = None
            except Exception:
              actual = None
            rollback = json.dumps({"cmd": "param_updated", "key": key, "value": actual})
            self.stream.get_messaging_channel().send(rollback)
            return

          ack = json.dumps({"cmd": "param_updated", "key": key, "value": ack_value})
          self.stream.get_messaging_channel().send(ack)
          return

        if cmd == "remove_param":
          key = payload.get("key")
          if key is not None:
            self.params.remove(str(key))
            ack = json.dumps({"cmd": "param_updated", "key": str(key), "value": None})
            self.stream.get_messaging_channel().send(ack)
          return
    except Exception:
      self.logger.exception("ExtendedStreamSession params command failure")
      return

    # 其餘指令交由上游原生 message_handler 處理
    super().message_handler(message)

  def _coerce_for_put(self, key: str, value: Any) -> Any:
    """
    依 self.params.get_type(key) 把 JSON 解碼後的值轉成原生 Params.put() 需要的精確型別。
    原生 python2cpp 以 type(dat) 精確比對，例如 int 寫入 FLOAT key 會 TypeError。
    """
    t = self.params.get_type(key)  # key 不存在時會拋 UnknownKeyName

    if t == ParamKeyType.BOOL:
      if isinstance(value, str):
        return value.strip().lower() in ("1", "true")
      return bool(value)
    if t == ParamKeyType.INT:
      return int(float(value))
    if t == ParamKeyType.FLOAT:
      return float(value)
    if t == ParamKeyType.STRING:
      if isinstance(value, bytes):
        return value.decode("utf-8")
      return value if isinstance(value, str) else str(value)
    if t == ParamKeyType.TIME:
      if isinstance(value, datetime):
        return value
      return datetime.fromisoformat(str(value))
    if t == ParamKeyType.JSON:
      if isinstance(value, str):
        value = json.loads(value)
      if not isinstance(value, (dict, list)):
        raise TypeError(f"JSON param {key} requires dict or list, got {type(value).__name__}")
      return value
    raise TypeError(f"Param {key} of type {t.name} cannot be written remotely")

  @staticmethod
  def _to_jsonable(value: Any) -> Any:
    return value.isoformat() if isinstance(value, datetime) else value



async def handle_get_stream_ext(
  state: ServerState,
  raw_body: bytes,
  content_type: str,
) -> tuple[int, bytes, str]:
  """
  擴充版 /stream 處理器：
  抽離 raw_capnp 與 max_hz 擴充欄位後再建立上游 StreamRequestBody，
  確保完全不需要修改 openpilot/system/webrtc/helpers.py。
  """
  if content_type != "application/json":
    return _json_response({"error": "unsupported media type"}, status=415)

  raw_dict = json.loads(raw_body)
  raw_capnp = bool(raw_dict.pop("raw_capnp", False))
  max_hz = float(raw_dict.pop("max_hz", 25.0))

  stream_dict = state.streams
  body = StreamRequestBody(**raw_dict)

  async with state.stream_lock:
    # 與上游一致：避免預熱請求（enabled=False）踢掉既有連線
    enabled = any(s.run_task and not s.run_task.done() and s.enabled for s in stream_dict.values())
    if enabled and not body.enabled:
      return _json_response({"error": "busy", "message": "someone else is connected."})

    # 關閉既有舊連線
    for sid, s in list(stream_dict.items()):
      if s.run_task and not s.run_task.done():
        try:
          ch = s.stream.get_messaging_channel()
          ch.send(json.dumps({"type": "disconnect", "data": "Another device has connected, closing this session."}))
        except Exception:
          pass
      await s.stop()
      stream_dict.pop(sid, None)

    # 建立擴充版工作階段
    session = ExtendedStreamSession(body, raw_capnp=raw_capnp, max_hz=max_hz)
    stream_dict[session.identifier] = session
    try:
      answer = await asyncio.wait_for(session.get_answer(), timeout=30)
      cloudlog.event(
        "webrtcd.session.ice_candidates",
        session_id=session.identifier,
        offer_candidates=_ice_candidates(body.sdp),
        answer_candidates=_ice_candidates(answer.sdp),
      )
    except Exception:
      await session.stop()
      stream_dict.pop(session.identifier, None)
      logging.getLogger("webrtcd").exception("Failed to create extended stream answer")
      raise

    session.start()

    def remove_finished_session(_: asyncio.Task[Any]) -> None:
      stream_dict.pop(session.identifier, None)
      schedule_teardown(state)

    if session.run_task is not None:
      session.run_task.add_done_callback(remove_finished_session)

  return _json_response({"sdp": answer.sdp, "type": answer.type})


class ExtendedWebrtcdHandler(WebrtcdHandler):
  """
  擴充版 HTTP 請求處理器：
  攔截 POST /stream 導向 handle_get_stream_ext，其餘路由（/schema、/notify 等）
  完整沿用上游 WebrtcdHandler。未來第二階段的 /api/version 與 /api/ui-bundle 亦加於此處。
  """

  def _dispatch_request(self) -> None:
    parsed = urlparse(self.path)
    if parsed.path == "/stream" and self.command == "POST":
      try:
        result = self._run(
          handle_get_stream_ext(self.server.state, self._read_body(), self.headers.get_content_type())
        )
      except Exception as e:
        logging.getLogger("webrtcd").exception("Unhandled error handling %s", self.path)
        result = _json_response({"error": "exception", "message": f"{type(e).__name__}: {e}"}, status=500)
      self._send(*result)
      return

    super()._dispatch_request()


def webrtcd_ext_thread(host: str, port: int) -> None:
  """啟動擴充版 webrtcd 服務（結構與上游 webrtcd_thread 一致）。"""
  logging.basicConfig(level=logging.INFO, handlers=[logging.StreamHandler()])
  prewarm_start = time.monotonic()
  prewarm_stream_session_imports()
  prewarm_end = time.monotonic()
  logging.getLogger("webrtcd").info(f"webrtc_ext prewarm finished in {(prewarm_end - prewarm_start) * 1000:.1f} ms")

  loop = asyncio.new_event_loop()
  asyncio.set_event_loop(loop)
  state = ServerState()

  server = WebrtcdHTTPServer((host, port), ExtendedWebrtcdHandler)
  server.state = state
  server.loop = loop

  http_thread = threading.Thread(target=server.serve_forever, name="webrtcd-ext-http", daemon=True)
  http_thread.start()

  shutting_down = False
  shutdown_tasks: set[asyncio.Task[Any]] = set()

  def request_shutdown() -> None:
    nonlocal shutting_down
    if shutting_down:
      return
    shutting_down = True
    task = loop.create_task(_shutdown(server, state, loop))
    shutdown_tasks.add(task)
    task.add_done_callback(shutdown_tasks.discard)

  for sig in (signal.SIGINT, signal.SIGTERM):
    loop.add_signal_handler(sig, request_shutdown)

  try:
    loop.run_forever()
  finally:
    server.server_close()
    loop.close()


def main() -> None:
  parser = argparse.ArgumentParser(description="Extended WebRTC daemon for Remote BIG=1 UI")
  # 預設監聽 0.0.0.0 以允許同區網內的大螢幕裝置直接連線
  parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to listen on")
  parser.add_argument("--port", type=int, default=5001, help="Port to listen on")
  args = parser.parse_args()

  webrtcd_ext_thread(args.host, args.port)


if __name__ == "__main__":
  main()
