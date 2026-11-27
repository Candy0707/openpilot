#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import base64
import ctypes
from datetime import datetime
import gc
import io
import json
import logging
import os
import signal
import tarfile
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlparse
import uuid

from openpilot.cereal import log, messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import ParamKeyType, Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.version import get_build_metadata
from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.system.webrtc.webrtcd import (
  CerealIncomingMessageProxy,
  CerealOutgoingMessageProxy,
  LivestreamBitrateController,
  ServerState,
  StreamSession,
  WebrtcdHandler,
  WebrtcdHTTPServer,
  _default_route_ip,
  _ice_candidates,
  _json_response,
  _shutdown,
  prewarm_stream_session_imports,
  schedule_teardown,
)

# =============================================================================
# 實機 glibc 記憶體碎片抑制與閒置回收機制（借鏡 opview 實車驗證設計）
# webrtcd_ext 具備多條執行緒（HTTP、asyncio、libdatachannel），glibc 預設最多開
# 「CPU 核心數 × 8」個 malloc arena，各 arena 獨立保留釋放後的空間會導致 RSS 只增不減：
#   1. 啟動建立執行緒前呼叫 mallopt(M_ARENA_MAX, 2) 限制 arena 上限。
#   2. 所有 session 結束並延遲 1 秒後執行 gc.collect() + malloc_trim(0) 歸還記憶體頁，
#      降低車機長跑時記憶體緊缺引發 commIssue 的風險。
# 非 glibc 環境（如 macOS、musl）時兩者皆安全略過並回傳 False。
# =============================================================================
M_ARENA_MAX = -8  # glibc <malloc.h> 常數定義
WEBRTCD_MALLOC_ARENAS = 2
MEMORY_RELEASE_DELAY_S = 1.0  # session 結束後等待 libdatachannel / asyncio 收尾再執行 trim

_libc: ctypes.CDLL | None | bool = None


def _glibc() -> ctypes.CDLL | None:
  """惰性載入 glibc 動態函式庫並宣告 mallopt / malloc_trim 函式簽章。"""
  global _libc
  if _libc is None:
    try:
      lib = ctypes.CDLL("libc.so.6")
      lib.mallopt.argtypes = [ctypes.c_int, ctypes.c_int]
      lib.mallopt.restype = ctypes.c_int
      lib.malloc_trim.argtypes = [ctypes.c_size_t]
      lib.malloc_trim.restype = ctypes.c_int
      _libc = lib
    except (OSError, AttributeError):
      _libc = False
  return _libc or None


def limit_malloc_arenas(n: int = WEBRTCD_MALLOC_ARENAS) -> bool:
  """限制 glibc malloc arena 數量；必須在建立任何背景執行緒之前呼叫。成功回傳 True。"""
  lib = _glibc()
  return lib is not None and lib.mallopt(M_ARENA_MAX, n) == 1


def _rss_mb() -> float | None:
  """從 /proc/self/statm 讀取目前行程實體常駐記憶體（RSS，單位 MB）。"""
  try:
    with open("/proc/self/statm") as f:
      return int(f.read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 1e6
  except (OSError, ValueError, IndexError):
    return None


def release_memory() -> tuple[bool, float | None, float | None]:
  """執行 gc.collect() 後呼叫 malloc_trim(0)，回傳 (是否實際歸還, trim 前 RSS MB, trim 後 RSS MB)。"""
  before = _rss_mb()
  gc.collect()
  lib = _glibc()
  released = bool(lib.malloc_trim(0)) if lib is not None else False
  return released, before, _rss_mb()


def schedule_memory_release(state: ServerState) -> None:
  """
  連線結束後延遲 MEMORY_RELEASE_DELAY_S 秒再歸還記憶體；
  若屆時已有新連線建立則跳過，避免在建立新 session 期間搶佔 CPU 時間。
  """
  def release() -> None:
    if state.streams:
      return
    t0 = time.monotonic()
    released, before, after = release_memory()
    logging.getLogger("webrtcd").info(
      "session ended: malloc_trim released=%s, RSS %s -> %s MB (%.0f ms)",
      released,
      f"{before:.0f}" if before is not None else "?",
      f"{after:.0f}" if after is not None else "?",
      (time.monotonic() - t0) * 1000,
    )

  asyncio.get_running_loop().call_later(MEMORY_RELEASE_DELAY_S, release)


def _json_default(obj: Any) -> Any:
  """處理 Capnp to_dict() 內可能殘留的 bytes 或非原生型別，避免 json.dumps 失敗。"""
  if isinstance(obj, bytes):
    return base64.b64encode(obj).decode("ascii")
  if isinstance(obj, datetime):
    return obj.isoformat()
  return str(obj)


class ThrottledCerealOutgoingProxy(CerealOutgoingMessageProxy):
  """
  精簡節流版 JSON 推播代理（直接沿用上游 CerealOutgoingMessageProxy）。

  設計目的：
  1. 完全沿用原生 webrtcd 的 JSON 封包格式 {"type", "logMonoTime", "valid", "data"}，
     讓遠端 Windows / macOS / Android / iOS 端徹底擺脫 pycapnp 依賴並方便雙向互傳訊息。
  2. 內建逐服務降頻節流（預設 25 Hz），避免 100 Hz 高頻服務（carState、controlsState）塞爆 Wi-Fi。
  3. 支援動態補訂服務 add_services()，解決啟動期間後續才建立的 SubMaster 漏訂問題。
  4. 內建無副作用壅塞退避機制：
     - WebRTC DataChannel 為可靠有序傳輸，佇列無上限；當網路變差時若持續寫入會導致舊車況積壓、
       畫面延遲持續擴大。
     - 網路順暢時 channel.send() 回傳 True，本機制零介入、不增加任何延遲。
     - 僅當 channel.send() 回傳 False（代表封包落入底層緩衝佇列）時暫停發送 CONGESTION_BACKOFF_S，
       期間仍持續呼叫 sm.update(0) 刷新最新值，讓恢復後直接送出最新車況。
     - 刻意不呼叫 buffered_amount()，避開 libdatachannel 實車呼叫後事件迴圈卡死與 SIGSEGV 的問題。
  """

  CONGESTION_BACKOFF_S = 0.2

  def __init__(self, services: list[str], enabled: bool = True, max_hz: float = 25.0):
    valid_services = self._filter_valid_services(services)
    super().__init__(valid_services, enabled=enabled)
    self._min_interval = 1.0 / max(max_hz, 1.0)
    self._last_sent: dict[str, float] = dict.fromkeys(self.services, 0.0)
    # 壅塞退避追蹤狀態（僅於 channel.send() 回傳 False 時啟動）
    self._skipped: int = 0
    self._congested_since: float | None = None
    self._backoff_until: float = 0.0

  @staticmethod
  def _filter_valid_services(services: list[str]) -> list[str]:
    """僅保留合法且非廢棄的 cereal service，避免非法名稱導致 SubMaster 初始化失敗。"""
    valid: list[str] = []
    seen: set[str] = set()
    for s in services:
      if s in seen:
        continue
      if s in log.Event.schema.fields and not s.endswith("DEPRECATED"):
        valid.append(s)
        seen.add(s)
    return valid

  def add_services(self, new_services: list[str]) -> list[str]:
    """動態追加訂閱服務清單，僅在出現尚未訂閱的合法服務時重建 SubMaster。"""
    to_add = [s for s in self._filter_valid_services(new_services) if s not in self.services]
    if not to_add:
      return list(self.services)

    self.services.extend(to_add)
    self.sm = messaging.SubMaster(self.services)
    for s in to_add:
      self._last_sent[s] = 0.0
    return list(self.services)

  def update(self) -> None:
    """
    沿用原生 SubMaster.update(0) 與 to_json()，並依每個 service 獨立節流送出 JSON；
    搭配 channel.send() is False 壅塞退避防止熱點／Wi-Fi 波動造成封包積壓。
    """
    if not self.services:
      return

    open_channels = [ch for ch in self.channels if ch.is_open()]
    if not open_channels:
      return

    now = time.monotonic()
    # 即使處於退避期間仍必須呼叫 sm.update(0)，確保 SubMaster 持續排空舊值並保留最新車況
    self.sm.update(0)

    if now < self._backoff_until:
      self._skipped += sum(1 for updated in self.sm.updated.values() if updated)
      return

    if self._congested_since is not None:
      self.logger.warning(
        "remote_ui data channel recovered after %.1fs, dropped %d messages",
        now - self._congested_since,
        self._skipped,
      )
      self._congested_since = None
      self._skipped = 0

    for service, updated in self.sm.updated.items():
      if not updated:
        continue
      if (now - self._last_sent.get(service, 0.0)) < self._min_interval:
        continue

      msg_dict = self.to_json(self.sm[service])
      mono_time, valid = self.sm.logMonoTime[service], self.sm.valid[service]
      outgoing_msg = {
        "type": service,
        "logMonoTime": mono_time,
        "valid": valid,
        "data": msg_dict,
      }
      encoded_msg = json.dumps(outgoing_msg, default=_json_default).encode("utf-8")
      self._last_sent[service] = now

      sent_now = True
      for channel in open_channels:
        if channel.is_open() and channel.send(encoded_msg) is False:
          sent_now = False

      if not sent_now:
        # 封包已進入底層緩衝佇列：本筆仍會送達，之後暫停 0.2s 讓佇列消化避免延遲累積
        self._backoff_until = now + self.CONGESTION_BACKOFF_S
        if self._congested_since is None:
          self._congested_since = now
          self.logger.warning("remote_ui data channel congested, backing off %.1fs", self.CONGESTION_BACKOFF_S)
        return


class ExtendedStreamSession(StreamSession):
  """
  擴充版 WebRTC 串流工作階段（繼承自原生 StreamSession）。

  擴充功能：
  1. 沿用原生 JSON 服務橋接，並直接建立 ThrottledCerealOutgoingProxy（避免先建上游 SubMaster 又覆蓋浪費記憶體）。
  2. 遠端 UI 模式（remote_ui=True）：
     - 將 WebRTCAnswerBuilder 的 bind_address 設為 None，避免車機同時有 LTE 與 WiFi 時預設路由落在 LTE、
       導致區網／熱點裝置無法建立 ICE 連線。
     - 取消 300 秒強制斷線逾時，比照 body session 持續維持連線。
     - 支援獨立控制視訊串流開關（預設 Offroad 不開影像、進 Onroad 再啟用），關閉影像時不中斷數據橋接。
  3. 擴充 message_handler：攔截 Params 同步讀寫與動態訂閱指令，其餘原生指令完整交回父類處理；
     同時支援透過 HTTP POST /api/control 觸發，避免客戶端呼叫 DataChannel.send() 觸發 GIL 互鎖。
  """

  # 單一參數快照序列化上限（64 KB），避免巨型二進位資料阻塞控制通道，同時完整相容大型 JSON 設定
  MAX_PARAM_JSON_BYTES = 65536

  def __init__(
    self,
    body: StreamRequestBody,
    remote_ui: bool = False,
    video_enabled: bool | None = None,
    max_hz: float = 25.0,
  ):
    from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
    from teleoprtc.builder import WebRTCAnswerBuilder

    self.identifier = str(uuid.uuid4())
    self.params = Params()
    self.remote_ui = remote_ui
    self.max_hz = max_hz
    self.enabled = body.enabled
    self.video_enabled = body.enabled if video_enabled is None else bool(video_enabled)

    # 遠端 UI 不綁定預設路由介面，確保車機同時有 LTE 與 WiFi/熱點時 ICE 能正確產生區網候選位址
    bind_address = None if self.remote_ui else _default_route_ip()
    builder = WebRTCAnswerBuilder(body.sdp, bind_address=bind_address)

    # 遠端 UI 模式下僅由 video_enabled 控制影像軌道與位元率控制器，讓 Offroad 期間不浪費編碼與頻寬
    track_enabled = self.video_enabled if self.remote_ui else self.enabled
    self.video_tracks = []
    for camera in body.cameras:
      track = LiveStreamVideoStreamTrack(camera, track_enabled)
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

    if self.remote_ui:
      # 遠端 UI 永遠配置可動態擴充的節流 JSON 推播代理（即使初始清單為空亦建立）
      self.outgoing_bridge = ThrottledCerealOutgoingProxy(
        body.bridge_services_out,
        enabled=True,
        max_hz=max_hz,
      )
    elif len(body.bridge_services_out) > 0:
      # 非遠端 UI 請求若帶有 outgoing 服務，亦沿用節流代理確保相容性
      self.outgoing_bridge = ThrottledCerealOutgoingProxy(
        body.bridge_services_out,
        enabled=self.enabled,
        max_hz=max_hz,
      )

    self.bitrate_controller = LivestreamBitrateController(
      self.stream.get_receiver_report_stats,
      self.params,
      track_enabled,
    )

    self.run_task: asyncio.Task | None = None
    self._cleanup_lock = asyncio.Lock()
    self._cleanup_done = False
    self.logger = logging.getLogger("webrtcd")
    cloudlog.warning(
      "New stream session (%s), video cameras %s, video enabled %s, incoming services %s, outgoing services %s, remote_ui %s",
      self.identifier,
      [t.id for t in self.video_tracks],
      track_enabled,
      body.bridge_services_in,
      body.bridge_services_out,
      self.remote_ui,
    )

  async def run_normal_session(self) -> None:
    """遠端 UI 模式不套用 300 秒逾時斷線；其餘原生呼叫維持上游 300 秒保護。"""
    if self.remote_ui:
      await self.stream.wait_for_disconnection()
      return
    await super().run_normal_session()

  def _dump_utf8_params(self) -> tuple[dict[str, Any], dict[str, int], dict[str, Any]]:
    """
    傾印 C4 真實 Params 快照、型別表與原生預設值表（零假資料）：
    - snapshot：目前實際寫入 C4 的真實設定值。
    - types：每個 key 對應的 ParamKeyType 整數值。
    - defaults：從 C4 原生 get_default_value() 讀取的真實預設值（供遠端 return_default=True 使用）。
    """
    snapshot: dict[str, Any] = {}
    types: dict[str, int] = {}
    defaults: dict[str, Any] = {}

    for raw_key in self.params.all_keys():
      key = raw_key.decode("utf-8", errors="ignore") if isinstance(raw_key, bytes) else str(raw_key)
      try:
        t = self.params.get_type(key)
        types[key] = int(t)  # 即使目前無值，也讓遠端知道真實型別

        # 讀取 C4 真實預設值（若有定義）
        default_val = self.params.get_default_value(key)
        if default_val is not None and not isinstance(default_val, bytes):
          json_default_val = self._to_jsonable(default_val)
          if len(json.dumps(json_default_val, ensure_ascii=False)) <= self.MAX_PARAM_JSON_BYTES:
            defaults[key] = json_default_val

        # 讀取 C4 目前真實設定值
        val = self.params.get(key)
        if val is None or isinstance(val, bytes):
          continue
        json_val = self._to_jsonable(val)
        if len(json.dumps(json_val, ensure_ascii=False)) <= self.MAX_PARAM_JSON_BYTES:
          snapshot[key] = json_val
      except (TypeError, ValueError, UnicodeError):
        continue
      except Exception:
        self.logger.exception("Failed to serialize Params key %s", key)
        continue

    return snapshot, types, defaults

  def _send_json_message(self, payload: dict[str, Any]) -> None:
    """安全送出 JSON 回覆訊息至 messaging_channel。"""
    if self.stream.has_messaging_channel():
      ch = self.stream.get_messaging_channel()
      if ch.is_open():
        ch.send(json.dumps(payload, default=_json_default))

  def message_handler(self, message: bytes | str) -> None:
    """
    攔截遠端 UI 送來的控制指令與視訊切換指令；其餘原生指令原封不動交回上游 StreamSession。
    """
    try:
      payload = json.loads(message) if isinstance(message, (bytes, str)) else None
      if isinstance(payload, dict):
        # 同時相容 cmd（RemoteParams / 動態訂閱）與 type（webrtcd 原生慣例）
        cmd = payload.get("cmd") or payload.get("type")

        # 遠端 UI 模式下的原生 livestreamVideoEnable：只切換影像串流，不中斷數據通道
        if cmd == "livestreamVideoEnable" and self.remote_ui:
          enabled = bool(payload.get("data", {}).get("enabled", False))
          self.video_enabled = enabled
          for track in self.video_tracks:
            track.enable(enabled)
          if self.bitrate_controller is not None:
            self.bitrate_controller.enable(enabled)
          # 開啟影像時主動向 encoderd 請求關鍵幀（IDR），縮短進入 Onroad 首幀延遲
          self.params.put("LivestreamRequestKeyframe", enabled)
          return

        if cmd == "subscribe_services":
          services = payload.get("services", [])
          if isinstance(services, list) and isinstance(self.outgoing_bridge, ThrottledCerealOutgoingProxy):
            current = self.outgoing_bridge.add_services([str(s) for s in services])
            self._send_json_message({"cmd": "services_subscribed", "services": current})
          return

        if cmd == "get_all_params":
          snapshot, types, defaults = self._dump_utf8_params()
          self._send_json_message({
            "cmd": "all_params",
            "params": snapshot,
            "types": types,
            "defaults": defaults,
          })
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
            # 寫入失敗：回傳 C4 實際值，讓遠端樂觀快取回滾為真實狀態
            self.logger.exception("put_param failed for key %s", key)
            try:
              actual = self._to_jsonable(self.params.get(key))
              if isinstance(actual, bytes):
                actual = None
            except Exception:
              actual = None
            self._send_json_message({"cmd": "param_updated", "key": key, "value": actual})
            return

          self._send_json_message({"cmd": "param_updated", "key": key, "value": ack_value})
          return

        if cmd == "remove_param":
          key = payload.get("key")
          if key is not None:
            self.params.remove(str(key))
            self._send_json_message({"cmd": "param_updated", "key": str(key), "value": None})
          return
    except Exception:
      self.logger.exception("ExtendedStreamSession params command failure")
      return

    # 其餘指令（livestreamCameraSwitch、livestreamSettings、clockSync、bridge_services_in 等）交由上游處理
    super().message_handler(message)

  def _coerce_for_put(self, key: str, value: Any) -> Any:
    """
    依 self.params.get_type(key) 把 JSON 解碼後的值轉成原生 Params.put() 需要的精確型別。
    原生 python2cpp 以 type(dat) 精確比對，例如 int 寫入 FLOAT key 會 TypeError。
    """
    t = self.params.get_type(key)  # key 不存在時會拋 UnknownKeyName

    if t == ParamKeyType.BOOL:
      if isinstance(value, bool):
        return value
      if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
      return bool(value)
    if t == ParamKeyType.INT:
      if isinstance(value, bool):
        return int(value)
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
  抽離 remote_ui、video_enabled 與 max_hz 擴充欄位後再建立上游 StreamRequestBody，
  確保完全不需要修改 openpilot/system/webrtc/helpers.py。
  """
  if content_type != "application/json":
    return _json_response({"error": "unsupported media type"}, status=415)

  raw_dict = json.loads(raw_body)
  remote_ui = bool(raw_dict.pop("remote_ui", False))
  video_enabled = raw_dict.pop("video_enabled", None)
  max_hz = float(raw_dict.pop("max_hz", 25.0))
  # 向後相容舊版欄位，已改為原生 JSON 通道故忽略 raw_capnp
  raw_dict.pop("raw_capnp", None)

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
    session = ExtendedStreamSession(
      body,
      remote_ui=remote_ui,
      video_enabled=video_enabled,
      max_hz=max_hz,
    )
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
      # 連線結束後排程 gc.collect + malloc_trim(0)，將閒置記憶體歸還作業系統
      schedule_memory_release(state)

    if session.run_task is not None:
      session.run_task.add_done_callback(remove_finished_session)

  return _json_response({"sdp": answer.sdp, "type": answer.type})


async def handle_post_control(
  state: ServerState,
  raw_body: bytes,
  content_type: str,
) -> tuple[int, bytes, str]:
  """
  獨立 HTTP 控制端點（POST /api/control）：
  - 接收遠端 UI 送來的 JSON 控制指令（Params 快照／讀寫、動態補訂服務、視訊與鏡頭切換），
    於 asyncio 事件迴圈中直接轉交目前活躍的 ExtendedStreamSession.message_handler() 處理。
  - 讓客戶端徹底免於在高流量下行期間呼叫 DataChannel.send()，根除 nanobind / libdatachannel GIL 互鎖死鎖。
  """
  if content_type != "application/json":
    return _json_response({"error": "unsupported media type"}, status=415)

  try:
    payload = json.loads(raw_body)
  except json.JSONDecodeError:
    return _json_response({"error": "bad request"}, status=400)

  if not isinstance(payload, dict):
    return _json_response({"error": "invalid_request", "message": "payload must be a JSON object"}, status=400)

  # 優先鎖定目前活躍的遠端 UI session，若無則退回最後一個活躍 session
  target_session: StreamSession | None = None
  for session in reversed(list(state.streams.values())):
    if session.run_task is not None and session.run_task.done():
      continue
    if getattr(session, "remote_ui", False):
      target_session = session
      break
    if target_session is None:
      target_session = session

  if target_session is None:
    return _json_response({"error": "no_active_session"}, status=409)

  # 剛建立連線時，等待最多 1 秒讓 server 端 DataChannel on_open 就緒，避免首筆 get_all_params 漏回
  deadline = time.monotonic() + 1.0
  while time.monotonic() < deadline:
    if target_session.stream.has_messaging_channel():
      try:
        if target_session.stream.get_messaging_channel().is_open():
          break
      except Exception:
        pass
    await asyncio.sleep(0.02)

  target_session.message_handler(raw_body)
  return _json_response({"ok": True})


def _enum_schema_to_map(enum_schema: Any) -> dict[str, int]:
  """
  將 pycapnp 的 EnumSchema 轉換為 {列舉名稱: 整數編號} 對照表。
  優先讀取 enumerants 字典，若版本差異無此屬性則退回讀取底層 node.enum.enumerants 索引。
  """
  enumerants = getattr(enum_schema, "enumerants", None)
  if isinstance(enumerants, dict) and enumerants:
    return {str(k): int(v) for k, v in enumerants.items()}

  node = getattr(enum_schema, "node", None)
  if node is not None and node.which() == "enum":
    # Cap'n Proto schema node 內的 enumerants 順序即為真實列舉整數編號（ordinal）
    return {str(e.name): idx for idx, e in enumerate(node.enum.enumerants)}

  return {}


def _extract_enums_from_list(
  list_schema: Any,
  elem_type_proto: Any,
  ancestors: frozenset[int],
) -> dict[str, Any]:
  """遞迴萃取 List 型別元素內部的列舉定義（支援 List(Enum)、List(Struct) 與巢狀 List）。"""
  elem_which = elem_type_proto.which()
  if elem_which not in ("enum", "struct", "list"):
    return {}

  try:
    elem_schema = getattr(list_schema, "elementType", None)
  except Exception:
    return {}

  if elem_schema is None:
    return {}

  if elem_which == "enum":
    enum_map = _enum_schema_to_map(elem_schema)
    return {"__list__": {"__enum__": enum_map}} if enum_map else {}

  if elem_which == "struct":
    sub = _extract_enums_from_struct(elem_schema, ancestors)
    return {"__list__": sub} if sub else {}

  if elem_which == "list":
    sub = _extract_enums_from_list(elem_schema, elem_type_proto.list.elementType, ancestors)
    return {"__list__": sub} if sub else {}

  return {}


def _extract_enums_from_field(
  field: Any,
  ancestors: frozenset[int] = frozenset(),
) -> dict[str, Any]:
  """從單一 Cap'n Proto Struct 欄位遞迴萃取所有真實列舉對照表（僅保留含列舉之分支）。"""
  proto = getattr(field, "proto", None)
  if proto is None:
    return {}

  kind = proto.which()
  if kind == "group":
    return _extract_enums_from_struct(field.schema, ancestors)

  if kind == "slot":
    slot_type = proto.slot.type
    type_which = slot_type.which()
    if type_which == "enum":
      enum_map = _enum_schema_to_map(field.schema)
      return {"__enum__": enum_map} if enum_map else {}
    if type_which == "struct":
      return _extract_enums_from_struct(field.schema, ancestors)
    if type_which == "list":
      return _extract_enums_from_list(field.schema, slot_type.list.elementType, ancestors)

  return {}


def _extract_enums_from_struct(
  struct_schema: Any,
  ancestors: frozenset[int] = frozenset(),
) -> dict[str, Any]:
  """
  遞迴走訪 Cap'n Proto Struct 的所有欄位並收集列舉對照表。
  使用路徑型祖先集合（ancestors）防止自參照循環遞迴，同時允許同結構體於不同兄弟欄位重複展開。
  """
  node = getattr(struct_schema, "node", None)
  node_id = getattr(node, "id", None)
  if node_id is not None:
    if node_id in ancestors:
      return {}
    ancestors = ancestors | {node_id}

  fields = getattr(struct_schema, "fields", None)
  if not isinstance(fields, dict):
    return {}

  result: dict[str, Any] = {}
  for field_name, field in fields.items():
    if field_name.endswith("DEPRECATED"):
      continue
    sub = _extract_enums_from_field(field, ancestors)
    if sub:
      result[field_name] = sub
  return result


def handle_get_enums(services_param: str) -> tuple[int, bytes, str]:
  """
  回傳指定 cereal services 的真實 Cap'n Proto 列舉名稱與整數編號對照表，
  供遠端純 Python SubMaster 精準還原 enum.raw 整數值（不影響原生 /schema 與 DataChannel 格式）。
  """
  services = [s.strip() for s in services_param.split(",") if s.strip()]
  enum_dict: dict[str, Any] = {}
  for s in services:
    if s in log.Event.schema.fields and not s.endswith("DEPRECATED"):
      extracted = _extract_enums_from_field(log.Event.schema.fields[s])
      if extracted:
        enum_dict[s] = extracted
  return _json_response(enum_dict)


def handle_get_version() -> tuple[int, bytes, str]:
  """
  回傳目前裝置的版號與 git commit 資訊（欄位結構對齊 athenad.py getVersion()），
  供 Windows / macOS / Android / iOS App 框架比對是否需要下載新版 UI 套件。
  """
  build_metadata = get_build_metadata()
  return _json_response({
    "version": build_metadata.openpilot.version,
    "remote": build_metadata.openpilot.git_normalized_origin,
    "branch": build_metadata.channel,
    "commit": build_metadata.openpilot.git_commit,
    "commit_date": build_metadata.openpilot.git_commit_date.strip("'").split()[0],
  })


def build_ui_bundle_tarball(base_dir: str = BASEDIR) -> bytes:
  """
  將純 Python UI 程式碼與靜態資源打包成 tar.gz，供遠端 App 框架下載後進行 compileall 編譯：
  - 僅收錄 UI 執行所需目錄，自動排除 __pycache__、.pyc、.so、.dylib 等平台相依產物。
  """
  include_paths = (
    "openpilot/__init__.py",
    "openpilot/selfdrive/__init__.py",
    "openpilot/selfdrive/ui",
    "openpilot/selfdrive/assets",
    "openpilot/system/__init__.py",
    "openpilot/system/ui",
  )

  def _tar_filter(tarinfo: tarfile.TarInfo) -> tarfile.TarInfo | None:
    parts = tarinfo.name.split("/")
    if "__pycache__" in parts or ".git" in parts:
      return None
    if tarinfo.name.endswith((".pyc", ".pyo", ".so", ".dylib", ".dll", ".o")):
      return None
    return tarinfo

  buf = io.BytesIO()
  with tarfile.open(fileobj=buf, mode="w:gz") as tar:
    for rel_path in include_paths:
      full_path = os.path.join(base_dir, rel_path)
      if os.path.exists(full_path):
        tar.add(full_path, arcname=rel_path, filter=_tar_filter)
  return buf.getvalue()


class ExtendedWebrtcdHandler(WebrtcdHandler):
  """
  擴充版 HTTP 請求處理器：
  - POST /stream：導向 handle_get_stream_ext。
  - POST /api/control：接收遠端控制指令並轉交目前活躍 session，避免 DataChannel 雙向互鎖。
  - GET /api/enums：回傳指定服務的真實 Cap'n Proto 列舉編號表。
  - GET /api/version：回傳版本與 git commit 資訊。
  - GET /api/ui-bundle：打包並下載純 Python UI 與資源檔（tar.gz）。
  - 其餘路由（/schema、/notify 等）完整沿用上游 WebrtcdHandler。
  """

  def _dispatch_request(self) -> None:
    parsed = urlparse(self.path)
    try:
      if parsed.path == "/stream" and self.command == "POST":
        result = self._run(
          handle_get_stream_ext(self.server.state, self._read_body(), self.headers.get_content_type())
        )
        self._send(*result)
        return

      if parsed.path == "/api/control" and self.command == "POST":
        result = self._run(
          handle_post_control(self.server.state, self._read_body(), self.headers.get_content_type())
        )
        self._send(*result)
        return

      if parsed.path == "/api/enums" and self.command in ("GET", "HEAD"):
        services = parse_qs(parsed.query).get("services", [""])[0]
        self._send(*handle_get_enums(services))
        return

      if parsed.path == "/api/version" and self.command in ("GET", "HEAD"):
        self._send(*handle_get_version())
        return

      if parsed.path == "/api/ui-bundle" and self.command in ("GET", "HEAD"):
        archive = build_ui_bundle_tarball()
        self._send(200, archive, "application/gzip")
        return
    except Exception as e:
      logging.getLogger("webrtcd").exception("Unhandled error handling %s", self.path)
      self._send(*_json_response({"error": "exception", "message": f"{type(e).__name__}: {e}"}, status=500))
      return

    super()._dispatch_request()


def webrtcd_ext_thread(host: str, port: int, arenas_limited: bool | None = None) -> None:
  """啟動擴充版 webrtcd 服務（結構與上游 webrtcd_thread 一致）。"""
  logging.basicConfig(level=logging.INFO, handlers=[logging.StreamHandler()])
  if arenas_limited is not None:
    logging.getLogger("webrtcd").info(
      "malloc arena limit %d: %s",
      WEBRTCD_MALLOC_ARENAS,
      "on" if arenas_limited else "unavailable (not glibc)",
    )
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
  # 必須在 webrtcd_ext_thread 建立 HTTP / asyncio / libdatachannel 任何執行緒之前限制 malloc arena
  arenas_limited = limit_malloc_arenas()
  parser = argparse.ArgumentParser(description="Extended WebRTC daemon for Remote BIG=1 UI")
  # 預設監聽 0.0.0.0 以允許同區網內的大螢幕裝置直接連線
  parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to listen on")
  parser.add_argument("--port", type=int, default=5001, help="Port to listen on")
  args = parser.parse_args()

  webrtcd_ext_thread(args.host, args.port, arenas_limited)


if __name__ == "__main__":
  main()