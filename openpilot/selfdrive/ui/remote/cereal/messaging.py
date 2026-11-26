from __future__ import annotations

import threading
import time
from typing import Any

from openpilot.selfdrive.ui.remote.webrtc_client import RemoteWebRTCClient


class RemoteEnum(str):
  """
  相容 Cap'n Proto _DynamicEnum 的列舉字串子類（純 Python，零 pycapnp 依賴）：
  - 繼承自 str，可直接與列舉名稱字串比對（例如 == 'wifi'、== 'good'）。
  - .raw 屬性回傳真實的整數列舉編號（int），完整支援 sidebar.py 等元件的數值比較與運算
    （如 strength.raw > 0、strength.raw + 1）。
  - 支援與 int、str 及原生 Capnp 列舉物件（具 int .raw 屬性）進行相等比較，且維持可雜湊性。
  """

  __slots__ = ("_raw",)

  def __new__(cls, name: str, raw: int = 0) -> RemoteEnum:
    obj = super().__new__(cls, name)
    obj._raw = int(raw)
    return obj

  @property
  def raw(self) -> int:
    return self._raw

  def __int__(self) -> int:
    return self._raw

  def __eq__(self, other: object) -> bool:
    if isinstance(other, RemoteEnum):
      return self._raw == other._raw and super().__eq__(other)
    if isinstance(other, int) and not isinstance(other, bool):
      return self._raw == other
    if isinstance(other, str):
      return super().__eq__(other)
    other_raw = getattr(other, "raw", None)
    if isinstance(other_raw, int) and not isinstance(other_raw, bool):
      return self._raw == other_raw
    return NotImplemented

  def __ne__(self, other: object) -> bool:
    eq = self.__eq__(other)
    if eq is NotImplemented:
      return NotImplemented
    return not eq

  def __hash__(self) -> int:
    return super().__hash__()


def _enum_zero_value(enum_map: dict[str, int]) -> RemoteEnum:
  """從 C4 真實列舉表找出編號 0 對應的列舉名稱並回傳 RemoteEnum(name, 0)。"""
  for name, raw_val in enum_map.items():
    if raw_val == 0:
      return RemoteEnum(name, 0)
  return RemoteEnum("", 0)


def _wrap_enum_value(value: Any, enum_map: dict[str, int]) -> RemoteEnum:
  """依據 C4 真實列舉對照表，將 JSON 字串或整數還原為具正確 .raw 整數編號的 RemoteEnum。"""
  if isinstance(value, str):
    return RemoteEnum(value, int(enum_map.get(value, 0)))
  if isinstance(value, int) and not isinstance(value, bool):
    for name, raw_val in enum_map.items():
      if raw_val == value:
        return RemoteEnum(name, value)
    return RemoteEnum(str(value), value)
  return _enum_zero_value(enum_map)


def _wrap_json_value(
  value: Any,
  enum_node: Any = None,
  schema_node: Any = None,
) -> Any:
  """
  遞迴將原生 webrtcd 送來的 JSON 資料包裝為支援屬性點存取與真實列舉編號的唯讀結構。
  """
  # 若此節點在 C4 schema 中為列舉型別，直接包裝為 RemoteEnum（含正確 int .raw）
  if isinstance(enum_node, dict) and "__enum__" in enum_node:
    return _wrap_enum_value(value, enum_node["__enum__"])

  if isinstance(value, dict):
    child_enum = enum_node if isinstance(enum_node, dict) else None
    child_schema = schema_node if isinstance(schema_node, dict) else None
    return RemoteStruct(value, enum_map=child_enum, schema=child_schema)

  if isinstance(value, list):
    elem_enum = enum_node.get("__list__") if isinstance(enum_node, dict) else None
    elem_schema = schema_node[0] if isinstance(schema_node, list) and len(schema_node) > 0 else None
    return [_wrap_json_value(v, elem_enum, elem_schema) for v in value]

  return value


def _schema_to_zero_skeleton(schema_node: Any, enum_node: Any = None) -> Any:
  """
  依據 C4 原生 GET /schema 與 GET /api/enums 回傳的真實結構建立零值骨架：
  - 僅用於尚未收到首包的服務，或 Cap'n Proto to_dict() 省略之未初始化指標欄位。
  - 原生 /schema 真實格式為：型別字串（"bool"、"uint32"、"float32"、"text" 等）、
    陣列（["float32"]、[{...}]）與巢狀欄位字典（{"technology": "text", ...}）。
  - 嚴格不偽造任何非零假數據；列舉欄位一律對齊編號 0 的真實列舉值。
  """
  if isinstance(enum_node, dict) and "__enum__" in enum_node:
    return _enum_zero_value(enum_node["__enum__"])

  if isinstance(schema_node, str):
    if schema_node == "bool":
      return False
    if schema_node in ("int8", "int16", "int32", "int64", "uint8", "uint16", "uint32", "uint64"):
      return 0
    if schema_node in ("float32", "float64"):
      return 0.0
    if schema_node in ("text", "data"):
      return ""
    return None

  if isinstance(schema_node, list):
    return []

  if isinstance(schema_node, dict):
    child_enum = enum_node if isinstance(enum_node, dict) else None
    return RemoteStruct({}, enum_map=child_enum, schema=schema_node)

  return 0


class RemoteStruct:
  """
  對應 Capnp StructReader 的輕量唯讀屬性包裝器（純 Python，零 pycapnp 依賴）。
  支援：
  - 屬性點存取：obj.vEgo、obj.laneLines[0].x、obj.networkStrength.raw
  - 字典鍵存取：obj['vEgo']
  - 自動補齊 Cap'n Proto to_dict() 省略之未初始化欄位（依 C4 真實 schema 延遲產生零值）
  - 轉回純字典：obj.to_dict()
  """

  __slots__ = ("_raw", "_enum_map", "_schema", "_cache")

  def __init__(
    self,
    data: dict[str, Any],
    enum_map: dict[str, Any] | None = None,
    schema: dict[str, Any] | None = None,
  ):
    object.__setattr__(self, "_raw", data)
    object.__setattr__(self, "_enum_map", enum_map)
    object.__setattr__(self, "_schema", schema)
    object.__setattr__(self, "_cache", {})

  def __getattr__(self, name: str) -> Any:
    cache: dict[str, Any] = object.__getattribute__(self, "_cache")
    if name in cache:
      return cache[name]

    raw: dict[str, Any] = object.__getattribute__(self, "_raw")
    enum_map: dict[str, Any] | None = object.__getattribute__(self, "_enum_map")
    schema: dict[str, Any] | None = object.__getattribute__(self, "_schema")

    child_enum = enum_map.get(name) if isinstance(enum_map, dict) else None
    child_schema = schema.get(name) if isinstance(schema, dict) else None

    if name in raw:
      wrapped = _wrap_json_value(raw[name], child_enum, child_schema)
      cache[name] = wrapped
      return wrapped

    # Cap'n Proto to_dict() 會省略未設定的子結構／清單／空指標，依真實 schema 回傳零值
    if isinstance(schema, dict) and name in schema:
      zero_val = _schema_to_zero_skeleton(child_schema, child_enum)
      cache[name] = zero_val
      return zero_val

    raise AttributeError(f"RemoteStruct 沒有欄位 '{name}'")

  def __getitem__(self, key: str) -> Any:
    return getattr(self, key)

  def __contains__(self, key: str) -> bool:
    raw: dict[str, Any] = object.__getattribute__(self, "_raw")
    schema: dict[str, Any] | None = object.__getattribute__(self, "_schema")
    return key in raw or (isinstance(schema, dict) and key in schema)

  def to_dict(self) -> dict[str, Any]:
    return dict(object.__getattribute__(self, "_raw"))

  def which(self) -> str:
    """相容 Capnp union 欄位的 which() 查詢（僅檢查實際收到的 _raw 啟用鍵）。"""
    raw: dict[str, Any] = object.__getattribute__(self, "_raw")
    for k, v in raw.items():
      if v is not None:
        return k
    return ""

  def __repr__(self) -> str:
    return f"RemoteStruct({object.__getattribute__(self, '_raw')!r})"


class _MutableMessage:
  """
  提供給 messaging.new_message('uiDebug') 使用的輕量可寫結構，
  讓 ui.py 寫入 msg.uiDebug.cpuTimeMillis 等效能統計時完全不依賴 pycapnp。
  """

  def __init__(self, service: str):
    self._service = service
    self.valid = True
    self.logMonoTime = int(time.monotonic() * 1e9)
    setattr(self, service, _MutableNamespace())

  def to_bytes(self) -> bytes:
    return b""


class _MutableNamespace:
  """任意屬性可寫入的輕量命名空間。"""


def new_message(service: str, size: int | None = None, valid: bool = True) -> _MutableMessage:
  """純 Python 版 new_message（供 ui.py 建立 uiDebug 等本機遙測訊息用）。"""
  msg = _MutableMessage(service)
  msg.valid = valid
  return msg


class SubMaster:
  """
  遠端版 SubMaster（純 Python 實作，零 msgq / pycapnp 依賴）。

  設計說明：
  1. 完整實作原生 SubMaster 的公開介面（__getitem__、update、all_alive、all_valid、all_checks 等）。
  2. 初始化時向 C4 拉取真實 /schema 與 /api/enums，用於建立零值骨架並精準還原所有列舉 .raw 整數值。
  3. 透過 RemoteWebRTCClient 動態訂閱 services，背景執行緒收到原生 webrtcd 送來的
     JSON 封包後包裝為 RemoteStruct 放入暫存區；主執行緒呼叫 update() 時微秒級交換同步。
  4. 嚴格區分真實資料與未收包狀態：收到 C4 首筆真實封包前 seen=False / alive=False；
     絕不在 __init__ 偷跑連線，由 connect_remote_ui() 統一啟動並等待核心資料就緒後才開窗。
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

    # 與原生 SubMaster 一致的狀態追蹤字典
    self.updated: dict[str, bool] = dict.fromkeys(self.services, False)
    self.seen: dict[str, bool] = dict.fromkeys(self.services, False)
    self.alive: dict[str, bool] = dict.fromkeys(self.services, False)
    self.valid: dict[str, bool] = dict.fromkeys(self.services, True)
    self.recv_time: dict[str, float] = dict.fromkeys(self.services, 0.0)
    self.recv_frame: dict[str, int] = dict.fromkeys(self.services, 0)
    self.logMonoTime: dict[str, int] = dict.fromkeys(self.services, 0)
    self.sock: dict[str, Any] = dict.fromkeys(self.services, None)

    # 執行緒安全暫存區：背景執行緒寫入，主執行緒在 update() 取走
    # 格式：{service: (wrapped_data, logMonoTime, valid, recv_mono)}
    self._staging_lock = threading.Lock()
    self._staging_msgs: dict[str, tuple[Any, int, bool, float]] = {}

    # 判定 service 存活的超時閾值（秒）
    self._alive_timeout = 2.0

    # 綁定全域 WebRTC 客戶端並註冊服務清單（不在此呼叫 start()，避免搶先握手漏訂服務）
    self._client = RemoteWebRTCClient.get_instance()

    # 向 C4 拉取真實 /schema 與 /api/enums（僅作零值骨架與列舉編號對照，絕不捏造假資料）
    self._schemas: dict[str, Any] = self._client.fetch_schema(self.services)
    self._enums: dict[str, Any] = self._client.fetch_enums(self.services)

    self.data: dict[str, Any] = {}
    for s in self.services:
      self.data[s] = _schema_to_zero_skeleton(self._schemas.get(s, {}), self._enums.get(s))

    self._client.register_services(self.services)
    self._client.add_service_callback(self._on_service_json)

  def __getitem__(self, s: str) -> Any:
    return self.data[s]

  def _on_service_json(self, service: str, payload: Any, log_mono_time: int, valid: bool) -> None:
    """
    [背景執行緒] 收到 C4 原生 webrtcd 推播的 JSON 服務封包時觸發。
    依據 C4 真實 schema 與列舉表完成包裝，僅保留每個 service 最新一筆真實資料。
    """
    if service not in self.data:
      return

    wrapped = _wrap_json_value(payload, self._enums.get(service), self._schemas.get(service))
    now = time.monotonic()
    with self._staging_lock:
      self._staging_msgs[service] = (wrapped, log_mono_time, valid, now)

  def update(self, timeout: int = 100) -> None:
    """
    [主執行緒] 將背景收到的最新遠端真實狀態同步至 self.data。
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

    # 更新本幀收到的真實資料
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

  def all_alive(self, service_list: list[str] | None = None) -> bool:
    return all(self.alive[s] for s in (service_list or self.services))

  def all_valid(self, service_list: list[str] | None = None) -> bool:
    return all(self.valid[s] for s in (service_list or self.services))

  def all_checks(self, service_list: list[str] | None = None) -> bool:
    target = service_list or self.services
    return self.all_alive(target) and self.all_valid(target)


class PubMaster:
  """
  遠端版 PubMaster（純 Python 安全 No-op 實作）。

  設計說明：
  ui.py 會建立 PubMaster(['uiDebug']) 並每幀發送效能統計。
  遠端裝置無本機 msgq 環境，因此覆寫為安全空操作，讓上層 ui.py 無需額外判斷。
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
