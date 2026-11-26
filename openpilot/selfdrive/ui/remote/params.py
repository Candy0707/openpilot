from __future__ import annotations

import json
import threading
from typing import Any
import datetime

from openpilot.common.params import Params as _NativeParams, ParamKeyType
from openpilot.selfdrive.ui.remote.webrtc_client import RemoteWebRTCClient


class Params(_NativeParams):
  """
  遠端版 Params（繼承自原生 openpilot.common.params.Params）。

  設計原則：
  1. 刻意不呼叫 super().__init__()，避免在桌面或平板端讀寫本機 /data/params 目錄。
  2. 讀取走本機共用快取（0 ms 延遲）：確保 ui.py 渲染迴圈與背景更新執行緒絕不卡頓。
  3. 寫入採樂觀更新（Optimistic Update）：立即更新本機快取以提供即時點擊回饋，
     同時透過 WebRTC messaging_channel 非同步寫回 C4 本機 Params。
  4. 相容 UIStateSP 特殊呼叫方式：支援 return_default=True、block=True，以及
     JSON/數值字串自動解析防呆。
  """

  # 跨實例共用快取（因各設定子頁面會各自實例化 Params()）
  _shared_cache: dict[str, Any] = {}
  _shared_types: dict[str, ParamKeyType] = {}
  _cache_lock = threading.Lock()
  _registered = False

  def __init__(self, d: str | None = None):
    self._client = RemoteWebRTCClient.get_instance()

    # 僅註冊控制訊息回呼，不在這裏呼叫 start()，把連線啟動權交給 SubMaster
    # 以確保所有服務清單皆已註冊完畢後才向 C4 發出第一次信令握手
    with self._cache_lock:
      if not Params._registered:
        self._client.add_control_callback(self._on_control_message)
        Params._registered = True

  @classmethod
  def _on_control_message(cls, data: dict[str, Any]) -> None:
    cmd = data.get("cmd")
    with cls._cache_lock:
      if cmd == "all_params":
        types = data.get("types", {})
        if isinstance(types, dict):
          for k, t in types.items():
            try:
              cls._shared_types[str(k)] = ParamKeyType(int(t))
            except (ValueError, TypeError):
              pass
        params_dict = data.get("params", {})
        if isinstance(params_dict, dict):
          for k, v in params_dict.items():
            if v is not None:
              cls._shared_cache[str(k)] = cls._coerce(str(k), v)
      elif cmd == "param_updated":
        key = data.get("key")
        val = data.get("value")
        if key is not None:
          if val is None:
            cls._shared_cache.pop(str(key), None)
          else:
            cls._shared_cache[str(key)] = cls._coerce(str(key), val)

  @classmethod
  def _coerce(cls, key: str, raw_val: Any) -> Any:
    """依 C4 同步的 ParamKeyType 將值還原為原生 Params.get() 會回傳的型別。呼叫端需已持有 _cache_lock 或不需鎖。"""
    if raw_val is None:
      return None
    t = cls._shared_types.get(key)
    try:
      if t == ParamKeyType.BOOL:
        if isinstance(raw_val, str):
          return raw_val.strip().lower() in ("1", "true")
        return bool(raw_val)
      if t == ParamKeyType.INT:
        return int(float(raw_val))
      if t == ParamKeyType.FLOAT:
        return float(raw_val)
      if t == ParamKeyType.STRING:
        return raw_val if isinstance(raw_val, str) else str(raw_val)
      if t == ParamKeyType.TIME:
        if isinstance(raw_val, datetime.datetime):
          return raw_val
        return datetime.datetime.fromisoformat(str(raw_val))
      if t == ParamKeyType.JSON:
        if isinstance(raw_val, str):
          return json.loads(raw_val)
        return raw_val
    except (ValueError, TypeError):
      return raw_val
    return cls._parse_value(raw_val)  # 型別未知時退回原本的啟發式解析

  @staticmethod
  def _parse_value(raw_val: Any) -> Any:
    if raw_val is None or not isinstance(raw_val, str):
      return raw_val
    stripped = raw_val.strip()
    if (stripped.startswith("{") and stripped.endswith("}")) or (stripped.startswith("[") and stripped.endswith("]")):
      try:
        return json.loads(stripped)
      except json.JSONDecodeError:
        pass
    return raw_val

  # =========================================================================
  # 讀取介面（覆寫原生底層 I/O，改讀記憶體快取）
  # =========================================================================

  def get(self, key: str, block: bool = False, return_default: bool = False) -> Any:
    with self._cache_lock:
      return self._shared_cache.get(key)  # 快取內已是轉型後的值

  def get_type(self, key: str) -> ParamKeyType:
    """遠端版 get_type：讀 C4 同步的型別表，未知時由 Python 型別推斷。"""
    with self._cache_lock:
      t = self._shared_types.get(key)
      if t is not None:
        return t
      val = self._shared_cache.get(key)
    if isinstance(val, bool):
      return ParamKeyType.BOOL
    if isinstance(val, int):
      return ParamKeyType.INT
    if isinstance(val, float):
      return ParamKeyType.FLOAT
    if isinstance(val, datetime.datetime):
      return ParamKeyType.TIME
    if isinstance(val, (dict, list)):
      return ParamKeyType.JSON
    return ParamKeyType.STRING

  def get_bool(self, key: str, block: bool = False) -> bool:
    """取得布林型態設定值（'1' 或 'true' 視為 True）。"""
    val = self.get(key)
    if val is None:
      return False
    if isinstance(val, bool):
      return val
    if isinstance(val, bytes):
      return val == b"1"
    return str(val).lower() in ("1", "true")

  def get_int(self, key: str, block: bool = False) -> int:
    """取得整數型態設定值，失敗時回傳 0。"""
    val = self.get(key)
    try:
      return int(float(val)) if val is not None else 0
    except (ValueError, TypeError):
      return 0

  def get_float(self, key: str, block: bool = False) -> float:
    """取得浮點數型態設定值，失敗時回傳 0.0。"""
    val = self.get(key)
    try:
      return float(val) if val is not None else 0.0
    except (ValueError, TypeError):
      return 0.0

  # =========================================================================
  # 寫入與刪除介面（樂觀更新本機快取 + 非同步送往 C4）
  # =========================================================================

  def put(self, key: str, value: Any, block: bool = False) -> None:
    with self._cache_lock:
      self._shared_cache[key] = self._coerce(key, value)

    payload_value = value.isoformat() if isinstance(value, datetime.datetime) else value
    self._client.send_control_command({
      "cmd": "put_param",
      "key": key,
      "value": payload_value,
      "block": block,
    })

  def put_bool(self, key: str, value: bool, block: bool = False) -> None:
    """寫入布林設定值並同步至 C4。"""
    self.put(key, value, block=block)

  def put_nonblocking(self, key: str, value: Any) -> None:
    """非阻塞寫入（遠端模式下所有 put 本身皆為非阻塞）。"""
    self.put(key, value)

  def put_bool_nonblocking(self, key: str, value: bool) -> None:
    """非阻塞寫入布林設定值。"""
    self.put_bool(key, value)

  def remove(self, key: str) -> None:
    """刪除指定設定值並同步至 C4。"""
    with self._cache_lock:
      self._shared_cache.pop(key, None)

    self._client.send_control_command({
      "cmd": "remove_param",
      "key": key,
    })

  def check_key(self, key: str) -> bool:
    """檢查 key 是否存在於快取中。"""
    with self._cache_lock:
      return key in self._shared_cache

  def all_keys(self) -> list[bytes]:
    """回傳目前快取中的所有 key（格式對齊原生回傳 list[bytes]）。"""
    with self._cache_lock:
      return [k.encode("utf-8") for k in self._shared_cache]
