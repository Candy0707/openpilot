from __future__ import annotations

import datetime
from enum import IntEnum, IntFlag
import json
import threading
from typing import Any

from openpilot.selfdrive.ui.remote.webrtc_client import RemoteWebRTCClient


class ParamKeyFlag(IntFlag):
  """完全對齊 openpilot/common/params.py 的 ParamKeyFlag 定義（不依賴 libparams_c.so）。"""

  PERSISTENT = 0x02
  CLEAR_ON_MANAGER_START = 0x04
  CLEAR_ON_ONROAD_TRANSITION = 0x08
  CLEAR_ON_OFFROAD_TRANSITION = 0x10
  DEVELOPMENT_ONLY = 0x40
  CLEAR_ON_IGNITION_ON = 0x80
  BACKUP = 0x100
  ALL = 0xFFFFFFFF


class ParamKeyType(IntEnum):
  """完全對齊 openpilot/common/params.py 的 ParamKeyType 定義（不依賴 libparams_c.so）。"""

  STRING = 0
  BOOL = 1
  INT = 2
  FLOAT = 3
  TIME = 4
  JSON = 5
  BYTES = 6


class UnknownKeyName(Exception):
  """完全對齊原生 Params 的未知參數鍵例外。"""


class Params:
  """
  遠端版 Params（純 Python 實作，透過 sys.modules 取代原生 openpilot.common.params.Params）。

  設計原則：
  1. 零 C 擴充依賴：完全不載入 libparams_c.so，可在 Windows / macOS / Android / iOS 直接運行。
  2. 零假資料快取：快取只保存 C4 實際同步回來的真實數值（_shared_cache）、真實型別表
     （_shared_types）與真實預設值表（_shared_defaults），絕不硬編碼任何假資料。
  3. 讀取 0 ms 延遲：確保 ui.py 60 FPS 渲染迴圈絕不因網路 I/O 卡頓。
  4. 寫入採樂觀更新＋失敗回滾：先更新本機快取提供即時回饋，同時透過 WebRTC 寫回 C4；
     若 C4 端驗證失敗會回傳真實值自動回滾快取。
  """

  # 跨實例共用快取（因各設定子頁面會各自實例化 Params()）
  _shared_cache: dict[str, Any] = {}
  _shared_types: dict[str, ParamKeyType] = {}
  _shared_defaults: dict[str, Any] = {}
  _cache_lock = threading.Lock()
  _registered = False

  def __init__(self, d: str | None = None):
    self.d = d or ""
    self._client = RemoteWebRTCClient.get_instance()

    # 僅註冊控制訊息回呼，絕不在此偷跑 start()，把連線啟動權交給 connect_remote_ui()
    with self._cache_lock:
      if not Params._registered:
        self._client.add_control_callback(self._on_control_message)
        Params._registered = True

  @classmethod
  def reset_cache(cls) -> None:
    """清空共用快取與註冊狀態（供單元測試或重新連線至新裝置時使用）。"""
    with cls._cache_lock:
      cls._shared_cache.clear()
      cls._shared_types.clear()
      cls._shared_defaults.clear()
      cls._registered = False

  @classmethod
  def _on_control_message(cls, data: dict[str, Any]) -> None:
    """
    [背景執行緒] 處理來自 C4 messaging_channel 的 Params 真實同步訊息：
    - all_params：取代整個快取、型別表與預設值表，確保無殘留過期資料。
    - param_updated：單鍵寫入確認、刪除或失敗回滾。
    """
    cmd = data.get("cmd")
    with cls._cache_lock:
      if cmd == "all_params":
        cls._shared_types.clear()
        cls._shared_defaults.clear()
        cls._shared_cache.clear()

        types = data.get("types", {})
        if isinstance(types, dict):
          for k, t in types.items():
            try:
              cls._shared_types[str(k)] = ParamKeyType(int(t))
            except (ValueError, TypeError):
              pass

        defaults = data.get("defaults", {})
        if isinstance(defaults, dict):
          for k, v in defaults.items():
            if v is not None:
              cls._shared_defaults[str(k)] = cls._coerce(str(k), v)

        params_dict = data.get("params", {})
        if isinstance(params_dict, dict):
          for k, v in params_dict.items():
            if v is not None:
              cls._shared_cache[str(k)] = cls._coerce(str(k), v)

      elif cmd == "param_updated":
        key = data.get("key")
        val = data.get("value")
        if key is not None:
          str_key = str(key)
          if val is None:
            cls._shared_cache.pop(str_key, None)
          else:
            cls._shared_cache[str_key] = cls._coerce(str_key, val)

  @classmethod
  def _coerce(cls, key: str, raw_val: Any) -> Any:
    """依 C4 同步的 ParamKeyType 將值還原為原生 Params.get() 會回傳的精確型別。"""
    if raw_val is None:
      return None
    t = cls._shared_types.get(key)
    try:
      if t == ParamKeyType.BOOL:
        if isinstance(raw_val, bool):
          return raw_val
        if isinstance(raw_val, str):
          return raw_val.strip().lower() in ("1", "true", "yes", "on")
        return bool(raw_val)
      if t == ParamKeyType.INT:
        if isinstance(raw_val, bool):
          return int(raw_val)
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
    except (ValueError, TypeError, json.JSONDecodeError):
      return raw_val
    return raw_val

  # =========================================================================
  # 讀取介面（全走真實快取，零假資料）
  # =========================================================================

  def get(self, key: str, block: bool = False, return_default: bool = False) -> Any:
    """
    取得指定 key 的真實設定值。
    若尚未設定且 return_default=True，僅回傳從 C4 同步來的真實預設值（無則回傳 None）。
    """
    str_key = key.decode("utf-8") if isinstance(key, bytes) else str(key)
    with self._cache_lock:
      val = self._shared_cache.get(str_key)
      if val is None and return_default:
        return self._shared_defaults.get(str_key)
      return val

  def get_default_value(self, key: str) -> Any:
    """回傳從 C4 同步來的真實 schema 預設值。"""
    str_key = key.decode("utf-8") if isinstance(key, bytes) else str(key)
    with self._cache_lock:
      return self._shared_defaults.get(str_key)

  def get_type(self, key: str) -> ParamKeyType:
    """遠端版 get_type：讀 C4 同步的真實型別表，未知時由值型別推斷。"""
    str_key = key.decode("utf-8") if isinstance(key, bytes) else str(key)
    with self._cache_lock:
      t = self._shared_types.get(str_key)
      if t is not None:
        return t
      val = self._shared_cache.get(str_key)
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
    """取得布林型態設定值（無值時回傳 False，對齊原生 params_get_bool 行為）。"""
    val = self.get(key)
    if val is None:
      return False
    if isinstance(val, bool):
      return val
    if isinstance(val, bytes):
      return val == b"1"
    return str(val).strip().lower() in ("1", "true")

  def get_int(self, key: str, block: bool = False) -> int:
    """取得整數型態設定值。"""
    val = self.get(key)
    try:
      return int(float(val)) if val is not None else 0
    except (ValueError, TypeError):
      return 0

  def get_float(self, key: str, block: bool = False) -> float:
    """取得浮點數型態設定值。"""
    val = self.get(key)
    try:
      return float(val) if val is not None else 0.0
    except (ValueError, TypeError):
      return 0.0

  # =========================================================================
  # 寫入與刪除介面（樂觀更新本機快取 + 非同步送往 C4）
  # =========================================================================

  def put(self, key: str, value: Any, block: bool = False) -> None:
    """寫入設定值並透過 WebRTC 非同步同步至 C4（寫入失敗由 C4 回傳真實值自動回滾）。"""
    str_key = key.decode("utf-8") if isinstance(key, bytes) else str(key)
    with self._cache_lock:
      self._shared_cache[str_key] = self._coerce(str_key, value)

    payload_value = value.isoformat() if isinstance(value, datetime.datetime) else value
    self._client.send_control_command({
      "cmd": "put_param",
      "key": str_key,
      "value": payload_value,
      "block": block,
    })

  def put_bool(self, key: str, value: bool, block: bool = False) -> None:
    """寫入布林設定值並同步至 C4。"""
    self.put(key, bool(value), block=block)

  def put_nonblocking(self, key: str, value: Any) -> None:
    """非阻塞寫入（遠端模式下所有 put 本身皆為非阻塞）。"""
    self.put(key, value)

  def put_bool_nonblocking(self, key: str, value: bool) -> None:
    """非阻塞寫入布林設定值。"""
    self.put_bool(key, value)

  def remove(self, key: str) -> None:
    """刪除指定設定值並同步至 C4。"""
    str_key = key.decode("utf-8") if isinstance(key, bytes) else str(key)
    with self._cache_lock:
      self._shared_cache.pop(str_key, None)

    self._client.send_control_command({
      "cmd": "remove_param",
      "key": str_key,
    })

  def check_key(self, key: str | bytes) -> bytes:
    """檢查 key 是否存在於 C4 同步的型別表或快取中（對齊原生回傳 bytes 鍵名）。"""
    str_key = key.decode("utf-8") if isinstance(key, bytes) else str(key)
    with self._cache_lock:
      exists = (str_key in self._shared_types) or (str_key in self._shared_cache)
    if not exists:
      raise UnknownKeyName(str_key)
    return str_key.encode("utf-8")

  def all_keys(self, flag: ParamKeyFlag = ParamKeyFlag.ALL) -> list[bytes]:
    """回傳目前已知的所有 key（格式對齊原生回傳 list[bytes]）。"""
    with self._cache_lock:
      keys = sorted(set(self._shared_types.keys()) | set(self._shared_cache.keys()))
      return [k.encode("utf-8") for k in keys]

  def clear_all(self, tx_flag: ParamKeyFlag = ParamKeyFlag.ALL) -> None:
    """遠端模式下不允許單方面清空裝置端所有參數（安全空操作）。"""
    return
