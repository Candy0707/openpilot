from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

# =============================================================================
# 遠端 UI 專用 WifiManager 安全空實作替身（No-Op Stub）
#
# 設計目的：
# 1. 原生 openpilot/system/ui/lib/wifi_manager.py 會透過 jeepney 連線本機 Linux
#    系統 D-Bus（NetworkManager）並啟動未受 stop() 完整回收的初始化背景執行緒，
#    在 WSL2 / PC 遠端 UI 關閉時容易觸發 RouterClosed 與 Bad file descriptor 例外。
# 2. 本模組完全不匯入 jeepney / networkmanager、不建立本機 D-Bus 連線、不啟動任何
#    背景執行緒與 atexit 鉤子，同時確保 Windows / macOS / Android 跨平台零阻礙啟動。
# 3. 完整對齊上游公開常數、列舉、資料類別與 WifiManager 介面，讓 network.py 與
#    tici_updater.py 無需任何修改即可正常載入與渲染。
# =============================================================================

TETHERING_IP_ADDRESS = "192.168.43.1"
DEFAULT_TETHERING_PASSWORD = "swagswagcomma"
SIGNAL_QUEUE_SIZE = 10
SCAN_PERIOD_SECONDS = 5

DEBUG = False

# NetworkManager 802.11 AP 旗標純整數常數（供 get_security_type 獨立運作，免匯入 networkmanager）
_NM_802_11_AP_FLAGS_NONE = 0x00000000
_NM_802_11_AP_FLAGS_PRIVACY = 0x00000001
_NM_802_11_AP_FLAGS_WPS = 0x00000002
_NM_802_11_AP_SEC_PAIR_WEP40 = 0x00000001
_NM_802_11_AP_SEC_PAIR_WEP104 = 0x00000002
_NM_802_11_AP_SEC_GROUP_WEP40 = 0x00000040
_NM_802_11_AP_SEC_GROUP_WEP104 = 0x00000080
_NM_802_11_AP_SEC_KEY_MGMT_PSK = 0x00000100
_NM_802_11_AP_SEC_KEY_MGMT_802_1X = 0x00000200


def normalize_ssid(ssid: str) -> str:
  return ssid.replace("’", "'")  # for iPhone hotspots


class SecurityType(IntEnum):
  OPEN = 0
  WPA = 1
  WPA2 = 2
  WPA3 = 3
  UNSUPPORTED = 4


class MeteredType(IntEnum):
  UNKNOWN = 0
  YES = 1
  NO = 2


def get_security_type(flags: int, wpa_flags: int, rsn_flags: int) -> SecurityType:
  wpa_props = wpa_flags | rsn_flags

  # obtained by looking at flags of networks in the office as reported by an Android phone
  supports_wpa = (
    _NM_802_11_AP_SEC_PAIR_WEP40
    | _NM_802_11_AP_SEC_PAIR_WEP104
    | _NM_802_11_AP_SEC_GROUP_WEP40
    | _NM_802_11_AP_SEC_GROUP_WEP104
    | _NM_802_11_AP_SEC_KEY_MGMT_PSK
  )

  if (flags == _NM_802_11_AP_FLAGS_NONE) or ((flags & _NM_802_11_AP_FLAGS_WPS) and not (wpa_props & supports_wpa)):
    return SecurityType.OPEN
  elif (
    (flags & _NM_802_11_AP_FLAGS_PRIVACY)
    and (wpa_props & supports_wpa)
    and not (wpa_props & _NM_802_11_AP_SEC_KEY_MGMT_802_1X)
  ):
    return SecurityType.WPA
  else:
    return SecurityType.UNSUPPORTED


@dataclass(frozen=True)
class Network:
  ssid: str
  strength: int
  security_type: SecurityType
  is_tethering: bool

  @classmethod
  def from_dbus(cls, ssid: str, aps: list[AccessPoint], is_tethering: bool) -> Network:
    # we only want to show the strongest AP for each Network/SSID
    strongest_ap = max(aps, key=lambda ap: ap.strength)
    security_type = get_security_type(strongest_ap.flags, strongest_ap.wpa_flags, strongest_ap.rsn_flags)

    return cls(
      ssid=ssid,
      strength=100 if is_tethering else strongest_ap.strength,
      security_type=security_type,
      is_tethering=is_tethering,
    )


@dataclass(frozen=True)
class AccessPoint:
  ssid: str
  bssid: str
  strength: int
  flags: int
  wpa_flags: int
  rsn_flags: int
  ap_path: str

  @classmethod
  def from_dbus(cls, ap_props: dict[str, tuple[str, Any]], ap_path: str) -> AccessPoint:
    ssid = bytes(ap_props["Ssid"][1]).decode("utf-8", "replace")
    bssid = str(ap_props["HwAddress"][1])
    strength = int(ap_props["Strength"][1])
    flags = int(ap_props["Flags"][1])
    wpa_flags = int(ap_props["WpaFlags"][1])
    rsn_flags = int(ap_props["RsnFlags"][1])

    return cls(
      ssid=ssid,
      bssid=bssid,
      strength=strength,
      flags=flags,
      wpa_flags=wpa_flags,
      rsn_flags=rsn_flags,
      ap_path=ap_path,
    )


class ConnectStatus(IntEnum):
  DISCONNECTED = 0
  CONNECTING = 1
  CONNECTED = 2


@dataclass(frozen=True)
class WifiState:
  ssid: str | None = None
  status: ConnectStatus = ConnectStatus.DISCONNECTED


class WifiManager:
  """
  遠端 UI 模式下的安全空實作 WifiManager：
  - 不連本機 D-Bus、不開掃描與狀態監聽執行緒、不執行 sudo sysctl。
  - ipv4_address 固定為空字串且 is_tethering_active() 為 False，使 AdvancedNetworkSettings
    自動將計量網路（Metered）開關保持停用狀態，避免遠端畫面出現無作用的互動按鈕。
  """

  def __init__(self):
    self._networks: list[Network] = []  # an unsorted list of available Networks. a Network can be comprised of multiple APs
    self._active = True  # used to not run when not in settings
    self._exit = True

    # Store wifi device path (遠端替身不連接本機 D-Bus)
    self._wifi_device: str | None = None

    # State
    self._connections: dict[str, str] = {}  # ssid -> connection path
    self._wifi_state: WifiState = WifiState()
    self._user_epoch: int = 0
    self._ipv4_address: str = ""
    self._current_network_metered: MeteredType = MeteredType.UNKNOWN
    self._tethering_password: str = ""
    self._ipv4_forward = False
    self._tethering_ssid = "weedle"

    self._callback_queue: list[Callable[[], None]] = []

    # Callbacks
    self._need_auth: list[Callable[[str], None]] = []
    self._activated: list[Callable[[], None]] = []
    self._forgotten: list[Callable[[str | None], None]] = []
    self._networks_updated: list[Callable[[list[Network]], None]] = []
    self._disconnected: list[Callable[[], None]] = []

  def add_callbacks(
    self,
    need_auth: Callable[[str], None] | None = None,
    activated: Callable[[], None] | None = None,
    forgotten: Callable[[str | None], None] | None = None,
    networks_updated: Callable[[list[Network]], None] | None = None,
    disconnected: Callable[[], None] | None = None,
  ) -> None:
    """保留回呼註冊介面，維持與 WifiManagerUI / AdvancedNetworkSettings 相容。"""
    if need_auth is not None:
      self._need_auth.append(need_auth)
    if activated is not None:
      self._activated.append(activated)
    if forgotten is not None:
      self._forgotten.append(forgotten)
    if networks_updated is not None:
      self._networks_updated.append(networks_updated)
    if disconnected is not None:
      self._disconnected.append(disconnected)

  @property
  def networks(self) -> list[Network]:
    # Sort by connected/connecting, then known, then strength, then alphabetically.
    return sorted(
      self._networks,
      key=lambda n: (n.ssid != self._wifi_state.ssid, not self.is_connection_saved(n.ssid), -n.strength, n.ssid.lower()),
    )

  @property
  def wifi_state(self) -> WifiState:
    return self._wifi_state

  @property
  def ipv4_address(self) -> str:
    return self._ipv4_address

  @property
  def current_network_metered(self) -> MeteredType:
    return self._current_network_metered

  @property
  def connecting_to_ssid(self) -> str | None:
    wifi_state = self._wifi_state
    return wifi_state.ssid if wifi_state.status == ConnectStatus.CONNECTING else None

  @property
  def connected_ssid(self) -> str | None:
    wifi_state = self._wifi_state
    return wifi_state.ssid if wifi_state.status == ConnectStatus.CONNECTED else None

  @property
  def tethering_password(self) -> str:
    return self._tethering_password

  def process_callbacks(self) -> None:
    # Call from UI thread to run any pending callbacks
    to_run, self._callback_queue = self._callback_queue, []
    for cb in to_run:
      cb()

  def set_active(self, active: bool) -> None:
    """僅更新啟用旗標，不觸發本機 Wi-Fi 掃描或背景執行緒。"""
    self._active = bool(active)

  def connect_to_network(self, ssid: str, password: str, hidden: bool = False) -> None:
    """安全空操作：遠端模式不操作本機電腦或平板的網路連線。"""
    del ssid, password, hidden

  def forget_connection(self, ssid: str, block: bool = False) -> None:
    """安全空操作：遠端模式不刪除本機網路設定檔。"""
    del ssid, block

  def activate_connection(self, ssid: str, block: bool = False) -> None:
    """安全空操作：遠端模式不切換本機網路連線。"""
    del ssid, block

  def is_tethering_active(self) -> bool:
    # Check ssid, not connected_ssid, to also catch connecting state
    return False

  def is_connection_saved(self, ssid: str) -> bool:
    return ssid in self._connections

  def set_tethering_password(self, password: str) -> None:
    """安全空操作：遠端模式不修改本機熱點密碼。"""
    del password

  def set_ipv4_forward(self, enabled: bool) -> None:
    """僅記錄狀態，不執行本機 sysctl 指令。"""
    self._ipv4_forward = bool(enabled)

  def set_tethering_active(self, active: bool) -> None:
    """安全空操作：遠端模式不啟閉本機熱點。"""
    del active

  def set_current_network_metered(self, metered: MeteredType) -> None:
    """安全空操作：遠端模式不修改本機計量網路設定。"""
    del metered

  def __del__(self):
    self.stop()

  def stop(self) -> None:
    """冪等安全停止介面：無背景執行緒與 D-Bus 連線需清理。"""
    self._exit = True


__all__ = [
  "AccessPoint",
  "ConnectStatus",
  "DEBUG",
  "DEFAULT_TETHERING_PASSWORD",
  "MeteredType",
  "Network",
  "SCAN_PERIOD_SECONDS",
  "SIGNAL_QUEUE_SIZE",
  "SecurityType",
  "TETHERING_IP_ADDRESS",
  "WifiManager",
  "WifiState",
  "get_security_type",
  "normalize_ssid",
]
