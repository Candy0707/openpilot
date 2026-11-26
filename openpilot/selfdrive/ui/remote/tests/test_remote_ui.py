#!/usr/bin/env python3
from __future__ import annotations

import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import logging
from pathlib import Path
import socket
import tarfile
import tempfile
import threading
from typing import Any
import unittest

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.remote import wait_for_remote_service
from openpilot.selfdrive.ui.remote.cereal.messaging import PubMaster, SubMaster, new_message
from openpilot.selfdrive.ui.remote.params import ParamKeyType, Params as RemoteParams, UnknownKeyName
from openpilot.selfdrive.ui.remote.webrtc_client import RemoteWebRTCClient


def _find_free_port() -> int:
  """向作業系統申請一個目前空閒的本機 TCP 埠號，避免與執行中的服務衝突。"""
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind(("127.0.0.1", 0))
    return int(s.getsockname()[1])


class _FakeChannel:
  """模擬 libdatachannel DataChannel，用於無網路環境下驗證雙向 JSON 指令。"""

  def __init__(self) -> None:
    self.sent: list[Any] = []
    self._open = True

  def is_open(self) -> bool:
    return self._open

  def send(self, msg: Any) -> None:
    self.sent.append(msg)


class _FakeStream:
  """模擬 teleoprtc Stream，提供假的 messaging_channel 供單元測試攔截回傳內容。"""

  def __init__(self, channel: _FakeChannel) -> None:
    self._channel = channel

  def has_messaging_channel(self) -> bool:
    return True

  def get_messaging_channel(self) -> _FakeChannel:
    return self._channel

  async def stop(self) -> None:
    return


class _FakeVideoTrack:
  """模擬 LiveStreamVideoStreamTrack，追蹤遠端 UI 切換視訊串流的開關狀態。"""

  def __init__(self, enabled: bool = False) -> None:
    self.enabled = enabled

  def enable(self, enabled: bool) -> None:
    self.enabled = bool(enabled)


class _FakeOutgoingBridge:
  """模擬 CerealOutgoingMessageProxy，確認遠端 UI 關閉影像時不會連帶停掉數據橋接。"""

  def __init__(self, enabled: bool = True) -> None:
    self.enabled = enabled

  def enable(self, enabled: bool) -> None:
    self.enabled = bool(enabled)


class TestRemoteUI(OpenpilotTestCase):
  """
  遠端 UI 核心模組與 webrtcd_ext 自動化測試套件（基於 OpenpilotTestCase / unittest）。

  涵蓋驗證項目：
  1. 無遠端服務時確實阻塞不放行，服務啟動後才放行。
  2. 三段式開窗閘門（WebRTC 連線 + 真實 Params 快照 + 核心服務真實資料）嚴格把關。
  3. 零假資料 Params 快取、C4 真實 defaults 與型別安全寫入／失敗回滾。
  4. 純 JSON SubMaster 屬性存取、RemoteStr.raw、零假資料 seen/alive 與動態補訂服務。
  5. 進 Onroad 才啟用道路串流、回 Offroad 自動停流並清除舊影格。
  6. C4 端 webrtcd_ext 型別轉型、關影不斷數據，以及 UI Bundle 打包過濾。
  """

  COMMA_HARDWARE_TEST = False

  def setUp(self) -> None:
    super().setUp()
    # 每個測試開始前重設全域單例與快取，確保測試彼此完全獨立
    RemoteWebRTCClient.reset_instance()
    RemoteParams.reset_cache()

  def tearDown(self) -> None:
    # 測試結束後清理單例與背景執行緒資源
    RemoteWebRTCClient.reset_instance()
    RemoteParams.reset_cache()
    super().tearDown()

  # ===========================================================================
  # 測試 1：無遠端服務時確實阻塞不放行，服務啟動後才放行
  # ===========================================================================
  def test_wait_for_remote_service_blocks_until_ready(self) -> None:
    port = _find_free_port()

    # 尚未啟動服務前應逾時回傳 False（正式模式無 timeout 會持續阻塞不開窗）
    self.assertFalse(wait_for_remote_service("127.0.0.1", port, poll_interval=0.05, timeout=0.15))

    class _SchemaHandler(BaseHTTPRequestHandler):
      def do_GET(self) -> None:
        body = json.dumps({"deviceState": {}}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

      def log_message(self, format: str, *args: object) -> None:  # noqa: A002
        pass

    server = ThreadingHTTPServer(("127.0.0.1", port), _SchemaHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    try:
      self.assertTrue(wait_for_remote_service("127.0.0.1", port, poll_interval=0.05, timeout=2.0))
    finally:
      server.shutdown()
      server.server_close()

  # ===========================================================================
  # 測試 2：三段式開窗閘門（連線 + 真實 Params + 真實核心資料齊備才放行）
  # ===========================================================================
  def test_wait_until_ready_three_stage_gate(self) -> None:
    client = RemoteWebRTCClient(host="127.0.0.1", port=_find_free_port())
    client._running = True  # 跳過真實網路執行緒，由測試精準推進三段狀態

    # 階段 0：尚未連線 -> 必阻擋
    self.assertFalse(client.wait_until_ready(required_services=["deviceState"], poll_interval=0.02, timeout=0.08))

    # 階段 1：僅連線成功，但尚未收到 all_params -> 仍阻擋
    client._connected = True
    client._connected_event.set()
    self.assertFalse(client.wait_until_ready(required_services=["deviceState"], poll_interval=0.02, timeout=0.08))

    # 階段 2：收到真實 all_params，但尚未收到 deviceState -> 仍阻擋
    client._on_messaging_data(json.dumps({
      "cmd": "all_params",
      "params": {"IsMetric": True},
      "types": {"IsMetric": int(ParamKeyType.BOOL)},
      "defaults": {},
    }))
    self.assertFalse(client.wait_until_ready(required_services=["deviceState"], poll_interval=0.02, timeout=0.08))

    # 階段 3：收到真實 deviceState JSON 封包 -> 立即放行開窗
    client._on_messaging_data(json.dumps({
      "type": "deviceState",
      "logMonoTime": 123456,
      "valid": True,
      "data": {"started": False, "networkType": "wifi"},
    }).encode("utf-8"))
    self.assertTrue(client.wait_until_ready(required_services=["deviceState"], poll_interval=0.02, timeout=1.0))

  # ===========================================================================
  # 測試 3：零假資料 Params 快取、真實 defaults 與型別安全寫入／回滾
  # ===========================================================================
  def test_remote_params_zero_fake_data_and_rollback(self, monkeypatch) -> None:
    client = RemoteWebRTCClient.get_instance()
    sent_cmds: list[dict[str, Any]] = []
    monkeypatch.setattr(client, "send_control_command", lambda payload: (sent_cmds.append(payload) or True))

    params = RemoteParams()

    # 尚未同步前不應有任何硬編碼假資料（包含先前曾有的 SpeedLimitMode 等）
    self.assertIsNone(params.get("SpeedLimitMode", return_default=True))
    self.assertIsNone(params.get("ShowDebugInfo", return_default=True))
    with self.assertRaises(UnknownKeyName):
      params.check_key("NonExistentKey")

    # 模擬收到 C4 傳來的真實快照、型別表與原生 defaults
    client._on_messaging_data(json.dumps({
      "cmd": "all_params",
      "params": {
        "ShowDebugInfo": False,
        "CruiseSpeed": 90,
        "AthenadUploadQueue": [{"id": "1"}],
      },
      "types": {
        "ShowDebugInfo": int(ParamKeyType.BOOL),
        "CruiseSpeed": int(ParamKeyType.INT),
        "SpeedLimitMode": int(ParamKeyType.INT),
        "AthenadUploadQueue": int(ParamKeyType.JSON),
      },
      "defaults": {
        "SpeedLimitMode": 2,
      },
    }))

    # 已設定鍵回傳真實值；未設定鍵於 return_default=True 時回傳 C4 真實 default
    self.assertIs(params.get("ShowDebugInfo"), False)
    self.assertIs(params.get_bool("ShowDebugInfo"), False)
    self.assertEqual(params.get("CruiseSpeed"), 90)
    self.assertIsNone(params.get("SpeedLimitMode"))
    self.assertEqual(params.get("SpeedLimitMode", return_default=True), 2)
    self.assertEqual(params.get("AthenadUploadQueue"), [{"id": "1"}])

    # 樂觀寫入與送出指令
    params.put_bool("ShowDebugInfo", True)
    self.assertIs(params.get_bool("ShowDebugInfo"), True)
    self.assertEqual(sent_cmds[-1], {"cmd": "put_param", "key": "ShowDebugInfo", "value": True, "block": False})

    # 模擬 C4 寫入失敗並回傳真實值回滾
    client._on_messaging_data(json.dumps({"cmd": "param_updated", "key": "ShowDebugInfo", "value": False}))
    self.assertIs(params.get_bool("ShowDebugInfo"), False)

# ===========================================================================
  # 測試 4：純 JSON SubMaster 存取、真實列舉 .raw 數值運算、零值骨架與動態補訂
  # ===========================================================================
  def test_remote_submaster_json_and_dynamic_subscribe(self, monkeypatch) -> None:
    client = RemoteWebRTCClient.get_instance()

    # 使用與 C4 原生 /schema 完全一致的真實輸出格式
    monkeypatch.setattr(client, "fetch_schema", lambda services, timeout=3.0: {
      "deviceState": {
        "started": "bool",
        "networkType": "text",
        "networkStrength": "text",
        "cpuTempC": ["float32"],
        "networkInfo": {
          "technology": "text",
          "channel": "uint16",
        },
      },
      "modelV2": {
        "laneLines": [{"x": ["float32"]}],
      },
      "onroadEvents": [
        {"name": "text", "permanent": "bool"},
      ],
    })

    # 模擬 C4 /api/enums 回傳的真實列舉對照表（含 List(Struct) 內部列舉）
    monkeypatch.setattr(client, "fetch_enums", lambda services, timeout=3.0: {
      "deviceState": {
        "networkType": {"__enum__": {"none": 0, "wifi": 1, "cell2G": 2, "cell3G": 3, "cell4G": 4, "cell5G": 5}},
        "networkStrength": {"__enum__": {"unknown": 0, "poor": 1, "moderate": 2, "good": 3, "great": 4}},
      },
      "onroadEvents": {
        "__list__": {
          "name": {"__enum__": {"canError": 0, "startup": 1}},
        },
      },
    })

    sent_cmds: list[dict[str, Any]] = []
    monkeypatch.setattr(client, "send_control_command", lambda payload: (sent_cmds.append(payload) or True))

    sm = SubMaster(["deviceState", "modelV2", "onroadEvents"])

    # 確認 SubMaster.__init__ 不會偷跑連線，確保所有服務註冊完畢後才握手
    self.assertFalse(client._running)

    # 尚未收到真實封包前：seen/alive 嚴格為 False，且列舉零值骨架 .raw 為 int 0 不報錯
    sm.update(0)
    self.assertFalse(sm.seen["deviceState"])
    self.assertFalse(sm.alive["deviceState"])
    self.assertIs(sm["deviceState"].started, False)
    self.assertEqual(sm["deviceState"].networkStrength, "unknown")
    self.assertEqual(sm["deviceState"].networkStrength.raw, 0)
    self.assertEqual(sm["deviceState"].cpuTempC, [])
    self.assertEqual(sm["deviceState"].networkInfo.channel, 0)
    self.assertEqual(sm["onroadEvents"], [])

    # 收到 C4 原生 JSON 封包（刻意省略 networkInfo 以驗證未初始化指標自動退回零值）
    client._on_messaging_data(json.dumps({
      "type": "deviceState",
      "logMonoTime": 999000,
      "valid": True,
      "data": {"started": True, "networkType": "wifi", "networkStrength": "good"},
    }).encode("utf-8"))
    client._on_messaging_data(json.dumps({
      "type": "modelV2",
      "logMonoTime": 999100,
      "valid": True,
      "data": {"laneLines": [{"x": [1.0, 2.0, 3.0]}]},
    }).encode("utf-8"))
    client._on_messaging_data(json.dumps({
      "type": "onroadEvents",
      "logMonoTime": 999200,
      "valid": True,
      "data": [{"name": "startup", "permanent": True}],
    }).encode("utf-8"))

    sm.update(0)
    self.assertTrue(sm.updated["deviceState"])
    self.assertTrue(sm.seen["deviceState"])
    self.assertTrue(sm.alive["deviceState"])
    self.assertIs(sm["deviceState"].started, True)

    # 驗證列舉字串比對、整數比對，以及 sidebar.py 第 123 行完全一致的 .raw 運算
    strength = sm["deviceState"].networkStrength
    self.assertEqual(strength, "good")
    self.assertEqual(strength, 3)
    self.assertEqual(strength.raw, 3)
    net_strength = max(0, min(5, strength.raw + 1)) if strength.raw > 0 else 0
    self.assertEqual(net_strength, 4)

    net_type = sm["deviceState"].networkType
    self.assertEqual(net_type, "wifi")
    self.assertEqual(net_type, 1)
    self.assertEqual(net_type.raw, 1)
    self.assertEqual(int(net_type), 1)

    # 驗證未出現在 JSON 中的子結構自動補齊零值，不會拋出 AttributeError
    self.assertEqual(sm["deviceState"].networkInfo.technology, "")
    self.assertEqual(sm["deviceState"].networkInfo.channel, 0)

    # 驗證 List(Struct) 內部列舉與巢狀資料
    self.assertEqual(sm["modelV2"].laneLines[0].x, [1.0, 2.0, 3.0])
    self.assertEqual(sm["onroadEvents"][0].name, "startup")
    self.assertEqual(sm["onroadEvents"][0].name.raw, 1)
    self.assertTrue(sm.all_checks())

    # 已連線後新增另一個 SubMaster，驗證自動觸發 subscribe_services 動態補訂
    client._connected = True
    _ = SubMaster(["carState"])
    self.assertTrue(
      any(c.get("cmd") == "subscribe_services" and "carState" in c.get("services", []) for c in sent_cmds)
    )

    # 驗證純 Python new_message 與 PubMaster No-op
    pm = PubMaster(["uiDebug"])
    msg = new_message("uiDebug")
    msg.uiDebug.cpuTimeMillis = 12.5
    pm.send("uiDebug", msg)

  # ===========================================================================
  # 測試 5：進 Onroad 才啟用串流、回 Offroad 停流並清除舊畫面
  # ===========================================================================
  def test_onroad_video_toggle_and_stale_frame_cleanup(self, monkeypatch) -> None:
    client = RemoteWebRTCClient(host="127.0.0.1", port=_find_free_port())
    client._connected = True

    sent_cmds: list[dict[str, Any]] = []
    monkeypatch.setattr(client, "send_control_command", lambda payload: (sent_cmds.append(payload) or True))

    # 預設 Offroad 不啟用視訊
    self.assertFalse(client.video_enabled)

    # 模擬進入 Onroad -> 送出原生 livestreamVideoEnable(True) 並允許寫入影格
    client.set_video_enabled(True)
    self.assertTrue(client.video_enabled)
    self.assertEqual(sent_cmds[-1], {"type": "livestreamVideoEnable", "data": {"enabled": True}})

    dummy_frame = np.zeros((10, 10, 3), dtype=np.uint8)
    client._on_decoded_frame(dummy_frame)
    frame, fid, _ = client.get_latest_frame()
    self.assertIsNotNone(frame)
    self.assertEqual(fid, 1)

    # 模擬離開 Onroad -> 送出 livestreamVideoEnable(False) 並清除舊畫面
    client.set_video_enabled(False)
    self.assertFalse(client.video_enabled)
    self.assertEqual(sent_cmds[-1], {"type": "livestreamVideoEnable", "data": {"enabled": False}})
    frame_after, fid_after, _ = client.get_latest_frame()
    self.assertIsNone(frame_after)
    self.assertEqual(fid_after, 0)

  # ===========================================================================
  # 測試 6：C4 端 webrtcd_ext 型別轉型、關影不斷數據、版本與 UI Bundle 打包
  # ===========================================================================
  def test_webrtcd_ext_params_coercion_video_toggle_and_bundle(self) -> None:
    from openpilot.common.params import ParamKeyType as NativeParamKeyType
    from openpilot.system.webrtc.webrtcd_ext import ExtendedStreamSession, build_ui_bundle_tarball

    class _MockNativeParams:
      """模擬 C4 原生 Params 嚴格型別檢查與 defaults 讀取行為。"""

      def __init__(self) -> None:
        self.store: dict[str, Any] = {"ShowDebugInfo": False, "CruiseSpeed": 80}
        self.types = {
          "ShowDebugInfo": NativeParamKeyType.BOOL,
          "LivestreamRequestKeyframe": NativeParamKeyType.BOOL,
          "CruiseSpeed": NativeParamKeyType.INT,
          "LatAccel": NativeParamKeyType.FLOAT,
          "LastTime": NativeParamKeyType.TIME,
          "CustomJson": NativeParamKeyType.JSON,
        }
        self.defaults = {"ShowDebugInfo": False, "CruiseSpeed": 60}

      def all_keys(self) -> list[bytes]:
        return [k.encode("utf-8") for k in self.types]

      def get_type(self, key: str) -> NativeParamKeyType:
        return self.types[key]

      def get_default_value(self, key: str) -> Any:
        return self.defaults.get(key)

      def get(self, key: str) -> Any:
        return self.store.get(key)

      def put(self, key: str, value: Any, block: bool = False) -> None:
        expected = {
          NativeParamKeyType.BOOL: bool,
          NativeParamKeyType.INT: int,
          NativeParamKeyType.FLOAT: float,
          NativeParamKeyType.TIME: datetime.datetime,
          NativeParamKeyType.JSON: (dict, list),
        }[self.types[key]]
        if not isinstance(value, expected):
          raise TypeError(f"expected {expected}, got {type(value)}")
        self.store[key] = value

      def remove(self, key: str) -> None:
        self.store.pop(key, None)

    # 以最小物件構造測試 ExtendedStreamSession 的轉型與指令處理
    fake_ch = _FakeChannel()
    fake_track = _FakeVideoTrack(enabled=False)
    fake_bridge = _FakeOutgoingBridge(enabled=True)

    session = object.__new__(ExtendedStreamSession)
    session.params = _MockNativeParams()
    session.stream = _FakeStream(fake_ch)
    session.remote_ui = True
    session.video_enabled = False
    session.video_tracks = [fake_track]
    session.bitrate_controller = None
    session.outgoing_bridge = fake_bridge
    session.logger = logging.getLogger("test_webrtcd_ext")

    # 驗證各型別轉型（包含先前觸發報錯的 BOOL 字串 '0' / '1'）
    self.assertIs(session._coerce_for_put("ShowDebugInfo", "0"), False)
    self.assertIs(session._coerce_for_put("ShowDebugInfo", "1"), True)
    self.assertEqual(session._coerce_for_put("CruiseSpeed", "105.0"), 105)
    self.assertEqual(session._coerce_for_put("LatAccel", "1.25"), 1.25)
    self.assertIsInstance(session._coerce_for_put("LastTime", "2026-11-27T01:00:00"), datetime.datetime)
    self.assertEqual(session._coerce_for_put("CustomJson", '{"a": 1}'), {"a": 1})

    # 驗證 remote_ui 模式下開關視訊不中斷 outgoing_bridge 數據通道
    session.message_handler(json.dumps({"type": "livestreamVideoEnable", "data": {"enabled": True}}))
    self.assertTrue(fake_track.enabled)
    self.assertTrue(fake_bridge.enabled)
    self.assertIs(session.params.store["LivestreamRequestKeyframe"], True)

    session.message_handler(json.dumps({"type": "livestreamVideoEnable", "data": {"enabled": False}}))
    self.assertFalse(fake_track.enabled)
    self.assertTrue(fake_bridge.enabled)
    self.assertIs(session.params.store["LivestreamRequestKeyframe"], False)

    # 驗證 get_all_params 同時包含 params、types 與真實 defaults
    session.message_handler(json.dumps({"cmd": "get_all_params"}))
    resp = json.loads(fake_ch.sent[-1])
    self.assertEqual(resp["cmd"], "all_params")
    self.assertIs(resp["params"]["ShowDebugInfo"], False)
    self.assertEqual(resp["defaults"]["CruiseSpeed"], 60)

    # 驗證 put_param 寫入與確認回傳
    session.message_handler(json.dumps({"cmd": "put_param", "key": "ShowDebugInfo", "value": "1"}))
    ack = json.loads(fake_ch.sent[-1])
    self.assertEqual(ack, {"cmd": "param_updated", "key": "ShowDebugInfo", "value": True})
    self.assertIs(session.params.store["ShowDebugInfo"], True)

    # 驗證 build_ui_bundle_tarball 排除 __pycache__、.pyc 與 .so
    with tempfile.TemporaryDirectory() as tmp_dir:
      tmp_path = Path(tmp_dir)
      ui_dir = tmp_path / "openpilot" / "selfdrive" / "ui"
      cache_dir = ui_dir / "__pycache__"
      cache_dir.mkdir(parents=True)
      (ui_dir / "ui.py").write_text("print('ok')\n", encoding="utf-8")
      (cache_dir / "ui.cpython-312.pyc").write_bytes(b"pyc")
      (ui_dir / "native.so").write_bytes(b"so")

      tar_bytes = build_ui_bundle_tarball(base_dir=str(tmp_path))
      with tarfile.open(fileobj=io.BytesIO(tar_bytes), mode="r:gz") as tar:
        names = tar.getnames()

      self.assertIn("openpilot/selfdrive/ui/ui.py", names)
      self.assertFalse(any("__pycache__" in n or n.endswith((".pyc", ".so")) for n in names))

  # ===========================================================================
  # 測試 7：驗證 C4 端 handle_get_enums 對真實 log.Event.schema 的萃取結果
  # ===========================================================================
  def test_webrtcd_ext_real_enums_endpoint(self) -> None:
    from openpilot.cereal import log
    from openpilot.system.webrtc.webrtcd_ext import handle_get_enums

    status, body, _ = handle_get_enums("deviceState,onroadEvents")
    self.assertEqual(status, 200)
    data = json.loads(body.decode("utf-8"))

    # 驗證 deviceState 真實列舉與原生 pycapnp 定義完全一致
    strength_map = data["deviceState"]["networkStrength"]["__enum__"]
    self.assertEqual(strength_map["unknown"], int(log.DeviceState.NetworkStrength.unknown))
    self.assertEqual(strength_map["good"], int(log.DeviceState.NetworkStrength.good))

    net_type_map = data["deviceState"]["networkType"]["__enum__"]
    self.assertEqual(net_type_map["none"], int(log.DeviceState.NetworkType.none))
    self.assertEqual(net_type_map["wifi"], int(log.DeviceState.NetworkType.wifi))

    # 驗證 List(Struct) 型態的 onroadEvents 也能正確萃取出內部 EventName 列舉表
    event_name_map = data["onroadEvents"]["__list__"]["name"]["__enum__"]
    self.assertIn("startup", event_name_map)
    self.assertIsInstance(event_name_map["startup"], int)

if __name__ == "__main__":
  unittest.main()