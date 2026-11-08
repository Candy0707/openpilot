import json
from types import SimpleNamespace

import pytest

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.webrtc.webrtcd import (
  CerealOutgoingMessageProxy, StreamRequestError, compact_model_v2, opview_tdx_compat, parse_stream_request,
)


def _xyz(n):
  return SimpleNamespace(x=[0.12345] * n, y=[1.0] * n, z=[-2.0] * n)


def _fake_model():
  return SimpleNamespace(frameId=7, position=_xyz(3), laneLines=[_xyz(3)] * 4, laneLineProbs=[0.91234] * 4,
                         roadEdges=[_xyz(3)] * 2, roadEdgeStds=[0.5] * 2, acceleration=SimpleNamespace(x=[0.33333] * 3))


class TestOpviewStreamRequest(OpenpilotTestCase):
  def test_opview_format(self):
    body, is_opview = parse_stream_request({"sdp": "x", "initCamera": "road", "bridge_services_in": [],
                                            "bridge_services_out": ["carState", "modelV2"], "someNewField": 1})
    assert is_opview
    assert body.cameras == ["road"]
    assert body.enabled is True
    assert body.bridge_services_out == ["carState", "modelV2"]

  def test_upstream_format_unchanged(self):
    body, is_opview = parse_stream_request({"sdp": "x", "cameras": ["wideRoad"], "enabled": False,
                                            "bridge_services_out": ["carState"]})
    assert not is_opview
    assert body.cameras == ["wideRoad"] and body.enabled is False

  def test_upstream_missing_fields_still_rejected(self):
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x"})
    assert e.value.status == 400

  def test_unknown_service_is_500(self):
    # opview 帶選用 service（tdx 等）被拒時靠 500 退回重試
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x", "initCamera": "road", "bridge_services_out": ["carState", "noSuchService"]})
    assert e.value.status == 500

  def test_renamed_services(self):
    # opview 用舊名稱；這版改名為 extrinsicsCalibration / narrowRoadCameraState
    body, _ = parse_stream_request({"sdp": "x", "initCamera": "road",
                                    "bridge_services_out": ["carState", "liveCalibration", "roadCameraState", "modelV2"]})
    assert body.bridge_services_out == ["carState", "extrinsicsCalibration", "narrowRoadCameraState", "modelV2"]
    proxy = CerealOutgoingMessageProxy(body.bridge_services_out, opview=True)
    assert proxy.out_names == {"extrinsicsCalibration": "liveCalibration", "narrowRoadCameraState": "roadCameraState"}

  def test_invalid_camera(self):
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x", "initCamera": "rear", "bridge_services_out": []})
    assert e.value.status == 400

  def test_lan_cannot_publish(self):
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x", "initCamera": "road", "bridge_services_in": ["testJoystick"]}, remote_is_local=False)
    assert e.value.status == 403
    # localhost（sunnylink / athena / body）照舊可以
    body, _ = parse_stream_request({"sdp": "x", "cameras": ["road"], "enabled": True, "bridge_services_in": ["testJoystick"]})
    assert body.bridge_services_in == ["testJoystick"]


  def test_privacy_blocks_driver_camera(self):
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x", "initCamera": "driver"}, allow_driver_camera=False)
    assert e.value.status == 403
    with pytest.raises(StreamRequestError):
      parse_stream_request({"sdp": "x", "cameras": ["driver"], "enabled": True}, allow_driver_camera=False)
    body, _ = parse_stream_request({"sdp": "x", "initCamera": "road"}, allow_driver_camera=False)
    assert body.cameras == ["road"]


class TestOpviewTdxCompat(OpenpilotTestCase):
  def test_sp_to_dp(self):
    # sunnypilot tdx（trafficStatus + roadEvent）→ opview 認得的 dp 格式（nextSpeed / congestion / events[]）
    sp = {"trafficStatus": {"sectionId": "1", "speed": 92, "nextSectionId": "2", "nextSpeed": 35, "status": "severeCongestion"},
          "roadEvent": {"sectionId": "2", "description": "前方:1|事故 佔用外側/車道/2|施工", "distance": 1200.0, "isActive": True}}
    d = opview_tdx_compat(sp)
    assert d["nextSpeed"] == 35 and d["congestion"] == "congested"
    assert d["events"] == [
      {"eventType": 1, "description": "事故 佔用外側/車道", "isCurrentSection": False},
      {"eventType": 2, "description": "施工", "isCurrentSection": False},
    ]
    assert opview_tdx_compat({**sp, "roadEvent": {"isActive": True, "description": "目前:3|壅塞"}})["events"][0]["isCurrentSection"]

  def test_status_mapping_and_inactive(self):
    for sp_status, dp_congestion in (("freeFlow", "smooth"), ("heavyTraffic", "slow"), ("severeCongestion", "congested"), ("unknown", "unknown")):
      d = opview_tdx_compat({"trafficStatus": {"nextSpeed": -1, "status": sp_status}, "roadEvent": {"isActive": False, "description": ""}})
      assert d["congestion"] == dp_congestion and d["events"] == []

  def test_dp_format_passthrough(self):
    dp = {"nextSpeed": 50, "congestion": "slow", "events": []}
    assert opview_tdx_compat(dp) is dp


class TestOpviewOutgoingProxy(OpenpilotTestCase):
  def test_compact_model(self):
    d = compact_model_v2(_fake_model())
    assert set(d) == {"frameId", "position", "laneLines", "laneLineProbs", "roadEdges", "roadEdgeStds", "acceleration"}
    assert d["laneLineProbs"][0] == 0.912 and d["acceleration"]["x"][0] == 0.333
    json.dumps(d)

  def _proxy(self, mocker, send_result=True):
    proxy = CerealOutgoingMessageProxy(["carState"], opview=True)
    proxy.sm = mocker.MagicMock()
    proxy.sm.updated = {"carState": True}
    proxy.sm.__getitem__.return_value = 1.5
    proxy.sm.logMonoTime = {"carState": 1}
    proxy.sm.valid = {"carState": True}
    ch = mocker.Mock(spec=["is_open", "send"])  # 不提供 buffered_amount：opview 路徑不得呼叫它
    ch.is_open.return_value = True
    ch.send.return_value = send_result
    proxy.add_channel(ch)
    return proxy, ch

  def test_congested_channel_backs_off(self, mocker):
    # send() 回 False（進了緩衝佇列）→ 暫停 OPVIEW_BACKOFF_S，期間不再送
    proxy, ch = self._proxy(mocker, send_result=False)
    proxy.update()
    assert ch.send.call_count == 1 and proxy.congested_since is not None
    proxy.last_sent.clear()
    proxy.update()
    assert ch.send.call_count == 1 and proxy.skipped == 1
    proxy.backoff_until = 0.0
    ch.send.return_value = True
    proxy.update()
    assert ch.send.call_count == 2 and proxy.congested_since is None

  def test_rate_limit(self, mocker):
    proxy, ch = self._proxy(mocker)
    proxy.update()
    proxy.update()  # 30 ms 內第二次不送
    assert ch.send.call_count == 1
    assert json.loads(ch.send.call_args.args[0]) == {"type": "carState", "logMonoTime": 1, "valid": True, "data": 1.5}
