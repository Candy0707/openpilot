import json
from types import SimpleNamespace

import pytest

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.webrtc.webrtcd import (
  CerealOutgoingMessageProxy, StreamRequestError, compact_model_v2, parse_stream_request,
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


class TestOpviewOutgoingProxy(OpenpilotTestCase):
  def test_compact_model(self):
    d = compact_model_v2(_fake_model())
    assert set(d) == {"frameId", "position", "laneLines", "laneLineProbs", "roadEdges", "roadEdgeStds", "acceleration"}
    assert d["laneLineProbs"][0] == 0.912 and d["acceleration"]["x"][0] == 0.333
    json.dumps(d)

  def _proxy(self, mocker, buffered):
    proxy = CerealOutgoingMessageProxy(["carState"], opview=True)
    proxy.sm = mocker.MagicMock()
    proxy.sm.updated = {"carState": True}
    proxy.sm.__getitem__.return_value = 1.5
    proxy.sm.logMonoTime = {"carState": 1}
    proxy.sm.valid = {"carState": True}
    ch = mocker.Mock()
    ch.is_open.return_value = True
    ch.buffered_amount.return_value = buffered
    proxy.add_channel(ch)
    return proxy, ch

  def test_congested_channel_skips(self, mocker):
    proxy, ch = self._proxy(mocker, CerealOutgoingMessageProxy.OPVIEW_MAX_BUFFERED_BYTES + 1)
    proxy.update()
    ch.send.assert_not_called()
    assert proxy.congested_since is not None and proxy.skipped == 1

  def test_rate_limit(self, mocker):
    proxy, ch = self._proxy(mocker, 0)
    proxy.update()
    proxy.update()  # 30 ms 內第二次不送
    assert ch.send.call_count == 1
    assert json.loads(ch.send.call_args.args[0]) == {"type": "carState", "logMonoTime": 1, "valid": True, "data": 1.5}
