import json
import threading
import urllib.request

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.opview.opviewd import CarList, make_server, HINT_OFFROAD, HINT_ONROAD

CARS = {
  "Toyota Corolla 2020": {"platform": "TOYOTA_COROLLA_TSS2", "make": "Toyota", "model": "Corolla", "year": ["2020"]},
  "Toyota RAV4 2022": {"platform": "TOYOTA_RAV4_TSS2_2022", "make": "Toyota", "model": "RAV4", "year": ["2022"]},
  "Hyundai Custin 2023": {"platform": "HYUNDAI_CUSTIN_1ST_GEN", "make": "Hyundai", "model": "Custin", "year": ["2023"]},
}


class FakeParams:
  def __init__(self):
    self.store = {"IsOffroad": True}

  def get(self, key):
    return self.store.get(key)

  def get_bool(self, key):
    return bool(self.store.get(key))

  def put(self, key, value):
    self.store[key] = value

  def remove(self, key):
    self.store.pop(key, None)


class TestOpviewd(OpenpilotTestCase):
  def setup_method(self):
    self.params = FakeParams()
    cars = CarList(self.params, path="/nonexistent/car_list.json")
    self.params.put("CarList", CARS)  # 檔案讀不到時退回 CarList 參數
    self.server = make_server("127.0.0.1", 0, self.params, cars)
    self.port = self.server.server_address[1]
    self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
    self.thread.start()

  def teardown_method(self):
    self.server.shutdown()
    self.server.server_close()

  def _req(self, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f"http://127.0.0.1:{self.port}{path}", data=data,
                                 headers={"Content-Type": "application/json"} if data else {})
    try:
      with urllib.request.urlopen(req, timeout=5) as r:
        return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
      return e.code, json.loads(e.read())

  def test_init(self):
    status, j = self._req("/api/init")
    assert status == 200 and j["branch"] == "sp" and j["isOffroad"] is True and "dp_dev_dashy" not in j

  def test_models_and_select(self):
    status, j = self._req("/api/models")
    assert status == 200
    assert j["model_list"] == {"Hyundai": ["Hyundai Custin 2023"], "Toyota": ["Toyota Corolla 2020", "Toyota RAV4 2022"]}
    assert j["selected_model"] == "" and j["apply_hint"] == HINT_OFFROAD

    status, _ = self._req("/api/models/select", {"selected_model": "Toyota Corolla 2020"})
    assert status == 200
    assert self.params.get("CarPlatformBundle") == {**CARS["Toyota Corolla 2020"], "name": "Toyota Corolla 2020"}
    assert self._req("/api/models")[1]["selected_model"] == "Toyota Corolla 2020"

    self.params.put("IsOffroad", False)
    status, j = self._req("/api/models/select", {"selected_model": "[AUTO]"})
    assert status == 200 and j["apply_hint"] == HINT_ONROAD
    assert self.params.get("CarPlatformBundle") is None

  def test_unknown_model(self):
    status, j = self._req("/api/models/select", {"selected_model": "Nope"})
    assert status == 400 and j["error"] == "unknown_model"
    assert self.params.get("CarPlatformBundle") is None

  def test_not_found(self):
    assert self._req("/api/settings")[0] == 404
