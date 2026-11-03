#!/usr/bin/env python3
"""
opview 主畫面的車型讀寫服務（:5088），給沒有 dragonpilot dashy 的 sunnypilot 分支使用。

只實作 opview 會用到的 dashy API（對照 opview lib/services/comma_models_api.dart、
lib/selfdrive/ui/settings/op_settings_link.dart）：
  GET  /api/init           → {"dp_dev_dashy": true, "isOffroad": bool, "branch": "sunnypilot"}
                             opview 只看有沒有 dp_dev_dashy 這個 key 來判斷「有本機設定服務」，並取得 isOffroad
  GET  /api/models         → {"model_list": {make: [車型名稱, …]}, "selected_model": "<名稱>" | ""}
  POST /api/models/select  body {"selected_model": "<名稱>" | "[AUTO]" | ""}
  GET  /                   → 302 轉到 sunnylink（opview「裝置設定」按鈕會開這個網址）

車型寫入 sunnypilot 自己的 CarPlatformBundle（和設定 → 車輛 → 選擇車型完全相同的格式），
card 在下一次 onroad 指紋辨識時讀取：offroad 時設定，下次發動就生效，不需要重開機。
車型名稱用 car_list.json 的 key（例如 "Toyota Corolla 2020-22"），不是 dp 的 platform 名稱。
"""
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

PORT = 5088
AUTO_MODEL = "[AUTO]"
SUNNYLINK_URL = "https://www.sunnylink.ai/"
CAR_LIST_JSON = os.path.join(BASEDIR, "openpilot", "sunnypilot", "selfdrive", "car", "car_list.json")
MAX_BODY = 64 * 1024

_car_list: dict[str, dict] | None = None
_car_list_lock = threading.Lock()


def load_car_list() -> dict[str, dict]:
  global _car_list
  with _car_list_lock:
    if _car_list is None:
      with open(CAR_LIST_JSON) as f:
        _car_list = json.load(f)
    return _car_list


def models_payload(params: Params) -> dict:
  by_make: dict[str, list[str]] = {}
  for name, data in load_car_list().items():
    by_make.setdefault(str(data.get("make") or "Other"), []).append(name)
  for names in by_make.values():
    names.sort()
  bundle = params.get("CarPlatformBundle") or {}
  selected = bundle.get("name", "") if isinstance(bundle, dict) else ""
  return {"model_list": dict(sorted(by_make.items())), "selected_model": selected}


def select_model(params: Params, name: str | None) -> tuple[int, dict]:
  if not name or name == AUTO_MODEL:
    params.remove("CarPlatformBundle")
    cloudlog.warning("opview_serverd: CarPlatformBundle cleared (auto fingerprint)")
    return 200, {"ok": True, "selected_model": ""}
  data = load_car_list().get(name)
  if data is None:
    return 400, {"ok": False, "error": f"unknown model: {name}"}
  params.put("CarPlatformBundle", {**data, "name": name})
  cloudlog.warning(f"opview_serverd: CarPlatformBundle set to {name!r} ({data.get('platform')})")
  return 200, {"ok": True, "selected_model": name}


class Handler(BaseHTTPRequestHandler):
  server_version = "opview_serverd"
  params: Params

  def _send_json(self, status: int, obj: dict) -> None:
    body = json.dumps(obj, ensure_ascii=False).encode()
    self.send_response(status)
    self.send_header("Content-Type", "application/json; charset=utf-8")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def do_GET(self) -> None:
    path = urlparse(self.path).path
    try:
      if path == "/api/init":
        self._send_json(200, {"dp_dev_dashy": True, "isOffroad": self.params.get_bool("IsOffroad"), "branch": "sunnypilot"})
      elif path == "/api/models":
        self._send_json(200, models_payload(self.params))
      elif path in ("/", "/index.html"):
        self.send_response(302)
        self.send_header("Location", SUNNYLINK_URL)
        self.send_header("Content-Length", "0")
        self.end_headers()
      else:
        self._send_json(404, {"error": "not found"})
    except Exception as e:
      cloudlog.exception("opview_serverd: GET failed")
      self._send_json(500, {"error": f"{type(e).__name__}: {e}"})

  def do_POST(self) -> None:
    path = urlparse(self.path).path
    try:
      if path != "/api/models/select":
        self._send_json(404, {"error": "not found"})
        return
      length = int(self.headers.get("Content-Length", 0))
      if length > MAX_BODY:
        self._send_json(413, {"error": "body too large"})
        return
      body = json.loads(self.rfile.read(length) or b"{}")
      if not isinstance(body, dict):
        self._send_json(400, {"error": "bad request"})
        return
      status, result = select_model(self.params, body.get("selected_model"))
      self._send_json(status, result)
    except json.JSONDecodeError:
      self._send_json(400, {"error": "bad request"})
    except Exception as e:
      cloudlog.exception("opview_serverd: POST failed")
      self._send_json(500, {"error": f"{type(e).__name__}: {e}"})

  def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override
    pass


class Server(ThreadingHTTPServer):
  daemon_threads = True
  allow_reuse_address = True


def main() -> None:
  Handler.params = Params()
  load_car_list()
  server = Server(("0.0.0.0", PORT), Handler)
  cloudlog.warning(f"opview_serverd listening on :{PORT}")
  server.serve_forever()


if __name__ == "__main__":
  main()
