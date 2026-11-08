"""
隱私模式（PrivacyMode，預設開啟）

開啟時：
  comma.ai
    - 不向 comma 註冊（manager 不呼叫 register()；已有 DongleId 照用，沒有就用 UNREGISTERED）
    - 不執行 athenad（comma 遠端連線：遠端取檔、即時影像、讀取 GPS、SSH 代理…），已在跑的 daemon 會被終止
    - 不執行 uploader（行車紀錄 / 影片上傳到 comma）
    - 不執行 statsd（comma 統計資料）
    - CommaConnectApi 一律拒絕連線（UI 的 prime 狀態、firehose、行程統計都不再向 comma 查詢）
  sunnylink（保留設定網頁與備份）
    - 不執行 statsd_sp（車輛參數 / 設定統計）、sunnylink_uploader
    - sunnylinkd 不上傳檔案、不轉送 log、不上傳統計
    - sunnylinkd 拒絕遠端呼叫：取檔 / 上傳、即時影像、讀 cereal 訊息（GPS 等）、列目錄、SSH 代理、SIM / 金鑰資訊
    - getParams 不回傳位置相關參數；PrivacyMode 本身不能從 sunnylink 遠端關閉

  車內與錄影（每次發動時鎖定，行駛中切換於下次發動生效）
    - 不執行 encoderd：不錄任何影片（道路 / 廣角 / 車內 / qcamera、縮圖、錄音）。loggerd 照常寫 rlog/qlog（沒有影像）
    - 不執行 dmonitoringmodeld：車內鏡頭畫面不做任何分析
    - dmonitoringd 不讀車內鏡頭，並停用駕駛監控（wheeltouch_step 的 driver_engaged 固定為 True，
      awareness 永遠 1.0、不發任何警示；仍持續送出 driverMonitoringState，selfdrived 不會報通訊錯誤）
    - webrtcd 拒絕車內鏡頭串流（opview / sunnylink 都一樣）
  opview 用的 stream_encoderd 只編碼要求的道路鏡頭、不寫檔，不受影響。

comma 雲端 / sunnylink 部分切換後需重新開機才完全生效。
"""
import os
import signal

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

PRIVACY_PARAM = "PrivacyMode"

# manager 不啟動的程序
COMMA_CLOUD_PROCS = ("manage_athenad", "uploader", "statsd")
SUNNYLINK_TELEMETRY_PROCS = ("statsd_sp", "sunnylink_uploader")
PRIVACY_BLOCKED_PROCS = COMMA_CLOUD_PROCS + SUNNYLINK_TELEMETRY_PROCS

# sunnylinkd 拒絕的遠端方法（athenad 的方法透過共用 dispatcher 也會出現在 sunnylinkd）
SUNNYLINK_BLOCKED_METHODS = frozenset({
  "getMessage",          # 讀任何 cereal 訊息：gpsLocation、liveLocationKalman、carState…
  "startStream",         # WebRTC 即時影像
  "listDataDirectory",   # 列出行車紀錄 / 影片
  "uploadFileToUrl",     # 上傳行車紀錄 / 影片
  "uploadFilesToUrls",
  "listUploadQueue",
  "cancelUpload",
  "setRouteViewed",
  "startLocalProxy",     # 遠端 SSH 代理
  "setNavDestination",
  "getSimInfo",          # IMEI / ICCID
  "getPublicKey",
  "getSshAuthorizedKeys",
  "getGithubUsername",
  "takeSnapshot",        # 舊版 athenad 才有；保險起見一併列入
})

# sunnylinkd getParams 不回傳的參數（位置資訊）
SUNNYLINK_BLOCKED_READ_PARAMS = frozenset({
  "LastGPSPosition",
  "LastGPSPositionLLK",
  "LocationFilterInitialState",
})


_onroad_latch: bool | None = None


def privacy_mode_onroad(started: bool, params: Params) -> bool:
  """發動時鎖定一次隱私模式狀態，整趟行駛不變。
  行駛中切換開關不會讓 dmonitoringmodeld / dmonitoringd 中途不一致（例如 model 停了但 dmonitoringd 還在等車內鏡頭結果
  → selfdrived 通訊錯誤），下次發動才套用。"""
  global _onroad_latch
  if not started:
    _onroad_latch = None
    return privacy_mode(params)
  if _onroad_latch is None:
    _onroad_latch = privacy_mode(params)
  return _onroad_latch


def privacy_mode(params: Params | None = None) -> bool:
  try:
    return (params or Params()).get_bool(PRIVACY_PARAM)
  except Exception:
    # 參數讀不到時以隱私優先
    return True


def stop_athenad_daemon(params: Params) -> None:
  """manage_athenad 是 DaemonProcess：manager 重啟也不會停（stop() 是空的），這裡依 AthenadPid 終止。"""
  pid = params.get("AthenadPid")
  if pid is None:
    return
  try:
    pid = int(pid)
    with open(f"/proc/{pid}/cmdline") as f:
      if "athena" not in f.read():
        return
    try:
      os.killpg(pid, signal.SIGTERM)  # DaemonProcess 以 setpgrp 啟動，連同 athenad 子程序一起停
    except OSError:
      os.kill(pid, signal.SIGTERM)
    cloudlog.warning(f"privacy mode: stopped athenad daemon (pid {pid})")
  except (OSError, ValueError):
    pass
  finally:
    params.remove("AthenadPid")
