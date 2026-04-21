import socket
import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached


class SystemMonitorRenderer(Widget):
  def __init__(self):
    super().__init__()
    # 1. 字體設定 (參考您的雷達範例)
    self.label_size = 26
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)

    # 2. 初始化 IP 與狀態文字
    self.ip_address = self._get_ip_address()
    self.hw_text = "讀取硬體資訊中..."

  def _get_ip_address(self) -> str:
    try:
      s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      s.connect(("8.8.8.8", 80))
      ip = s.getsockname()[0]
      s.close()
      return f"IP: {ip}"
    except Exception:
      return "IP: Offline"

  def _update_state(self) -> None:
    """
    從 ui_state 更新硬體數據
    """
    sm = ui_state.sm
    if sm.updated['deviceState']:
      device_state = sm['deviceState']

      # CPU 平均使用率
      cpu_usages = device_state.cpuUsagePercent
      cpu_usage = sum(cpu_usages) / len(cpu_usages) if len(cpu_usages) > 0 else 0.0

      # GPU 與 RAM 使用率
      gpu_usage = device_state.gpuUsagePercent
      mem_usage = device_state.memoryUsagePercent

      # 尋找全機最高溫
      all_temps = list(device_state.cpuTempC) + list(device_state.gpuTempC)
      max_temp = max(all_temps) if len(all_temps) > 0 else 0.0

      # 風扇轉速百分比
      fan_speed_pct = int(device_state.fanSpeedPercentDesired)

      # 更新左側要顯示的硬體文字
      self.hw_text = f"CPU: {cpu_usage:.0f}%  |  GPU: {gpu_usage}%  |  RAM: {mem_usage}%  |  Max Temp: {max_temp:.1f}°C  |  Fan: {fan_speed_pct}%"

  def _render(self, rect: rl.Rectangle) -> None:
    """
    執行畫面渲染 (直接由 Openpilot UI 引擎呼叫並傳入螢幕範圍 rect)
    """
    # 如果還沒抓到資料，可以選擇不畫，或畫預設文字
    if not self.hw_text:
      return

    # 右側 IP 資訊 X 軸：靠右對齊
    # 利用 measure_text_cached 精準取得 IP 文字的寬度
    ip_size = measure_text_cached(self._font_bold, self.ip_address, self.label_size, 0)
    # 螢幕總寬度 - 文字寬度 - 右側邊距 (例如 40)
    ip_x = rect.width - ip_size.x - 80

    # --- 設定顏色 ---
    # 這裡使用白色，如果您需要半透明，可以使用 rl.Color(255, 255, 255, 200)
    text_color = rl.WHITE

    # --- 座標計算 ---
    text_size = measure_text_cached(self._font_bold, self.hw_text, self.label_size, 0)
    bottom_y = rect.height - text_size.y

    # 左側硬體資訊 X 軸：固定靠左 (例如距離左邊緣 40 像素)
    hw_x = 80

    # --- 繪製文字 ---
    # 1. 畫左下角的硬體狀態
    rl.draw_text_ex(self._font_bold, self.hw_text, rl.Vector2(hw_x, bottom_y), self.label_size, 0, text_color)

    # 2. 畫右下角的 IP 位址
    rl.draw_text_ex(self._font_bold, self.ip_address, rl.Vector2(ip_x, bottom_y), self.label_size, 0, text_color)
