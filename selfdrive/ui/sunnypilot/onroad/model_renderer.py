"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.sunnypilot.onroad.chevron_metrics import ChevronMetrics
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath


class ModelRendererSP(ModelRenderer):
  def __init__(self):
    super().__init__()
    self.rainbow_path = RainbowPath()
    self.chevron_metrics = ChevronMetrics()
    self._font_bold = gui_app.font(FontWeight.BOLD)

    self.leftBlindspot = False
    self.rightBlindspot = False

  def _render(self, rect: rl.Rectangle):
    super()._render(rect)
    sm = ui_state.sm
    carState = sm['carState']
    self.leftBlindspot = carState.leftBlindspot
    self.rightBlindspot = carState.rightBlindspot

  def get_lane_line_color(self, line) -> rl.Color:
    alpha = np.clip(self._lane_line_probs[line], 0.0, 0.7)
    color = rl.Color(255, 255, 255, int(alpha * 255))
    if (self.leftBlindspot and line == 1) or (self.rightBlindspot and line == 2):
      color = rl.Color(255, 165, 0, 255)
    return color

  def _draw_path(self, sm):
    if not self._path.projected_points.size:
      return

    # 從 SubMaster 讀取縱向規劃資料
    lp_sp = sm["longitudinalPlanSP"]
    acm = lp_sp.adaptiveCoastingModule

    # 執行 ACM 整合軌跡繪製
    self._draw_acm_integrated_path(acm, sm)

  def _draw_acm_integrated_path(self, acm, sm):
    # 1️⃣ 先繪製原廠基本的路徑背景 (包含原本的信心彩虹色)
    super()._draw_path(sm)

    path_x = self._path.raw_points[:, 0]
    proj_pts = self._path.projected_points

    # 🛑 防呆攔截：若無路徑資料，直接退出 (移除 acm.state == 'disabled'，確保隨時顯示)
    if path_x.size < 2 or proj_pts.size < 2:
      return

    # 取得 Openpilot 物理路徑的視界極限 (防止線段疊加在畫面盡頭閃爍)
    max_path_x = path_x[-1] - 0.5

    # 2️⃣ 動態讀取來自通訊協定 (Capnp) 的百分比邊界計算物理距離
    target_dist = acm.targetDist

    # 防止剛啟動或異常時 target_dist 為 0 造成畫面崩潰
    if target_dist < 0.1:
      return

    exit_dist = target_dist * acm.exitPercent  # ⚪ 動態讀取：通常為 1.00 (100% 退出線)
    brake_dist = target_dist * acm.coastEndPercent  # 🟡 動態讀取：通常為 0.80 (80% 微煞車起點)
    danger_dist = target_dist * acm.safeDistPercent  # 🔴 動態讀取：通常為 0.70 (70% 危險交接線)

    # 🛡️ 原色保護區：交接線下方再扣除 10%
    pre_danger_dist = target_dist * 0.10

    # 3️⃣ 繪製路面連續漸變霓虹地毯 (Continuous Blend)
    def get_stop_for_dist_idx(dist):
      idx = np.searchsorted(path_x, dist)
      idx = np.clip(idx, 0, len(proj_pts) // 2 - 1)
      track_y = proj_pts[idx][1]
      # 計算畫面上相對應的 Y 軸停靠點百分比
      stop = 1.0 - (track_y - self._rect.y) / self._rect.height
      return float(np.clip(stop, 0.0, 1.0))

    raw_s_pre = get_stop_for_dist_idx(pre_danger_dist)
    raw_s_danger = get_stop_for_dist_idx(danger_dist)
    raw_s_brake = get_stop_for_dist_idx(brake_dist)
    raw_s_exit = get_stop_for_dist_idx(exit_dist)

    # 🛠️ 絕對防崩潰 8-Stop 推擠演算法
    # 嚴格保持單調遞增，徹底解決 OpenGL 漸層蒸發與閃爍的問題！
    s_pre = max(0.0, raw_s_pre)
    s_danger = max(s_pre + 0.001, raw_s_danger)
    s_brake = max(s_danger + 0.001, raw_s_brake)
    s_exit = max(s_brake + 0.001, raw_s_exit)

    # 若超出畫面邊界 (1.0)，則由後往前強壓回來
    if s_exit > 0.995:
      s_exit = 0.995
      s_brake = min(s_brake, s_exit - 0.001)
      s_danger = min(s_danger, s_brake - 0.001)
      s_pre = min(s_pre, s_danger - 0.001)

    s_exit_end = s_exit + 0.002

    # 定義各標線的核心顏色 (降低透明度，讓 GPU 自動融合出柔和過渡)
    c_red = rl.Color(255, 60, 60, 150)  # 交接區 (紅色)
    c_orange = rl.Color(255, 150, 0, 150)  # 微煞車緩衝區 (橘色)
    c_green = rl.Color(0, 255, 150, 150)  # 舒適滑行區 (綠色)
    c_clear = rl.Color(0, 0, 0, 0)  # 透明區 / 原廠顏色保護區

    # 依照距離遠近設置漸變色標 (Stops) - 嚴格保持 8 個點
    # 💡 重大改變：不再設定「突兀切換」，直接將顏色綁定在防線上，讓 GPU 在區間內平滑漸變
    stops = [
      0.0,
      s_pre,  # 🛡️ 60% 保護線 (透明)
      s_danger,  # 🔴 70% 交接線 (紅色) -> GPU 自動渲染: 透明 平滑過渡至 紅色
      s_brake,  # 🟡 80% 微煞線 (橘色) -> GPU 自動渲染: 紅色 平滑過渡至 橘色
      s_exit,  # 🟢 100% 退出線 (綠色) -> GPU 自動渲染: 橘色 平滑過渡至 綠色
      s_exit_end,  # 退出線後轉回透明
      0.999,
      1.0,
    ]

    # 對應的漸層顏色陣列 - 嚴格保持 8 個顏色以防崩潰
    colors = [c_clear, c_clear, c_red, c_orange, c_green, c_clear, c_clear, c_clear]

    # 繪製路面漸變多邊形
    grad = Gradient(start=(0.0, 1.0), end=(0.0, 0.0), colors=colors, stops=stops)
    draw_polygon(self._rect, self._path.projected_points, gradient=grad)

    # 4️⃣ 繪製實體標記線與距離文字 (釘死在路徑右側)
    def draw_mark(dist, color, thickness, label):
      # 🛡️ 超視距隱藏防護：若線段遠於畫面極限，直接不畫，防止擠在天平線上閃爍
      if dist > max_path_x:
        return

      idx = np.searchsorted(path_x, dist)
      idx = np.clip(idx, 0, len(proj_pts) // 2 - 1)
      # 取得路徑左右兩側的投影點
      p_l, p_r = proj_pts[idx], proj_pts[len(proj_pts) - 1 - idx]
      # 畫出橫跨軌跡的橫線
      rl.draw_line_ex(rl.Vector2(p_l[0], p_l[1]), rl.Vector2(p_r[0], p_r[1]), thickness, color)
      if label:
        sz = measure_text_cached(self._font_bold, label, 30, 0)
        pos = rl.Vector2(p_r[0] + 8, p_r[1] - sz.y / 2)
        # 繪製文字陰影增強可讀性
        rl.draw_text_ex(self._font_bold, label, rl.Vector2(pos.x + 1, pos.y + 1), 30, 0, rl.BLACK)
        rl.draw_text_ex(self._font_bold, label, pos, 30, 0, color)

    # 依序畫出所有動態邊界標記線
    draw_mark(exit_dist, rl.Color(255, 255, 255, 150), 4, f"退出：{exit_dist:.1f}m")
    draw_mark(brake_dist, rl.Color(255, 215, 0, 200), 8, f"微煞：{brake_dist:.1f}m")
    draw_mark(danger_dist, rl.Color(255, 60, 60, 220), 10, f"交接：{danger_dist:.1f}m")
