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
    # 1️⃣ 先繪製原廠基本的路徑背景
    super()._draw_path(sm)

    path_x = self._path.raw_points[:, 0]
    proj_pts = self._path.projected_points

    # 🛑 防呆攔截：若無路徑資料、或目前處於無車狀態 (noLead)，則不繪製 ACM 特有的標記
    # 備註：Cereal 讀取枚舉時會轉為字串
    if path_x.size < 2 or proj_pts.size < 2 or acm.state == 'noLead':
      return

    # 2️⃣ 基於新版 ACM 的固定百分比邊界計算物理距離
    target_dist = acm.targetDist
    exit_dist = target_dist * 1.00  # ⚪ 100% 退出線
    coast_dist = target_dist * 0.98  # 🟢 98% 滑行啟動線
    brake_dist = target_dist * 0.90  # 🟡 90% 微煞車起點
    danger_dist = target_dist * 0.75  # 🔴 75% 危險交接線
    lead_dist_actual = acm.leadDist

    # 3️⃣ 繪製路面彩虹地毯 (Gradient Veil)
    def get_stop_for_dist_idx(dist):
      idx = np.searchsorted(path_x, dist)
      idx = np.clip(idx, 0, len(proj_pts) // 2 - 1)
      track_y = proj_pts[idx][1]
      # 計算畫面上相對應的 Y 軸停靠點百分比
      stop = 1.0 - (track_y - self._rect.y) / self._rect.height
      return np.clip(stop, 0.0, 1.0)

    s_exit = get_stop_for_dist_idx(exit_dist)
    s_coast = get_stop_for_dist_idx(coast_dist)
    s_brake = get_stop_for_dist_idx(brake_dist)
    s_danger = get_stop_for_dist_idx(danger_dist)

    # 定義各區間顏色
    c_red = rl.Color(255, 60, 60, 100)  # 75% 以下：危險重煞區 (紅色)
    c_orange = rl.Color(255, 150, 0, 100)  # 90%~75%：微煞車緩衝區 (橘色)
    c_green = rl.Color(0, 255, 150, 100)  # 98%~90%：舒適滑行區 (綠色)
    c_clear = rl.Color(0, 0, 0, 0)  # 100% 以上：透明不干涉區

    # 依照距離遠近設置漸變色標 (Stops)
    stops = [
      0.0,
      max(0.0, s_danger - 0.02),
      s_danger,  # 自車到 75%：危險紅
      s_brake,  # 75% 到 90%：微煞橘
      s_coast,  # 90% 到 98%：滑行綠
      s_exit,  # 98% 到 100%：轉向透明
      1.0,
    ]
    colors = [c_red, c_red, c_orange, c_orange, c_green, c_green, c_clear]

    # 繪製路面漸變多邊形
    grad = Gradient(start=(0.0, 1.0), end=(0.0, 0.0), colors=colors, stops=stops)
    draw_polygon(self._rect, self._path.projected_points, gradient=grad)

    # 4️⃣ 繪製實體標記線與距離文字 (釘死在路徑右側)
    def draw_mark(dist, color, thickness, label):
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

    # 依序畫出四條固定邊界界線
    draw_mark(exit_dist, rl.Color(255, 255, 255, 150), 4, f"100% 退出：{exit_dist:.1f}m")
    draw_mark(coast_dist, rl.Color(0, 255, 150, 200), 6, f"98% 滑行：{coast_dist:.1f}m")
    draw_mark(brake_dist, rl.Color(255, 215, 0, 200), 8, f"90% 微煞：{brake_dist:.1f}m")
    draw_mark(danger_dist, rl.Color(255, 60, 60, 220), 10, f"75% 交接：{danger_dist:.1f}m")

    # 畫出前車當下的實體位置標記
    draw_mark(lead_dist_actual, rl.Color(255, 255, 255, 255), 6, f"前車：{lead_dist_actual:.1f}m")
