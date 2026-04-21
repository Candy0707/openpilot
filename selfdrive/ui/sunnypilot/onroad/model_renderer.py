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

    # 直接讀取 SubMaster
    lp_sp = sm["longitudinalPlanSP"]
    acm = lp_sp.adaptiveCoastingModule

    self._draw_acm_integrated_path(acm, sm)

  def _draw_acm_integrated_path(self, acm, sm):
    # 1️⃣ 先打底：畫出完整的原廠軌跡
    super()._draw_path(sm)

    # 取得原始路徑的距離數據 (x 軸) 用於查找索引
    path_x = self._path.raw_points[:, 0]
    proj_pts = self._path.projected_points

    if path_x.size < 2 or proj_pts.size < 2:
      return

    target_dist = acm.targetDist
    safety_dist = target_dist * acm.dynamicSafety
    danger_dist = target_dist * acm.dynamicDanger
    exit_dist = target_dist * acm.stockControl
    lead_dist = target_dist * acm.leadDist
    has_lead = acm.state >= 2

    # --- 輔助函式：畫標記線與【右側邊緣釘死】文字 ---
    def draw_mark(dist, color, thickness, label):
      idx = np.searchsorted(path_x, dist)
      idx = np.clip(idx, 0, len(proj_pts) // 2 - 1)

      # 拿取左右對應頂點
      p_l = proj_pts[idx]
      p_r = proj_pts[len(proj_pts) - 1 - idx]

      v_l = rl.Vector2(p_l[0], p_l[1])
      v_r = rl.Vector2(p_r[0], p_r[1])

      # 🟢 修復：直接使用傳入的 color，不再去呼叫報錯的 color.r
      rl.draw_line_ex(v_l, v_r, thickness, color)

      if label:
        sz = measure_text_cached(self._font_bold, label, 30, 0)
        # 文字釘死在右頂點 (v_r)，加上偏移 (x+8) 確保不重疊
        text_pos = rl.Vector2(v_r.x + 8, v_r.y - sz.y / 2)

        # 畫文字底色陰影提高閱讀性
        rl.draw_text_ex(self._font_bold, label, rl.Vector2(text_pos.x + 1, text_pos.y + 1), 30, 0, rl.BLACK)
        rl.draw_text_ex(self._font_bold, label, text_pos, 30, 0, color)

    # 2️⃣ 繪製漸變面紗 (包含 5 段區間著色)
    def get_stop_for_dist_idx(dist):
      idx = np.searchsorted(path_x, dist)
      idx = np.clip(idx, 0, len(proj_pts) // 2 - 1)
      track_y = proj_pts[idx][1]
      stop = 1.0 - (track_y - self._rect.y) / self._rect.height
      return np.clip(stop, 0.0, 1.0)

    s_exit = get_stop_for_dist_idx(exit_dist)
    s_danger = get_stop_for_dist_idx(danger_dist)
    s_safety = get_stop_for_dist_idx(safety_dist)
    s_target = get_stop_for_dist_idx(target_dist)

    # 顏色配置
    c_red = rl.Color(255, 60, 60, 100)
    c_yellow = rl.Color(255, 215, 0, 100)
    c_green = rl.Color(0, 255, 150, 100)
    c_clear = rl.Color(0, 0, 0, 0)  # 乾淨透明 (露出原廠)

    stops = [
      0.0,
      max(0.0, s_exit - 0.05),
      s_exit,  # 自車 -> 退出區間：透明 -> 紅色
      s_danger - 0.03,
      s_danger,  # 退出區間 -> 煞車區間：紅色 -> 黃色
      s_safety - 0.03,
      s_safety,  # 煞車區間 -> 滑行區間：黃色 -> 綠色
      s_target,
      min(1.0, s_target + 0.1),  # 滑行區間 -> 目標距離：綠色 -> 透明
      1.0,
    ]
    colors = [c_clear, c_clear, c_red, c_red, c_yellow, c_yellow, c_green, c_green, c_clear, c_clear]

    grad = Gradient(start=(0.0, 1.0), end=(0.0, 0.0), colors=colors, stops=stops)
    draw_polygon(self._rect, self._path.projected_points, gradient=grad)

    # 3️⃣ 繪製 5 條【右側釘死】的實體標記線與文字
    # 傳入的顏色我都調成 200 或 220 左右的 Alpha 值，讓線條稍微有點半透明不刺眼
    draw_mark(exit_dist, rl.Color(255, 60, 60, 200), 10, f"{tr('exit_dist')}：{exit_dist:.1f}m")
    draw_mark(danger_dist, rl.Color(255, 150, 0, 200), 8, f"{tr('safety_dist')}：{danger_dist:.1f}m")
    draw_mark(safety_dist, rl.Color(255, 215, 0, 200), 6, f"{tr('danger_dist')}：{safety_dist:.1f}m")
    draw_mark(target_dist, rl.Color(0, 255, 150, 200), 5, f"{tr('target_dist')}：{target_dist:.1f}m")

    if has_lead:
      draw_mark(lead_dist, rl.Color(255, 255, 255, 220), 6, f"{tr('leadDist')}：{lead_dist:.1f}m")
