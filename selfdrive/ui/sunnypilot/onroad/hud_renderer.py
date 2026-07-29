"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math
import time

import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiRenderer, DeveloperUiState, get_bottom_dev_ui_offset
from openpilot.selfdrive.ui.sunnypilot.onroad.road_name import RoadNameRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.rocket_fuel import RocketFuel
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_limit import SpeedLimitRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.smart_cruise_control import SmartCruiseControlRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.turn_signal import TurnSignalController
from openpilot.selfdrive.ui.sunnypilot.onroad.circular_alerts import CircularAlertsRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_renderer import SpeedRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.radar_ui import RadarUiRenderer

from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer, UI_CONFIG, FONT_SIZES, COLORS, CRUISE_DISABLED_CHAR
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached

SLA_ACTIVE_COLOR = rl.Color(0x91, 0x9b, 0x95, 0xff)


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.developer_ui = DeveloperUiRenderer()
    self.road_name_renderer = RoadNameRenderer()
    self.rocket_fuel = RocketFuel()
    self.speed_limit_renderer = SpeedLimitRenderer()
    self.smart_cruise_control_renderer = SmartCruiseControlRenderer()
    self.turn_signal_controller = TurnSignalController()
    self.circular_alerts_renderer = CircularAlertsRenderer()
    self.speed_renderer = SpeedRenderer()
    self._torque_bar = TorqueBar(scale=3.0, always=True)

    self.pcm_cruise_speed: bool = True
    self.show_icbm_status: bool = False
    self.icbm_active_counter: int = 0
    self.speed_cluster: float = 0.0
    self.speed_conv: float = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH

    # --- TDX 路況預警變數 (由 dp 版移植) ---
    self.tdx_speed: int = -1
    self.tdx_next_speed: int = -1
    self.tdx_status: str = "UNKNOWN"
    self.tdx_event_active: bool = False
    self.tdx_event_desc: str = ""

  def _update_state(self) -> None:
    if ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      # 重置 TDX 變數，避免殘留上次熄火前的資料
      self.tdx_speed = -1
      self.tdx_next_speed = -1
      self.tdx_status = "UNKNOWN"
      self.tdx_event_active = False
      self.tdx_event_desc = ""
      return

    if ui_state.CP_SP is not None:
      self.pcm_cruise_speed = ui_state.CP_SP.pcmCruiseSpeed
    self.speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed_cluster = ui_state.sm['carState'].cruiseState.speedCluster * self.speed_conv

    super()._update_state()
    self.road_name_renderer.update()
    self.speed_limit_renderer.update()
    self.smart_cruise_control_renderer.update()
    self.turn_signal_controller.update()
    self.circular_alerts_renderer.update()
    self.speed_renderer.update()
    self._update_tdx_state()

  def _update_tdx_state(self) -> None:
    """讀取 TDX 即時路況與事件 (由 dp 版移植)"""
    try:
      tdx = ui_state.sm['tdx']
      self.tdx_speed = tdx.trafficStatus.speed
      self.tdx_next_speed = tdx.trafficStatus.nextSpeed
      self.tdx_status = str(tdx.trafficStatus.status)
      self.tdx_event_active = tdx.roadEvent.isActive

      raw_desc = str(tdx.roadEvent.description)
      # 解碼並只取純文字
      if raw_desc and ":" in raw_desc:
        loc_part, events_part = raw_desc.split(":", 1)
        clean_events = []
        for evt in events_part.split("/"):
          parts = evt.split("|")
          # 如果有代碼，就只取 | 後面的文字；否則保留原樣
          clean_events.append(parts[1] if len(parts) > 1 else evt)
        # 重新組裝為純文字供畫面繪製
        self.tdx_event_desc = f"{loc_part}: {' / '.join(clean_events)}"
      else:
        self.tdx_event_desc = raw_desc
    except Exception:
      pass

  def _get_icbm_status(self):
    if not self.pcm_cruise_speed and ui_state.sm['carControl'].enabled:
      if round(self.set_speed) != round(self.speed_cluster):
        self.icbm_active_counter = 3 * gui_app.target_fps  # 3 seconds usually
      elif self.icbm_active_counter > 0:
        self.icbm_active_counter -= 1
    else:
      self.icbm_active_counter = 0

    self.show_icbm_status = self.icbm_active_counter > 0

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    long_plan_sp = ui_state.sm['longitudinalPlanSP']
    long_override = ui_state.sm['carControl'].cruiseControl.override
    self._get_icbm_status()

    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if long_plan_sp.speedLimit.assist.active:
        set_speed_color = SLA_ACTIVE_COLOR if long_override else rl.Color(0, 0xff, 0, 0xff)
        max_color = SLA_ACTIVE_COLOR if long_override else rl.Color(0x80, 0xd8, 0xa6, 0xff)
      else:
        if ui_state.status == UIStatus.ENGAGED:
          max_color = COLORS.ENGAGED
        elif ui_state.status == UIStatus.DISENGAGED:
          max_color = COLORS.DISENGAGED
        elif ui_state.status == UIStatus.OVERRIDE:
          max_color = COLORS.OVERRIDE

    max_str_size = 60 if self.show_icbm_status else 40
    max_str_y = 15 if self.show_icbm_status else 27

    max_text = str(round(self.speed_cluster)) if self.show_icbm_status else tr("MAX")
    max_text_width = measure_text_cached(self._font_semi_bold, max_text, max_str_size).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + max_str_y),
      max_str_size,
      0,
      max_color,
    )

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    self.speed_renderer.render(rect)

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)

    if ui_state.torque_bar:
      torque_rect = rect
      if ui_state.developer_ui in (DeveloperUiState.BOTTOM, DeveloperUiState.BOTH):
        torque_rect = rl.Rectangle(rect.x, rect.y, rect.width, rect.height - get_bottom_dev_ui_offset())
      self._torque_bar.render(torque_rect)

    self.developer_ui.render(rect)
    self.radar_ui.render(rect)
    self.road_name_renderer.render(rect)
    self.speed_limit_renderer.render(rect)
    self._draw_set_speed(rect)
    self.smart_cruise_control_renderer.render(rect)
    self.turn_signal_controller.render(rect)
    self.circular_alerts_renderer.render(rect)
    self.rocket_fuel.render(rect, ui_state.sm)

    self._draw_tdx_info(rect)

  def set_model_renderer(self, model_renderer):
    self.model_renderer = model_renderer
    self.radar_ui = RadarUiRenderer(model_renderer)

  def _draw_tdx_info(self, rect: rl.Rectangle) -> None:
    """繪製 TDX 即時路況(前方車速)與事件跑馬燈 (由 dp 版移植，位置改配合 sp 版面)"""
    if self.tdx_next_speed <= 0 and not self.tdx_event_active:
      return

    bg_padding_x = 45
    bg_padding_y = 20

    if self.tdx_status == "GREEN":
      speed_color = rl.Color(128, 216, 166, 255)
    elif self.tdx_status == "YELLOW":
      speed_color = rl.Color(255, 204, 0, 255)
    elif self.tdx_status == "RED":
      speed_color = rl.Color(255, 100, 100, 255)
    else:
      speed_color = rl.WHITE

    # ==========================================
    # 前方車速: 放在頂部列下方、置中偏右，避開左側 set speed 區塊
    # ==========================================
    if 0 < self.tdx_next_speed <= 150:
      speed_text = f"前方車速: {self.tdx_next_speed} km/h"
      tdx_speed_font_size = 60
      speed_size = measure_text_cached(self._font_semi_bold, speed_text, tdx_speed_font_size)

      top_y = rect.y + UI_CONFIG.header_height + 25
      speed_x = rect.x + rect.width - speed_size.x - bg_padding_x - 40

      bg_rect = rl.Rectangle(
        speed_x - bg_padding_x, top_y - bg_padding_y,
        speed_size.x + bg_padding_x * 2, speed_size.y + bg_padding_y * 2,
      )
      rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(0, 0, 0, 160))
      rl.draw_text_ex(self._font_semi_bold, speed_text, rl.Vector2(speed_x, top_y), tdx_speed_font_size, 0, speed_color)

    # ==========================================
    # 事件跑馬燈: 貼齊 rect 底部，並扣除 developer_ui 下方保留高度，
    # 避免與 sp 的 developer_ui / turn_signal / circular_alerts / rocket_fuel 重疊
    # ==========================================
    if self.tdx_event_active and self.tdx_event_desc:
      bottom_offset = 0.0
      if ui_state.developer_ui in (DeveloperUiState.BOTTOM, DeveloperUiState.BOTH):
        bottom_offset = get_bottom_dev_ui_offset()

      tdx_event_font_size = 50
      max_text_width = rect.width - 200

      text = self.tdx_event_desc
      text_size = measure_text_cached(self._font_semi_bold, text, tdx_event_font_size)
      text_width = text_size.x
      line_height = text_size.y

      display_width = min(text_width, max_text_width)

      event_bg_height = line_height + bg_padding_y * 2
      event_y = rect.y + rect.height - bottom_offset - event_bg_height - 20

      event_x = rect.x + rect.width / 2 - display_width / 2
      event_bg_rect = rl.Rectangle(
        event_x - bg_padding_x, event_y,
        display_width + bg_padding_x * 2, event_bg_height,
      )

      # 呼吸燈閃爍警告背景
      alpha = 130 + int(50 * math.sin(time.time() * 5))
      rl.draw_rectangle_rounded(event_bg_rect, 0.2, 10, rl.Color(220, 50, 50, alpha))

      draw_y = event_y + bg_padding_y

      if text_width > max_text_width:
        # 文字超長 -> 裁切 + 跑馬燈來回捲動
        rl.begin_scissor_mode(int(event_bg_rect.x), int(event_bg_rect.y), int(event_bg_rect.width), int(event_bg_rect.height))

        extra_width = text_width - max_text_width
        scroll_speed = 80.0
        scroll_duration = extra_width / scroll_speed
        pause_duration = 2.0

        cycle_time = time.time() % ((scroll_duration + pause_duration) * 2)

        if cycle_time < pause_duration:
          offset = 0.0
        elif cycle_time < pause_duration + scroll_duration:
          progress = (cycle_time - pause_duration) / scroll_duration
          offset = extra_width * progress
        elif cycle_time < pause_duration * 2 + scroll_duration:
          offset = extra_width
        else:
          progress = (cycle_time - pause_duration * 2 - scroll_duration) / scroll_duration
          offset = extra_width * (1 - progress)

        draw_x = event_x - offset
        rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(draw_x, draw_y), tdx_event_font_size, 0, rl.WHITE)

        rl.end_scissor_mode()
      else:
        rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(event_x, draw_y), tdx_event_font_size, 0, rl.WHITE)
