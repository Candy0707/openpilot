#!/usr/bin/env python3
import os
import time

from openpilot.selfdrive.ui.remote import REMOTE_UI, connect_remote_ui, install_remote_hooks

# 遠端模式：在匯入任何 UI 模組前統一掛載遠端 Params 與 messaging，並阻塞等待遠端服務就緒
if REMOTE_UI:
  install_remote_hooks()

from openpilot.cereal import messaging
from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.common.realtime import Priority, config_realtime_process, set_core_affinity
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
from openpilot.selfdrive.ui.ui_state import ui_state

BIG_UI = gui_app.big_ui()


def main():
  cores = {5, }

  config_realtime_process(0, Priority.CTRL_HIGH)

  # 遠端模式開窗閘門：此時 ui_state 的 SubMaster 已註冊完所有服務，
  # 等待 WebRTC 連通＋真實 Params 快照＋核心真實資料全部到位後才往下開窗
  if REMOTE_UI:
    connect_remote_ui()
    ui_state.update()

  gui_app.init_window("UI")
  if BIG_UI:
    MainLayout()
  else:
    MiciMainLayout()

  pm = messaging.PubMaster(['uiDebug'])
  for should_render, frame_time, cpu_time in gui_app.render():
    extra_start = time.monotonic()
    ui_state.update()

    if should_render:
      # reaffine after power save offlines our core
      if COMMA_HARDWARE and os.sched_getaffinity(0) != cores:
        try:
          set_core_affinity(list(cores))
        except OSError:
          pass

      extra_cpu = time.monotonic() - extra_start
      msg = messaging.new_message('uiDebug')
      msg.uiDebug.cpuTimeMillis = (cpu_time + extra_cpu) * 1000
      msg.uiDebug.frameTimeMillis = frame_time * 1000
      pm.send('uiDebug', msg)


if __name__ == "__main__":
  main()