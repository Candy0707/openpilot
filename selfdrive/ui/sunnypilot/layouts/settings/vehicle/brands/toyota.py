"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp

from opendbc.car.toyota.values import ToyotaFlags
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP



DESCRIPTIONS = {
  'enforce_stock_longitudinal': tr_noop(
    'sunnypilot will not take over control of gas and brakes. Factory Toyota longitudinal control will be used.'
  ),
}


class ToyotaSettings(BrandSettings):
  def __init__(self):
    super().__init__()

    self.enforce_stock_longitudinal = toggle_item_sp(
      lambda: tr("Enforce Factory Longitudinal Control"),
      description=lambda: tr(DESCRIPTIONS["enforce_stock_longitudinal"]),
      initial_state=self.params.get_bool("ToyotaEnforceStockLongitudinal"),
      callback=self._on_enable_enforce_stock_longitudinal,
      enabled=lambda: not self.engaged,
    )

    self.enable_angle_control = toggle_item_sp(
      lambda: tr("Enable Angle Control (TSS2)"),
      description=lambda: tr("Enable Using Angle Control"),
      initial_state=self.params.get_bool("ToyotaEnableAngleControl"),
      callback=self._on_enable_angle_control,
      enabled=lambda: not self.engaged,
    )

    self.enable_auto_hold = multiple_button_item_sp(
      lambda: tr("Enable AUTO HOLD (TSS2)"),
      lambda: tr("For use on vehicles without auto braking.<br>" +
                 "ON：Start when the vehicle is in Drive (D) gear.<br>" +
                 "SPEED：Start when the vehicle speed is greater than 5 m/s"),
      buttons=[lambda: tr("OFF"), lambda: tr("ON"), lambda: tr("SPEED")],
      button_width=300,
      callback=self._set_enable_auto_hold,
      selected_index=self.params.get("ToyotaEnableAutoHold", return_default=True),
    )

    self.items = []
    self.items.append(self.enforce_stock_longitudinal)
    if self.CP.flags & ToyotaFlags.TSS2.value:
      self.items.append(self.enable_angle_control)
    if self.CP.flags & ToyotaFlags.TSS2.value and \
        not self.CP.flags & ToyotaFlags.RADAR_ACC.value and \
        not self.CP.flags & ToyotaFlags.SECOC.value:
      self.items.append(self.enable_auto_hold)

  def _on_enable_enforce_stock_longitudinal(self, state: bool):
    if state:
      def confirm_callback(result: int):
        if result == DialogResult.CONFIRM:
          self.params.put_bool("ToyotaEnforceStockLongitudinal", True)
          if self.params.get_bool("AlphaLongitudinalEnabled"):
            self.params.put_bool("AlphaLongitudinalEnabled", False)
          self.params.put_bool("OnroadCycleRequested", True)
        else:
          self.enforce_stock_longitudinal.action_item.set_state(False)

      content = (f"<h1>{self.enforce_stock_longitudinal.title}</h1><br>" +
                 f"<p>{self.enforce_stock_longitudinal.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True)
      gui_app.set_modal_overlay(dlg, callback=confirm_callback)

    else:
      self.params.put_bool("ToyotaEnforceStockLongitudinal", False)
      self.params.put_bool("OnroadCycleRequested", True)

  def _on_enable_angle_control(self, state: bool):
    if state:
      def confirm_callback(result: int):
        if result == DialogResult.CONFIRM:
          self.params.put_bool("ToyotaEnableAngleControl", True)
          self.params.put_bool("OnroadCycleRequested", True)
        else:
          self.enable_angle_control.action_item.set_state(False)

      content = (f"<h1>{self.enable_angle_control.title}</h1><br>" +
                 f"<p>{self.enable_angle_control.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True)
      gui_app.set_modal_overlay(dlg, callback=confirm_callback)

    else:
      self.params.put_bool("ToyotaEnableAngleControl", False)
      self.params.put_bool("OnroadCycleRequested", True)

  def _set_enable_auto_hold(self, selected_index: int):
      def confirm_callback(result: int):
        if result == DialogResult.CONFIRM:
          self.params.put("ToyotaEnableAutoHold", selected_index)

      content = (f"<h1>{self.enable_auto_hold.title}</h1><br>" +
                 f"<p>{self.enable_auto_hold.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True)
      gui_app.set_modal_overlay(dlg, callback=confirm_callback)

  def update_settings(self):
    pass