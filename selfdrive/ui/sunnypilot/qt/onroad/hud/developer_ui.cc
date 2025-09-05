/**
The MIT License

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Last updated: July 29, 2024
***/

#include "selfdrive/ui/sunnypilot/qt/onroad/hud/developer_ui.h"

#include <cmath>

#include "common/util.h"

developer_ui::developer_ui(QWidget *parent) : QObject(parent)
{
}

void developer_ui::updateState(const UIState &s)
{
  is_metric = s.scene.is_metric;

  /*
  latActive = car_control.getLatActive();
  madsEnabled = car_state.getMadsEnabled();
  // ############################## DEV UI START ##############################
  lead_d_rel = radar_state.getLeadOne().getDRel();
  lead_v_rel = radar_state.getLeadOne().getVRel();
  lead_status = radar_state.getLeadOne().getStatus();
  lateralState = QString::fromStdString(cs_sp.getLateralState());
  angleSteers = car_state.getSteeringAngleDeg();
  steerAngleDesired = cs.getLateralControlState().getPidState().getSteeringAngleDesiredDeg();
  curvature = cs.getCurvature();
  roll = sm["liveParameters"].getLiveParameters().getRoll();
  memoryUsagePercent = sm["deviceState"].getDeviceState().getMemoryUsagePercent();
  devUiInfo = s.scene.dev_ui_info;
  gpsAccuracy = is_gps_location_external ? gpsLocation.getHorizontalAccuracy() : 1.0; //External reports accuracy, internal does not.
  altitude = gpsLocation.getAltitude();
  vEgo = car_state.getVEgo();
  aEgo = car_state.getAEgo();
  steeringTorqueEps = car_state.getSteeringTorqueEps();
  bearingAccuracyDeg = gpsLocation.getBearingAccuracyDeg();
  bearingDeg = gpsLocation.getBearingDeg();
  torquedUseParams = (ltp.getUseParams() || s.scene.live_torque_toggle) && !s.scene.torqued_override;
  latAccelFactorFiltered = ltp.getLatAccelFactorFiltered();
  frictionCoefficientFiltered = ltp.getFrictionCoefficientFiltered();
  liveValid = ltp.getLiveValid();
  // ############################## DEV UI END ##############################
  */}

void developer_ui::drawText(QPainter &p, int x, int y, const QString &text, QColor color)
{
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});
  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

// ############################## DEV UI START ##############################
void developer_ui::drawCenteredLeftText(QPainter &p, int x, int y, const QString &text1, QColor color1, const QString &text2, const QString &text3, QColor color2)
{
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text1 + " ");
  QRect real_rect = fm.boundingRect(init_rect, 0, text1 + " ");
  real_rect.moveCenter({x, y});

  QRect init_rect3 = fm.boundingRect(text3);
  QRect real_rect3 = fm.boundingRect(init_rect3, 0, text3);
  real_rect3.moveTop(real_rect.top());
  real_rect3.moveLeft(real_rect.right() + 135);

  QRect init_rect2 = fm.boundingRect(text2);
  QRect real_rect2 = fm.boundingRect(init_rect2, 0, text2);
  real_rect2.moveTop(real_rect.top());
  real_rect2.moveRight(real_rect.right() + 125);

  p.setPen(color1);
  p.drawText(real_rect, Qt::AlignLeft | Qt::AlignVCenter, text1);

  p.setPen(color2);
  p.drawText(real_rect2, Qt::AlignRight | Qt::AlignVCenter, text2);
  p.drawText(real_rect3, Qt::AlignLeft | Qt::AlignVCenter, text3);
}

int developer_ui::drawDevUiElement(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color)
{
  p.setFont(InterFont(38, QFont::Bold));
  QColor white = QColor(0, 0, 0, 0);
  drawCenteredLeftText(p, x, y, label, white, value, units, color);

  return 430;
}

void developer_ui::drawDevUI(QPainter &p)
{
  p.save();

  QRect bar_rect1(rect().left(), rect().bottom() - 60, rect().width(), 61);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRect(bar_rect1);
  int x = bar_rect1.left();
  int y = bar_rect1.center().y();

  int rw = 90;

  UiElement aEgoElement = DeveloperUi::getAEgo(aEgo);
  rw += drawDevUiElement(p, rw, y, aEgoElement.value, aEgoElement.label, aEgoElement.units, aEgoElement.color);

  UiElement vEgoLeadElement = DeveloperUi::getVEgoLead(lead_status, lead_v_rel, vEgo, is_metric);
  rw += drawDevUiElement(p, rw, y, vEgoLeadElement.value, vEgoLeadElement.label, vEgoLeadElement.units, vEgoLeadElement.color);

  if (torquedUseParams)
  {
    UiElement frictionCoefficientFilteredElement = DeveloperUi::getFrictionCoefficientFiltered(frictionCoefficientFiltered, liveValid);
    rw += drawDevUiElement(p, rw, y, frictionCoefficientFilteredElement.value, frictionCoefficientFilteredElement.label, frictionCoefficientFilteredElement.units, frictionCoefficientFilteredElement.color);

    UiElement latAccelFactorFilteredElement = DeveloperUi::getLatAccelFactorFiltered(latAccelFactorFiltered, liveValid);
    rw += drawDevUiElement(p, rw, y, latAccelFactorFilteredElement.value, latAccelFactorFilteredElement.label, latAccelFactorFilteredElement.units, latAccelFactorFilteredElement.color);
  }
  else
  {
    UiElement steeringTorqueEpsElement = DeveloperUi::getSteeringTorqueEps(steeringTorqueEps);
    rw += drawDevUiElement(p, rw, y, steeringTorqueEpsElement.value, steeringTorqueEpsElement.label, steeringTorqueEpsElement.units, steeringTorqueEpsElement.color);

    UiElement bearingDegElement = DeveloperUi::getBearingDeg(bearingAccuracyDeg, bearingDeg);
    rw += drawDevUiElement(p, rw, y, bearingDegElement.value, bearingDegElement.label, bearingDegElement.units, bearingDegElement.color);
  }

  UiElement altitudeElement = DeveloperUi::getAltitude(gpsAccuracy, altitude);
  rw += drawDevUiElement(p, rw, y, altitudeElement.value, altitudeElement.label, altitudeElement.units, altitudeElement.color);

  p.restore();
}
// ############################## DEV UI END ##############################



// ********** metrics **********

// Add Relative Distance to Primary Lead Car

// Add Vehicle Current Acceleration
// Unit: m/s²
UiElement DeveloperUi::getAEgo(float a_ego) {
  QString value = QString::number(a_ego, 'f', 1);
  QColor color = QColor(255, 255, 255, 255);

  return UiElement(value, "ACC.", "m/s²", color);
}

// Add Relative Velocity to Primary Lead Car
// Unit: kph if metric, else mph
UiElement DeveloperUi::getVEgoLead(bool lead_status, float lead_v_rel, float v_ego, bool is_metric) {
  QString value = lead_status ? QString::number((lead_v_rel + v_ego) * (is_metric ? MS_TO_KPH : MS_TO_MPH), 'f', 0) : "-";
  QColor color = QColor(255, 255, 255, 255);

  if (lead_status) {
    // Red if approaching faster than 10mph
    // Orange if approaching (negative)
    if (lead_v_rel < -4.4704) {
      color = QColor(255, 0, 0, 255);
    } else if (lead_v_rel < 0) {
      color = QColor(255, 188, 0, 255);
    }
  }

  return UiElement(value, "L.S.", is_metric ? tr("km/h") : tr("mph"), color);
}

// Add Friction Coefficient Raw from torqued
// Unit: None
UiElement DeveloperUi::getFrictionCoefficientFiltered(float friction_coefficient_filtered, bool live_valid) {
  QString value = QString::number(friction_coefficient_filtered, 'f', 3);
  QColor color = live_valid ? QColor(0, 255, 0, 255) : QColor(255, 255, 255, 255);

  return UiElement(value, "FRIC.", "", color);
}

// Add Lateral Acceleration Factor Raw from torqued
// Unit: m/s²
UiElement DeveloperUi::getLatAccelFactorFiltered(float lat_accel_factor_filtered, bool live_valid) {
  QString value = QString::number(lat_accel_factor_filtered, 'f', 3);
  QColor color = live_valid ? QColor(0, 255, 0, 255) : QColor(255, 255, 255, 255);

  return UiElement(value, "L.A.", "m/s²", color);
}

// Add Steering Torque from Car EPS
// Unit: Newton Meters
UiElement DeveloperUi::getSteeringTorqueEps(float steering_torque_eps) {
  QString value = QString::number(std::fabs(steering_torque_eps), 'f', 1);
  QColor color = QColor(255, 255, 255, 255);

  return UiElement(value, "E.T.", "N·dm", color);
}

// Add Bearing Degree and Direction from Car (Compass)
// Unit: Meters
UiElement DeveloperUi::getBearingDeg(float bearing_accuracy_deg, float bearing_deg) {
  QString value = (bearing_accuracy_deg != 180.00) ? QString("%1%2%3").arg(QString::number(bearing_deg, 'd', 0)).arg("°").arg("") : "-";
  QColor color = QColor(255, 255, 255, 255);
  QString dir_value;

  if (bearing_accuracy_deg != 180.00) {
    if (((bearing_deg >= 337.5) && (bearing_deg <= 360)) || ((bearing_deg >= 0) && (bearing_deg <= 22.5))) {
      dir_value = "N";
    } else if ((bearing_deg > 22.5) && (bearing_deg < 67.5)) {
      dir_value = "NE";
    } else if ((bearing_deg >= 67.5) && (bearing_deg <= 112.5)) {
      dir_value = "E";
    } else if ((bearing_deg > 112.5) && (bearing_deg < 157.5)) {
      dir_value = "SE";
    } else if ((bearing_deg >= 157.5) && (bearing_deg <= 202.5)) {
      dir_value = "S";
    } else if ((bearing_deg > 202.5) && (bearing_deg < 247.5)) {
      dir_value = "SW";
    } else if ((bearing_deg >= 247.5) && (bearing_deg <= 292.5)) {
      dir_value = "W";
    } else if ((bearing_deg > 292.5) && (bearing_deg < 337.5)) {
      dir_value = "NW";
    }
  } else {
    dir_value = "OFF";
  }

  return UiElement(QString("%1 | %2").arg(dir_value).arg(value), "B.D.", "", color);
}

// Add Altitude of Current Location
// Unit: Meters
UiElement DeveloperUi::getAltitude(float gps_accuracy, float altitude) {
  QString value = (gps_accuracy != 0.00) ? QString::number(altitude, 'f', 1) : "-";
  QColor color = QColor(255, 255, 255, 255);

  return UiElement(value, "ALT.", "m", color);
}
