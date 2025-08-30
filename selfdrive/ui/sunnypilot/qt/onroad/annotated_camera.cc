/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/util.h"
#include "selfdrive/ui/sunnypilot/qt/onroad/annotated_camera.h"

AnnotatedCameraWidgetSP::AnnotatedCameraWidgetSP(VisionStreamType type, QWidget *parent) : AnnotatedCameraWidget(type, parent) {
  QObject::disconnect(uiState(), &UIState::uiUpdate, this, &AnnotatedCameraWidget::updateState);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &AnnotatedCameraWidget::updateState);
}

void AnnotatedCameraWidgetSP::updateState(const UIState &s) {
  AnnotatedCameraWidget::updateState(s);

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
    */
}

void AnnotatedCameraWidgetSP::drawText(QPainter &p, int x, int y, const QString &text, QColor color) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});
  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

// ############################## DEV UI START ##############################
void AnnotatedCameraWidgetSP::drawCenteredLeftText(QPainter &p, int x, int y, const QString &text1, QColor color1, const QString &text2, const QString &text3, QColor color2) {
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

int AnnotatedCameraWidgetSP::drawDevUiRight(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color) {
  p.setFont(InterFont(30 * 2, QFont::Bold));
  drawText(p, x + 92, y + 80, value, color);

  p.setFont(InterFont(28, QFont::Bold));
  drawText(p, x + 92, y + 80 + 42, label, alpha);

  if (units.length() > 0) {
    p.save();
    p.translate(x + 54 + 30 - 3 + 92 + 30, y + 37 + 25);
    p.rotate(-90);
    drawText(p, 0, 0, units, alpha);
    p.restore();
  }

  return 110;
}

void AnnotatedCameraWidgetSP::drawRightDevUi(QPainter &p, int x, int y) {
  int rh = 5;
  int ry = y;

  UiElement dRelElement = DeveloperUi::getDRel(lead_status, lead_d_rel);
  rh += drawDevUiRight(p, x, ry, dRelElement.value, dRelElement.label, dRelElement.units, dRelElement.color);
  ry = y + rh;

  UiElement vRelElement = DeveloperUi::getVRel(lead_status, lead_v_rel, is_metric);
  rh += drawDevUiRight(p, x, ry, vRelElement.value, vRelElement.label, vRelElement.units, vRelElement.color);
  ry = y + rh;

  UiElement steeringAngleDegElement = DeveloperUi::getSteeringAngleDeg(angleSteers, madsEnabled, latActive);
  rh += drawDevUiRight(p, x, ry, steeringAngleDegElement.value, steeringAngleDegElement.label, steeringAngleDegElement.units, steeringAngleDegElement.color);
  ry = y + rh;

  if (lateralState == "torque") {
    UiElement actualLateralAccelElement = DeveloperUi::getActualLateralAccel(curvature, vEgo, roll, madsEnabled, latActive);
    rh += drawDevUiRight(p, x, ry, actualLateralAccelElement.value, actualLateralAccelElement.label, actualLateralAccelElement.units, actualLateralAccelElement.color);
  } else {
    UiElement steeringAngleDesiredDegElement = DeveloperUi::getSteeringAngleDesiredDeg(madsEnabled, latActive, steerAngleDesired, angleSteers);
    rh += drawDevUiRight(p, x, ry, steeringAngleDesiredDegElement.value, steeringAngleDesiredDegElement.label, steeringAngleDesiredDegElement.units, steeringAngleDesiredDegElement.color);
  }
  ry = y + rh;

  UiElement memoryUsagePercentElement = DeveloperUi::getMemoryUsagePercent(memoryUsagePercent);
  rh += drawDevUiRight(p, x, ry, memoryUsagePercentElement.value, memoryUsagePercentElement.label, memoryUsagePercentElement.units, memoryUsagePercentElement.color);
  ry = y + rh;

  rh += 25;
  p.setBrush(QColor(0, 0, 0, 0));
  QRect ldu(x, y, 184, rh);
}

int AnnotatedCameraWidgetSP::drawNewDevUi(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color) {
  p.setFont(InterFont(38, QFont::Bold));
  QColor white = QColor(0, 0, 0, 0);
  drawCenteredLeftText(p, x, y, label, white, value, units, color);

  return 430;
}

void AnnotatedCameraWidgetSP::drawNewDevUi2(QPainter &p, int x, int y) {
  int rw = 90;

  UiElement aEgoElement = DeveloperUi::getAEgo(aEgo);
  rw += drawNewDevUi(p, rw, y, aEgoElement.value, aEgoElement.label, aEgoElement.units, aEgoElement.color);

  UiElement vEgoLeadElement = DeveloperUi::getVEgoLead(lead_status, lead_v_rel, vEgo, is_metric);
  rw += drawNewDevUi(p, rw, y, vEgoLeadElement.value, vEgoLeadElement.label, vEgoLeadElement.units, vEgoLeadElement.color);

  if (torquedUseParams) {
    UiElement frictionCoefficientFilteredElement = DeveloperUi::getFrictionCoefficientFiltered(frictionCoefficientFiltered, liveValid);
    rw += drawNewDevUi(p, rw, y, frictionCoefficientFilteredElement.value, frictionCoefficientFilteredElement.label, frictionCoefficientFilteredElement.units, frictionCoefficientFilteredElement.color);

    UiElement latAccelFactorFilteredElement = DeveloperUi::getLatAccelFactorFiltered(latAccelFactorFiltered, liveValid);
    rw += drawNewDevUi(p, rw, y, latAccelFactorFilteredElement.value, latAccelFactorFilteredElement.label, latAccelFactorFilteredElement.units, latAccelFactorFilteredElement.color);
  } else {
    UiElement steeringTorqueEpsElement = DeveloperUi::getSteeringTorqueEps(steeringTorqueEps);
    rw += drawNewDevUi(p, rw, y, steeringTorqueEpsElement.value, steeringTorqueEpsElement.label, steeringTorqueEpsElement.units, steeringTorqueEpsElement.color);

    UiElement bearingDegElement = DeveloperUi::getBearingDeg(bearingAccuracyDeg, bearingDeg);
    rw += drawNewDevUi(p, rw, y, bearingDegElement.value, bearingDegElement.label, bearingDegElement.units, bearingDegElement.color);
  }

  UiElement altitudeElement = DeveloperUi::getAltitude(gpsAccuracy, altitude);
  rw += drawNewDevUi(p, rw, y, altitudeElement.value, altitudeElement.label, altitudeElement.units, altitudeElement.color);
}

// ############################## DEV UI END ##############################