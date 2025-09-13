/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */
#include "common/swaglog.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/sunnypilot/qt/onroad/hud.h"

HudRendererSP::HudRendererSP() : HudRenderer()
{
  turnSignalWidget = new TurnSignalWidget();

  SteerWidget = new CircleWidget();
  SteerWidget->setup(-180.0f, 180.0f, "°", 192);
  SteerWidget->setoffset(0, 0);

  LongWidget = new CircleWidget();
  LongWidget->setup(-2.0f, 2.0f, " m/s²", 192);
  LongWidget->setoffset(0, 240);
}

void HudRendererSP::updateState(const UIState &s)
{
  const SubMaster &sm = *(s.sm);
  HudRenderer::updateState(s);
  turnSignalWidget->updateState(s);

  const auto car_params = sm["carParams"].getCarParams();
  const auto car_state = sm["carState"].getCarState();
  const auto car_control = sm["carControl"].getCarControl();
  const auto cs = sm["controlsState"].getControlsState();
  const bool cs_alive = sm.alive("controlsState");
  const auto radar_state = sm["radarState"].getRadarState();
  const auto ltp = sm["liveTorqueParameters"].getLiveTorqueParameters();
  const auto is_gps_location_external = sm.rcv_frame("gpsLocationExternal") > 1;
  const auto gpsLocation = is_gps_location_external ? sm["gpsLocationExternal"].getGpsLocationExternal() : sm["gpsLocation"].getGpsLocation();
  const auto live_map_data = sm["liveMapDataSP"].getLiveMapDataSP();

  enabled = car_state.getCruiseState().getAvailable();
  CS_Steer = car_state.getSteeringAngleDeg();
  CS_Long = car_state.getAEgo();

  latActive = car_control.getLatActive();
  longActive = car_control.getLongActive();

  CC_Steer = car_control.getActuators().getSteeringAngleDeg();
  CC_Long = car_control.getActuators().getAccel();

  road_name = QString::fromStdString(live_map_data.getRoadName());

  SteerWidget->updateState(s);
  SteerWidget->setValue(enabled, latActive, -CS_Steer, -CC_Steer);

  LongWidget->updateState(s);
  LongWidget->setValue(enabled, longActive, CS_Long, CC_Long);

  static int reverse_delay = 0;
  bool reverse_allowed = false;
  if (int(car_state.getGearShifter()) != 4)
  {
    reverse_delay = 0;
    reverse_allowed = false;
  }
  else
  {
    reverse_delay += 50;
    if (reverse_delay >= 1000)
    {
      reverse_allowed = true;
    }
  }

  reversing = reverse_allowed;
  is_metric = s.scene.is_metric;

  // Handle older routes where vEgoCluster is not set
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  speed *= is_metric ? MS_TO_KPH : MS_TO_MPH;

  steerOverride = car_state.getSteeringPressed();

  devUiInfo = s.scene.dev_ui_info;

  speedUnit = is_metric ? tr("km/h") : tr("mph");
  lead_d_rel = radar_state.getLeadOne().getDRel();
  lead_v_rel = radar_state.getLeadOne().getVRel();
  lead_status = radar_state.getLeadOne().getStatus();
  steerControlType = car_params.getSteerControlType();
  actuators = car_control.getActuators();
  torqueLateral = steerControlType == cereal::CarParams::SteerControlType::TORQUE;
  angleSteers = car_state.getSteeringAngleDeg();
  desiredCurvature = cs.getDesiredCurvature();
  curvature = cs.getCurvature();
  roll = sm["liveParameters"].getLiveParameters().getRoll();
  memoryUsagePercent = sm["deviceState"].getDeviceState().getMemoryUsagePercent();
  gpsAccuracy = is_gps_location_external ? gpsLocation.getHorizontalAccuracy() : 1.0; // External reports accuracy, internal does not.
  altitude = gpsLocation.getAltitude();
  vEgo = car_state.getVEgo();
  aEgo = car_state.getAEgo();
  steeringTorqueEps = car_state.getSteeringTorqueEps();
  bearingAccuracyDeg = gpsLocation.getBearingAccuracyDeg();
  bearingDeg = gpsLocation.getBearingDeg();
  torquedUseParams = ltp.getUseParams();
  latAccelFactorFiltered = ltp.getLatAccelFactorFiltered();
  frictionCoefficientFiltered = ltp.getFrictionCoefficientFiltered();
  liveValid = ltp.getLiveValid();
}

void HudRendererSP::draw(QPainter &p, const QRect &surface_rect)
{
  HudRenderer::draw(p, surface_rect);
  bool EN = false;
  EN = params.getBool("ShowRoadName");
  if (EN)
  {
    drawRoadName(p, surface_rect);
  }
  EN = params.getBool("ShowTurnSignals");
  if (EN)
  {
    turnSignalWidget->draw(p, surface_rect);
  }
  EN = params.getBool("ShowSteerState");
  if (EN)
  {
    SteerWidget->draw(p, surface_rect);
  }
  EN = params.getBool("ShowLongState");
  if (EN)
  {
    LongWidget->draw(p, surface_rect);
  }
  if (!reversing)
  {
    // Right Dev UI
    if (devUiInfo == 1)
    {
      QRect rect_right(surface_rect.right() - (UI_BORDER_SIZE * 2), UI_BORDER_SIZE * 1.5, 184, 170);
      drawRightDevUI(p, surface_rect.right() - 184 - UI_BORDER_SIZE * 2, UI_BORDER_SIZE * 2 + rect_right.height());
    }

    // Bottom Dev UI
    if (devUiInfo == 2)
    {
      QRect rect_bottom(surface_rect.left(), surface_rect.bottom() - 60, surface_rect.width(), 61);
      p.setPen(Qt::NoPen);
      p.setBrush(QColor(0, 0, 0, 100));
      p.drawRect(rect_bottom);
      drawBottomDevUI(p, rect_bottom.left(), rect_bottom.center().y());
    }

    if (devUiInfo == 3)
    {
      // Right Dev UI
      QRect rect_right(surface_rect.right() - (UI_BORDER_SIZE * 2), UI_BORDER_SIZE * 1.5, 184, 170);
      drawRightDevUI(p, surface_rect.right() - 184 - UI_BORDER_SIZE * 2, UI_BORDER_SIZE * 2 + rect_right.height());

      // Bottom Dev UI
      QRect rect_bottom(surface_rect.left(), surface_rect.bottom() - 60, surface_rect.width(), 61);
      p.setPen(Qt::NoPen);
      p.setBrush(QColor(0, 0, 0, 100));
      p.drawRect(rect_bottom);
      drawBottomDevUI(p, rect_bottom.left(), rect_bottom.center().y());
    }
  }
}

void HudRendererSP::drawText(QPainter &p, int x, int y, const QString &text, QColor color)
{
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});
  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

int HudRendererSP::drawRightDevUIElement(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color)
{

  p.setFont(InterFont(28, QFont::Bold));
  x += 92;
  y += 80;
  drawText(p, x, y, label);

  p.setFont(InterFont(30 * 2, QFont::Bold));
  y += 65;
  drawText(p, x, y, value, color);

  p.setFont(InterFont(28, QFont::Bold));

  if (units.length() > 0)
  {
    p.save();
    x += 120;
    y -= 25;
    p.translate(x, y);
    p.rotate(-90);
    drawText(p, 0, 0, units);
    p.restore();
  }

  return 130;
}

void HudRendererSP::drawRightDevUI(QPainter &p, int x, int y)
{
  int rh = 5;
  int ry = y;

  UiElement dRelElement = DeveloperUi::getDRel(lead_status, lead_d_rel);
  rh += drawRightDevUIElement(p, x, ry, dRelElement.value, dRelElement.label, dRelElement.units, dRelElement.color);
  ry = y + rh;

  UiElement vRelElement = DeveloperUi::getVRel(lead_status, lead_v_rel, is_metric, speedUnit);
  rh += drawRightDevUIElement(p, x, ry, vRelElement.value, vRelElement.label, vRelElement.units, vRelElement.color);
  ry = y + rh;

  UiElement steeringAngleDegElement = DeveloperUi::getSteeringAngleDeg(angleSteers, latActive, steerOverride);
  rh += drawRightDevUIElement(p, x, ry, steeringAngleDegElement.value, steeringAngleDegElement.label, steeringAngleDegElement.units, steeringAngleDegElement.color);
  ry = y + rh;

  UiElement actuatorsOutputLateralElement = DeveloperUi::getActuatorsOutputLateral(steerControlType, actuators, desiredCurvature, vEgo, roll, latActive, steerOverride);
  rh += drawRightDevUIElement(p, x, ry, actuatorsOutputLateralElement.value, actuatorsOutputLateralElement.label, actuatorsOutputLateralElement.units, actuatorsOutputLateralElement.color);
  ry = y + rh;

  UiElement actualLateralAccelElement = DeveloperUi::getActualLateralAccel(curvature, vEgo, roll, latActive, steerOverride);
  rh += drawRightDevUIElement(p, x, ry, actualLateralAccelElement.value, actualLateralAccelElement.label, actualLateralAccelElement.units, actualLateralAccelElement.color);
}

int HudRendererSP::drawBottomDevUIElement(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color)
{
  p.setFont(InterFont(38, QFont::Bold));
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(label + " ");
  QRect real_rect = fm.boundingRect(init_rect, 0, label + " ");
  real_rect.moveCenter({x, y});

  QRect init_rect2 = fm.boundingRect(value);
  QRect real_rect2 = fm.boundingRect(init_rect2, 0, value);
  real_rect2.moveTop(real_rect.top());
  real_rect2.moveLeft(real_rect.right() + 10);

  QRect init_rect3 = fm.boundingRect(units);
  QRect real_rect3 = fm.boundingRect(init_rect3, 0, units);
  real_rect3.moveTop(real_rect.top());
  real_rect3.moveLeft(real_rect2.right() + 10);

  p.setPen(Qt::white);
  p.drawText(real_rect, Qt::AlignLeft | Qt::AlignVCenter, label);

  p.setPen(color);
  p.drawText(real_rect2, Qt::AlignRight | Qt::AlignVCenter, value);
  p.drawText(real_rect3, Qt::AlignLeft | Qt::AlignVCenter, units);
  return 430;
}

void HudRendererSP::drawBottomDevUI(QPainter &p, int x, int y)
{
  int rw = 90;

  UiElement aEgoElement = DeveloperUi::getAEgo(aEgo);
  rw += drawBottomDevUIElement(p, rw, y, aEgoElement.value, aEgoElement.label, aEgoElement.units, aEgoElement.color);

  UiElement vEgoLeadElement = DeveloperUi::getVEgoLead(lead_status, lead_v_rel, vEgo, is_metric, speedUnit);
  rw += drawBottomDevUIElement(p, rw, y, vEgoLeadElement.value, vEgoLeadElement.label, vEgoLeadElement.units, vEgoLeadElement.color);

  if (torqueLateral && torquedUseParams)
  {
    UiElement frictionCoefficientFilteredElement = DeveloperUi::getFrictionCoefficientFiltered(frictionCoefficientFiltered, liveValid);
    rw += drawBottomDevUIElement(p, rw, y, frictionCoefficientFilteredElement.value, frictionCoefficientFilteredElement.label, frictionCoefficientFilteredElement.units, frictionCoefficientFilteredElement.color);

    UiElement latAccelFactorFilteredElement = DeveloperUi::getLatAccelFactorFiltered(latAccelFactorFiltered, liveValid);
    rw += drawBottomDevUIElement(p, rw, y, latAccelFactorFilteredElement.value, latAccelFactorFilteredElement.label, latAccelFactorFilteredElement.units, latAccelFactorFilteredElement.color);
  }
  else
  {
    UiElement steeringTorqueEpsElement = DeveloperUi::getSteeringTorqueEps(steeringTorqueEps);
    rw += drawBottomDevUIElement(p, rw, y, steeringTorqueEpsElement.value, steeringTorqueEpsElement.label, steeringTorqueEpsElement.units, steeringTorqueEpsElement.color);

    UiElement bearingDegElement = DeveloperUi::getBearingDeg(bearingAccuracyDeg, bearingDeg);
    rw += drawBottomDevUIElement(p, rw, y, bearingDegElement.value, bearingDegElement.label, bearingDegElement.units, bearingDegElement.color);
  }

  UiElement altitudeElement = DeveloperUi::getAltitude(gpsAccuracy, altitude);
  rw += drawBottomDevUIElement(p, rw, y, altitudeElement.value, altitudeElement.label, altitudeElement.units, altitudeElement.color);
}

void HudRendererSP::drawRoadName(QPainter &p, const QRect &surface_rect)
{
  if (road_name.isEmpty())
  {
    road_name = "None";
  }

  // Set font first to measure text
  p.setFont(InterFont(40, QFont::Normal));
  QFontMetrics fm(p.font());

  // Calculate required width based on text + padding
  int text_width = fm.horizontalAdvance(road_name);
  int padding = 40;
  int rect_width = text_width + padding;

  // Set minimum and maximum widths
  int min_width = 200;
  int max_width = surface_rect.width() - 40;
  rect_width = std::max(min_width, std::min(rect_width, max_width));

  // Position road name at the top center
  QRect road_rect(surface_rect.width() / 2 - rect_width / 2, 5, rect_width, 60);
  p.setPen(QPen(QColor(255, 255, 255, 100), 1));
  p.setBrush(QColor(0, 0, 0, 120));
  p.drawRoundedRect(road_rect, 6, 6);

  // Truncate long road names if they still don't fit
  p.setPen(QColor(255, 255, 255, 255));
  QString truncated = fm.elidedText(road_name, Qt::ElideRight, road_rect.width() - 20);
  p.drawText(road_rect, Qt::AlignCenter, truncated);
}