/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include "selfdrive/ui/qt/onroad/hud.h"
#include "selfdrive/ui/sunnypilot/qt/onroad/hud/turnsignal_ui.h"
#include "selfdrive/ui/sunnypilot/qt/onroad/hud/CircleWidget.h"
#include "selfdrive/ui/sunnypilot/qt/onroad/developer_ui/developer_ui.h"

class HudRendererSP : public HudRenderer {
  Q_OBJECT

public:
  explicit HudRendererSP();
  void updateState(const UIState &s) override;
  void draw(QPainter &p, const QRect &surface_rect) override;

protected:
  TurnSignalWidget *turnSignalWidget;
  CircleWidget *SteerWidget;
  CircleWidget *LongWidget;

private:
  Params params;
  QString road_name;

  void drawRoadName(QPainter &p, const QRect &surface_rect);

  void drawText(QPainter &p, int x, int y, const QString &text, QColor color = Qt::white);
  void drawRightDevUI(QPainter &p, int x, int y);
  int drawRightDevUIElement(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color);
  int drawBottomDevUIElement(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color);
  void drawBottomDevUI(QPainter &p, int x, int y);

  bool enabled = false;
  bool longActive = false;
  float CS_Steer = 0.0f;
  float CC_Steer = 0.0f;
  float CS_Long = 0.0f;
  float CC_Long = 0.0f;

  bool lead_status;
  float lead_d_rel;
  float lead_v_rel;
  bool torqueLateral;
  float angleSteers;
  float desiredCurvature;
  float curvature;
  float roll;
  int memoryUsagePercent;
  int devUiInfo;
  float gpsAccuracy;
  float altitude;
  float vEgo;
  float aEgo;
  float steeringTorqueEps;
  float bearingAccuracyDeg;
  float bearingDeg;
  bool torquedUseParams;
  float latAccelFactorFiltered;
  float frictionCoefficientFiltered;
  bool liveValid;
  QString speedUnit;
  bool latActive;
  bool steerOverride;
  bool reversing;
  cereal::CarParams::SteerControlType steerControlType;
  cereal::CarControl::Actuators::Reader actuators;
};
