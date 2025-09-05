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

#pragma once

#include "selfdrive/ui/sunnypilot/ui.h"
#include "selfdrive/ui/sunnypilot/qt/util.h"

struct UiElement {
  QString value{};
  QString label{};
  QString units{};
  QColor color{};

  explicit UiElement(const QString &value = "", const QString &label = "", const QString &units = "", const QColor &color = QColor(255, 255, 255, 255))
    : value(value), label(label), units(units), color(color) {}
};

class developer_ui : public QObject {
  Q_OBJECT

public:
  explicit developer_ui(QWidget *parent = nullptr);
  void drawDevUI(QPainter &p);
  int drawDevUiElement(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color);
  void drawCenteredLeftText(QPainter &p, int x, int y, const QString &text1, QColor color1, const QString &text2, const QString &text3, QColor color2);
  void drawText(QPainter &p, int x, int y, const QString &text, QColor color);
  void updateState(const UIState &s);


  QColor alpha = QColor(0, 0, 0, 255);
  bool is_metric = false;
  bool latActive = false;
  bool madsEnabled = false;
  bool lead_status;
  float lead_d_rel = 0;
  float lead_v_rel = 0;
  QString lateralState;
  float angleSteers = 0;
  float steerAngleDesired = 0;
  int memoryUsagePercent = 0;
  float curvature;
  float roll;
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

  static UiElement getAEgo(float a_ego);
  static UiElement getVEgoLead(bool lead_status, float lead_v_rel, float v_ego, bool is_metric);
  static UiElement getFrictionCoefficientFiltered(float friction_coefficient_filtered, bool live_valid);
  static UiElement getLatAccelFactorFiltered(float lat_accel_factor_filtered, bool live_valid);
  static UiElement getSteeringTorqueEps(float steering_torque_eps);
  static UiElement getBearingDeg(float bearing_accuracy_deg, float bearing_deg);
  static UiElement getAltitude(float gps_accuracy, float altitude);
};
