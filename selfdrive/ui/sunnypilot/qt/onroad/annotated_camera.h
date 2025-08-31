/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include "selfdrive/ui/qt/onroad/annotated_camera.h"
#include "selfdrive/ui/sunnypilot/qt/util.h"
#include "selfdrive/ui/sunnypilot/qt/onroad/developer_ui/developer_ui.h"

class AnnotatedCameraWidgetSP : public AnnotatedCameraWidget {
  Q_OBJECT

public:
  explicit AnnotatedCameraWidgetSP(VisionStreamType type, QWidget *parent = nullptr);
  void updateState(const UIState &s) override;


private:
  void paintGL() override;


  // ############################## DEV UI START ##############################
  void drawHud(QPainter &p);
  void drawText(QPainter &p, int x, int y, const QString &text, QColor color);
  int drawDevUiRight(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color);
  int drawNewDevUi(QPainter &p, int x, int y, const QString &value, const QString &label, const QString &units, QColor &color);
  void drawRightDevUi(QPainter &p, int x, int y);
  void drawNewDevUi2(QPainter &p, int x, int y);
  void drawCenteredLeftText(QPainter &p, int x, int y, const QString &text1, QColor color1, const QString &text2, const QString &text3, QColor color2);
  // ############################## DEV UI END ##############################


  // ############################## DEV UI START ##############################
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
  // ############################## DEV UI END ##############################
};


