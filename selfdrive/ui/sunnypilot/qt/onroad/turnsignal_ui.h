

#pragma once

#include "selfdrive/ui/sunnypilot/ui.h"
#include "selfdrive/ui/sunnypilot/qt/util.h"

class TurnSignalWidget : public QObject
{
  Q_OBJECT

public:
  explicit TurnSignalWidget(QWidget *parent = nullptr);
  void updateState(const UIState &s);
  void draw(QPainter &p, const QRect &surface_rect);

protected:
  bool ShowTurnSignals = false;

  void drawTurnSignal(QPainter &p, int circleX, int circleY, bool isLeft);

private:
  // 閃爍控制
  QTimer *blinkTimer;
  bool blinkState = false;

  // 位置
  int x = 0;
  int y = 0;

  // 畫圈圈
  int size = 0;
  int distance = 0;

  // 畫箭頭
  int arrowSize = 0;

  // 0完全透明 255不透明
  const int Alpha = 200;
  const QColor circle_off = QColor(22, 156, 69, 255);
  const QColor circle_on = QColor(30, 215, 96, 255);
  const QColor arrow_off = QColor(9, 56, 27, 255);
  const QColor arrow_on = QColor(255, 255, 255, 255);
};