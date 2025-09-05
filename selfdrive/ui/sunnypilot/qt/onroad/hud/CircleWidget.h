#pragma once

#include "selfdrive/ui/sunnypilot/ui.h"
#include "selfdrive/ui/sunnypilot/qt/util.h"

class CircleWidget : public QObject
{
  Q_OBJECT

public:
  // 建構函式設定位置 (x,y) 與正方形尺寸
  explicit CircleWidget(QWidget *parent = nullptr);

  void draw(QPainter &p, const QRect &surface_rect);
  void updateState(const UIState &s);

protected:


private:
  Params params;
  int x;
  int y;
  int size;
  double innerArcWidth;
  double angle1;
  double angle2;
  double minAngle;
  double maxAngle;

    // 畫弧線 + 尾端光暈
  void drawArcWithTail(QPainter &painter, const QPointF &center,
                       double radius, double penWidth, double angle,
                       QColor colorStart, QColor colorMid, QColor colorEnd);
};
