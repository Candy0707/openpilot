/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/sunnypilot/qt/onroad/hud.h"

HudRendererSP::HudRendererSP() : HudRenderer()
{
}

void HudRendererSP::updateState(const UIState &s)
{
  HudRenderer::updateState(s);
}

void HudRendererSP::draw(QPainter &p, const QRect &surface_rect)
{
  HudRenderer::draw(p, surface_rect);

  // Test box
  drawSurfaceRect(p, surface_rect);
}

void HudRendererSP::drawSurfaceRect(QPainter &p, const QRect &surface_rect)
{
  // 設定畫筆 (藍色，線寬 6)
  QPen pen(QColor(0, 0, 255), 6); // RGB藍色
  p.setPen(pen);

  // 填充背景
  p.setBrush(QColor(0, 100, 100, 100));

  // 畫矩形框
  p.drawRect(surface_rect);

  // 如果要在矩形內顯示大小
  QString sizeStr = QString("%1 x %2").arg(surface_rect.width()).arg(surface_rect.height());
  p.setPen(Qt::white);
  p.setFont(QFont("Arial", 16, QFont::Bold));
  p.drawText(surface_rect.topLeft() + QPoint(10, 25), sizeStr);
}