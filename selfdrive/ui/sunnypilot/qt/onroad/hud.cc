/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/onroad/hud.h"

HudRendererSP::HudRendererSP() : HudRenderer()
{
  EN = false;
  turnSignalWidget = new TurnSignalWidget();
  circleWidget = new CircleWidget();
}

void HudRendererSP::updateState(const UIState &s)
{
  HudRenderer::updateState(s);
  turnSignalWidget->updateState(s);
  circleWidget->updateState(s);
}

void HudRendererSP::draw(QPainter &p, const QRect &surface_rect)
{
  HudRenderer::draw(p, surface_rect);

  EN = params.getBool("ShowTurnSignals");
  if (EN)
  {
    turnSignalWidget->draw(p, surface_rect);
  }

  EN = params.getBool("ShowSteeringAngle");
  if (EN)
  {
    circleWidget->draw(p, surface_rect);
  }
}