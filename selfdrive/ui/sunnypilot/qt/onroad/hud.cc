/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */
#include "common/swaglog.h"
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

  bool enabled = false;
  bool latActive = false;
  bool longActive = false;
  float CS_Steer = 0.0f;
  float CC_Steer = 0.0f;
  float CS_Long = 0.0f;
  float CC_Long = 0.0f;

  try
  {
    if (sm.alive("carState") && sm.rcv_frame("carState") > 0)
    {
      const auto car_state = sm["carState"].getCarState();
      CS_Steer = car_state.getSteeringAngleDeg();
      CS_Long = car_state.getAEgo();
    }
    if (sm.alive("carControl") && sm.rcv_frame("carControl") > 0)
    {
      const auto car_control = sm["carControl"].getCarControl();
      enabled = car_control.getEnabled();
      latActive = car_control.getLatActive();
      longActive = car_control.getLongActive();
      if (car_control.hasActuators())
      {
        CC_Steer = car_control.getActuators().getSteeringAngleDeg();
        CC_Long = car_control.getActuators().getAccel();
      }
    }
  }
  catch (const std::exception &e)
  {
    LOGE("HUD UpdateState Exception: %s", e.what());
  }

  SteerWidget->updateState(s);
  SteerWidget->setValue(enabled, latActive, CS_Steer, CC_Steer);

  LongWidget->updateState(s);
  LongWidget->setValue(enabled, longActive, CS_Long, CC_Long);
}

void HudRendererSP::draw(QPainter &p, const QRect &surface_rect)
{
  HudRenderer::draw(p, surface_rect);
  bool EN = false;
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
}