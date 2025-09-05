#include "selfdrive/ui/sunnypilot/qt/onroad/hud/CircleWidget.h"
#include <QRadialGradient>
#include <QConicalGradient>
#include <QPen>
#include <QBrush>
#include <QFont>

CircleWidget::CircleWidget(QWidget *parent) : QObject(parent)
{
    angle1 = 0;
    angle2 = 0;
    minAngle = -180;
    maxAngle = 180;
    size = 192;
    innerArcWidth = 20;
}

void CircleWidget::updateState(const UIState &s)
{
    const SubMaster &sm = *(s.sm);
    const auto car_state = sm["carState"].getCarState();
    const auto controls_state = sm["controlsState"].getControlsState();

    angle1 = car_state.getSteeringAngleDeg();
    angle2 = controls_state.getLateralControlState().getPidState().getSteeringAngleDesiredDeg();
}

void CircleWidget::draw(QPainter &p, const QRect &surface_rect)
{
    p.setRenderHint(QPainter::Antialiasing, true);

    x = surface_rect.topRight().x() - size - UI_BORDER_SIZE;
    y = surface_rect.topRight().y() + 240 + UI_BORDER_SIZE;

    // 設定圓心
    QPointF center(x + size / 2.0, y + size / 2.0);

    double outerPenWidth = 3; // 外框線寬 (固定)
    // 半徑設定：外圈貼邊，內圈緊貼外圈
    double radiusOuter = size / 2.0;
    double radiusInner1 = radiusOuter - outerPenWidth / 2.0 - innerArcWidth / 2.0;
    double radiusInner2 = radiusInner1 - innerArcWidth / 2.0 - innerArcWidth / 2.0;

    // -----------------------
    // HUD 半透明光暈背景
    // -----------------------
    QRadialGradient bgGlow(center, radiusOuter * 1.2);
    bgGlow.setColorAt(0.0, QColor(255, 255, 255, 60));
    bgGlow.setColorAt(0.7, QColor(255, 255, 255, 10));
    bgGlow.setColorAt(1.0, QColor(255, 255, 255, 0));
    p.setBrush(bgGlow);
    p.setPen(Qt::NoPen);
    p.drawEllipse(center, radiusOuter * 1.1, radiusOuter * 1.1);

    // -----------------------
    // 外框圓
    // -----------------------
    QPen penOuter(QColor(255, 255, 255, 200), outerPenWidth);
    p.setPen(penOuter);
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(center, radiusOuter, radiusOuter);

    // -----------------------
    // 中心文字 + 光暈描邊
    // -----------------------
    QString angleText = QString::number(angle1, 'f', 1) + "°";
    QFont font = p.font();
    font.setPointSize(size / 10);
    font.setBold(true);
    p.setFont(font);

    // 使用 QRectF 畫文字，避免 QPointF 對齊錯誤
    QRectF textRect(center.x() - size / 4, center.y() - size / 4, size / 2, size / 2);

    // 光暈描邊
    QPen glowPen(QColor(255, 255, 255, 120));
    p.setPen(glowPen);
    p.drawText(textRect.translated(1, 1), Qt::AlignCenter, angleText);
    p.drawText(textRect.translated(-1, -1), Qt::AlignCenter, angleText);

    // 正文字
    p.setPen(Qt::white);
    p.drawText(textRect, Qt::AlignCenter, angleText);

    // -----------------------
    // 內圈弧線 + 尾端光暈
    // -----------------------
    drawArcWithTail(p, center, radiusInner1, innerArcWidth, angle1,
                    QColor(0, 255, 0), QColor(255, 255, 0), QColor(255, 0, 0));
    drawArcWithTail(p, center, radiusInner2, innerArcWidth, angle2,
                    QColor(30, 128, 30), QColor(128, 128, 30), QColor(128, 30, 30));
}

void CircleWidget::drawArcWithTail(QPainter &p, const QPointF &center,
                                   double radius, double penWidth, double angle,
                                   QColor colorStart, QColor colorMid, QColor colorEnd)
{
    // 上下限保護
    angle = qBound(minAngle, angle, maxAngle);

    QRectF rect(center.x() - radius, center.y() - radius, radius * 2, radius * 2);

    // 動態漸層強度
    double factor = qAbs(angle) / 180.0;
    auto adjustColor = [factor](QColor c)
    {
        int r = static_cast<int>(c.red() * factor + 50 * (1 - factor));
        int g = static_cast<int>(c.green() * factor + 50 * (1 - factor));
        int b = static_cast<int>(c.blue() * factor + 50 * (1 - factor));
        return QColor(r, g, b);
    };

    QColor start = adjustColor(colorStart);
    QColor mid = adjustColor(colorMid);
    QColor end = adjustColor(colorEnd);

    // 畫弧線
    QConicalGradient grad(center, 90);
    grad.setColorAt(0.0, start);
    grad.setColorAt(0.25, mid);
    grad.setColorAt(0.5, end);
    grad.setColorAt(0.75, mid);
    grad.setColorAt(1.0, start);

    QPen pen(QBrush(grad), penWidth, Qt::SolidLine, Qt::RoundCap);
    p.setPen(pen);
    p.drawArc(rect, 90 * 16, -angle * 16);

    // 尾端光暈
    double radEnd = (90 - angle) * M_PI / 180.0;
    QPointF endPoint(center.x() + radius * std::cos(radEnd),
                     center.y() - radius * std::sin(radEnd));

    QColor tailColor = end;
    int alpha = 80 + static_cast<int>(100 * factor);
    tailColor.setAlpha(alpha);
    double glowRadius = penWidth * (1.0 + factor);

    QRadialGradient tailGlow(endPoint, glowRadius);
    tailGlow.setColorAt(0.0, tailColor);
    tailGlow.setColorAt(1.0, QColor(tailColor.red(), tailColor.green(),
                                    tailColor.blue(), 0));

    p.setBrush(tailGlow);
    p.setPen(Qt::NoPen);
    p.drawEllipse(endPoint, glowRadius, glowRadius);
}
