from openpilot.common.swaglog import cloudlog
from dataclasses import dataclass
import pyray as rl

from cereal import car


@dataclass
class RadarElement:
  trackId: int #uint

  dRel: float
  yRel: float
  vRel: float

  aRel: float
  yvRel: float

  measured: bool

  color: rl.Color

  # ===== Screen Space =====
  scale_x: int
  scale_y: int

class RadarData():
  def __init__(self):
    self.Points: list[RadarElement] = []

  def update(self, model, radar: 'car.RadarData.RadarPoint') -> list[RadarElement]:
    if model is None or radar is None:
      return []

    self.Points = []

    # 已經放置的點，用於避免重疊 (x,y tuples)
    placed_coords = set()
    MIN_DIST = 2  # 不重疊範圍，2x2m

    for point in radar:

      # 防重疊
      coord_key = (int(point.dRel / MIN_DIST), int(point.yRel / MIN_DIST))
      if coord_key in placed_coords:
        continue  # 忽略重疊點

     #轉換成螢幕座標
      screen_pt = model.map(point.dRel, point.yRel)
      if screen_pt is None:
        continue

      # 根據距離決定顏色
      color = self.radar_point_color(point.dRel, point.yRel)

      # 初始化 RadarElement
      radar_element = RadarElement(
        trackId=point.trackId,
        dRel=point.dRel,
        yRel=-point.yRel,
        vRel=point.vRel,
        aRel=point.aRel,
        yvRel=point.yvRel,
        measured=point.measured,
        color=color,
        scale_x=int(screen_pt[0]),
        scale_y=int(screen_pt[1]),
      )

      self.Points.append(radar_element)

    return self.Points

  def radar_point_color(dRel: float, yRel: float) -> rl.Color:
      abs_y = abs(yRel)

      # --------------------------------------------------
      # 1️⃣ 危險程度 t（0=綠 → 0.5=黃 → 1=紅）
      # --------------------------------------------------
      if dRel > 50 and abs_y > 3:
        t = 0.0
      elif dRel > 50 and abs_y <= 3:
        t = 0.33
      elif dRel < 10 or abs_y < 2:
        t = 1.0
      else:
        t = 0.66

      # --------------------------------------------------
      # 2️⃣ 顏色漸變（直接算，不用 helper）
      # --------------------------------------------------
      if t <= 0.5:
        # 綠 → 黃
        r = int(255 * (t * 2))
        g = 255
        b = 0
      else:
        # 黃 → 紅
        r = 255
        g = int(255 * (1 - (t - 0.5) * 2))
        b = 0

      # --------------------------------------------------
      # 3️⃣ 透明度漸變（40% ~ 80%）
      # --------------------------------------------------
      d = max(0.0, min(dRel, 50.0))
      alpha = int(255 * (0.8 - (d / 50.0) * 0.4))

      return rl.Color(r, g, b, alpha)
