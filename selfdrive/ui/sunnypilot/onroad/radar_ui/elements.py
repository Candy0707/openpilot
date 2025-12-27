from dataclasses import dataclass
from cereal import car
import pyray as rl

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

  def update(self, radar: 'car.RadarData.RadarPoint') -> list[RadarElement]:
    self.Points = []

    # 已經放置的點，用於避免重疊 (x,y tuples)
    placed_coords = set()
    MIN_DIST = 2  # 不重疊範圍，2x2m

    for point in radar:

      # 防重疊
      coord_key = (int(point.dRel / MIN_DIST), int(point.yRel / MIN_DIST))
      if coord_key in placed_coords:
        continue  # 忽略重疊點


      # 根據距離決定顏色
      if point.dRel < 10:
        color = rl.RED
      elif point.dRel < 20:
        color = rl.YELLOW
      else:
        color = rl.GREEN

      # 初始化 RadarElement
      radar_element = RadarElement(
        trackId=point.trackId,
        dRel=point.dRel,
        yRel=point.yRel,
        vRel=point.vRel,
        aRel=point.aRel,
        yvRel=point.yvRel,
        measured=point.measured,
        color=color,
      )

      self.Points.append(radar_element)

    return self.Points


