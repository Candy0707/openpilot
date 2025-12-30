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

    STOP_SPEED = 0.3   # m/s，靜止門檻
    MIN_DIST = 2.0     # 2x2 m 防重疊格
    placed = {}

    for point in radar:

      # 靜止物件不要
      if abs(point.vRel) < STOP_SPEED:
        continue

      # 防重疊格子
      coord_key = (
        int(point.dRel / MIN_DIST),
        int(point.yRel / MIN_DIST),
      )

      # 若該格已有更好的點 → continue
      if coord_key in placed:
        best = placed[coord_key]
        if not is_better(point, best):
          continue

      # 記錄此格目前最好的點
      placed[coord_key] = point

     #轉換成螢幕座標
      screen_pt = model.map(point.dRel, -point.yRel)
      if screen_pt is None:
        continue

      # 根據距離決定顏色
      color = self.radar_point_color(point.dRel, point.yRel)

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
        scale_x=int(screen_pt[0]),
        scale_y=int(screen_pt[1]),
      )

      self.Points.append(radar_element)

    return self.Points

  def radar_point_color(self, dRel: float, yRel: float) -> rl.Color:
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

def is_better(a, b):
  """
  回傳 True 代表 a 比 b 更值得顯示
  優先順序：
    1. |vRel| 大（高速）
    2. |yRel| 小（前方）
    3. dRel 小（距離近）
  """
  if abs(a.vRel) != abs(b.vRel):
    return abs(a.vRel) > abs(b.vRel)

  if abs(a.yRel) != abs(b.yRel):
    return abs(a.yRel) < abs(b.yRel)

  return a.dRel < b.dRel
