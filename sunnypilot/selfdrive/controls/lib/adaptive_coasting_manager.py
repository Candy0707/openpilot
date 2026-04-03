"""
Adaptive Coasting Manager (ACM)
負責處理中高速跟車切入時的舒適度，透過運動學公式計算滑行與輕微減速需求。
"""
import time
import numpy as np
from cereal import messaging, custom
from openpilot.common.swaglog import cloudlog

class AdaptiveCoastingManager:
  def __init__(self):
    self.active = False
    self.just_disabled = False
    self._active_prev = False

  def update(self, sm: messaging.SubMaster, v_ego: float, t_follow: float) -> float | None:
    """
    核心邏輯：計算舒適的防護加速度
    回傳值：
      - None: 狀況超出處理範圍 (距離過遠、或危險需棄權)
      - 0.0:  純滑行 (放油門)
      - float: 需要的輕微減速度 (a_req)
    """
    radarState = sm['radarState']
    lead = radarState.leadOne

    # 1. 觸發門檻基本檢查
    # 必須有前車，且本車正在接近前車 (v_rel < 0)
    if not lead.status or lead.vRel >= 0:
      self._set_inactive()
      return None

    d_rel = lead.dRel
    v_rel = lead.vRel

    # 2. 安全距離定義 (含 4.0m 底盤保險)
    d_safe = max(4.0, (v_ego * t_follow) * 0.75)

    # 距離大於安全距離 -> 不介入
    if d_rel >= d_safe:
      self._set_inactive()
      return None

    # 3. 碰撞判定 (運動學公式)
    # 預留 2.0m 作為最終停止緩衝
    s_stop = max(0.2, d_rel - 2.0)
    a_req = -(v_rel**2) / (2 * s_stop)

    # 4. 安全閥 (Comfort Valve)
    # 如果物理需求跌破 -1.0 m/s^2，代表狀況不適合滑行，立即棄權交由 MPC 處理
    if a_req < -1.0:
      self._set_inactive()
      return None

    # 5. 輸出決策
    self._set_active(v_ego, a_req)

    # 判斷是否需要介入煞車
    # 這裡加入一點點的 Hysteresis (-0.1)，防止 0.0 邊緣高頻震盪
    if a_req < -0.1:
      return a_req  # 輕煞車
    else:
      return 0.0    # 純滑行

  def _set_active(self, v_ego, a_req):
    self.active = True
    self.just_disabled = False
    if not self._active_prev:
      cloudlog.info(f"ACM ON: v={v_ego*3.6:.0f}kph, a_req={a_req:.2f}")
    self._active_prev = True

  def _set_inactive(self):
    self.active = False
    self.just_disabled = self._active_prev
    if self.just_disabled:
      cloudlog.info("ACM OFF")
    self._active_prev = False