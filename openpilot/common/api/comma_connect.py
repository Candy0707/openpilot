import os

from openpilot.common.api.base import BaseApi
from openpilot.sunnypilot.privacy import privacy_mode

API_HOST = os.getenv('API_HOST', 'https://api.commadotai.com')


class CommaConnectApi(BaseApi):
  def __init__(self, dongle_id):
    super().__init__(dongle_id, API_HOST)
    self.user_agent = "openpilot-"

  def api_get(self, endpoint, *args, **kwargs):
    # 隱私模式：任何對 comma.ai API 的請求都不送出（註冊、prime 狀態、firehose、行程統計…）
    if privacy_mode():
      raise ConnectionError(f"PrivacyMode: comma.ai API blocked ({endpoint})")
    return super().api_get(endpoint, *args, **kwargs)
