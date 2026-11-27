#pragma once

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
}

#include "system/loggerd/encoder/encoder.h"
#include "system/loggerd/loggerd.h"

class FfmpegEncoder : public VideoEncoder {
public:
  FfmpegEncoder(const EncoderInfo &encoder_info, int in_width, int in_height);
  ~FfmpegEncoder();
  int encode_frame(VisionBuf* buf, VisionIpcBufExtra *extra);
  void encoder_open();
  void encoder_close();
  void set_bitrate(int bitrate);
  void request_keyframe();

private:
  int segment_num = -1;
  int counter = 0;
  bool is_open = false;

  // 遠端串流關鍵幀請求旗標（跨執行緒安全）與位元率不支援單次提示旗標
  std::atomic<bool> keyframe_requested{false};
  bool bitrate_warned = false;

  AVCodecContext *codec_ctx = nullptr;
  AVFrame *frame = NULL;
  std::vector<uint8_t> convert_buf;
  std::vector<uint8_t> downscale_buf;
};