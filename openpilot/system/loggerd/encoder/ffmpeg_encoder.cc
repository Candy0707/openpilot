#include "system/loggerd/encoder/ffmpeg_encoder.h"

#include <fcntl.h>
#include <unistd.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>

#define __STDC_CONSTANT_MACROS

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/dict.h>
#include <libavutil/imgutils.h>
}

#include "common/swaglog.h"
#include "common/util.h"
#include "common/yuv.h"

const int env_debug_encoder = (getenv("DEBUG_ENCODER") != NULL) ? atoi(getenv("DEBUG_ENCODER")) : 0;

FfmpegEncoder::FfmpegEncoder(const EncoderInfo &encoder_info, int in_width, int in_height)
    : VideoEncoder(encoder_info, in_width, in_height) {
  frame = av_frame_alloc();
  assert(frame);
  frame->format = AV_PIX_FMT_YUV420P;
  frame->width = out_width;
  frame->height = out_height;
  frame->linesize[0] = out_width;
  frame->linesize[1] = out_width/2;
  frame->linesize[2] = out_width/2;

  convert_buf.resize(in_width * in_height * 3 / 2);

  if (in_width != out_width || in_height != out_height) {
    downscale_buf.resize(out_width * out_height * 3 / 2);
  }
}

FfmpegEncoder::~FfmpegEncoder() {
  encoder_close();
  av_frame_free(&frame);
}

void FfmpegEncoder::encoder_open() {
  auto codec_id = encoder_info.get_settings(in_width).encode_type == cereal::EncodeIndex::Type::QCAMERA_H264
                      ? AV_CODEC_ID_H264
                      : AV_CODEC_ID_FFVHUFF;
  const AVCodec *codec = avcodec_find_encoder(codec_id);

  this->codec_ctx = avcodec_alloc_context3(codec);
  assert(this->codec_ctx);
  this->codec_ctx->width = frame->width;
  this->codec_ctx->height = frame->height;
  this->codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
  this->codec_ctx->time_base = (AVRational){ 1, encoder_info.fps };

  AVDictionary *opts = nullptr;
  if (codec_id == AV_CODEC_ID_H264) {
    // 低延遲即時串流調優：
    // - gop_size = fps：每 1 秒輸出 1 張 IDR 關鍵幀（取代預設 250 幀），大幅縮短連線首幀延遲。
    // - max_b_frames = 0 / LOW_DELAY：停用 B-frame 重排，消除編碼緩衝延遲。
    // - thread_count = 2：限制 libx264 執行緒數，避免模擬環境吃滿多核 CPU（原本高達 509%）。
    // - repeat-headers=1：確保每張 IDR 關鍵幀前都自動附帶 SPS/PPS，供遠端解碼器即時起播。
    this->codec_ctx->gop_size = encoder_info.fps > 0 ? encoder_info.fps : 20;
    this->codec_ctx->max_b_frames = 0;
    this->codec_ctx->thread_count = 2;
    this->codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;

    av_dict_set(&opts, "preset", "ultrafast", 0);
    av_dict_set(&opts, "tune", "zerolatency", 0);
    av_dict_set(&opts, "x264-params", "repeat-headers=1:annexb=1", 0);
  }

  int err = avcodec_open2(this->codec_ctx, codec, &opts);
  av_dict_free(&opts);
  assert(err >= 0);

  keyframe_requested.store(false, std::memory_order_release);
  bitrate_warned = false;
  is_open = true;
  segment_num++;
  counter = 0;
}

void FfmpegEncoder::encoder_close() {
  if (!is_open) return;

  avcodec_free_context(&codec_ctx);
  is_open = false;
}

void FfmpegEncoder::set_bitrate(int bitrate) {
  (void)bitrate;
  // 軟體編碼器不支援運行中熱改位元率：僅首次提示一次，避免 webrtcd 位元率控制持續洗版拖累效能
  if (!bitrate_warned) {
    LOGW("adaptive bitrate is not supported for ffmpeg encoder %s (further warnings suppressed)", encoder_info.publish_name);
    bitrate_warned = true;
  }
}

void FfmpegEncoder::request_keyframe() {
  // 標記於下一張送編碼的影格強制產生 IDR 關鍵幀（搭配 repeat-headers=1 自動附帶 SPS/PPS）
  keyframe_requested.store(true, std::memory_order_release);
}

int FfmpegEncoder::encode_frame(VisionBuf* buf, VisionIpcBufExtra *extra) {
  assert(buf->width == this->in_width);
  assert(buf->height == this->in_height);

  uint8_t *cy = convert_buf.data();
  uint8_t *cu = cy + in_width * in_height;
  uint8_t *cv = cu + (in_width / 2) * (in_height / 2);
  yuv::nv12_to_i420(buf->y, buf->stride,
                    buf->uv, buf->stride,
                    cy, in_width,
                    cu, in_width/2,
                    cv, in_width/2,
                    in_width, in_height);

  if (downscale_buf.size() > 0) {
    uint8_t *out_y = downscale_buf.data();
    uint8_t *out_u = out_y + frame->width * frame->height;
    uint8_t *out_v = out_u + (frame->width / 2) * (frame->height / 2);
    yuv::i420_scale(cy, in_width,
                    cu, in_width/2,
                    cv, in_width/2,
                    in_width, in_height,
                    out_y, frame->width,
                    out_u, frame->width/2,
                    out_v, frame->width/2,
                    frame->width, frame->height);
    frame->data[0] = out_y;
    frame->data[1] = out_u;
    frame->data[2] = out_v;
  } else {
    frame->data[0] = cy;
    frame->data[1] = cu;
    frame->data[2] = cv;
  }
  frame->pts = counter*50*1000; // 50ms per frame

  // 依外部 request_keyframe() 請求決定本幀是否強制輸出 I-frame（IDR），送出後自動還原為自動排程
  if (keyframe_requested.exchange(false, std::memory_order_acq_rel)) {
    frame->pict_type = AV_PICTURE_TYPE_I;
  } else {
    frame->pict_type = AV_PICTURE_TYPE_NONE;
  }

  int ret = counter;

  int err = avcodec_send_frame(this->codec_ctx, frame);
  if (err < 0) {
    LOGE("avcodec_send_frame error %d", err);
    ret = -1;
  }

  AVPacket pkt = {};
  pkt.data = NULL;
  pkt.size = 0;
  while (ret >= 0) {
    err = avcodec_receive_packet(this->codec_ctx, &pkt);
    if (err == AVERROR_EOF) {
      break;
    } else if (err == AVERROR(EAGAIN)) {
      // Encoder might need a few frames on startup to get started. Keep going
      ret = 0;
      break;
    } else if (err < 0) {
      LOGE("avcodec_receive_packet error %d", err);
      ret = -1;
      break;
    }

    if (env_debug_encoder) {
      printf("%20s got %8d bytes flags %8x idx %4d id %8d\n", encoder_info.publish_name, pkt.size, pkt.flags, counter, extra->frame_id);
    }

    publisher_publish(segment_num, counter, *extra,
      (pkt.flags & AV_PKT_FLAG_KEY) ? V4L2_BUF_FLAG_KEYFRAME : 0,
      kj::arrayPtr<capnp::byte>(pkt.data, (size_t)0), // TODO: get the header
      kj::arrayPtr<capnp::byte>(pkt.data, pkt.size));

    counter++;
  }
  av_packet_unref(&pkt);
  return ret;
}