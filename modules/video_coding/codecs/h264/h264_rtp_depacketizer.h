/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 */

#ifndef MODULES_VIDEO_CODING_CODECS_H264_H264_RTP_DEPACKETIZER_H_
#define MODULES_VIDEO_CODING_CODECS_H264_H264_RTP_DEPACKETIZER_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "absl/types/optional.h"
#include "api/function_view.h"
#include "modules/video_coding/rtp_video_depacketizer.h"

namespace webrtc {

class H264RtpDepacketizer final : public RtpVideoDepacketizer {
 public:
  explicit H264RtpDepacketizer(
      const std::map<std::string, std::string>& codec_params);
  ~H264RtpDepacketizer() override = default;

  absl::optional<FrameBoundaries> GetFrameBoundaries(
      rtc::ArrayView<const uint8_t> rtp_payload) override;

  absl::optional<Frame> AssembleFrame(
      size_t num_packets,
      rtc::FunctionView<const RtpPacketReceived&(size_t index)> get_packet)
      override;

 private:
  struct PpsInfo {
    int sps_id = -1;
    size_t size = 0;
    std::unique_ptr<uint8_t[]> data;
  };

  struct SpsInfo {
    size_t size = 0;
    int width = -1;
    int height = -1;
    std::unique_ptr<uint8_t[]> data;
  };

  void InsertSpsPpsNalus(rtc::ArrayView<const uint8_t> sps,
                         rtc::ArrayView<const uint8_t> pps) {}

  const bool sps_pps_idr_is_keyframe_;
  std::map<uint32_t, PpsInfo> pps_data_;
  std::map<uint32_t, SpsInfo> sps_data_;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_CODECS_H264_H264_RTP_DEPACKETIZER_H_
