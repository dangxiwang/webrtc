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

#ifndef MODULES_RTP_RTCP_SOURCE_RTP_VIDEO_DEPACKETIZER_H264_H_
#define MODULES_RTP_RTCP_SOURCE_RTP_VIDEO_DEPACKETIZER_H264_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "modules/rtp_rtcp/source/rtp_video_depacketizer.h"

namespace webrtc {

class H264RtpDepacketizer final : public RtpVideoDepacketizer {
 public:
  explicit H264RtpDepacketizer(absl::string_view sprop_base64);
  ~H264RtpDepacketizer() override = default;

  absl::optional<FrameBoundaries> GetFrameBoundaries(
      rtc::ArrayView<const uint8_t> rtp_payload) override;

  absl::optional<Frame> AssembleFrame(
      rtc::ArrayView<const RtpPacket*> rtp_packets) override;

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

  bool seen_idr_ = false;
  bool seen_sps_ = false;
};

}  // namespace webrtc

#endif  // MODULES_RTP_RTCP_SOURCE_RTP_VIDEO_DEPACKETIZER_H264_H_
