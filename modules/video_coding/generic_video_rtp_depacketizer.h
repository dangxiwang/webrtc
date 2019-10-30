/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_VIDEO_CODING_GENERIC_VIDEO_RTP_DEPACKETIZER_H_
#define MODULES_VIDEO_CODING_GENERIC_VIDEO_RTP_DEPACKETIZER_H_

#include <cstdint>
#include <memory>

#include "absl/types/optional.h"
#include "api/array_view.h"
#include "api/function_view.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/video_coding/rtp_video_depacketizer.h"

namespace webrtc {

class GenericVideoRtpDepacketizer final : public RtpVideoDepacketizer {
 public:
  GenericVideoRtpDepacketizer() = default;
  ~GenericVideoRtpDepacketizer() override = default;

  absl::optional<FrameBoundaries> GetFrameBoundaries(
      rtc::ArrayView<const uint8_t> rtp_payload) override;
  absl::optional<Frame> AssembleFrame(
      size_t num_packets,
      rtc::FunctionView<const RtpPacketReceived&(size_t index)> packets)
      override;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_GENERIC_VIDEO_RTP_DEPACKETIZER_H_
