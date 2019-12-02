/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_RTP_RTCP_SOURCE_RTP_VIDEO_DEPACKETIZER_GENERIC_H_
#define MODULES_RTP_RTCP_SOURCE_RTP_VIDEO_DEPACKETIZER_GENERIC_H_

#include <cstdint>
#include <memory>

#include "absl/types/optional.h"
#include "api/array_view.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/rtp_rtcp/source/rtp_video_depacketizer.h"

namespace webrtc {

class GenericVideoRtpDepacketizer final : public RtpVideoDepacketizer {
 public:
  GenericVideoRtpDepacketizer() = default;
  ~GenericVideoRtpDepacketizer() override = default;

  absl::optional<FrameBoundaries> GetFrameBoundaries(
      rtc::ArrayView<const uint8_t> rtp_payload) override;
  absl::optional<Frame> AssembleFrame(
      rtc::ArrayView<const RtpPacket*> rtp_packets) override;
};

}  // namespace webrtc

#endif  // MODULES_RTP_RTCP_SOURCE_RTP_VIDEO_DEPACKETIZER_GENERIC_H_
