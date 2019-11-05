/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_RTP_RTCP_SOURCE_RTP_VIDEO_DEPACKETIZER_H_
#define MODULES_RTP_RTCP_SOURCE_RTP_VIDEO_DEPACKETIZER_H_

#include <memory>

#include "absl/types/optional.h"
#include "api/array_view.h"
#include "api/video/encoded_image.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/rtp_rtcp/source/rtp_video_header.h"

namespace webrtc {

class RtpVideoDepacketizer {
 public:
  struct FrameBoundaries {
    // Some profiles do not have clear markers for beginning or end of frame.
    // In such cases VideoPacketBuffer should guess frame boundaries itself
    // e.g. based on marker bit and different rtp fields in neigbour packets.
    absl::optional<bool> begins_frame;
    absl::optional<bool> ends_frame;
    bool begins_keyframe = false;
  };
  struct Frame {
    RTPVideoHeader video_header;
    rtc::scoped_refptr<EncodedImageBuffer> bitstream;
  };

  virtual ~RtpVideoDepacketizer() = default;

  // Returns nullopt is packet doesn't look valid.
  virtual absl::optional<FrameBoundaries> GetFrameBoundaries(
      rtc::ArrayView<const uint8_t> rtp_payload) = 0;
  // Returns nullopt if packets doesn't form a valid frame, e.g. if they turn
  // out to be from different frames.
  virtual absl::optional<Frame> AssembleFrame(
      rtc::ArrayView<const RtpPacket*> rtp_packets) = 0;
};

}  // namespace webrtc

#endif  // MODULES_RTP_RTCP_SOURCE_RTP_VIDEO_DEPACKETIZER_H_
