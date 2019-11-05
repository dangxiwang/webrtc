/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "modules/rtp_rtcp/source/rtp_video_depacketizer_raw.h"

#include <memory>

#include "absl/types/optional.h"
#include "api/array_view.h"
#include "common_video/generic_frame_descriptor/generic_frame_info.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

namespace webrtc {

absl::optional<RtpVideoDepacketizer::Frame>
RtpVideoDepacketizerRaw::AssembleFrame(
    rtc::ArrayView<const RtpPacket*> rtp_packets) {
  size_t frame_size = 0;
  for (const RtpPacket* rtp_packet : rtp_packets) {
    frame_size += rtp_packet->payload_size();
  }

  if (frame_size == 0) {
    // Padding?
    return absl::nullopt;
  }

  absl::optional<Frame> frame(absl::in_place);
  frame->bitstream = EncodedImageBuffer::Create(frame_size);
  uint8_t* write_at = frame->bitstream->data();
  uint8_t* const ends_at = frame->bitstream->data() + frame->bitstream->size();
  for (const RtpPacket* rtp_packet : rtp_packets) {
    rtc::ArrayView<const uint8_t> rtp_payload = rtp_packet->payload();
    RTC_CHECK_LE(write_at + rtp_payload.size(), ends_at);
    memcpy(write_at, rtp_payload.data(), rtp_payload.size());
    write_at += rtp_payload.size();
  }
  RTC_DCHECK_EQ(write_at, ends_at);
  // Leave frame->video_header with default values: current packetizer has zero
  // knowledge how to fill any of its properties.
  return frame;
}

}  // namespace webrtc
