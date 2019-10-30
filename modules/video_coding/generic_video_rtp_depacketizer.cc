/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/generic_video_rtp_depacketizer.h"

namespace webrtc {
namespace {

constexpr uint8_t kKeyFrameBit = 0x01;
constexpr uint8_t kFirstPacketBit = 0x02;
constexpr uint8_t kExtendedHeaderBit = 0x04;
constexpr size_t kBasesHeaderLength = 1;
constexpr size_t kExtendedHeaderLength = 2;

size_t HeaderSize(rtc::ArrayView<const uint8_t> rtp_payload) {
  if (!rtp_payload.empty() && (rtp_payload[0] & kExtendedHeaderBit) != 0) {
    return kBasesHeaderLength + kExtendedHeaderLength;
  } else {
    return kBasesHeaderLength;
  }
}

bool ValidPayload(rtc::ArrayView<const uint8_t> rtp_payload) {
  return HeaderSize(rtp_payload) <= rtp_payload.size();
}

}  // namespace

absl::optional<RtpVideoDepacketizer::FrameBoundaries>
GenericVideoRtpDepacketizer::GetFrameBoundaries(
    rtc::ArrayView<const uint8_t> rtp_payload) {
  if (!ValidPayload(rtp_payload)) {
    return absl::nullopt;
  }
  RtpVideoDepacketizer::FrameBoundaries boundaries;
  boundaries.begins_frame = (rtp_payload[0] & kFirstPacketBit) != 0;
  boundaries.ends_frame = absl::nullopt;  // Marker bit is the only way.
  boundaries.begins_keyframe = (rtp_payload[0] & kKeyFrameBit) != 0;
  return boundaries;
}

absl::optional<RtpVideoDepacketizer::Frame>
GenericVideoRtpDepacketizer::AssembleFrame(
    size_t num_packets,
    rtc::FunctionView<const RtpPacketReceived&(size_t index)> packets) {
  size_t frame_size = 0;
  // First loop: validate the packets and calculate frame size.
  for (size_t i = 0; i < num_packets; ++i) {
    rtc::ArrayView<const uint8_t> rtp_payload = packets(i).payload();
    if (!ValidPayload(rtp_payload)) {
      return absl::nullopt;
    }
    frame_size += rtp_payload.size() - HeaderSize(rtp_payload);
  }

  RtpVideoDepacketizer::Frame frame;
  // Second loop: concatinate bitstream.
  frame.bitstream = EncodedImageBuffer::Create(frame_size);
  int offset = 0;
  for (size_t i = 0; i < num_packets; ++i) {
    rtc::ArrayView<const uint8_t> rtp_payload = packets(i).payload();
    rtc::ArrayView<const uint8_t> codec_payload =
        rtp_payload.subview(HeaderSize(rtp_payload));
    RTC_CHECK_LE(offset + codec_payload.size(), frame.bitstream->size());
    memcpy(frame.bitstream->data() + offset, codec_payload.data(),
           codec_payload.size());
    offset += codec_payload.size();
  }
  RTC_DCHECK_EQ(offset, frame_size);

  // Set video_header.
  RTC_DCHECK_GT(num_packets, 0);
  rtc::ArrayView<const uint8_t> rtp_payload = packets(0).payload();
  RTC_DCHECK_GE(rtp_payload.size(), HeaderSize(rtp_payload));
  uint8_t base_header = rtp_payload[0];
  frame.video_header.frame_type = (base_header & kKeyFrameBit) != 0
                                      ? VideoFrameType::kVideoFrameKey
                                      : VideoFrameType::kVideoFrameDelta;
  frame.video_header.codec = kVideoCodecGeneric;

  if ((base_header & kExtendedHeaderBit) != 0) {
    frame.video_header.generic.emplace();
    frame.video_header.generic->frame_id =
        ((rtp_payload[1] & 0x7F) << 8) | rtp_payload[2];
  }

  return frame;
}

}  // namespace webrtc
