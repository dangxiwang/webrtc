/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "modules/rtp_rtcp/source/rtp_video_depacketizer_vp8.h"

#include <memory>

#include "absl/types/optional.h"
#include "api/array_view.h"
#include "modules/rtp_rtcp/source/rtp_video_depacketizer.h"

namespace webrtc {
namespace {
//
// VP8 format:
//
// Payload descriptor
//       0 1 2 3 4 5 6 7
//      +-+-+-+-+-+-+-+-+
//      |X|R|N|S|PartID | (REQUIRED)
//      +-+-+-+-+-+-+-+-+
// X:   |I|L|T|K|  RSV  | (OPTIONAL)
//      +-+-+-+-+-+-+-+-+
// I:   |   PictureID   | (OPTIONAL)
//      +-+-+-+-+-+-+-+-+
// L:   |   TL0PICIDX   | (OPTIONAL)
//      +-+-+-+-+-+-+-+-+
// T/K: |TID:Y| KEYIDX  | (OPTIONAL)
//      +-+-+-+-+-+-+-+-+
//
// Payload header (considered part of the actual payload, sent to decoder)
//       0 1 2 3 4 5 6 7
//      +-+-+-+-+-+-+-+-+
//      |Size0|H| VER |P|
//      +-+-+-+-+-+-+-+-+
//      |      ...      |
//      +               +

struct Vp8Header {
  int temporal_id;
  int picture_id;
  int tl0_picture_id;
  size_t offset;
};

absl::optional<Vp8Header> Parse(rtc::ArrayView<const uint8_t> rtp_payload) {
  return absl::nullopt;
}

}  // namespace

Vp8RtpDepacketizer::~Vp8RtpDepacketizer() = default;

absl::optional<RtpVideoDepacketizer::FrameBoundaries>
Vp8RtpDepacketizer::GetFrameBoundaries(
    rtc::ArrayView<const uint8_t> rtp_payload) {
  if (rtp_payload.empty()) {
    return absl::nullopt;
  }
  absl::optional<FrameBoundaries> result(absl::in_place);
  uint8_t required_header = rtp_payload[0];
  result->begins_frame = (required_header & 0b0010'0000) != 0;
  result->ends_frame = absl::nullopt;  // Not in the rtp payload.
  if (result->begins_frame) {
    absl::optional<Vp8Header> vp8_header = Parse(rtp_payload);
    if (vp8_header == absl::nullopt) {
      return absl::nullopt;
    }
    size_t codec_payload_offset = vp8_header->offset;
    if (rtp_payload.size() <= codec_payload_offset) {
      return absl::nullopt;
    }
    result->begins_keyframe =
        !(rtp_payload[codec_payload_offset] & 0b0000'0001);
  }
  return result;
}

absl::optional<RtpVideoDepacketizer::Frame> Vp8RtpDepacketizer::AssembleFrame(
    rtc::ArrayView<const RtpPacket*> rtp_packets) {
  RTC_DCHECK(!rtp_packets.empty());
  Frame frame;
  absl::optional<Vp8Header> first_vp8_header = Parse(rtp_packets[0]->payload());
  if (first_vp8_header == absl::nullopt) {
    return absl::nullopt;
  }
  auto& vp8_rtp_header =
      frame.video_header.video_type_header.emplace<RTPVideoHeaderVP8>();
  vp8_rtp_header.temporalIdx = first_vp8_header->temporal_id;
  // ...

  size_t frame_size = 0;
  for (const RtpPacket* rtp_packet : rtp_packets) {
    absl::optional<Vp8Header> vp8_header = Parse(rtp_packet->payload());
    if (vp8_header == absl::nullopt) {
      return absl::nullopt;
    }
    if (vp8_header->offset < rtp_packet->payload_size()) {
      return absl::nullopt;
    }
    // Sanity check packet belongs to the same frame.
    if (vp8_header->picture_id != first_vp8_header->picture_id ||
        vp8_header->temporal_id != first_vp8_header->temporal_id) {
      return absl::nullopt;
    }
    frame_size += (rtp_packet->payload_size() - vp8_header->offset);
  }

  if (frame_size == 0) {
    return absl::nullopt;
  }

  frame.bitstream = EncodedImageBuffer::Create(frame_size);

  uint8_t* write_at = frame.bitstream->data();
  for (const RtpPacket* rtp_packet : rtp_packets) {
    absl::optional<Vp8Header> vp8_header = Parse(rtp_packet->payload());
    rtc::ArrayView<const uint8_t> codec_payload =
        rtp_packet->payload().subview(vp8_header->offset);
    memcpy(write_at, codec_payload.data(), codec_payload.size());
    write_at += codec_payload.size();
  }
  RTC_DCHECK_EQ(write_at - frame.bitstream->data(), frame_size);

  if (frame.bitstream->data()[0] & 1) {
    if (frame_size < 10) {
      return absl::nullopt;
    }
    frame.video_header.frame_type = VideoFrameType::kVideoFrameKey;
    frame.video_header.width = frame.bitstream->data()[6];
    frame.video_header.height = frame.bitstream->data()[8];
  } else {
    frame.video_header.frame_type = VideoFrameType::kVideoFrameDelta;
  }
  return frame;
}

}  // namespace webrtc
