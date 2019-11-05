/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "modules/rtp_rtcp/source/rtp_video_depacketizer_h264.h"

#include <memory>

#include "absl/types/optional.h"
#include "api/array_view.h"
#include "modules/rtp_rtcp/source/h264_sprop_parameter_sets.h"
#include "modules/rtp_rtcp/source/rtp_header_extensions.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/rtp_rtcp/source/rtp_video_depacketizer.h"
#include "system_wrappers/include/field_trial.h"

namespace webrtc {

H264RtpDepacketizer::H264RtpDepacketizer(absl::string_view sprop_base64)
    : sps_pps_idr_is_keyframe_(
          field_trial::IsEnabled("WebRTC-SpsPpsIdrIsH264Keyframe")) {
  H264SpropParameterSets sprop_decoder;
  if (sprop_base64 != nullptr && sprop_decoder.DecodeSprop(sprop_base64)) {
    InsertSpsPpsNalus(sprop_decoder.sps_nalu(), sprop_decoder.pps_nalu());
  }
}

absl::optional<RtpVideoDepacketizer::FrameBoundaries>
H264RtpDepacketizer::GetFrameBoundaries(
    rtc::ArrayView<const uint8_t> rtp_payload) {
  // Preparse.
  bool is_fua_not_first = true;
  bool is_fua_not_last = true;
  FrameBoundaries boundaries;
  if (is_fua_not_first) {
    boundaries.begins_frame = false;
  }
  if (is_fua_not_last) {
    boundaries.ends_frame = false;
  }

  bool has_idr = true, has_sps = true;
  if (has_idr) {
    // Check if there is known sps/pps for this idr.
    if (has_sps || seen_sps_) {
      boundaries.begins_keyframe = true;
      boundaries.begins_frame = true;
    } else {
      // Can't make a keyframe, but remember this idr. If there is later an sps
      // nalu, it can be declared to start a key frame.
      seen_idr_ = true;
    }
  }
  if (has_sps) {
    if (!has_idr && seen_idr_) {
      // There is a later idr! Can assembe a key frame now.
      boundaries.begins_keyframe = true;
      boundaries.begins_frame = true;
    } else {
      seen_sps_ = true;
    }
    // Save the sps.
  }

  return boundaries;
}

absl::optional<RtpVideoDepacketizer::Frame> H264RtpDepacketizer::AssembleFrame(
    rtc::ArrayView<const RtpPacket*> rtp_packets) {
  absl::optional<Frame> result(absl::in_place);

  bool has_idr = true, has_sps = false;
  if (has_idr) {
    // inject related sps and pps.
    result->video_header.frame_type = VideoFrameType::kVideoFrameKey;
  } else {
    result->video_header.frame_type = VideoFrameType::kVideoFrameDelta;
  }
  if (has_sps) {
    // may be skip it, asume it was inserted before the appropriate idr.
  }
  RtpVideoDepacketizer::Frame frame;
  frame.video_header.codec = VideoCodecType::kVideoCodecH264;
  // Payload-specific rtp header extension...
  rtp_packets[0]->GetExtension<FrameMarkingExtension>(
      &frame.video_header.frame_marking);
  // Unwrap strap/single-nalu/fua into regular nalus, if needed.
  return frame;
}

}  // namespace webrtc
