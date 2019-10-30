/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "modules/video_coding/codecs/h264/h264_rtp_depacketizer.h"

#include <memory>

#include "absl/types/optional.h"
#include "api/array_view.h"
#include "media/base/media_constants.h"
#include "modules/rtp_rtcp/source/rtp_header_extensions.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/video_coding/h264_sprop_parameter_sets.h"
#include "modules/video_coding/rtp_video_depacketizer.h"
#include "system_wrappers/include/field_trial.h"

namespace webrtc {

H264RtpDepacketizer::H264RtpDepacketizer(
    const std::map<std::string, std::string>& codec_params)
    : sps_pps_idr_is_keyframe_(
          field_trial::IsEnabled("WebRTC-SpsPpsIdrIsH264Keyframe")) {
  H264SpropParameterSets sprop_decoder;
  auto sprop_base64_it =
      codec_params.find(cricket::kH264FmtpSpropParameterSets);
  if (sprop_base64_it != codec_params.end() &&
      sprop_decoder.DecodeSprop(sprop_base64_it->second.c_str())) {
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
    size_t num_packets,
    rtc::FunctionView<const RtpPacketReceived&(size_t index)> get_packet) {
  Frame result;
  bool has_idr = true, has_sps = false;
  if (has_idr) {
    // inject related sps and pps.
    result.video_header.frame_type = VideoFrameType::kVideoFrameKey;
  } else {
    result.video_header.frame_type = VideoFrameType::kVideoFrameDelta;
  }
  if (has_sps) {
    // may be skip it, asume it was inserted before the appropriate idr.
  }
  RtpVideoDepacketizer::Frame frame;
  frame.video_header.codec = VideoCodecType::kVideoCodecH264;
  get_packet(0).GetExtension<FrameMarkingExtension>(
      &frame.video_header.frame_marking);
  // Unwrap strap/single-nalu/fua into regular nalus, if needed.
  return frame;
}

}  // namespace webrtc
