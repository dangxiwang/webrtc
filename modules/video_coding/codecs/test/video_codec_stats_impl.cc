/*
 *  Copyright (c) 2023 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/codecs/test/video_codec_stats_impl.h"

#include "rtc_base/checks.h"

namespace webrtc {
namespace test {
namespace {
using Frame = VideoCodecStats::Frame;
using Stream = VideoCodecStats::Stream;
}  // namespace

std::vector<Frame> VideoCodecStatsImpl::Slice(
    absl::optional<Filter> filter) const {
  std::vector<Frame> frames;
  for (const auto& fs : frames_) {
    if (filter.has_value()) {
      if (filter->first_frame.has_value() &&
          fs.second.frame_num < *filter->first_frame) {
        continue;
      }
      if (filter->last_frame.has_value() &&
          fs.second.frame_num > *filter->last_frame) {
        continue;
      }
      if (filter->spatial_idx.has_value() &&
          fs.second.spatial_idx != *filter->spatial_idx) {
        continue;
      }
      if (filter->temporal_idx.has_value() &&
          fs.second.temporal_idx > *filter->temporal_idx) {
        continue;
      }
    }
    frames.push_back(fs.second);
  }
  return frames;
}

Stream VideoCodecStatsImpl::Aggregate(
    const std::vector<Frame>& frames,
    absl::optional<DataRate> bitrate,
    absl::optional<Frequency> framerate) const {
  Stream stream;
  return stream;
}

void VideoCodecStatsImpl::LogMetrics(const std::vector<Frame>& frames,
                                     MetricsLogger* logger,
                                     std::string test_case_name) const {}

Frame* VideoCodecStatsImpl::AddFrame(int frame_num,
                                     uint32_t timestamp_rtp,
                                     int spatial_idx) {
  Frame frame;
  frame.frame_num = frame_num;
  frame.timestamp_rtp = timestamp_rtp;
  frame.spatial_idx = spatial_idx;

  FrameId frame_id;
  frame_id.frame_num = frame_num;
  frame_id.spatial_idx = spatial_idx;

  RTC_CHECK(frames_.find(frame_id) == frames_.end())
      << "Frame with frame_num=" << frame_num
      << " and spatial_idx=" << spatial_idx << " already exists";

  frames_[frame_id] = frame;

  if (frame_num_.find(timestamp_rtp) == frame_num_.end()) {
    frame_num_[timestamp_rtp] = frame_num;
  }

  return &frames_[frame_id];
}

Frame* VideoCodecStatsImpl::GetFrame(uint32_t timestamp_rtp, int spatial_idx) {
  if (frame_num_.find(timestamp_rtp) == frame_num_.end()) {
    return nullptr;
  }

  FrameId frame_id;
  frame_id.frame_num = frame_num_[timestamp_rtp];
  frame_id.spatial_idx = spatial_idx;

  if (frames_.find(frame_id) == frames_.end()) {
    return nullptr;
  }

  return &frames_[frame_id];
}

}  // namespace test
}  // namespace webrtc
