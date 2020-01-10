/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "video/video_source_sink_controller.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "rtc_base/numerics/optional_conversions.h"

namespace webrtc {

VideoSourceSinkController::VideoSourceSinkController(
    rtc::VideoSinkInterface<VideoFrame>* sink,
    rtc::VideoSourceInterface<VideoFrame>* source)
    : sink_(sink),
      source_(source),
      degradation_preference_(DegradationPreference::DISABLED) {
  RTC_DCHECK(sink_);
}

void VideoSourceSinkController::SetSource(
    rtc::VideoSourceInterface<VideoFrame>* source,
    DegradationPreference degradation_preference) {
  rtc::VideoSourceInterface<VideoFrame>* old_source;
  rtc::VideoSinkWants wants;
  {
    rtc::CritScope lock(&crit_);
    old_source = source_;
    source_ = source;
    degradation_preference_ = degradation_preference;
    wants = CurrentSettingsToSinkWantsInternal();
  }
  if (old_source != source && old_source)
    old_source->RemoveSink(sink_);
  if (!source)
    return;
  source->AddOrUpdateSink(sink_, wants);
}

void VideoSourceSinkController::PushSourceSinkSettings() {
  rtc::VideoSourceInterface<VideoFrame>* source;
  rtc::VideoSinkWants wants;
  {
    rtc::CritScope lock(&crit_);
    source = source_;
    wants = CurrentSettingsToSinkWantsInternal();
  }
  if (!source)
    return;
  source->AddOrUpdateSink(sink_, wants);
}

VideoSourceRestrictions VideoSourceSinkController::restrictions() const {
  rtc::CritScope lock(&crit_);
  return restrictions_;
}

absl::optional<size_t> VideoSourceSinkController::pixels_per_frame_upper_limit()
    const {
  rtc::CritScope lock(&crit_);
  return pixels_per_frame_upper_limit_;
}

absl::optional<double> VideoSourceSinkController::frame_rate_upper_limit()
    const {
  rtc::CritScope lock(&crit_);
  return frame_rate_upper_limit_;
}

bool VideoSourceSinkController::rotation_applied() const {
  rtc::CritScope lock(&crit_);
  return rotation_applied_;
}

int VideoSourceSinkController::resolution_alignment() const {
  rtc::CritScope lock(&crit_);
  return resolution_alignment_;
}

void VideoSourceSinkController::SetRestrictions(
    VideoSourceRestrictions restrictions) {
  rtc::CritScope lock(&crit_);
  restrictions_ = std::move(restrictions);
}

void VideoSourceSinkController::SetPixelsPerFrameUpperLimit(
    absl::optional<size_t> pixels_per_frame_upper_limit) {
  rtc::CritScope lock(&crit_);
  pixels_per_frame_upper_limit_ = std::move(pixels_per_frame_upper_limit);
}

void VideoSourceSinkController::SetFrameRateUpperLimit(
    absl::optional<double> frame_rate_upper_limit) {
  rtc::CritScope lock(&crit_);
  frame_rate_upper_limit_ = std::move(frame_rate_upper_limit);
}

void VideoSourceSinkController::SetRotationApplied(bool rotation_applied) {
  rtc::CritScope lock(&crit_);
  rotation_applied_ = rotation_applied;
}

void VideoSourceSinkController::SetResolutionAlignment(
    int resolution_alignment) {
  rtc::CritScope lock(&crit_);
  resolution_alignment_ = resolution_alignment;
}

rtc::VideoSinkWants VideoSourceSinkController::CurrentSettingsToSinkWants()
    const {
  rtc::CritScope lock(&crit_);
  return CurrentSettingsToSinkWantsInternal();
}

rtc::VideoSinkWants
VideoSourceSinkController::CurrentSettingsToSinkWantsInternal() const {
  rtc::VideoSinkWants wants;
  wants.rotation_applied = rotation_applied_;
  // |wants.black_frames| is not used, it always has its default value false.
  wants.max_pixel_count =
      OptionalSizeTToInt(restrictions_.max_pixels_per_frame());
  wants.target_pixel_count =
      OptionalSizeTToOptionalInt(restrictions_.target_pixels_per_frame());
  wants.max_framerate_fps = OptionalDoubleToInt(restrictions_.max_frame_rate());
  wants.resolution_alignment = resolution_alignment_;
  {
    // Clear any constraints from the current sink wants that don't apply to
    // the used degradation_preference.
    switch (degradation_preference_) {
      case DegradationPreference::BALANCED:
        break;
      case DegradationPreference::MAINTAIN_FRAMERATE:
        wants.max_framerate_fps = std::numeric_limits<int>::max();
        break;
      case DegradationPreference::MAINTAIN_RESOLUTION:
        wants.max_pixel_count = std::numeric_limits<int>::max();
        wants.target_pixel_count.reset();
        break;
      case DegradationPreference::DISABLED:
        wants.max_pixel_count = std::numeric_limits<int>::max();
        wants.target_pixel_count.reset();
        wants.max_framerate_fps = std::numeric_limits<int>::max();
    }
  }
  wants.max_pixel_count = std::min(
      wants.max_pixel_count, OptionalSizeTToInt(pixels_per_frame_upper_limit_));
  wants.max_framerate_fps = std::min(
      wants.max_framerate_fps, OptionalDoubleToInt(frame_rate_upper_limit_));
  return wants;
}

}  // namespace webrtc
