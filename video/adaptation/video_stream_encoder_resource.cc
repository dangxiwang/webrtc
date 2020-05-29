/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "video/adaptation/video_stream_encoder_resource.h"

#include <algorithm>
#include <utility>

namespace webrtc {

VideoStreamEncoderResource::VideoStreamEncoderResource(std::string name)
    : lock_(),
      name_(std::move(name)),
      encoder_queue_(nullptr),
      resource_adaptation_queue_(nullptr),
      usage_state_(absl::nullopt),
      listeners_() {}

VideoStreamEncoderResource::~VideoStreamEncoderResource() {}

void VideoStreamEncoderResource::RegisterEncoderTaskQueue(
    TaskQueueBase* encoder_queue) {
  RTC_DCHECK(!encoder_queue_);
  RTC_DCHECK(encoder_queue);
  encoder_queue_ = encoder_queue;
}

void VideoStreamEncoderResource::RegisterAdaptationTaskQueue(
    TaskQueueBase* resource_adaptation_queue) {
  rtc::CritScope crit(&lock_);
  RTC_DCHECK(!resource_adaptation_queue_);
  RTC_DCHECK(resource_adaptation_queue);
  resource_adaptation_queue_ = resource_adaptation_queue;
}

void VideoStreamEncoderResource::UnregisterAdaptationTaskQueue() {
  rtc::CritScope crit(&lock_);
  RTC_DCHECK(resource_adaptation_queue_);
  RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
  resource_adaptation_queue_ = nullptr;
}

void VideoStreamEncoderResource::AddResourceListener(
    ResourceListener* listener) {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue());
  listeners_.push_back(listener);
}

void VideoStreamEncoderResource::RemoveResourceListener(
    ResourceListener* listener) {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue());
  auto it = std::find(listeners_.begin(), listeners_.end(), listener);
  RTC_DCHECK(it != listeners_.end());
  listeners_.erase(it);
}

std::string VideoStreamEncoderResource::Name() const {
  return name_;
}

absl::optional<ResourceUsageState> VideoStreamEncoderResource::UsageState()
    const {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue());
  return usage_state_;
}

void VideoStreamEncoderResource::ClearUsageState() {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue());
  usage_state_ = absl::nullopt;
}

bool VideoStreamEncoderResource::IsAdaptationUpAllowed(
    const VideoStreamInputState& input_state,
    const VideoSourceRestrictions& restrictions_before,
    const VideoSourceRestrictions& restrictions_after,
    rtc::scoped_refptr<Resource> reason_resource) const {
  return true;
}

void VideoStreamEncoderResource::OnAdaptationApplied(
    const VideoStreamInputState& input_state,
    const VideoSourceRestrictions& restrictions_before,
    const VideoSourceRestrictions& restrictions_after,
    rtc::scoped_refptr<Resource> reason_resource) {}

void VideoStreamEncoderResource::OnResourceUsageStateMeasured(
    ResourceUsageState usage_state) {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue());
  usage_state_ = usage_state;
  for (auto& listener : listeners_) {
    listener->OnResourceUsageStateMeasured(this);
  }
}

TaskQueueBase* VideoStreamEncoderResource::encoder_queue() const {
  return encoder_queue_;
}

TaskQueueBase* VideoStreamEncoderResource::resource_adaptation_queue() const {
  rtc::CritScope crit(&lock_);
  RTC_DCHECK(resource_adaptation_queue_);
  RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
  return resource_adaptation_queue_;
}

}  // namespace webrtc
