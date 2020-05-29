/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef VIDEO_ADAPTATION_VIDEO_STREAM_ENCODER_RESOURCE_H_
#define VIDEO_ADAPTATION_VIDEO_STREAM_ENCODER_RESOURCE_H_

#include <string>
#include <vector>

#include "absl/types/optional.h"
#include "api/task_queue/task_queue_base.h"
#include "call/adaptation/resource.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/synchronization/sequence_checker.h"

namespace webrtc {

class VideoStreamEncoderResource : public Resource {
 public:
  ~VideoStreamEncoderResource() override;

  // Registering task queues must be performed as part of initialization.
  void RegisterEncoderTaskQueue(TaskQueueBase* encoder_queue);

  // Resource implementation.
  void RegisterAdaptationTaskQueue(
      TaskQueueBase* resource_adaptation_queue) override;
  void UnregisterAdaptationTaskQueue() override;
  void AddResourceListener(ResourceListener* listener) override;
  void RemoveResourceListener(ResourceListener* listener) override;
  std::string Name() const override;
  absl::optional<ResourceUsageState> UsageState() const override;
  void ClearUsageState() override;
  // Default implementations, may be overriden again by child classes.
  bool IsAdaptationUpAllowed(
      const VideoStreamInputState& input_state,
      const VideoSourceRestrictions& restrictions_before,
      const VideoSourceRestrictions& restrictions_after,
      rtc::scoped_refptr<Resource> reason_resource) const override;
  void OnAdaptationApplied(
      const VideoStreamInputState& input_state,
      const VideoSourceRestrictions& restrictions_before,
      const VideoSourceRestrictions& restrictions_after,
      rtc::scoped_refptr<Resource> reason_resource) override;

 protected:
  explicit VideoStreamEncoderResource(std::string name);

  void OnResourceUsageStateMeasured(ResourceUsageState usage_state);

  // The caller is responsible for ensuring the task queue is still valid.
  TaskQueueBase* encoder_queue() const;
  // Validity of returned pointer is ensured by only allowing this method to be
  // called on the adaptation task queue. Designed for use with RTC_GUARDED_BY.
  // For posting from a different queue, use
  // MaybePostTaskToResourceAdaptationQueue() instead, which only posts if the
  // task queue is currently registered.
  TaskQueueBase* resource_adaptation_queue() const;
  template <typename Closure>
  void MaybePostTaskToResourceAdaptationQueue(Closure&& closure) {
    rtc::CritScope crit(&lock_);
    if (!resource_adaptation_queue_)
      return;
    resource_adaptation_queue_->PostTask(ToQueuedTask(closure));
  }

 private:
  rtc::CriticalSection lock_;
  const std::string name_;
  // Treated as const after initialization.
  TaskQueueBase* encoder_queue_;
  TaskQueueBase* resource_adaptation_queue_ RTC_GUARDED_BY(lock_);
  absl::optional<ResourceUsageState> usage_state_
      RTC_GUARDED_BY(resource_adaptation_queue());
  std::vector<ResourceListener*> listeners_
      RTC_GUARDED_BY(resource_adaptation_queue());
};

}  // namespace webrtc

#endif  // VIDEO_ADAPTATION_VIDEO_STREAM_ENCODER_RESOURCE_H_
