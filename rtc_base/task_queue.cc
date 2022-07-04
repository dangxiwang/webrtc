/*
 *  Copyright 2019 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "rtc_base/task_queue.h"

#include "api/task_queue/task_queue_base.h"

namespace rtc {

TaskQueue::TaskQueue(
    std::unique_ptr<webrtc::TaskQueueBase, webrtc::TaskQueueDeleter> task_queue)
    : impl_(task_queue.release()) {}

TaskQueue::~TaskQueue() {
  // There might running task that tries to rescheduler itself to the TaskQueue
  // and not yet aware TaskQueue destructor is called.
  // Calling back to TaskQueue::PostTask need impl_ pointer still be valid, so
  // do not invalidate impl_ pointer until Delete returns.
  impl_->Delete();
}

bool TaskQueue::IsCurrent() const {
  return impl_->IsCurrent();
}

void TaskQueue::PostTask(absl::AnyInvocable<void() &&> task) {
  impl_->PostTask(std::move(task));
}

void TaskQueue::PostDelayedTask(absl::AnyInvocable<void() &&> task,
                                uint32_t milliseconds) {
  impl_->PostDelayedTask(std::move(task), milliseconds);
}

void TaskQueue::PostDelayedHighPrecisionTask(absl::AnyInvocable<void() &&> task,
                                             uint32_t milliseconds) {
  impl_->PostDelayedHighPrecisionTask(std::move(task), milliseconds);
}

void TaskQueue::PostDelayedTaskWithPrecision(
    webrtc::TaskQueueBase::DelayPrecision precision,
    absl::AnyInvocable<void() &&> task,
    uint32_t milliseconds) {
  impl_->PostDelayedTaskWithPrecision(precision, std::move(task), milliseconds);
}

}  // namespace rtc
