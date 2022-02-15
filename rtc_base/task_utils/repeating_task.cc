/*
 *  Copyright 2019 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/task_utils/repeating_task.h"

#include "absl/memory/memory.h"
#include "rtc_base/logging.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/task_utils/to_queued_task.h"
#include "rtc_base/time_utils.h"

namespace webrtc {
namespace webrtc_repeating_task_impl {

RepeatingTaskBase::RepeatingTaskBase(
    TaskQueueBase* task_queue,
    TaskQueueBase::DelayPrecision precision,
    TimeDelta first_delay,
    Clock* clock,
    rtc::scoped_refptr<PendingTaskSafetyFlag> alive_flag)
    : task_queue_(task_queue),
      precision_(precision),
      clock_(clock),
      next_run_time_(clock_->CurrentTime() + first_delay),
      alive_flag_(std::move(alive_flag)) {}

RepeatingTaskBase::~RepeatingTaskBase() = default;

bool RepeatingTaskBase::Run() {
  RTC_DCHECK_RUN_ON(task_queue_);
  // Return true to tell the TaskQueue to destruct this object.
  if (!alive_flag_->alive())
    return true;

  TimeDelta delay = RunClosure();
  RTC_DCHECK_GE(delay, TimeDelta::Zero());

  // A delay of +infinity means that the task should not be run again.
  // Alternatively, the closure might have stopped this task. In either which
  // case we return true to destruct this object.
  if (delay.IsPlusInfinity() || !alive_flag_->alive())
    return true;

  TimeDelta lost_time = clock_->CurrentTime() - next_run_time_;
  next_run_time_ += delay;
  delay -= lost_time;
  delay = std::max(delay, TimeDelta::Zero());

  task_queue_->PostDelayedTaskWithPrecision(precision_, absl::WrapUnique(this),
                                            delay);

  // Return false to tell the TaskQueue to not destruct this object since we
  // have taken ownership with absl::WrapUnique.
  return false;
}

}  // namespace webrtc_repeating_task_impl

void RepeatingTaskHandle::Stop() {
  if (repeating_task_) {
    repeating_task_->SetNotAlive();
    repeating_task_ = nullptr;
  }
}

bool RepeatingTaskHandle::Running() const {
  return repeating_task_ != nullptr;
}

namespace webrtc_repeating_task_impl {
// These methods are empty, but can be externally equipped with actions using
// dtrace.
void RepeatingTaskHandleDTraceProbeStart() {}
void RepeatingTaskHandleDTraceProbeDelayedStart() {}
void RepeatingTaskImplDTraceProbeRun() {}
}  // namespace webrtc_repeating_task_impl
}  // namespace webrtc
