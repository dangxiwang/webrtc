/*
 *  Copyright 2016 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// This file contains the implementation of TaskQueue for Mac and iOS.
// The implementation uses Grand Central Dispatch queues (GCD) to
// do the actual task queuing.

#include "rtc_base/task_queue_gcd.h"

#include <string.h>

#include <dispatch/dispatch.h>

#include "absl/memory/memory.h"
#include "absl/strings/string_view.h"
#include "api/task_queue/queued_task.h"
#include "api/task_queue/task_queue_base.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {
namespace {

int TaskQueuePriorityToGCD(TaskQueueFactory::Priority priority) {
  switch (priority) {
    case TaskQueueFactory::Priority::NORMAL:
      return DISPATCH_QUEUE_PRIORITY_DEFAULT;
    case TaskQueueFactory::Priority::HIGH:
      return DISPATCH_QUEUE_PRIORITY_HIGH;
    case TaskQueueFactory::Priority::LOW:
      return DISPATCH_QUEUE_PRIORITY_LOW;
  }
}

class TaskQueueGcd : public TaskQueueBase {
 public:
  TaskQueueGcd(absl::string_view queue_name, int gcd_priority);

  void Delete() override;
  void PostTask(std::unique_ptr<QueuedTask> task) override;
  void PostDelayedTask(std::unique_ptr<QueuedTask> task,
                       uint32_t milliseconds) override;

 private:
  struct TaskContext {
    TaskContext(TaskQueueGcd* queue, std::unique_ptr<QueuedTask> task)
        : queue(queue), task(std::move(task)) {}

    TaskQueueGcd* const queue;
    std::unique_ptr<QueuedTask> task;
  };

  ~TaskQueueGcd() override;
  static void RunTask(void* task_context);
  static void SetNotActive(void* task_queue);
  static void DeleteContext(void* task_queue);

  dispatch_queue_t queue_;
  bool is_active_;
};

TaskQueueGcd::TaskQueueGcd(absl::string_view queue_name, int gcd_priority);
    : queue_(dispatch_queue_create(std::string(queue_name).c_str(),
                                   DISPATCH_QUEUE_SERIAL)),
      is_active_(true) {
  RTC_CHECK(queue_);
  dispatch_set_context(queue_, this);
  // Assign a finalizer that will delete the context when the last reference
  // to the queue is released.  This may run after the TaskQueue object has
  // been deleted.
  dispatch_set_finalizer_f(queue_, &DeleteContext);

  dispatch_set_target_queue(queue_, dispatch_get_global_queue(gcd_priority, 0));
    }

    TaskQueueGcd::~TaskQueueGcd() = default;

    TaskQueueGcd::Delete() {
      RTC_DCHECK(!IsCurrent());
      // Implementation/behavioral note:
      // Dispatch queues are reference counted via calls to dispatch_retain and
      // dispatch_release. Pending blocks submitted to a queue also hold a
      // reference to the queue until they have finished. Once all references to
      // a queue have been released, the queue will be deallocated by the
      // system. This is why we check the context before running tasks.

      // Use dispatch_sync to set the context to null to guarantee that there's
      // not a race between checking the context and using it from a task.
      dispatch_sync_f(queue_, this, &SetNotActive);
      dispatch_release(queue_);
    }

    void TaskQueueGcd::PostTask(std::unique_ptr<QueuedTask> task) {
      auto* context = new TaskContext(this, std::move(task));
      dispatch_async_f(queue_, context, &RunTask);
    }

    void TaskQueueGcd::PostDelayedTask(std::unique_ptr<QueuedTask> task,
                                       uint32_t milliseconds) {
      auto* context = new TaskContext(this, std::move(task));
      dispatch_after_f(
          dispatch_time(DISPATCH_TIME_NOW, milliseconds * NSEC_PER_MSEC),
          queue_, context, &RunTask);
    }

// static
    void TaskQueueGcd::RunTask(void* task_context) {
      std::unique_ptr<TaskContext> tc(static_cast<TaskContext*>(task_context));
      if (tc->queue->is_active_) {
        CurrentTaskQueueSetter set_current(tc->queue);
        if (!tc->task->Run())
          tc->task.release();
      }
    }

    // static
    void TaskQueueGcd::SetNotActive(void* task_queue) {
      static_cast<TaskQueueGcd*>(task_queue)->is_active_ = false;
    }

    // static
    void TaskQueueGcd::DeleteContext(void* task_queue) {
      delete static_cast<TaskQueueGcd*>(task_queue);
    }

    }  // namespace

    std::unique_ptr<TaskQueueFactory> CreateTaskQueueGcdFactory() {
      return absl::make_unique<TaskQueueGcdFactory>();
    }

    }  // namespace webrtc
