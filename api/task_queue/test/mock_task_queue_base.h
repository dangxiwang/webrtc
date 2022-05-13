/*
 *  Copyright 2022 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_TASK_QUEUE_TEST_MOCK_TASK_QUEUE_BASE_H_
#define API_TASK_QUEUE_TEST_MOCK_TASK_QUEUE_BASE_H_

#include <memory>

#include "api/task_queue/task_queue_base.h"
#include "test/gmock.h"

namespace webrtc {

class MockTaskQueueBase : public TaskQueueBase {
 public:
  MOCK_METHOD(void, Delete, (), (override));
  MOCK_METHOD(void, PostTask, (std::unique_ptr<QueuedTask>), (override));
  MOCK_METHOD(void,
              PostDelayedTask,
              (std::unique_ptr<QueuedTask>, uint32_t),
              (override));
  MOCK_METHOD(void,
              PostDelayedHighPrecisionTask,
              (std::unique_ptr<QueuedTask>, uint32_t),
              (override));
};

static_assert(!std::is_abstract_v<MockTaskQueueBase>);

}  // namespace webrtc

#endif  // API_TASK_QUEUE_TEST_MOCK_TASK_QUEUE_BASE_H_
