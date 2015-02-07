/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "testing/gmock/include/gmock/gmock.h"
#include "testing/gtest/include/gtest/gtest.h"
#include "webrtc/modules/interface/module.h"
#include "webrtc/modules/utility/source/process_thread_impl.h"
#include "webrtc/system_wrappers/interface/tick_util.h"

namespace webrtc {

using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::SetArgPointee;

class MockModule : public Module {
 public:
  MOCK_METHOD0(TimeUntilNextProcess, int64_t());
  MOCK_METHOD0(Process, int32_t());
};

ACTION_P(SetEvent, event) {
  event->Set();
}

ACTION_P(Increment, counter) {
  ++(*counter);
}

ACTION_P(SetTimestamp, ptr) {
  *ptr = TickTime::MillisecondTimestamp();
}

TEST(ProcessThreadImpl, StartStop) {
  ProcessThreadImpl thread;
  EXPECT_EQ(0, thread.Start());
  EXPECT_EQ(0, thread.Stop());
}

TEST(ProcessThreadImpl, MultipleStartStop) {
  ProcessThreadImpl thread;
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(0, thread.Start());
    EXPECT_EQ(0, thread.Stop());
  }
}

// Verifies that we get at least call back to Process() on the worker thread.
TEST(ProcessThreadImpl, ProcessCall) {
  ProcessThreadImpl thread;
  ASSERT_EQ(0, thread.Start());

  rtc::scoped_ptr<EventWrapper> event(EventWrapper::Create());

  MockModule module;
  EXPECT_CALL(module, TimeUntilNextProcess()).WillRepeatedly(Return(0));
  EXPECT_CALL(module, Process())
      .WillOnce(DoAll(SetEvent(event.get()), Return(0)))
      .WillRepeatedly(Return(0));

  ASSERT_EQ(0, thread.RegisterModule(&module));
  EXPECT_EQ(kEventSignaled, event->Wait(100));
  EXPECT_EQ(0, thread.Stop());
}

// Same as ProcessCall except the module is registered before the
// call to Start().
TEST(ProcessThreadImpl, ProcessCall2) {
  ProcessThreadImpl thread;
  rtc::scoped_ptr<EventWrapper> event(EventWrapper::Create());

  MockModule module;
  EXPECT_CALL(module, TimeUntilNextProcess()).WillRepeatedly(Return(0));
  EXPECT_CALL(module, Process())
      .WillOnce(DoAll(SetEvent(event.get()), Return(0)))
      .WillRepeatedly(Return(0));

  ASSERT_EQ(0, thread.RegisterModule(&module));
  ASSERT_EQ(thread.Start(), 0);
  EXPECT_EQ(kEventSignaled, event->Wait(100));
  EXPECT_EQ(thread.Stop(), 0);
}

// Tests setting up a module for callbacks and then unregister that module.
// After unregistration, we should not receive any further callbacks.
TEST(ProcessThreadImpl, Deregister) {
  ProcessThreadImpl thread;
  rtc::scoped_ptr<EventWrapper> event(EventWrapper::Create());

  int process_count = 0;
  MockModule module;
  EXPECT_CALL(module, TimeUntilNextProcess()).WillRepeatedly(Return(0));
  EXPECT_CALL(module, Process())
      .WillOnce(DoAll(SetEvent(event.get()),
                      Increment(&process_count),
                      Return(0)))
      .WillRepeatedly(DoAll(Increment(&process_count), Return(0)));

  ASSERT_EQ(0, thread.RegisterModule(&module));
  ASSERT_EQ(0, thread.Start());

  EXPECT_EQ(kEventSignaled, event->Wait(100));
  ASSERT_EQ(0, thread.DeRegisterModule(&module));
  EXPECT_GE(process_count, 1);
  int count_after_deregister = process_count;

  // We shouldn't get any more callbacks.
  EXPECT_EQ(kEventTimeout, event->Wait(20));
  EXPECT_EQ(count_after_deregister, process_count);
  EXPECT_EQ(0, thread.Stop());
}

// Helper function for testing receiving a callback after a certain amount of
// time.  There's some variance of timing built into it to reduce chance of
// flakiness on bots.
void ProcessCallAfterAFewMs(int64_t milliseconds) {
  ProcessThreadImpl thread;
  ASSERT_EQ(0, thread.Start());

  rtc::scoped_ptr<EventWrapper> event(EventWrapper::Create());

  MockModule module;
  int64_t start_time = 0;
  int64_t called_time = 0;
  EXPECT_CALL(module, TimeUntilNextProcess())
      .WillOnce(DoAll(SetTimestamp(&start_time),
                      Return(milliseconds)))
      .WillRepeatedly(Return(milliseconds));
  EXPECT_CALL(module, Process())
      .WillOnce(DoAll(SetTimestamp(&called_time),
                      SetEvent(event.get()),
                      Return(0)))
      .WillRepeatedly(Return(0));

  EXPECT_EQ(0, thread.RegisterModule(&module));

  // Add a buffer of 50ms due to slowness of some trybots
  // (e.g. win_drmemory_light)
  EXPECT_EQ(kEventSignaled, event->Wait(milliseconds + 50));
  ASSERT_EQ(0, thread.Stop());

  ASSERT_GT(start_time, 0);
  ASSERT_GT(called_time, 0);
  // Use >= instead of > since due to rounding and timer accuracy (or lack
  // thereof), can make the test run in "0"ms time.
  EXPECT_GE(called_time, start_time);
  // Check for an acceptable range.
  uint32 diff = called_time - start_time;
  EXPECT_GE(diff, milliseconds - 15);
  EXPECT_LT(diff, milliseconds + 15);
}

// DISABLED for now since the virtual build bots are too slow :(
// TODO(tommi): Fix.
TEST(ProcessThreadImpl, DISABLED_ProcessCallAfter5ms) {
  ProcessCallAfterAFewMs(5);
}

// DISABLED for now since the virtual build bots are too slow :(
// TODO(tommi): Fix.
TEST(ProcessThreadImpl, DISABLED_ProcessCallAfter50ms) {
  ProcessCallAfterAFewMs(50);
}

// DISABLED for now since the virtual build bots are too slow :(
// TODO(tommi): Fix.
TEST(ProcessThreadImpl, DISABLED_ProcessCallAfter200ms) {
  ProcessCallAfterAFewMs(200);
}

// Runs callbacks with the goal of getting up to 50 callbacks within a second
// (on average 1 callback every 20ms).  On real hardware, we're usually pretty
// close to that, but the test bots that run on virtual machines, will
// typically be in the range 30-40 callbacks.
// DISABLED for now since this can take up to 2 seconds to run on the slowest
// build bots.
// TODO(tommi): Fix.
TEST(ProcessThreadImpl, DISABLED_Process50Times) {
  ProcessThreadImpl thread;
  ASSERT_EQ(0, thread.Start());

  rtc::scoped_ptr<EventWrapper> event(EventWrapper::Create());

  MockModule module;
  int callback_count = 0;
  // Ask for a callback after 20ms.
  EXPECT_CALL(module, TimeUntilNextProcess())
      .WillRepeatedly(Return(20));
  EXPECT_CALL(module, Process())
      .WillRepeatedly(DoAll(Increment(&callback_count),
                            Return(0)));

  EXPECT_EQ(0, thread.RegisterModule(&module));

  EXPECT_EQ(kEventTimeout, event->Wait(1000));
  ASSERT_EQ(0, thread.Stop());

  printf("Callback count: %i\n", callback_count);
  // Check that we got called back up to 50 times.
  // Some of the try bots run on slow virtual machines, so the lower bound
  // is much more relaxed to avoid flakiness.
  EXPECT_GE(callback_count, 25);
  EXPECT_LE(callback_count, 50);
}

// Tests that we can wake up the worker thread to give us a callback right
// away when we know the thread is sleeping.
TEST(ProcessThreadImpl, WakeUp) {
  ProcessThreadImpl thread;
  ASSERT_EQ(0, thread.Start());

  rtc::scoped_ptr<EventWrapper> started(EventWrapper::Create());
  rtc::scoped_ptr<EventWrapper> called(EventWrapper::Create());

  MockModule module;
  int64_t start_time = 0;
  int64_t called_time = 0;
  // Ask for a callback after 1000ms first, then 0ms.
  EXPECT_CALL(module, TimeUntilNextProcess())
      .WillOnce(DoAll(SetTimestamp(&start_time),
                      SetEvent(started.get()),
                      Return(1000)))
      .WillRepeatedly(Return(0));
  EXPECT_CALL(module, Process())
      .WillOnce(DoAll(SetTimestamp(&called_time),
                      SetEvent(called.get()),
                      Return(0)))
      .WillRepeatedly(Return(0));

  EXPECT_EQ(0, thread.RegisterModule(&module));

  EXPECT_EQ(kEventSignaled, started->Wait(100));
  thread.WakeUp(&module);
  EXPECT_EQ(kEventSignaled, called->Wait(100));
  ASSERT_EQ(0, thread.Stop());

  ASSERT_GT(start_time, 0);
  ASSERT_GT(called_time, 0);
  EXPECT_GE(called_time, start_time);
  uint32 diff = called_time - start_time;
  // We should have been called back much quicker than 1sec.
  EXPECT_LE(diff, 100u);
}

}  // namespace webrtc
