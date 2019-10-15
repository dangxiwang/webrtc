/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/synchronization/sequence_checker.h"

#include <memory>
#include <utility>

#include "api/function_view.h"
#include "rtc_base/event.h"
#include "rtc_base/platform_thread.h"
#include "rtc_base/task_queue_for_test.h"
#include "rtc_base/thread_checker.h"
#include "test/gtest.h"

namespace webrtc {
namespace {

// This class is dead code, but its purpose is to make sure that
// SequenceChecker is compatible with the RTC_GUARDED_BY and RTC_RUN_ON
// attributes that are checked at compile-time.
class CompileTimeTestForGuardedBy {
 public:
  int CalledOnSequence() RTC_RUN_ON(sequence_checker_) { return guarded_; }

  void CallMeFromSequence() {
    RTC_DCHECK_RUN_ON(&sequence_checker_) << "Should be called on sequence";
    guarded_ = 41;
  }

 private:
  int guarded_ RTC_GUARDED_BY(sequence_checker_);
  ::webrtc::SequenceChecker sequence_checker_;
};

void RunOnDifferentThread(rtc::FunctionView<void()> run) {
  struct Object {
    static void Run(void* obj) {
      auto* me = static_cast<Object*>(obj);
      me->run();
      me->thread_has_run_event.Set();
    }

    rtc::FunctionView<void()> run;
    rtc::Event thread_has_run_event;
  } object{run};

  rtc::PlatformThread thread(&Object::Run, &object, "thread");
  thread.Start();
  EXPECT_TRUE(object.thread_has_run_event.Wait(1000));
  thread.Stop();
}

}  // namespace

TEST(SequenceCheckerTest, CallsAllowedOnSameThread) {
  SequenceChecker sequence_checker;
  EXPECT_TRUE(sequence_checker.IsCurrent());
}

TEST(SequenceCheckerTest, DestructorAllowedOnDifferentThread) {
  auto sequence_checker = std::make_unique<SequenceChecker>();
  RunOnDifferentThread([&] {
    // Verify that the destructor doesn't assert when called on a different
    // thread.
    sequence_checker.reset();
  });
}

TEST(SequenceCheckerTest, Detach) {
  SequenceChecker sequence_checker;
  sequence_checker.Detach();
  RunOnDifferentThread([&] { EXPECT_TRUE(sequence_checker.IsCurrent()); });
}

TEST(SequenceCheckerTest, DetachFromThreadAndUseOnTaskQueue) {
  SequenceChecker sequence_checker;
  sequence_checker.Detach();
  TaskQueueForTest queue;
  queue.SendTask([&] { EXPECT_TRUE(sequence_checker.IsCurrent()); },
                 RTC_FROM_HERE);
}

TEST(SequenceCheckerTest, DetachFromTaskQueueAndUseOnThread) {
  TaskQueueForTest queue;
  queue.SendTask(
      [] {
        SequenceChecker sequence_checker;
        sequence_checker.Detach();
        RunOnDifferentThread(
            [&] { EXPECT_TRUE(sequence_checker.IsCurrent()); });
      },
      RTC_FROM_HERE);
}

TEST(SequenceCheckerTest, MethodNotAllowedOnDifferentThreadInDebug) {
  SequenceChecker sequence_checker;
  RunOnDifferentThread(
      [&] { EXPECT_EQ(sequence_checker.IsCurrent(), !RTC_DCHECK_IS_ON); });
}

TEST(SequenceCheckerTest, MethodNotAllowedOnDifferentTaskQueueInDebug) {
  SequenceChecker sequence_checker;
  TaskQueueForTest queue;
  queue.SendTask(
      [&] { EXPECT_EQ(sequence_checker.IsCurrent(), !RTC_DCHECK_IS_ON); },
      RTC_FROM_HERE);
}

TEST(SequenceCheckerTest, DetachFromTaskQueueInDebug) {
  SequenceChecker sequence_checker;
  sequence_checker.Detach();

  TaskQueueForTest queue1;
  queue1.SendTask([&] { EXPECT_TRUE(sequence_checker.IsCurrent()); },
                  RTC_FROM_HERE);

  // IsCurrent should return false in debug builds after moving to
  // another task queue.
  TaskQueueForTest queue2;
  queue2.SendTask(
      [&] { EXPECT_EQ(sequence_checker.IsCurrent(), !RTC_DCHECK_IS_ON); },
      RTC_FROM_HERE);
}

class TestAnnotations {
 public:
  TestAnnotations() : test_var_(false) {}

  void ModifyTestVar() {
    RTC_DCHECK_RUN_ON(&checker_);
    test_var_ = true;
  }

 private:
  bool test_var_ RTC_GUARDED_BY(&checker_);
  SequenceChecker checker_;
};

TEST(SequenceCheckerTest, TestAnnotations) {
  TestAnnotations annotations;
  annotations.ModifyTestVar();
}

#if GTEST_HAS_DEATH_TEST && !defined(WEBRTC_ANDROID)

void TestAnnotationsOnWrongQueue() {
  TestAnnotations annotations;
  TaskQueueForTest queue;
  queue.SendTask([&] { annotations.ModifyTestVar(); }, RTC_FROM_HERE);
}

#if RTC_DCHECK_IS_ON
TEST(SequenceCheckerTest, TestAnnotationsOnWrongQueueDebug) {
  ASSERT_DEATH({ TestAnnotationsOnWrongQueue(); }, "");
}
#else
TEST(SequenceCheckerTest, TestAnnotationsOnWrongQueueRelease) {
  TestAnnotationsOnWrongQueue();
}
#endif
#endif  // GTEST_HAS_DEATH_TEST
}  // namespace webrtc
