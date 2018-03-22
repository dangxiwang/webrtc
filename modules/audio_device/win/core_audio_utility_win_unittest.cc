/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// #include "test/gmock.h"
#include "modules/audio_device/win/core_audio_utility_win.h"
#include "rtc_base/logging.h"
#include "test/gtest.h"

// using ::testing::_;
// using ::testing::AtLeast;
// using ::testing::Ge;
// using ::testing::Invoke;
// using ::testing::NiceMock;
// using ::testing::NotNull;

namespace webrtc {
namespace win {

namespace {

#define ABORT_TEST_IF_NOT(requirements_satisfied)                        \
  do {                                                                   \
    bool fail = false;                                                   \
    if (ShouldAbortTest(requirements_satisfied, #requirements_satisfied, \
                        &fail)) {                                        \
      if (fail)                                                          \
        FAIL();                                                          \
      else                                                               \
        return;                                                          \
    }                                                                    \
  } while (false)

bool ShouldAbortTest(bool requirements_satisfied,
                     const char* requirements_expression,
                     bool* should_fail) {
  if (!requirements_satisfied) {
    RTC_LOG(LS_ERROR) << "Requirement(s) not satisfied ("
                      << requirements_expression << ")";
    // TODO(henrika): improve hardcoded condition to determine if test should
    // fail or be ignored.
    *should_fail = false;
    return true;
  }
  *should_fail = false;
  return false;
}

}  // namespace

// CoreAudioUtilityTest test fixture.
class CoreAudioUtilityTest : public ::testing::Test {
 protected:
  CoreAudioUtilityTest() {
    // We must initialize the COM library on a thread before we calling any of
    // the library functions. All COM functions will return CO_E_NOTINITIALIZED
    // otherwise.
    EXPECT_TRUE(com_init_.Succeeded());

    // Configure logging.
    rtc::LogMessage::LogToDebug(rtc::LS_INFO);
    rtc::LogMessage::LogTimestamps();
    rtc::LogMessage::LogThreads();
  }

  virtual ~CoreAudioUtilityTest() {}

  /*
  bool DevicesAvailable() {
    return CoreAudioUtil::IsSupported() &&
           CoreAudioUtil::NumberOfActiveDevices(eCapture) > 0 &&
           CoreAudioUtil::NumberOfActiveDevices(eRender) > 0;
  */

  bool DevicesAvailable() { return CoreAudioUtility::IsSupported(); }

 private:
  ScopedCOMInitializer com_init_;
};

TEST_F(CoreAudioUtilityTest, Test1) {
  ABORT_TEST_IF_NOT(DevicesAvailable());
}

}  // namespace win
}  // namespace webrtc
