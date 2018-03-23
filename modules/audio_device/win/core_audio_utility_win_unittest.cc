/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_device/win/core_audio_utility_win.h"
#include "modules/audio_device/audio_device_description.h"
#include "rtc_base/arraysize.h"
#include "rtc_base/logging.h"
#include "test/gtest.h"

using Microsoft::WRL::ComPtr;

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

// CoreAudioUtilityWinTest test fixture.
class CoreAudioUtilityWinTest : public ::testing::Test {
 protected:
  CoreAudioUtilityWinTest() {
    // We must initialize the COM library on a thread before we calling any of
    // the library functions. All COM functions will return CO_E_NOTINITIALIZED
    // otherwise.
    EXPECT_TRUE(com_init_.Succeeded());

    // Configure logging.
    rtc::LogMessage::LogToDebug(rtc::LS_INFO);
    rtc::LogMessage::LogTimestamps();
    rtc::LogMessage::LogThreads();
  }

  virtual ~CoreAudioUtilityWinTest() {}

  bool DevicesAvailable() {
    return CoreAudioUtility::IsSupported() &&
           CoreAudioUtility::NumberOfActiveDevices(eCapture) > 0 &&
           CoreAudioUtility::NumberOfActiveDevices(eRender) > 0;
  }

 private:
  ScopedCOMInitializer com_init_;
};

TEST_F(CoreAudioUtilityWinTest, NumberOfActiveDevices) {
  ABORT_TEST_IF_NOT(DevicesAvailable());
  int render_devices = CoreAudioUtility::NumberOfActiveDevices(eRender);
  EXPECT_GT(render_devices, 0);
  int capture_devices = CoreAudioUtility::NumberOfActiveDevices(eCapture);
  EXPECT_GT(capture_devices, 0);
  int total_devices = CoreAudioUtility::NumberOfActiveDevices(eAll);
  EXPECT_EQ(total_devices, render_devices + capture_devices);
}

TEST_F(CoreAudioUtilityWinTest, CreateDeviceEnumerator) {
  ABORT_TEST_IF_NOT(DevicesAvailable());
  ComPtr<IMMDeviceEnumerator> enumerator =
      CoreAudioUtility::CreateDeviceEnumerator();
  EXPECT_TRUE(enumerator.Get());
}

TEST_F(CoreAudioUtilityWinTest, CreateDefaultDevice) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  struct {
    EDataFlow flow;
    ERole role;
  } data[] = {{eRender, eConsole},         {eRender, eCommunications},
              {eRender, eMultimedia},      {eCapture, eConsole},
              {eCapture, eCommunications}, {eCapture, eMultimedia}};

  // Create default devices for all flow/role combinations above.
  ComPtr<IMMDevice> audio_device;
  for (size_t i = 0; i < arraysize(data); ++i) {
    audio_device = CoreAudioUtility::CreateDevice(
        AudioDeviceDescription::kDefaultDeviceId, data[i].flow, data[i].role);
    EXPECT_TRUE(audio_device.Get());
    EXPECT_EQ(data[i].flow, CoreAudioUtility::GetDataFlow(audio_device.Get()));
  }

  // Only eRender and eCapture are allowed as flow parameter.
  audio_device = CoreAudioUtility::CreateDevice(
      AudioDeviceDescription::kDefaultDeviceId, eAll, eConsole);
  EXPECT_FALSE(audio_device.Get());
}

TEST_F(CoreAudioUtilityWinTest, GetFriendlyName) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  // Get name and ID of default device used for recording.
  ComPtr<IMMDevice> audio_device = CoreAudioUtility::CreateDevice(
      AudioDeviceDescription::kDefaultDeviceId, eCapture, eConsole);
  AudioDeviceName device_name;
  HRESULT hr =
      CoreAudioUtility::GetDeviceName(audio_device.Get(), &device_name);
  EXPECT_TRUE(SUCCEEDED(hr));

  // Use unique ID as input to GetFriendlyName() and compare the result
  // with the already obtained friendly name for the default capture device.
  std::string friendly_name = CoreAudioUtility::GetFriendlyName(
      device_name.unique_id, eCapture, eConsole);
  EXPECT_EQ(friendly_name, device_name.device_name);

  // Same test as above but for playback.
  audio_device = CoreAudioUtility::CreateDevice(
      AudioDeviceDescription::kDefaultDeviceId, eRender, eConsole);
  hr = CoreAudioUtility::GetDeviceName(audio_device.Get(), &device_name);
  EXPECT_TRUE(SUCCEEDED(hr));
  friendly_name = CoreAudioUtility::GetFriendlyName(device_name.unique_id,
                                                    eRender, eConsole);
  EXPECT_EQ(friendly_name, device_name.device_name);
}

}  // namespace win
}  // namespace webrtc
