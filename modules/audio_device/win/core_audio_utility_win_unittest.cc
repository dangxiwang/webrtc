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
        AudioDeviceName::kDefaultDeviceId, data[i].flow, data[i].role);
    EXPECT_TRUE(audio_device.Get());
    EXPECT_EQ(data[i].flow, CoreAudioUtility::GetDataFlow(audio_device.Get()));
  }

  // Only eRender and eCapture are allowed as flow parameter.
  audio_device = CoreAudioUtility::CreateDevice(
      AudioDeviceName::kDefaultDeviceId, eAll, eConsole);
  EXPECT_FALSE(audio_device.Get());
}

TEST_F(CoreAudioUtilityWinTest, CreateDevice) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  // Get name and ID of default device used for playback.
  ComPtr<IMMDevice> default_render_device = CoreAudioUtility::CreateDevice(
      AudioDeviceName::kDefaultDeviceId, eRender, eConsole);
  AudioDeviceName default_render_name =
      CoreAudioUtility::GetDeviceName(default_render_device.Get());
  EXPECT_TRUE(default_render_name.IsValid());

  // Use the unique ID as input to CreateDevice() and create a corresponding
  // IMMDevice. The data-flow direction and role parameters are ignored for
  // this scenario.
  ComPtr<IMMDevice> audio_device = CoreAudioUtility::CreateDevice(
      default_render_name.unique_id, EDataFlow(), ERole());
  EXPECT_TRUE(audio_device.Get());

  // Verify that the two IMMDevice interfaces represents the same endpoint
  // by comparing their unique IDs.
  AudioDeviceName device_name =
      CoreAudioUtility::GetDeviceName(audio_device.Get());
  EXPECT_EQ(default_render_name.unique_id, device_name.unique_id);
}

TEST_F(CoreAudioUtilityWinTest, GetDefaultDeviceName) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  struct {
    EDataFlow flow;
    ERole role;
  } data[] = {{eRender, eConsole},
              {eRender, eCommunications},
              {eCapture, eConsole},
              {eCapture, eCommunications}};

  // Get name and ID of default devices for all flow/role combinations above.
  ComPtr<IMMDevice> audio_device;
  AudioDeviceName device_name;
  for (size_t i = 0; i < arraysize(data); ++i) {
    audio_device = CoreAudioUtility::CreateDevice(
        AudioDeviceName::kDefaultDeviceId, data[i].flow, data[i].role);
    device_name = CoreAudioUtility::GetDeviceName(audio_device.Get());
    EXPECT_TRUE(device_name.IsValid());
  }
}

TEST_F(CoreAudioUtilityWinTest, GetFriendlyName) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  // Get name and ID of default device used for recording.
  ComPtr<IMMDevice> audio_device = CoreAudioUtility::CreateDevice(
      AudioDeviceName::kDefaultDeviceId, eCapture, eConsole);
  AudioDeviceName device_name =
      CoreAudioUtility::GetDeviceName(audio_device.Get());
  EXPECT_TRUE(device_name.IsValid());

  // Use unique ID as input to GetFriendlyName() and compare the result
  // with the already obtained friendly name for the default capture device.
  std::string friendly_name = CoreAudioUtility::GetFriendlyName(
      device_name.unique_id, eCapture, eConsole);
  EXPECT_EQ(friendly_name, device_name.device_name);

  // Same test as above but for playback.
  audio_device = CoreAudioUtility::CreateDevice(
      AudioDeviceName::kDefaultDeviceId, eRender, eConsole);
  device_name = CoreAudioUtility::GetDeviceName(audio_device.Get());
  friendly_name = CoreAudioUtility::GetFriendlyName(device_name.unique_id,
                                                    eRender, eConsole);
  EXPECT_EQ(friendly_name, device_name.device_name);
}

TEST_F(CoreAudioUtilityWinTest, GetInputDeviceNames) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  AudioDeviceNames device_names;
  EXPECT_TRUE(CoreAudioUtility::GetInputDeviceNames(&device_names));
  // Number of elements in the list should be two more than the number of
  // active devices since we always add default and default communication
  // devices on index 0 and 1.
  EXPECT_EQ(static_cast<int>(device_names.size()),
            2 + CoreAudioUtility::NumberOfActiveDevices(eCapture));
}

TEST_F(CoreAudioUtilityWinTest, GetOutputDeviceNames) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  AudioDeviceNames device_names;
  EXPECT_TRUE(CoreAudioUtility::GetOutputDeviceNames(&device_names));
  // Number of elements in the list should be two more than the number of
  // active devices since we always add default and default communication
  // devices on index 0 and 1.
  EXPECT_EQ(static_cast<int>(device_names.size()),
            2 + CoreAudioUtility::NumberOfActiveDevices(eRender));
}

TEST_F(CoreAudioUtilityWinTest, CreateClient) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  EDataFlow data[] = {eRender, eCapture};

  // Obtain reference to an IAudioClient interface for a default audio endpoint
  // device specified by two different data flows and the |eConsole| role.
  for (size_t i = 0; i < arraysize(data); ++i) {
    ComPtr<IAudioClient> client = CoreAudioUtility::CreateClient(
        AudioDeviceName::kDefaultDeviceId, data[i], eConsole);
    EXPECT_TRUE(client.Get());
  }
}

TEST_F(CoreAudioUtilityWinTest, GetSharedModeMixFormat) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  ComPtr<IAudioClient> client = CoreAudioUtility::CreateClient(
      AudioDeviceName::kDefaultDeviceId, eRender, eConsole);
  EXPECT_TRUE(client.Get());

  // Perform a simple sanity test of the acquired format structure.
  WAVEFORMATPCMEX format;
  EXPECT_TRUE(SUCCEEDED(
      CoreAudioUtility::GetSharedModeMixFormat(client.Get(), &format)));
  EXPECT_GE(format.Format.nChannels, 1);
  EXPECT_GE(format.Format.nSamplesPerSec, 8000u);
  EXPECT_GE(format.Format.wBitsPerSample, 16);
  EXPECT_GE(format.Samples.wValidBitsPerSample, 16);
  EXPECT_EQ(format.Format.wFormatTag, WAVE_FORMAT_EXTENSIBLE);
}

TEST_F(CoreAudioUtilityWinTest, IsFormatSupported) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  // Create a default render client.
  ComPtr<IAudioClient> client = CoreAudioUtility::CreateClient(
      AudioDeviceName::kDefaultDeviceId, eRender, eConsole);
  EXPECT_TRUE(client.Get());

  // Get the default, shared mode, mixing format.
  WAVEFORMATPCMEX format;
  EXPECT_TRUE(SUCCEEDED(
      CoreAudioUtility::GetSharedModeMixFormat(client.Get(), &format)));

  // In shared mode, the audio engine always supports the mix format.
  EXPECT_TRUE(CoreAudioUtility::IsFormatSupported(
      client.Get(), AUDCLNT_SHAREMODE_SHARED, &format));

  // Use an invalid format and verify that it is not supported.
  format.Format.nSamplesPerSec += 1;
  EXPECT_FALSE(CoreAudioUtility::IsFormatSupported(
      client.Get(), AUDCLNT_SHAREMODE_SHARED, &format));
}

TEST_F(CoreAudioUtilityWinTest, GetDevicePeriod) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  EDataFlow data[] = {eRender, eCapture};

  // Verify that the device periods are valid for the default render and
  // capture devices.
  for (size_t i = 0; i < arraysize(data); ++i) {
    ComPtr<IAudioClient> client;
    REFERENCE_TIME shared_time_period = 0;
    REFERENCE_TIME exclusive_time_period = 0;
    client = CoreAudioUtility::CreateClient(AudioDeviceName::kDefaultDeviceId,
                                            data[i], eConsole);
    EXPECT_TRUE(client.Get());
    EXPECT_TRUE(SUCCEEDED(CoreAudioUtility::GetDevicePeriod(
        client.Get(), AUDCLNT_SHAREMODE_SHARED, &shared_time_period)));
    EXPECT_GT(shared_time_period, 0);
    EXPECT_TRUE(SUCCEEDED(CoreAudioUtility::GetDevicePeriod(
        client.Get(), AUDCLNT_SHAREMODE_EXCLUSIVE, &exclusive_time_period)));
    EXPECT_GT(exclusive_time_period, 0);
    EXPECT_LE(exclusive_time_period, shared_time_period);
  }
}

TEST_F(CoreAudioUtilityWinTest, GetPreferredAudioParameters) {
  ABORT_TEST_IF_NOT(DevicesAvailable());

  EDataFlow data[] = {eRender, eCapture};

  // Verify that the preferred audio parameters are OK for the default render
  // and capture devices.
  for (size_t i = 0; i < arraysize(data); ++i) {
    AudioParameters params;
    EXPECT_TRUE(SUCCEEDED(CoreAudioUtility::GetPreferredAudioParameters(
        AudioDeviceName::kDefaultDeviceId, data[i] == eRender, &params)));
    EXPECT_TRUE(params.is_valid());
    EXPECT_TRUE(params.is_complete());
  }
}

}  // namespace win
}  // namespace webrtc
