/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_DEVICE_INCLUDE_MOCK_AUDIO_DEVICE_H_
#define MODULES_AUDIO_DEVICE_INCLUDE_MOCK_AUDIO_DEVICE_H_

#include <string>

#include "api/make_ref_counted.h"
#include "modules/audio_device/include/audio_device.h"
#include "test/gmock.h"

namespace webrtc {
namespace test {

class MockAudioDeviceModule : public AudioDeviceModule {
 public:
  static rtc::scoped_refptr<MockAudioDeviceModule> CreateNice() {
    return rtc::make_ref_counted<::testing::NiceMock<MockAudioDeviceModule>>();
  }
  static rtc::scoped_refptr<MockAudioDeviceModule> CreateStrict() {
    return rtc::make_ref_counted<
        ::testing::StrictMock<MockAudioDeviceModule>>();
  }

  // AudioDeviceModule.
  MOCK_METHOD(int32_t,
              RegisterAudioCallback,
              (AudioTransport * audioCallback),
              (override));
  MOCK_METHOD(int32_t, Init, (), (override));
  MOCK_METHOD(int32_t, Terminate, (), (override));
  MOCK_METHOD(bool, Initialized, (), (const, override));
  MOCK_METHOD(int32_t, InitPlayout, (), (override));
  MOCK_METHOD(bool, PlayoutIsInitialized, (), (const, override));
  MOCK_METHOD(int32_t, InitRecording, (), (override));
  MOCK_METHOD(bool, RecordingIsInitialized, (), (const, override));
  MOCK_METHOD(int32_t, StartPlayout, (), (override));
  MOCK_METHOD(int32_t, StopPlayout, (), (override));
  MOCK_METHOD(bool, Playing, (), (const, override));
  MOCK_METHOD(int32_t, StartRecording, (), (override));
  MOCK_METHOD(int32_t, StopRecording, (), (override));
  MOCK_METHOD(bool, Recording, (), (const, override));
  MOCK_METHOD(int32_t, InitSpeaker, (), (override));
  MOCK_METHOD(bool, SpeakerIsInitialized, (), (const, override));
  MOCK_METHOD(int32_t, InitMicrophone, (), (override));
  MOCK_METHOD(bool, MicrophoneIsInitialized, (), (const, override));
  MOCK_METHOD(int32_t,
              StereoPlayoutIsAvailable,
              (bool* available),
              (const, override));
  MOCK_METHOD(int32_t, SetStereoPlayout, (bool enable), (override));
  MOCK_METHOD(int32_t, StereoPlayout, (bool* enabled), (const, override));
  MOCK_METHOD(int32_t,
              StereoRecordingIsAvailable,
              (bool* available),
              (const, override));
  MOCK_METHOD(int32_t, SetStereoRecording, (bool enable), (override));
  MOCK_METHOD(int32_t, StereoRecording, (bool* enabled), (const, override));
  MOCK_METHOD(int32_t, PlayoutDelay, (uint16_t * delayMS), (const, override));
  MOCK_METHOD(bool, BuiltInAECIsAvailable, (), (const, override));
  MOCK_METHOD(bool, BuiltInAGCIsAvailable, (), (const, override));
  MOCK_METHOD(bool, BuiltInNSIsAvailable, (), (const, override));
  MOCK_METHOD(int32_t, EnableBuiltInAEC, (bool enable), (override));
  MOCK_METHOD(int32_t, EnableBuiltInAGC, (bool enable), (override));
  MOCK_METHOD(int32_t, EnableBuiltInNS, (bool enable), (override));
  MOCK_METHOD(int32_t, GetPlayoutUnderrunCount, (), (const, override));
};
}  // namespace test
}  // namespace webrtc

#endif  // MODULES_AUDIO_DEVICE_INCLUDE_MOCK_AUDIO_DEVICE_H_
