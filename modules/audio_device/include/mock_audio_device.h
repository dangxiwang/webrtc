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

#include "modules/audio_device/include/audio_device.h"
#include "rtc_base/ref_counted_object.h"
#include "test/gmock.h"

namespace webrtc {
namespace test {

class MockAudioDeviceModule : public AudioDeviceModule {
 public:
  static rtc::scoped_refptr<MockAudioDeviceModule> CreateNice() {
    return new rtc::RefCountedObject<
        ::testing::NiceMock<MockAudioDeviceModule>>();
  }
  static rtc::scoped_refptr<MockAudioDeviceModule> CreateStrict() {
    return new rtc::RefCountedObject<
        ::testing::StrictMock<MockAudioDeviceModule>>();
  }

  // AudioDeviceModule.
  MOCK_METHOD(int32_t, ActiveAudioLayer, (AudioLayer*), (const, override));
  MOCK_METHOD(int32_t, RegisterAudioCallback, (AudioTransport*), (override));
  MOCK_METHOD(int32_t, Init, (), (override));
  MOCK_METHOD(int32_t, Terminate, (), (override));
  MOCK_METHOD(bool, Initialized, (), (const, override));
  MOCK_METHOD(int16_t, PlayoutDevices, (), (override));
  MOCK_METHOD(int16_t, RecordingDevices, (), (override));
  MOCK_METHOD(int32_t,
              PlayoutDeviceName,
              (uint16_t index,
               char name[kAdmMaxDeviceNameSize],
               char guid[kAdmMaxGuidSize]),
              (override));
  MOCK_METHOD(int32_t,
              RecordingDeviceName,
              (uint16_t index,
               char name[kAdmMaxDeviceNameSize],
               char guid[kAdmMaxGuidSize]),
              (override));
  MOCK_METHOD(int32_t, SetPlayoutDevice, (uint16_t index), (override));
  MOCK_METHOD(int32_t,
              SetPlayoutDevice,
              (WindowsDeviceType device),
              (override));
  MOCK_METHOD(int32_t, SetRecordingDevice, (uint16_t index), (override));
  MOCK_METHOD(int32_t,
              SetRecordingDevice,
              (WindowsDeviceType device),
              (override));
  MOCK_METHOD(int32_t, PlayoutIsAvailable, (bool*), (override));
  MOCK_METHOD(int32_t, InitPlayout, (), (override));
  MOCK_METHOD(bool, PlayoutIsInitialized, (), (const, override));
  MOCK_METHOD(int32_t, RecordingIsAvailable, (bool*), (override));
  MOCK_METHOD(int32_t, InitRecording, (), (override));
  MOCK_METHOD(bool, RecordingIsInitialized, (), (const, override));
  MOCK_METHOD(int32_t, StartPlayout, (), (override));
  MOCK_METHOD(int32_t, StopPlayout, (), (override));
  MOCK_METHOD(bool, Playing, (), (const, override));
  MOCK_METHOD(int32_t, StartRecording, (), (override));
  MOCK_METHOD(int32_t, StopRecording, (), (override));
  MOCK_METHOD(bool, Recording, (), (const, override));
  MOCK_METHOD(int32_t, SetAGC, (bool enable), (override));
  MOCK_METHOD(bool, AGC, (), (const, override));
  MOCK_METHOD(int32_t, InitSpeaker, (), (override));
  MOCK_METHOD(bool, SpeakerIsInitialized, (), (const, override));
  MOCK_METHOD(int32_t, InitMicrophone, (), (override));
  MOCK_METHOD(bool, MicrophoneIsInitialized, (), (const, override));
  MOCK_METHOD(int32_t, SpeakerVolumeIsAvailable, (bool*), (override));
  MOCK_METHOD(int32_t, SetSpeakerVolume, (uint32_t volume), (override));
  MOCK_METHOD(int32_t, SpeakerVolume, (uint32_t*), (const, override));
  MOCK_METHOD(int32_t, MaxSpeakerVolume, (uint32_t*), (const, override));
  MOCK_METHOD(int32_t, MinSpeakerVolume, (uint32_t*), (const, override));
  MOCK_METHOD(int32_t, MicrophoneVolumeIsAvailable, (bool*), (override));
  MOCK_METHOD(int32_t, SetMicrophoneVolume, (uint32_t volume), (override));
  MOCK_METHOD(int32_t, MicrophoneVolume, (uint32_t*), (const, override));
  MOCK_METHOD(int32_t, MaxMicrophoneVolume, (uint32_t*), (const, override));
  MOCK_METHOD(int32_t, MinMicrophoneVolume, (uint32_t*), (const, override));
  MOCK_METHOD(int32_t, SpeakerMuteIsAvailable, (bool*), (override));
  MOCK_METHOD(int32_t, SetSpeakerMute, (bool enable), (override));
  MOCK_METHOD(int32_t, SpeakerMute, (bool*), (const, override));
  MOCK_METHOD(int32_t, MicrophoneMuteIsAvailable, (bool*), (override));
  MOCK_METHOD(int32_t, SetMicrophoneMute, (bool enable), (override));
  MOCK_METHOD(int32_t, MicrophoneMute, (bool*), (const, override));
  MOCK_METHOD(int32_t, StereoPlayoutIsAvailable, (bool*), (const, override));
  MOCK_METHOD(int32_t, SetStereoPlayout, (bool enable), (override));
  MOCK_METHOD(int32_t, StereoPlayout, (bool*), (const, override));
  MOCK_METHOD(int32_t, StereoRecordingIsAvailable, (bool*), (const, override));
  MOCK_METHOD(int32_t, SetStereoRecording, (bool enable), (override));
  MOCK_METHOD(int32_t, StereoRecording, (bool*), (const, override));
  MOCK_METHOD(int32_t, PlayoutDelay, (uint16_t*), (const, override));
  MOCK_METHOD(bool, BuiltInAECIsAvailable, (), (const, override));
  MOCK_METHOD(bool, BuiltInAGCIsAvailable, (), (const, override));
  MOCK_METHOD(bool, BuiltInNSIsAvailable, (), (const, override));
  MOCK_METHOD(int32_t, EnableBuiltInAEC, (bool enable), (override));
  MOCK_METHOD(int32_t, EnableBuiltInAGC, (bool enable), (override));
  MOCK_METHOD(int32_t, EnableBuiltInNS, (bool enable), (override));
  MOCK_METHOD(int32_t, GetPlayoutUnderrunCount, (), (const, override));
#if defined(WEBRTC_IOS)
  MOCK_METHOD(int,
              GetPlayoutAudioParameters,
              (AudioParameters*),
              (const, override));
  MOCK_METHOD(int,
              GetRecordAudioParameters,
              (AudioParameters*),
              (const, override));
#endif  // WEBRTC_IOS
};
}  // namespace test
}  // namespace webrtc

#endif  // MODULES_AUDIO_DEVICE_INCLUDE_MOCK_AUDIO_DEVICE_H_
