/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_DEVICE_WIN_AUDIO_DEVICE_CORE_AUDIO_WIN_H_
#define MODULES_AUDIO_DEVICE_WIN_AUDIO_DEVICE_CORE_AUDIO_WIN_H_

#include <memory>
#include <string>

#include "api/optional.h"
#include "modules/audio_device/audio_device_generic.h"
#include "modules/audio_device/fine_audio_buffer.h"
#include "modules/audio_device/win/core_audio_utility_win.h"
#include "rtc_base/platform_thread.h"
#include "rtc_base/thread_annotations.h"
#include "rtc_base/thread_checker.h"

namespace webrtc {
namespace win {

class AudioDeviceCoreAudio final : public AudioDeviceGeneric {
 public:
  AudioDeviceCoreAudio();
  ~AudioDeviceCoreAudio();

  int32_t ActiveAudioLayer(
      AudioDeviceModule::AudioLayer& audioLayer) const override;

  void AttachAudioBuffer(AudioDeviceBuffer* audio_buffer) override;

  InitStatus Init() override;
  int32_t Terminate() override;
  bool Initialized() const override;

  int16_t PlayoutDevices() override;
  int16_t RecordingDevices() override;
  int32_t PlayoutDeviceName(uint16_t index,
                            char name[kAdmMaxDeviceNameSize],
                            char guid[kAdmMaxGuidSize]) override;
  int32_t RecordingDeviceName(uint16_t index,
                              char name[kAdmMaxDeviceNameSize],
                              char guid[kAdmMaxGuidSize]) override {
    return 0;
  }

  int32_t SetPlayoutDevice(uint16_t index) override;
  int32_t SetPlayoutDevice(
      AudioDeviceModule::WindowsDeviceType device) override;
  int32_t SetRecordingDevice(uint16_t index) override;
  int32_t SetRecordingDevice(
      AudioDeviceModule::WindowsDeviceType device) override;

  int32_t PlayoutIsAvailable(bool& available) override { return 0; }
  int32_t InitPlayout() override;
  bool PlayoutIsInitialized() const override;
  int32_t RecordingIsAvailable(bool& available) override { return 0; }
  int32_t InitRecording() override;
  bool RecordingIsInitialized() const override;

  int32_t StartPlayout() override;
  int32_t StopPlayout() override;
  bool Playing() const override;
  int32_t StartRecording() override;
  int32_t StopRecording() override;
  bool Recording() const override;

  int32_t InitSpeaker() override { return 0; }
  bool SpeakerIsInitialized() const override { return false; }
  int32_t InitMicrophone() override { return 0; }
  bool MicrophoneIsInitialized() const override { return false; }

  int32_t SpeakerVolumeIsAvailable(bool& available) override { return 0; }
  int32_t SetSpeakerVolume(uint32_t volume) override { return 0; }
  int32_t SpeakerVolume(uint32_t& volume) const override { return 0; }
  int32_t MaxSpeakerVolume(uint32_t& maxVolume) const override { return 0; }
  int32_t MinSpeakerVolume(uint32_t& minVolume) const override { return 0; }

  int32_t MicrophoneVolumeIsAvailable(bool& available) override { return 0; }
  int32_t SetMicrophoneVolume(uint32_t volume) override { return 0; }
  int32_t MicrophoneVolume(uint32_t& volume) const override { return 0; }
  int32_t MaxMicrophoneVolume(uint32_t& maxVolume) const override { return 0; }
  int32_t MinMicrophoneVolume(uint32_t& minVolume) const override { return 0; }

  int32_t SpeakerMuteIsAvailable(bool& available) override { return 0; }
  int32_t SetSpeakerMute(bool enable) override { return 0; }
  int32_t SpeakerMute(bool& enabled) const override { return 0; }

  int32_t MicrophoneMuteIsAvailable(bool& available) override { return 0; }
  int32_t SetMicrophoneMute(bool enable) override { return 0; }
  int32_t MicrophoneMute(bool& enabled) const override { return 0; }

  int32_t StereoPlayoutIsAvailable(bool& available) override;
  int32_t SetStereoPlayout(bool enable) override { return 0; }
  int32_t StereoPlayout(bool& enabled) const override { return 0; }
  int32_t StereoRecordingIsAvailable(bool& available) override;
  int32_t SetStereoRecording(bool enable) override { return 0; }
  int32_t StereoRecording(bool& enabled) const override { return 0; }

  int32_t PlayoutDelay(uint16_t& delayMS) const override { return 0; }

  bool BuiltInAECIsAvailable() const override { return false; }
  int32_t EnableBuiltInAEC(bool enable) override { return 0; }

  AudioDeviceCoreAudio(const AudioDeviceCoreAudio&) = delete;
  AudioDeviceCoreAudio& operator=(const AudioDeviceCoreAudio&) = delete;

  void RunRenderThread();
  void RunCaptureThread();

 private:
  void StopRenderThread();
  void StopCaptureThread();
  bool RenderAudioFromSource(uint64_t device_frequency);
  bool CaptureAudioToSink();
  int EstimateOutputLatencyMillis(uint64_t device_frequency);
  rtc::Optional<int> EstimateInputLatencyMillis(uint64_t capture_time_100ns);

  rtc::ThreadChecker main_thread_checker_;
  std::unique_ptr<ScopedCOMInitializer> com_initializer_;

  bool initialized_ RTC_GUARDED_BY(main_thread_checker_) = false;
  // TODO(henrika): do we really need these states?
  bool playout_is_initialized_ RTC_GUARDED_BY(main_thread_checker_) = false;
  bool recording_is_initialized_ RTC_GUARDED_BY(main_thread_checker_) = false;

  std::string output_device_id_ RTC_GUARDED_BY(main_thread_checker_);
  std::string input_device_id_ RTC_GUARDED_BY(main_thread_checker_);

  // Raw pointer handle provided to us in AttachAudioBuffer(). Owned by the
  // AudioDeviceModuleImpl class and set by AudioDeviceModule::Create().
  AudioDeviceBuffer* audio_device_buffer_ RTC_GUARDED_BY(main_thread_checker_) =
      nullptr;

  // TODO(henrika): thread annotations...

  Microsoft::WRL::ComPtr<IAudioClient> audio_output_client_
      RTC_GUARDED_BY(main_thread_checker_);
  Microsoft::WRL::ComPtr<IAudioClient> audio_input_client_
      RTC_GUARDED_BY(main_thread_checker_);

  Microsoft::WRL::ComPtr<IAudioClock> audio_output_clock_;

  Microsoft::WRL::ComPtr<IAudioRenderClient> audio_render_client_
      RTC_GUARDED_BY(main_thread_checker_);
  Microsoft::WRL::ComPtr<IAudioCaptureClient> audio_capture_client_
      RTC_GUARDED_BY(main_thread_checker_);

  WAVEFORMATEXTENSIBLE output_format_ = {};
  WAVEFORMATEXTENSIBLE input_format_ = {};

  uint32_t output_endpoint_buffer_size_frames_ = 0;
  uint32_t input_endpoint_buffer_size_frames_ = 0;

  rtc::Optional<double> qpc_to_100ns_ RTC_GUARDED_BY(main_thread_checker_);

  // Counts the number of audio frames written to the endpoint buffer.
  uint64_t num_output_frames_written_ = 0;

  ScopedHandle audio_samples_render_event_;
  ScopedHandle stop_render_event_;

  ScopedHandle audio_samples_capture_event_;
  ScopedHandle stop_capture_event_;

  // TODO(henrika): improve comments...
  // Rendering is driven by this thread (which has no message loop).
  // All OnMoreData() callbacks will be called from this thread.
  std::unique_ptr<rtc::PlatformThread> render_thread_;
  std::unique_ptr<rtc::PlatformThread> capture_thread_;

  std::unique_ptr<FineAudioBuffer> fine_audio_output_buffer_;
  std::unique_ptr<FineAudioBuffer> fine_audio_input_buffer_;
};

}  // namespace win
}  // namespace webrtc

#endif  //  MODULES_AUDIO_DEVICE_WIN_AUDIO_DEVICE_CORE_AUDIO_WIN_H_
