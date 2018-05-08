/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_device/win/core_audio_output_win.h"

#include "modules/audio_device/audio_device_buffer.h"
#include "modules/audio_device/fine_audio_buffer.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/ptr_util.h"

using Microsoft::WRL::ComPtr;

namespace webrtc {

using win::CoreAudioUtility;

namespace win_adm {

CoreAudioOutput::CoreAudioOutput()
    : CoreAudioBase(CoreAudioBase::Direction::kOutput) {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
}

CoreAudioOutput::~CoreAudioOutput() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  Terminate();
}

int CoreAudioOutput::Init() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  return 0;
}

int CoreAudioOutput::Terminate() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  StopPlayout();
  return 0;
}

int CoreAudioOutput::NumDevices() const {
  RTC_DCHECK_RUN_ON(&thread_checker_);
  return CoreAudioUtility::NumberOfActiveDevices(eRender);
}

int CoreAudioOutput::SetDevice(int index) {
  RTC_DLOG(INFO) << __FUNCTION__ << ": " << index;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  if (initialized_) {
    return -1;
  }

  std::string device_id = GetDeviceID(index);
  RTC_DLOG(INFO) << "index=" << index << " => device_id: " << device_id;
  device_id_ = device_id;

  return device_id_.empty() ? -1 : 0;
}

int CoreAudioOutput::SetDevice(AudioDeviceModule::WindowsDeviceType device) {
  RTC_DLOG(INFO) << __FUNCTION__ << ": " << device;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  return SetDevice((device == AudioDeviceModule::kDefaultDevice) ? 0 : 1);
}

int CoreAudioOutput::DeviceName(int index,
                                char name[kAdmMaxDeviceNameSize],
                                char guid[kAdmMaxGuidSize]) {
  RTC_DLOG(INFO) << __FUNCTION__ << ": " << index;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  RTC_DCHECK(name);
  return CoreAudioBase::DeviceName(index, name, guid);
}

void CoreAudioOutput::AttachAudioBuffer(AudioDeviceBuffer* audio_buffer) {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  audio_device_buffer_ = audio_buffer;
}

bool CoreAudioOutput::PlayoutIsInitialized() const {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  return initialized_;
}

int CoreAudioOutput::InitPlayout() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  RTC_DCHECK(!initialized_);
  RTC_DCHECK(!playing_);
  // RTC_DCHECK(!audio_client_.Get());

  // Create and IAudioClient client and store the valid interface pointer in
  // |audio_client_|. The base class will use optimal output parameters and do
  // an event driven shared mode initialization. The utilized format will be
  // stored in |format_| and can be used for configuration and allocation of
  // audio buffers.
  if (!CoreAudioBase::Init()) {
    return -1;
  }
  RTC_DCHECK(audio_client_.Get());

  // Configure the playout side of the audio device buffer using the |format_|.
  RTC_DCHECK(audio_device_buffer_);
  WAVEFORMATEX* format = &format_.Format;
  RTC_DCHECK_EQ(format->wFormatTag, WAVE_FORMAT_EXTENSIBLE);
  audio_device_buffer_->SetPlayoutSampleRate(format->nSamplesPerSec);
  audio_device_buffer_->SetPlayoutChannels(format->nChannels);

  // Create a modified audio buffer class which allows us to ask for any number
  // of samples (and not only multiple of 10ms) to match the optimal
  // buffer size per callback used by Core Audio.
  // TODO(henrika): can we use a shared buffer instead?
  fine_audio_buffer_ = rtc::MakeUnique<FineAudioBuffer>(audio_device_buffer_);

  // Create an IAudioRenderClient client for an initialized IAudioClient.
  // The IAudioRenderClient interface enables us to write output data to
  // a rendering endpoint buffer.
  ComPtr<IAudioRenderClient> audio_render_client =
      CoreAudioUtility::CreateRenderClient(audio_client_.Get());
  if (!audio_render_client.Get())
    return -1;

  // Store valid COM interfaces.
  audio_render_client_ = audio_render_client;
  // audio_clock_ = audio_clock;

  initialized_ = true;
  return 0;
}

int CoreAudioOutput::StartPlayout() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  // if (fine_audio_buffer_) {
  //  fine_audio_buffer_->ResetPlayout();
  // }
  if (!Start()) {
    return -1;
  }
  playing_ = true;
  return 0;
}

int CoreAudioOutput::StopPlayout() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  if (!initialized_ || !playing_) {
    return 0;
  }
  if (!Stop()) {
    RTC_LOG(LS_ERROR) << "StopPlayout failed";
    return -1;
  }
  // thread_checker_aaudio_.DetachFromThread();
  initialized_ = false;
  playing_ = false;
  return 0;
}

bool CoreAudioOutput::Playing() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  return playing_;
}

}  // namespace win_adm

}  // namespace webrtc
