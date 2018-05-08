/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_device/win/core_audio_base_win.h"

#include <string>

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

using Microsoft::WRL::ComPtr;

namespace webrtc {

using win::CoreAudioUtility;

namespace win_adm {

namespace {

enum DefaultDeviceType {
  kDefault = 0,
  kDefaultCommunications = (kDefault + 1),
  kDefaultDeviceTypeMaxCount = (kDefaultCommunications + 1),
};

const char* DirectionToString(CoreAudioBase::Direction direction) {
  switch (direction) {
    case CoreAudioBase::Direction::kOutput:
      return "Output";
    case CoreAudioBase::Direction::kInput:
      return "Input";
    default:
      return "Unkown";
  }
}

}  // namespace

CoreAudioBase::CoreAudioBase(Direction direction)
    : direction_(direction), format_() {
  RTC_DLOG(INFO) << __FUNCTION__ << "[" << DirectionToString(direction) << "]";

  // Create the event which the audio engine will signal each time a buffer
  // becomes ready to be processed by the client.
  audio_samples_event_.Set(CreateEvent(nullptr, false, false, nullptr));
  RTC_DCHECK(audio_samples_event_.IsValid());
}

CoreAudioBase::~CoreAudioBase() {
  RTC_DLOG(INFO) << __FUNCTION__;
}

EDataFlow CoreAudioBase::GetDataFlow() const {
  return direction_ == CoreAudioBase::Direction::kOutput ? eRender : eCapture;
}

int CoreAudioBase::NumberOfActiveDevices() const {
  return CoreAudioUtility::NumberOfActiveDevices(GetDataFlow());
}

int CoreAudioBase::NumberOfEnumeratedDevices() const {
  return NumberOfActiveDevices() + kDefaultDeviceTypeMaxCount;
}

bool CoreAudioBase::IsDefaultDevice(int index) const {
  return index == kDefault;
}

bool CoreAudioBase::IsDefaultCommunicationsDevice(int index) const {
  return index == kDefaultCommunications;
}

bool CoreAudioBase::IsInput() const {
  return direction_ == CoreAudioBase::Direction::kInput;
}

bool CoreAudioBase::IsOutput() const {
  return direction_ == CoreAudioBase::Direction::kInput;
}

std::string CoreAudioBase::GetDeviceID(int index) const {
  if (index > NumberOfEnumeratedDevices() - 1) {
    RTC_LOG(LS_ERROR) << "Invalid device index";
    return std::string();
  }

  std::string device_id;
  if (IsDefaultDevice(index)) {
    device_id = IsInput() ? CoreAudioUtility::GetDefaultInputDeviceID()
                          : CoreAudioUtility::GetDefaultOutputDeviceID();
  } else if (IsDefaultCommunicationsDevice(index)) {
    device_id = IsInput() ? CoreAudioUtility::GetCommunicationsInputDeviceID()
                          : CoreAudioUtility::GetCommunicationsOutputDeviceID();
  } else {
    AudioDeviceNames device_names;
    bool ok = IsInput() ? CoreAudioUtility::GetInputDeviceNames(&device_names)
                        : CoreAudioUtility::GetOutputDeviceNames(&device_names);
    if (ok) {
      device_id = device_names[index].unique_id;
    }
  }
  return device_id;
}

int CoreAudioBase::DeviceName(int index,
                              char name[kAdmMaxDeviceNameSize],
                              char guid[kAdmMaxGuidSize]) const {
  RTC_DLOG(INFO) << __FUNCTION__ << "[" << DirectionToString(direction())
                 << "]";
  if (index > NumberOfEnumeratedDevices() - 1) {
    RTC_LOG(LS_ERROR) << "Invalid device index";
    return -1;
  }

  AudioDeviceNames device_names;
  bool ok = IsInput() ? CoreAudioUtility::GetInputDeviceNames(&device_names)
                      : CoreAudioUtility::GetOutputDeviceNames(&device_names);
  if (!ok) {
    RTC_LOG(LS_ERROR) << "Failed to get the device name";
    return -1;
  }

  rtc::strcpyn(reinterpret_cast<char*>(name), kAdmMaxDeviceNameSize,
               device_names[index].device_name.c_str());
  RTC_DLOG(INFO) << "name: " << name;
  if (guid != nullptr) {
    rtc::strcpyn(reinterpret_cast<char*>(guid), kAdmMaxGuidSize,
                 device_names[index].unique_id.c_str());
    RTC_DLOG(INFO) << "guid: " << guid;
  }
  return 0;
}

bool CoreAudioBase::Init() {
  RTC_DLOG(INFO) << __FUNCTION__ << "[" << DirectionToString(direction())
                 << "]";
  RTC_DCHECK(!device_id_.empty());
  RTC_DCHECK(audio_device_buffer_);

  // Use an existing |device_id_| and set parameters which are required to
  // create an audio client. It is up to the parent class to set |device_id_|.
  // TODO(henrika): improve device notification.
  std::string device_id = device_id_;
  EDataFlow data_flow = eRender;
  ERole role = eConsole;
  if (IsInput() && device_id_ == CoreAudioUtility::GetDefaultInputDeviceID() ||
      IsOutput() &&
          device_id_ == CoreAudioUtility::GetDefaultOutputDeviceID()) {
    device_id = AudioDeviceName::kDefaultDeviceId;
    data_flow = eRender;
    role = eConsole;
  } else if (IsInput() &&
                 device_id_ ==
                     CoreAudioUtility::GetCommunicationsInputDeviceID() ||
             IsOutput() &&
                 device_id_ ==
                     CoreAudioUtility::GetCommunicationsOutputDeviceID()) {
    device_id = AudioDeviceName::kDefaultCommunicationsDeviceId;
    data_flow = eRender;
    role = eCommunications;
  }

  // Create an IAudioClient interface which enables us to create and initialize
  // an audio stream between an audio application and the audio engine.
  ComPtr<IAudioClient> audio_client =
      CoreAudioUtility::CreateClient(device_id, data_flow, role);
  if (!audio_client.Get()) {
    return false;
  }

  // Retrieve preferred audio input or output parameters for the given client.
  AudioParameters params;
  if (FAILED(CoreAudioUtility::GetPreferredAudioParameters(audio_client.Get(),
                                                           &params))) {
    return false;
  }

  // Define the output WAVEFORMATEXTENSIBLE format in |format_|.
  WAVEFORMATEX* format = &format_.Format;
  format->wFormatTag = WAVE_FORMAT_EXTENSIBLE;
  format->nChannels = params.channels();
  format->nSamplesPerSec = params.sample_rate();
  format->wBitsPerSample = params.bits_per_sample();
  format->nBlockAlign = (format->wBitsPerSample / 8) * format->nChannels;
  format->nAvgBytesPerSec = format->nSamplesPerSec * format->nBlockAlign;
  format->cbSize = sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX);
  // Add the parts which are unique for the WAVE_FORMAT_EXTENSIBLE structure.
  format_.Samples.wValidBitsPerSample = params.bits_per_sample();
  // TODO(henrika): improve (common for input and output?)
  format_.dwChannelMask = params.channels() == 1
                              ? SPEAKER_FRONT_CENTER
                              : SPEAKER_FRONT_LEFT | SPEAKER_FRONT_RIGHT;
  format_.SubFormat = KSDATAFORMAT_SUBTYPE_PCM;
  RTC_DLOG(INFO) << CoreAudioUtility::WaveFormatExToString(&format_);

  // Verify that the format is supported.
  if (!CoreAudioUtility::IsFormatSupported(
          audio_client.Get(), AUDCLNT_SHAREMODE_SHARED, &format_)) {
    return false;
  }

  // Initialize the audio stream between the client and the device in shared
  // mode using event-driven buffer handling.
  if (FAILED(CoreAudioUtility::SharedModeInitialize(
          audio_client.Get(), &format_, audio_samples_event_,
          &endpoint_buffer_size_frames_))) {
    return false;
  }

  // Check device period and the preferred buffer size and log a warning if
  // WebRTC's buffer size is not an even divisor of the preferred buffer size
  // in Core Audio.
  // TODO(henrik): sort out if a non-perfect match really is an issue.
  REFERENCE_TIME device_period;
  if (FAILED(CoreAudioUtility::GetDevicePeriod(
          audio_client.Get(), AUDCLNT_SHAREMODE_SHARED, &device_period))) {
    return false;
  }
  const double device_period_in_seconds =
      static_cast<double>(
          CoreAudioUtility::ReferenceTimeToTimeDelta(device_period)
              .ToMilliseconds()) /
      1000.0L;
  const int preferred_frames_per_buffer =
      static_cast<int>(params.sample_rate() * device_period_in_seconds + 0.5);
  RTC_DLOG(INFO) << "preferred_frames_per_buffer: "
                 << preferred_frames_per_buffer;
  if (preferred_frames_per_buffer % params.frames_per_buffer()) {
    RTC_LOG(WARNING) << "Buffer size of " << params.frames_per_buffer()
                     << " is not an even divisor of "
                     << preferred_frames_per_buffer;
  }

  // Store valid COM interfaces.
  audio_client_ = audio_client;

  return true;
}

bool CoreAudioBase::Start() {
  RTC_DLOG(INFO) << __FUNCTION__ << "[" << DirectionToString(direction())
                 << "]";
  return true;
}

bool CoreAudioBase::Stop() {
  RTC_DLOG(INFO) << __FUNCTION__ << "[" << DirectionToString(direction())
                 << "]";
  return true;
}

}  // namespace win_adm

}  // namespace webrtc
