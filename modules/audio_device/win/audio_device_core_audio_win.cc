/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_device/win/audio_device_core_audio_win.h"

#include "api/array_view.h"
#include "api/units/time_delta.h"
#include "rtc_base/arraysize.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/safe_conversions.h"
#include "rtc_base/ptr_util.h"
#include "rtc_base/stringutils.h"
#include "rtc_base/zero_memory.h"
// TODO(henrika): to be removed
// #include "system_wrappers/include/sleep.h"

using Microsoft::WRL::ComPtr;

namespace webrtc {
namespace win {

namespace {

enum DefaultDeviceType {
  kDefault = 0,
  kDefaultCommunications = (kDefault + 1),
  kDefaultDeviceTypeMaxCount = (kDefaultCommunications + 1),
};

// Returns number of active audio output devices.
int NumberOfActiveOutputDevices() {
  return CoreAudioUtility::NumberOfActiveDevices(eRender);
}

// Returns total number of enumerated audio output devices which is the sum
// of all active devices plus two extra (one default and one default
// communications).
int NumberOfEnumeratedOutputDevices() {
  return NumberOfActiveOutputDevices() + kDefaultDeviceTypeMaxCount;
}

// Returns number of active audio input devices.
int NumberOfActiveInputDevices() {
  return CoreAudioUtility::NumberOfActiveDevices(eCapture);
}

// Returns total number of enumerated audio input devices which is the sum
// of all active devices plus two extra (one default and one default
// communications).
int NumberOfEnumeratedInputDevices() {
  return NumberOfActiveInputDevices() + kDefaultDeviceTypeMaxCount;
}

void RunOutput(void* obj) {
  RTC_DCHECK(obj);
  AudioDeviceCoreAudio* adm = reinterpret_cast<AudioDeviceCoreAudio*>(obj);
  adm->RunRenderThread();
}

void RunInput(void* obj) {
  RTC_DCHECK(obj);
  AudioDeviceCoreAudio* adm = reinterpret_cast<AudioDeviceCoreAudio*>(obj);
  adm->RunCaptureThread();
}

}  // namespace

AudioDeviceCoreAudio::AudioDeviceCoreAudio()
    : com_initializer_(new ScopedCOMInitializer(ScopedCOMInitializer::kMTA)),
      output_format_() {
  RTC_DLOG(INFO) << "AudioDeviceCoreAudio()";
  RTC_DCHECK(com_initializer_->Succeeded());

  // Create the event which the audio engine will signal each time an output
  // a buffer becomes ready to be processed by the client.
  audio_samples_render_event_.Set(CreateEvent(nullptr, false, false, nullptr));
  RTC_DCHECK(audio_samples_render_event_.IsValid());

  // Event to be be set in Stop() when rendering shall stop.
  stop_render_event_.Set(CreateEvent(nullptr, false, false, nullptr));
  RTC_DCHECK(stop_render_event_.IsValid());

  // Create the event which the audio engine will signal each time an input
  // a buffer becomes ready to be processed by the client.
  audio_samples_capture_event_.Set(CreateEvent(nullptr, false, false, nullptr));
  RTC_DCHECK(audio_samples_capture_event_.IsValid());

  // Event to be be set in Stop() when capturing shall stop.
  stop_capture_event_.Set(CreateEvent(nullptr, false, false, nullptr));
  RTC_DCHECK(stop_capture_event_.IsValid());
}

AudioDeviceCoreAudio::~AudioDeviceCoreAudio() {
  RTC_DLOG(INFO) << "~AudioDeviceCoreAudio()";
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
}

int32_t AudioDeviceCoreAudio::ActiveAudioLayer(
    AudioDeviceModule::AudioLayer& audioLayer) const {
  audioLayer = AudioDeviceModule::kWindowsCoreAudio2;
  return 0;
}

void AudioDeviceCoreAudio::AttachAudioBuffer(AudioDeviceBuffer* audio_buffer) {
  RTC_DLOG(INFO) << "AttachAudioBuffer";
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  audio_device_buffer_ = audio_buffer;
}

AudioDeviceGeneric::InitStatus AudioDeviceCoreAudio::Init() {
  RTC_DLOG(INFO) << "Init";
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  if (initialized_) {
    return InitStatus::OK;
  }

  // TODO(henrika): add more here...

  initialized_ = true;
  return InitStatus::OK;
}

int32_t AudioDeviceCoreAudio::Terminate() {
  RTC_DLOG(INFO) << "Terminate";
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  if (!initialized_) {
    return 0;
  }
  StopPlayout();
  initialized_ = false;
  return 0;
}

bool AudioDeviceCoreAudio::Initialized() const {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  return initialized_;
}

int16_t AudioDeviceCoreAudio::PlayoutDevices() {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  return CoreAudioUtility::NumberOfActiveDevices(eRender);
}

int16_t AudioDeviceCoreAudio::RecordingDevices() {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  return CoreAudioUtility::NumberOfActiveDevices(eCapture);
}

int32_t AudioDeviceCoreAudio::PlayoutDeviceName(
    uint16_t index,
    char name[kAdmMaxDeviceNameSize],
    char guid[kAdmMaxGuidSize]) {
  RTC_DLOG(INFO) << "PlayoutDeviceName: " << index;
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DCHECK(name);

  if (index > NumberOfEnumeratedOutputDevices() - 1) {
    RTC_LOG(LS_ERROR) << "Invalid device index";
    return -1;
  }

  AudioDeviceNames output_device_names;
  if (!CoreAudioUtility::GetOutputDeviceNames(&output_device_names)) {
    RTC_LOG(LS_ERROR) << "Output device enumeration failed";
    return -1;
  }

  rtc::strcpyn(reinterpret_cast<char*>(name), kAdmMaxDeviceNameSize,
               output_device_names[index].device_name.c_str());
  if (guid != nullptr) {
    rtc::strcpyn(reinterpret_cast<char*>(guid), kAdmMaxGuidSize,
                 output_device_names[index].unique_id.c_str());
  }
  return 0;
}

int32_t AudioDeviceCoreAudio::SetPlayoutDevice(uint16_t index) {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DLOG(INFO) << "SetPlayoutDevice: " << index;
  if (playout_is_initialized_) {
    return -1;
  }

  if (index > NumberOfEnumeratedOutputDevices() - 1) {
    RTC_LOG(LS_ERROR) << "Invalid device index";
    return -1;
  }

  std::string device_id;
  if (index == kDefault) {
    device_id = CoreAudioUtility::GetDefaultOutputDeviceID();
  } else if (index == kDefaultCommunications) {
    device_id = CoreAudioUtility::GetCommunicationsOutputDeviceID();
  } else {
    AudioDeviceNames output_device_names;
    if (!CoreAudioUtility::GetOutputDeviceNames(&output_device_names)) {
      RTC_LOG(LS_ERROR) << "Output device enumeration failed";
      return -1;
    }
    device_id = output_device_names[index].unique_id;
  }
  RTC_DLOG(INFO) << "index=" << index << " => device_id: " << device_id;

  output_device_id_ = device_id;
  return output_device_id_.empty() ? -1 : 0;
}

int32_t AudioDeviceCoreAudio::SetPlayoutDevice(
    AudioDeviceModule::WindowsDeviceType device) {
  return SetPlayoutDevice((device == AudioDeviceModule::kDefaultDevice) ? 0
                                                                        : 1);
}

int32_t AudioDeviceCoreAudio::SetRecordingDevice(uint16_t index) {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DLOG(INFO) << "SetRecordingDevice: " << index;
  if (recording_is_initialized_) {
    return -1;
  }

  if (index > NumberOfEnumeratedInputDevices() - 1) {
    RTC_LOG(LS_ERROR) << "Invalid device index";
    return -1;
  }

  std::string device_id;
  if (index == kDefault) {
    device_id = CoreAudioUtility::GetDefaultInputDeviceID();
  } else if (index == kDefaultCommunications) {
    device_id = CoreAudioUtility::GetCommunicationsInputDeviceID();
  } else {
    AudioDeviceNames input_device_names;
    if (!CoreAudioUtility::GetInputDeviceNames(&input_device_names)) {
      RTC_LOG(LS_ERROR) << "Input device enumeration failed";
      return -1;
    }
    device_id = input_device_names[index].unique_id;
  }
  RTC_DLOG(INFO) << "index=" << index << " => device_id: " << device_id;

  input_device_id_ = device_id;
  return input_device_id_.empty() ? -1 : 0;
}

int32_t AudioDeviceCoreAudio::SetRecordingDevice(
    AudioDeviceModule::WindowsDeviceType device) {
  return SetRecordingDevice((device == AudioDeviceModule::kDefaultDevice) ? 0
                                                                          : 1);
}

int32_t AudioDeviceCoreAudio::InitPlayout() {
  RTC_DLOG(INFO) << "InitPlayout";
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DCHECK(!output_device_id_.empty());

  // TODO(henrika): is there really a need for support of restart without
  // closing?
  // RTC_DCHECK(!audio_output_client_.Get());

  if (playout_is_initialized_) {
    return 0;
  }
  if (Playing()) {
    return -1;
  }

  // Use the |output_device_id_| and set parameters which are required to
  // create an audio client.
  // TODO(henrika): improve device notification.
  std::string device_id = output_device_id_;
  EDataFlow data_flow = eRender;
  ERole role = eConsole;
  if (output_device_id_ == CoreAudioUtility::GetDefaultOutputDeviceID()) {
    device_id = AudioDeviceName::kDefaultDeviceId;
    data_flow = eRender;
    role = eConsole;
  } else if (output_device_id_ ==
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
    return -1;
  }

  // Retrieve preferred audio output parameters.
  AudioParameters params;
  if (FAILED(CoreAudioUtility::GetPreferredAudioParameters(audio_client.Get(),
                                                           &params))) {
    return -1;
  }

  // Define the output WAVEFORMATEXTENSIBLE format in |output_format_|.
  WAVEFORMATEX* format = &output_format_.Format;
  format->wFormatTag = WAVE_FORMAT_EXTENSIBLE;
  format->nChannels = params.channels();
  format->nSamplesPerSec = params.sample_rate();
  format->wBitsPerSample = params.bits_per_sample();
  format->nBlockAlign = (format->wBitsPerSample / 8) * format->nChannels;
  format->nAvgBytesPerSec = format->nSamplesPerSec * format->nBlockAlign;
  format->cbSize = sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX);
  // Add the parts which are unique for the WAVE_FORMAT_EXTENSIBLE structure.
  output_format_.Samples.wValidBitsPerSample = params.bits_per_sample();
  // TODO(henrika): improve.
  output_format_.dwChannelMask = params.channels() == 1
                                     ? SPEAKER_FRONT_CENTER
                                     : SPEAKER_FRONT_LEFT | SPEAKER_FRONT_RIGHT;
  output_format_.SubFormat = KSDATAFORMAT_SUBTYPE_PCM;
  RTC_DLOG(INFO) << CoreAudioUtility::WaveFormatExToString(&output_format_);

  // Verify that the format is supported.
  if (!CoreAudioUtility::IsFormatSupported(
          audio_client.Get(), AUDCLNT_SHAREMODE_SHARED, &output_format_)) {
    return -1;
  }

  // Initialize the audio stream between the client and the device in shared
  // mode using event-driven buffer handling.
  if (FAILED(CoreAudioUtility::SharedModeInitialize(
          audio_client.Get(), &output_format_, audio_samples_render_event_,
          &output_endpoint_buffer_size_frames_))) {
    return -1;
  }

  // Set AudioDeviceBuffer parameters.
  if (audio_device_buffer_) {
    audio_device_buffer_->SetPlayoutSampleRate(params.sample_rate());
    audio_device_buffer_->SetPlayoutChannels(params.channels());
  }

  // Create a modified audio buffer class which allows us to ask for any number
  // of samples (and not only multiple of 10ms) to match the optimal buffer
  // size per callback used by Core Audio.
  fine_audio_output_buffer_ =
      rtc::MakeUnique<FineAudioBuffer>(audio_device_buffer_);

  // Check device period and the preferred buffer size and log a warning if
  // WebRTC's buffer size is not an even divisor of the preferred buffer size
  // in Core Audio.
  // TODO(henrik): sort out if a non-perfect match really is an issue.
  REFERENCE_TIME device_period;
  if (FAILED(CoreAudioUtility::GetDevicePeriod(
          audio_client.Get(), AUDCLNT_SHAREMODE_SHARED, &device_period))) {
    return -1;
  }
  const double device_period_in_seconds =
      static_cast<double>(
          CoreAudioUtility::ReferenceTimeToTimeDelta(device_period).ms()) /
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

  // Create an IAudioRenderClient client for an initialized IAudioClient.
  // The IAudioRenderClient interface enables us to write output data to
  // a rendering endpoint buffer.
  ComPtr<IAudioRenderClient> audio_render_client =
      CoreAudioUtility::CreateRenderClient(audio_client.Get());
  if (!audio_render_client.Get())
    return -1;

  ComPtr<IAudioClock> audio_clock =
      CoreAudioUtility::CreateAudioClock(audio_client.Get());
  if (!audio_clock.Get())
    return -1;

  // Store valid COM interfaces.
  audio_output_client_ = audio_client;
  audio_render_client_ = audio_render_client;
  audio_output_clock_ = audio_clock;

  playout_is_initialized_ = true;

  return 0;
}

bool AudioDeviceCoreAudio::PlayoutIsInitialized() const {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  return playout_is_initialized_;
}

int32_t AudioDeviceCoreAudio::InitRecording() {
  RTC_DLOG(INFO) << "InitRecording";
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DCHECK(!input_device_id_.empty());

  // TODO(henrika): is there really a need for support of restart without
  // closing?
  // RTC_DCHECK(!audio_input_client_.Get());

  if (recording_is_initialized_) {
    return 0;
  }
  if (Recording()) {
    return -1;
  }

  // Use the |input_device_id_| and set parameters which are required to
  // create an audio client.
  // TODO(henrika): improve device notification.
  std::string device_id = input_device_id_;
  EDataFlow data_flow = eCapture;
  ERole role = eConsole;
  if (input_device_id_ == CoreAudioUtility::GetDefaultInputDeviceID()) {
    device_id = AudioDeviceName::kDefaultDeviceId;
    data_flow = eCapture;
    role = eConsole;
  } else if (input_device_id_ ==
             CoreAudioUtility::GetCommunicationsInputDeviceID()) {
    device_id = AudioDeviceName::kDefaultCommunicationsDeviceId;
    data_flow = eCapture;
    role = eCommunications;
  }

  // Create an IAudioClient interface which enables us to create and initialize
  // an audio stream between an audio application and the audio engine.
  ComPtr<IAudioClient> audio_client =
      CoreAudioUtility::CreateClient(device_id, data_flow, role);
  if (!audio_client.Get()) {
    return -1;
  }

  // Retrieve preferred audio input parameters.
  AudioParameters params;
  if (FAILED(CoreAudioUtility::GetPreferredAudioParameters(audio_client.Get(),
                                                           &params))) {
    return -1;
  }

  // Define the input WAVEFORMATEXTENSIBLE format in |input_format_|.
  WAVEFORMATEX* format = &input_format_.Format;
  format->wFormatTag = WAVE_FORMAT_EXTENSIBLE;
  format->nChannels = params.channels();
  format->nSamplesPerSec = params.sample_rate();
  format->wBitsPerSample = params.bits_per_sample();
  format->nBlockAlign = (format->wBitsPerSample / 8) * format->nChannels;
  format->nAvgBytesPerSec = format->nSamplesPerSec * format->nBlockAlign;
  format->cbSize = sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX);
  // Add the parts which are unique for the WAVE_FORMAT_EXTENSIBLE structure.
  input_format_.Samples.wValidBitsPerSample = params.bits_per_sample();
  // TODO(henrika): improve.
  input_format_.dwChannelMask = params.channels() == 1
                                    ? SPEAKER_FRONT_CENTER
                                    : SPEAKER_FRONT_LEFT | SPEAKER_FRONT_RIGHT;
  input_format_.SubFormat = KSDATAFORMAT_SUBTYPE_PCM;
  RTC_DLOG(INFO) << CoreAudioUtility::WaveFormatExToString(&input_format_);

  // Verify that the format is supported.
  if (!CoreAudioUtility::IsFormatSupported(
          audio_client.Get(), AUDCLNT_SHAREMODE_SHARED, &input_format_)) {
    return -1;
  }

  // Initialize the audio stream between the client and the device in shared
  // mode using event-driven buffer handling.
  if (FAILED(CoreAudioUtility::SharedModeInitialize(
          audio_client.Get(), &input_format_, audio_samples_capture_event_,
          &input_endpoint_buffer_size_frames_))) {
    return -1;
  }

  // Set AudioDeviceBuffer parameters.
  if (audio_device_buffer_) {
    audio_device_buffer_->SetRecordingSampleRate(params.sample_rate());
    audio_device_buffer_->SetRecordingChannels(params.channels());
  }

  // Create a modified audio buffer class which allows us to supply any number
  // of samples (and not only multiple of 10ms) to match the optimal buffer
  // size per callback used by Core Audio.
  // TODO(henrika): can we share one FineAudioBuffer?
  fine_audio_input_buffer_ =
      rtc::MakeUnique<FineAudioBuffer>(audio_device_buffer_);

  // Check device period and the preferred buffer size and log a warning if
  // WebRTC's buffer size is not an even divisor of the preferred buffer size
  // in Core Audio.
  // TODO(henrik): sort out if a non-perfect match really is an issue.
  REFERENCE_TIME device_period;
  if (FAILED(CoreAudioUtility::GetDevicePeriod(
          audio_client.Get(), AUDCLNT_SHAREMODE_SHARED, &device_period))) {
    return -1;
  }
  const double device_period_in_seconds =
      static_cast<double>(
          CoreAudioUtility::ReferenceTimeToTimeDelta(device_period).ms()) /
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

  // Create an IAudioCaptureClient client for an initialized IAudioClient.
  // The IAudioCaptureClient interface enables a client to read input data from
  // a capture endpoint buffer.
  ComPtr<IAudioCaptureClient> audio_capture_client =
      CoreAudioUtility::CreateCaptureClient(audio_client.Get());
  if (!audio_capture_client.Get())
    return -1;

  // Query performance frequency.
  LARGE_INTEGER ticks_per_sec = {};
  qpc_to_100ns_.reset();
  if (::QueryPerformanceFrequency(&ticks_per_sec)) {
    double qpc_ticks_per_second =
        rtc::dchecked_cast<double>(ticks_per_sec.QuadPart);
    qpc_to_100ns_ = 10000000.0 / qpc_ticks_per_second;
  }

  // Store valid COM interfaces.
  audio_input_client_ = audio_client;
  audio_capture_client_ = audio_capture_client;

  recording_is_initialized_ = true;
  return 0;
}

bool AudioDeviceCoreAudio::RecordingIsInitialized() const {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  return recording_is_initialized_;
}

int32_t AudioDeviceCoreAudio::StartPlayout() {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DLOG(INFO) << "StartPlayout";
  if (!playout_is_initialized_) {
    return -1;
  }
  if (Playing()) {
    return 0;
  }
  if (fine_audio_output_buffer_) {
    fine_audio_output_buffer_->ResetPlayout();
  }

  if (!CoreAudioUtility::FillRenderEndpointBufferWithSilence(
          audio_output_client_.Get(), audio_render_client_.Get())) {
    RTC_LOG(LS_WARNING) << "Failed to prepare output endpoint with "
                           "FillRenderEndpointBufferWithSilence";
  }

  num_output_frames_written_ = output_endpoint_buffer_size_frames_;

  render_thread_.reset(new rtc::PlatformThread(
      RunOutput, this, "wasapi_render_thread", rtc::kRealtimePriority));
  render_thread_->Start();
  if (!render_thread_->IsRunning()) {
    StopRenderThread();
    RTC_LOG(LS_ERROR) << "Failed to start rendering thread";
    return -1;
  }
  RTC_DLOG(INFO) << "Started thread with name: " << render_thread_->name();

  // Start streaming data between the endpoint buffer and the audio engine.
  _com_error error = audio_output_client_->Start();
  if (error.Error() != S_OK) {
    StopRenderThread();
    RTC_LOG(LS_ERROR) << "IAudioClient::Start failed: "
                      << CoreAudioUtility::ErrorToString(error);
    return -1;
  }

  return 0;
}

int32_t AudioDeviceCoreAudio::StopPlayout() {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DLOG(INFO) << "StopPlayout";
  if (!playout_is_initialized_) {
    return 0;
  }
  if (!Playing()) {
    return 0;
  }

  // Stop output streaming and the internal rendering thread.
  _com_error error = audio_output_client_->Stop();
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioClient::Stop failed: "
                      << CoreAudioUtility::ErrorToString(error);
  }
  StopRenderThread();

  // Flush all pending data and reset the audio clock stream position to 0.
  error = audio_output_client_->Reset();
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioClient::Reset failed: "
                      << CoreAudioUtility::ErrorToString(error);
  }

  // Extra safety check to ensure that the buffers are cleared.
  // If the buffers are not cleared correctly, the next call to Start()
  // would fail with AUDCLNT_E_BUFFER_ERROR at IAudioRenderClient::GetBuffer().
  UINT32 num_queued_frames = 0;
  audio_output_client_->GetCurrentPadding(&num_queued_frames);
  RTC_DCHECK_EQ(0u, num_queued_frames);

  playout_is_initialized_ = false;
  return 0;
}

bool AudioDeviceCoreAudio::Playing() const {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  return render_thread_ != nullptr;
}

int32_t AudioDeviceCoreAudio::StartRecording() {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DLOG(INFO) << "StartRecording";
  if (!recording_is_initialized_) {
    return -1;
  }
  if (Recording()) {
    return 0;
  }
  if (fine_audio_input_buffer_) {
    fine_audio_input_buffer_->ResetRecord();
  }

  // num_output_frames_written_ = output_endpoint_buffer_size_frames_;

  capture_thread_.reset(new rtc::PlatformThread(
      RunInput, this, "wasapi_capture_thread", rtc::kRealtimePriority));
  capture_thread_->Start();
  if (!capture_thread_->IsRunning()) {
    StopCaptureThread();
    RTC_LOG(LS_ERROR) << "Failed to start capture thread";
    return -1;
  }
  RTC_DLOG(INFO) << "Started thread with name: " << capture_thread_->name();

  // Start streaming data between the endpoint buffer and the audio engine.
  _com_error error = audio_input_client_->Start();
  if (error.Error() != S_OK) {
    StopCaptureThread();
    RTC_LOG(LS_ERROR) << "IAudioClient::Start failed: "
                      << CoreAudioUtility::ErrorToString(error);
    return -1;
  }

  return 0;
}

int32_t AudioDeviceCoreAudio::StopRecording() {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DLOG(INFO) << "StopRecording";
  if (!recording_is_initialized_) {
    return 0;
  }
  if (!Recording()) {
    return 0;
  }

  // Stop output streaming and the internal capture thread.
  _com_error error = audio_input_client_->Stop();
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioClient::Stop failed: "
                      << CoreAudioUtility::ErrorToString(error);
  }
  StopCaptureThread();

  // Flush all pending data and reset the audio clock stream position to 0.
  error = audio_input_client_->Reset();
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioClient::Reset failed: "
                      << CoreAudioUtility::ErrorToString(error);
  }

  qpc_to_100ns_.reset();
  recording_is_initialized_ = false;
  return 0;
}

bool AudioDeviceCoreAudio::Recording() const {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  return capture_thread_ != nullptr;
}

// TODO(henrika): improve.
int32_t AudioDeviceCoreAudio::StereoPlayoutIsAvailable(bool& available) {
  available = true;
  return 0;
}

int32_t AudioDeviceCoreAudio::StereoRecordingIsAvailable(bool& available) {
  available = true;
  return 0;
}

void AudioDeviceCoreAudio::RunRenderThread() {
  RTC_DLOG(INFO) << "RunRenderThread starts...";
  ScopedCOMInitializer com_initializer(ScopedCOMInitializer::kMTA);
  // TODO(henrika): difference between "Pro Audio" and "Audio"?
  ScopedMMCSSRegistration mmcss_registration(L"Pro Audio");
  RTC_DCHECK(com_initializer.Succeeded());
  RTC_DCHECK(mmcss_registration.Succeeded());
  RTC_DCHECK(stop_render_event_.IsValid());
  RTC_DCHECK(audio_samples_render_event_.IsValid());

  bool playing = true;
  bool error = false;
  HANDLE wait_array[] = {stop_render_event_.Get(),
                         audio_samples_render_event_.Get()};

  // The device frequency is the frequency generated by the hardware clock in
  // the audio device. The GetFrequency() method reports a constant frequency.
  UINT64 device_frequency = 0;
  _com_error result = audio_output_clock_->GetFrequency(&device_frequency);
  if ((error = result.Error()) != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioClock::GetFrequency failed: "
                      << CoreAudioUtility::ErrorToString(error);
  }

  // Keep rendering audio until the stop event or the stream-switch event
  // is signaled. An error event can also break the main thread loop.
  while (playing && !error) {
    // Wait for a close-down event, stream-switch event or a new render event.
    DWORD wait_result = WaitForMultipleObjects(arraysize(wait_array),
                                               wait_array, false, INFINITE);

    switch (wait_result) {
      case WAIT_OBJECT_0 + 0:
        // |stop_render_event_| has been set.
        playing = false;
        break;
      case WAIT_OBJECT_0 + 1:
        // |audio_samples_render_event_| has been set.
        error = !RenderAudioFromSource(device_frequency);
        break;
      default:
        error = true;
        break;
    }
  }

  if (playing && error) {
    RTC_LOG(LS_ERROR) << "WASAPI rendering failed.";
    // Stop audio rendering since something has gone wrong in our main thread
    // loop. Note that, we are still in a "started" state, hence a Stop() call
    // is required to join the thread properly.
    audio_output_client_->Stop();

    // TODO(henrika): notify clients that something has gone wrong and that
    // this stream should be destroyed instead of reused in the future.
  }

  RTC_DLOG(INFO) << "...RunRenderThread stops";
}

void AudioDeviceCoreAudio::StopRenderThread() {
  RTC_DLOG(INFO) << "StopRenderThread";
  if (render_thread_) {
    if (render_thread_->IsRunning()) {
      RTC_DLOG(INFO) << "Sets stop_render_event...";
      SetEvent(stop_render_event_.Get());
      RTC_DLOG(INFO) << "PlatformThread::Stop...";
      render_thread_->Stop();
    }

    render_thread_.reset();

    // Ensure that we don't quit the main thread loop immediately next
    // time StartPlayout() is called.
    ResetEvent(stop_render_event_.Get());
  }

  // TODO(henrika): clear callback?
}

void AudioDeviceCoreAudio::RunCaptureThread() {
  RTC_DLOG(INFO) << "RunCaptureThread starts...";
  ScopedCOMInitializer com_initializer(ScopedCOMInitializer::kMTA);
  // TODO(henrika): difference between "Pro Audio" and "Audio"?
  ScopedMMCSSRegistration mmcss_registration(L"Pro Audio");
  RTC_DCHECK(com_initializer.Succeeded());
  RTC_DCHECK(mmcss_registration.Succeeded());
  RTC_DCHECK(stop_capture_event_.IsValid());
  RTC_DCHECK(audio_samples_capture_event_.IsValid());

  bool recording = true;
  bool error = false;
  HANDLE wait_array[] = {stop_capture_event_.Get(),
                         audio_samples_capture_event_.Get()};

  // Keep capturing audio until the stop event or the stream-switch event
  // is signaled. An error event can also break the main thread loop.
  while (recording && !error) {
    // Wait for a close-down event, stream-switch event or a new render event.
    DWORD wait_result = WaitForMultipleObjects(arraysize(wait_array),
                                               wait_array, false, INFINITE);

    switch (wait_result) {
      case WAIT_OBJECT_0 + 0:
        // |stop_capture_event_| has been set.
        recording = false;
        break;
      case WAIT_OBJECT_0 + 1:
        // |audio_samples_capture_event_| has been set.
        error = !CaptureAudioToSink();
        break;
      default:
        error = true;
        break;
    }
  }

  if (recording && error) {
    RTC_LOG(LS_ERROR) << "WASAPI capture failed.";
    // Stop audio capturing since something has gone wrong in our main thread
    // loop. Note that, we are still in a "started" state, hence a Stop() call
    // is required to join the thread properly.
    audio_input_client_->Stop();

    // TODO(henrika): notify clients that something has gone wrong and that
    // this stream should be destroyed instead of reused in the future.
  }

  RTC_DLOG(INFO) << "...RunCaptureThread stops";
}

void AudioDeviceCoreAudio::StopCaptureThread() {
  RTC_DLOG(INFO) << "StopCaptureThread";
  if (capture_thread_) {
    if (capture_thread_->IsRunning()) {
      RTC_DLOG(INFO) << "Sets stop_capture_event...";
      SetEvent(stop_capture_event_.Get());
      RTC_DLOG(INFO) << "PlatformThread::Stop...";
      capture_thread_->Stop();
    }

    capture_thread_.reset();

    // Ensure that we don't quit the main thread loop immediately next
    // time StartRecording() is called.
    ResetEvent(stop_capture_event_.Get());
  }

  // TODO(henrika): clear callback?
}

bool AudioDeviceCoreAudio::RenderAudioFromSource(UINT64 device_frequency) {
  // RTC_DLOG(INFO) << "RenderAudioFromSource";

  // Get the padding value which indicates the amount of valid unread data that
  // the endpoint buffer currently contains.
  UINT32 num_unread_frames = 0;
  _com_error error =
      audio_output_client_->GetCurrentPadding(&num_unread_frames);
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioClient::GetCurrentPadding failed: "
                      << CoreAudioUtility::ErrorToString(error);
    return false;
  }

  // Contains how much new data we can write to the buffer without the risk of
  // overwriting previously written data that the audio engine has not yet read
  // from the buffer. I.e., it is the maximum buffer size we can request when
  // calling IAudioRenderClient::GetBuffer().
  UINT32 num_requested_frames =
      output_endpoint_buffer_size_frames_ - num_unread_frames;
  // RTC_DLOG(INFO) << "num_requested_frames: " << num_requested_frames;

  // Request all available space in the rendering endpoint buffer into which the
  // client can later write an audio packet.
  uint8_t* audio_data;
  error = audio_render_client_->GetBuffer(num_requested_frames, &audio_data);
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioRenderClient::GetBuffer failed: "
                      << CoreAudioUtility::ErrorToString(error);
    return false;
  }

  // TODO(henrika): only update the latency estimate N times per second to
  // save resources.
  // TODO(henrika): note that FineAudioBuffer adds latency as well.
  int playout_delay_ms = EstimateOutputLatencyMillis(device_frequency);
  // RTC_DLOG(INFO) << "playout_delay_ms: " << playout_delay_ms;

  // Get audio data from WebRTC and write it to the allocated buffer in
  // |audio_data|.
  fine_audio_output_buffer_->GetPlayoutData(
      rtc::MakeArrayView(
          reinterpret_cast<int16_t*>(audio_data),
          num_requested_frames * output_format_.Format.nChannels),
      playout_delay_ms);

  // Release the buffer space acquired in IAudioRenderClient::GetBuffer.
  error = audio_render_client_->ReleaseBuffer(num_requested_frames, 0);
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioRenderClient::ReleaseBuffer failed: "
                      << CoreAudioUtility::ErrorToString(error);
    return false;
  }

  num_output_frames_written_ += num_requested_frames;

  return true;
}

bool AudioDeviceCoreAudio::CaptureAudioToSink() {
  UINT32 num_frames_in_next_packet = 0;
  _com_error error =
      audio_capture_client_->GetNextPacketSize(&num_frames_in_next_packet);
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioCaptureClient::GetNextPacketSize failed: "
                      << CoreAudioUtility::ErrorToString(error);
    return false;
  }
  if (num_frames_in_next_packet == 0) {
    // No audio is recorded. Return already here to avoid allocating memory
    // by calling IAudioCaptureClient::GetBuffer.
    return true;
  }

  // Drain the WASAPI capture buffer fully.
  while (num_frames_in_next_packet > 0) {
    uint8_t* audio_data;
    UINT32 num_frames_to_read = 0;
    DWORD flags = 0;
    UINT64 device_position_frames = 0;
    UINT64 capture_time_100ns = 0;
    error = audio_capture_client_->GetBuffer(&audio_data, &num_frames_to_read,
                                             &flags, &device_position_frames,
                                             &capture_time_100ns);
    if (error.Error() == AUDCLNT_S_BUFFER_EMPTY) {
      // The call succeeded but no capture data is available to be read.
      // Return and start waiting for new capture event
      RTC_DCHECK_EQ(num_frames_to_read, 0u);
      return true;
    }
    if (error.Error() != S_OK) {
      RTC_LOG(LS_ERROR) << "IAudioCaptureClient::GetBuffer failed: "
                        << CoreAudioUtility::ErrorToString(error);
      return false;
    }

    // TODO(henrika): only update the latency estimate N times per second to
    // save resources.
    // TODO(henrika): note that FineAudioBuffer adds latency as well.
    auto opt_record_delay_ms = EstimateInputLatencyMillis(capture_time_100ns);

    // The data in the packet is not correlated with the previous packet's
    // device position; possibly due to a stream state transition or timing
    // glitch. The behavior of the AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY flag
    // is undefined on the application's first call to GetBuffer after Start.
    if (device_position_frames != 0 &&
        flags & AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY) {
      RTC_DLOG(LS_WARNING) << "AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY";
    }
    // The time at which the device's stream position was recorded is uncertain.
    // Thus, the client might be unable to accurately set a time stamp for the
    // current data packet.
    if (flags & AUDCLNT_BUFFERFLAGS_TIMESTAMP_ERROR) {
      RTC_DLOG(LS_WARNING) << "AUDCLNT_BUFFERFLAGS_TIMESTAMP_ERROR";
    }

    // Treat all of the data in the packet as silence and ignore the actual
    // data values when AUDCLNT_BUFFERFLAGS_SILENT is set.
    if (flags & AUDCLNT_BUFFERFLAGS_SILENT) {
      rtc::ExplicitZeroMemory(
          audio_data, input_format_.Format.nBlockAlign * num_frames_to_read);
      RTC_DLOG(LS_WARNING) << "Captured audio is replaced by silence";
    } else {
      // Copy recorded audio in |audio_data| to the WebRTC sink using the
      // FineAudioBuffer object.
      // TODO(henrika): fix delay estimation.
      int record_delay_ms = 0;
      if (opt_record_delay_ms) {
        record_delay_ms = *opt_record_delay_ms;
        // RTC_DLOG(INFO) << "record_delay_ms: " << record_delay_ms;
      }
      fine_audio_input_buffer_->DeliverRecordedData(
          rtc::MakeArrayView(
              reinterpret_cast<const int16_t*>(audio_data),
              input_format_.Format.nChannels * num_frames_to_read),

          record_delay_ms);
    }

    error = audio_capture_client_->ReleaseBuffer(num_frames_to_read);
    if (error.Error() != S_OK) {
      RTC_LOG(LS_ERROR) << "IAudioCaptureClient::ReleaseBuffer failed: "
                        << CoreAudioUtility::ErrorToString(error);
      return false;
    }

    error =
        audio_capture_client_->GetNextPacketSize(&num_frames_in_next_packet);
    if (error.Error() != S_OK) {
      RTC_LOG(LS_ERROR) << "IAudioCaptureClient::GetNextPacketSize failed: "
                        << CoreAudioUtility::ErrorToString(error);
      return false;
    }
  }

  return true;
}

// TODO(henrika): IAudioClock2::GetDevicePosition could perhaps be used here
// instead. Tried it once, but i crashed for capture devices.ini
int AudioDeviceCoreAudio::EstimateOutputLatencyMillis(
    uint64_t device_frequency) {
  UINT64 position = 0;
  UINT64 qpc_position = 0;
  int delay_ms = 0;
  // Get the device position through output parameter |position|. This is the
  // stream position of the sample that is currently playing through the
  // speakers.
  _com_error error = audio_output_clock_->GetPosition(&position, &qpc_position);
  if (error.Error() == S_OK) {
    // Number of frames already played out through the speaker.
    const uint64_t num_played_out_frames =
        output_format_.Format.nSamplesPerSec * position / device_frequency;

    // Number of frames that have been written to the buffer but not yet
    // played out corresponding to the estimated latency measured in number
    // of audio frames.
    const uint64_t delay_frames =
        num_output_frames_written_ - num_played_out_frames;

    // Convert latency in number of frames into milliseconds.
    webrtc::TimeDelta delay =
        webrtc::TimeDelta::us(delay_frames * win::kNumMicrosecsPerSec /
                              output_format_.Format.nSamplesPerSec);
    delay_ms = delay.ms();
  }
  return delay_ms;
}

rtc::Optional<int> AudioDeviceCoreAudio::EstimateInputLatencyMillis(
    uint64_t capture_time_100ns) {
  if (!qpc_to_100ns_) {
    return rtc::nullopt;
  }
  // Input parameter |capture_time_100ns| contains the performance counter at
  // the time that the audio endpoint device recorded the device position of
  // the first audio frame in the data packet converted into 100ns units.
  // We derive a delay estimate by:
  // - sampling the current performance counter (qpc_now_raw),
  // - converting it into 100ns time units (now_time_100ns), and
  // - subtracting |capture_time_100ns| from now_time_100ns.
  LARGE_INTEGER perf_counter_now = {};
  if (!::QueryPerformanceCounter(&perf_counter_now)) {
    return rtc::nullopt;
  }
  uint64_t qpc_now_raw = perf_counter_now.QuadPart;
  uint64_t now_time_100ns = qpc_now_raw * (*qpc_to_100ns_);
  webrtc::TimeDelta delay =
      webrtc::TimeDelta::us(0.1 * (now_time_100ns - capture_time_100ns) + 0.5);
  return delay.ms();
}

}  // namespace win
}  // namespace webrtc
