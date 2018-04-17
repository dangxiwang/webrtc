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

#include "rtc_base/arraysize.h"
#include "rtc_base/logging.h"
#include "rtc_base/stringutils.h"
// TODO(henrika): to be removed
#include "system_wrappers/include/sleep.h"

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

/*
int NumberOfInputDevices() {
  return CoreAudioUtility::NumberOfActiveDevices(eCapture);
}*/

void RunOutput(void* obj) {
  RTC_DCHECK(obj);
  AudioDeviceCoreAudio* adm = reinterpret_cast<AudioDeviceCoreAudio*>(obj);
  adm->RunRenderThread();
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

  // Event to be be set in Stop() when capturing shall stop.
  stop_render_event_.Set(CreateEvent(nullptr, false, false, nullptr));
  RTC_DCHECK(stop_render_event_.IsValid());
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
  if (playing_) {
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

  // Create an IAudioRenderClient client for an initialized IAudioClient.
  // The IAudioRenderClient interface enables us to write output data to
  // a rendering endpoint buffer.
  ComPtr<IAudioRenderClient> audio_render_client =
      CoreAudioUtility::CreateRenderClient(audio_client.Get());
  if (!audio_render_client.Get())
    return -1;

  // Store valid COM interfaces.
  audio_output_client_ = audio_client;
  audio_render_client_ = audio_render_client;

  playout_is_initialized_ = true;

  return 0;
}

bool AudioDeviceCoreAudio::PlayoutIsInitialized() const {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  return playout_is_initialized_;
}

int32_t AudioDeviceCoreAudio::StartPlayout() {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DLOG(INFO) << "StartPlayout";
  if (!playout_is_initialized_) {
    return -1;
  }
  if (playing_) {
    return 0;
  }

  // Fill up with silence here...

  render_thread_.reset(new rtc::PlatformThread(
      RunOutput, this, "wasapi_render_thread", rtc::kRealtimePriority));
  render_thread_->Start();
  if (!render_thread_->IsRunning()) {
    StopRenderThread();
    RTC_LOG(LS_ERROR) << "Failed to start rendering thread";
    return -1;
  }
  RTC_DLOG(INFO) << "Started thread with name: " << render_thread_->name();

  // Start render client here...

  playing_ = true;
  return 0;
}

int32_t AudioDeviceCoreAudio::StopPlayout() {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DLOG(INFO) << "StopPlayout";
  if (!playout_is_initialized_) {
    return 0;
  }
  if (!render_thread_) {
    return 0;
  }

  StopRenderThread();

  // TODO(henrika): reset audio output client...

  playout_is_initialized_ = false;
  playing_ = false;
  return 0;
}

bool AudioDeviceCoreAudio::Playing() const {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  return playing_;
}

int32_t AudioDeviceCoreAudio::SetRecordingDevice(uint16_t index) {
  RTC_DCHECK_RUN_ON(&main_thread_checker_);
  RTC_DLOG(INFO) << "SetRecordingDevice: " << index;
  if (recording_is_initialized_) {
    return -1;
  }
  return 0;
}

// TODO(henrika): improve.
int32_t AudioDeviceCoreAudio::StereoPlayoutIsAvailable(bool& available) {
  available = true;
  return 0;
}

void AudioDeviceCoreAudio::RunRenderThread() {
  RTC_DLOG(INFO) << "RunRenderThread starts...";
  ScopedCOMInitializer com_initializer(ScopedCOMInitializer::kMTA);

  bool playing = true;
  bool error = false;
  HANDLE wait_array[] = {stop_render_event_.Get(),
                         audio_samples_render_event_.Get()};

  // TODO(henrika): get device frequency here....

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
        // error = !RenderAudioFromSource(device_frequency);
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
    // time Start() is called.
    ResetEvent(stop_render_event_.Get());
  }

  // TODO(henrika): clear callback?
}

}  // namespace win
}  // namespace webrtc
