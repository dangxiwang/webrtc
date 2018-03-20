/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "sdk/android/src/jni/audio_device/audio_manager.h"

#include <utility>

#include "rtc_base/arraysize.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/platform_thread.h"
#include "sdk/android/generated_audio_jni/jni/WebRtcAudioManager_jni.h"
#include "sdk/android/src/jni/audio_device/audio_common.h"
#include "sdk/android/src/jni/jni_helpers.h"

namespace webrtc {

namespace android_adm {

// AudioManager::JavaAudioManager implementation
AudioManager::JavaAudioManager::JavaAudioManager(
    const ScopedJavaLocalRef<jobject>& audio_manager)
    : env_(audio_manager.env()), audio_manager_(audio_manager) {
  RTC_LOG(INFO) << "JavaAudioManager::ctor";
}

AudioManager::JavaAudioManager::~JavaAudioManager() {
  RTC_LOG(INFO) << "JavaAudioManager::~dtor";
}

bool AudioManager::JavaAudioManager::Init() {
  thread_checker_.CalledOnValidThread();
  return Java_WebRtcAudioManager_init(env_, audio_manager_);
}

void AudioManager::JavaAudioManager::Close() {
  thread_checker_.CalledOnValidThread();
  Java_WebRtcAudioManager_dispose(env_, audio_manager_);
}

bool AudioManager::JavaAudioManager::IsCommunicationModeEnabled() {
  thread_checker_.CalledOnValidThread();
  return Java_WebRtcAudioManager_isCommunicationModeEnabled(env_,
                                                            audio_manager_);
}

bool AudioManager::JavaAudioManager::IsDeviceBlacklistedForOpenSLESUsage() {
  thread_checker_.CalledOnValidThread();
  return Java_WebRtcAudioManager_isDeviceBlacklistedForOpenSLESUsage(
      env_, audio_manager_);
}

// AudioManager implementation
AudioManager::AudioManager()
    : audio_layer_(AudioDeviceModule::kPlatformDefaultAudio),
      initialized_(false),
      hardware_aec_(false),
      hardware_agc_(false),
      hardware_ns_(false),
      low_latency_playout_(false),
      low_latency_record_(false),
      delay_estimate_in_milliseconds_(0) {
  RTC_LOG(INFO) << "ctor";
  j_audio_manager_.reset(
      new JavaAudioManager(Java_WebRtcAudioManager_Constructor(
          AttachCurrentThreadIfNeeded(), jni::jlongFromPointer(this))));
}

AudioManager::~AudioManager() {
  RTC_LOG(INFO) << "dtor";
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  Close();
}

void AudioManager::SetActiveAudioLayer(
    AudioDeviceModule::AudioLayer audio_layer) {
  RTC_LOG(INFO) << "SetActiveAudioLayer: " << audio_layer;
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  RTC_DCHECK(!initialized_);
  // Store the currently utilized audio layer.
  audio_layer_ = audio_layer;
  // The delay estimate can take one of two fixed values depending on if the
  // device supports low-latency output or not. However, it is also possible
  // that the user explicitly selects the high-latency audio path, hence we use
  // the selected |audio_layer| here to set the delay estimate.
  delay_estimate_in_milliseconds_ =
      (audio_layer == AudioDeviceModule::kAndroidJavaAudio)
          ? kHighLatencyModeDelayEstimateInMilliseconds
          : kLowLatencyModeDelayEstimateInMilliseconds;
  RTC_LOG(INFO) << "delay_estimate_in_milliseconds: "
                << delay_estimate_in_milliseconds_;
}

SLObjectItf AudioManager::GetOpenSLEngine() {
  RTC_LOG(INFO) << "GetOpenSLEngine";
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  // Only allow usage of OpenSL ES if such an audio layer has been specified.
  if (audio_layer_ != AudioDeviceModule::kAndroidOpenSLESAudio &&
      audio_layer_ !=
          AudioDeviceModule::kAndroidJavaInputAndOpenSLESOutputAudio) {
    RTC_LOG(INFO)
        << "Unable to create OpenSL engine for the current audio layer: "
        << audio_layer_;
    return nullptr;
  }
  // OpenSL ES for Android only supports a single engine per application.
  // If one already has been created, return existing object instead of
  // creating a new.
  if (engine_object_.Get() != nullptr) {
    RTC_LOG(WARNING) << "The OpenSL ES engine object has already been created";
    return engine_object_.Get();
  }
  // Create the engine object in thread safe mode.
  const SLEngineOption option[] = {
      {SL_ENGINEOPTION_THREADSAFE, static_cast<SLuint32>(SL_BOOLEAN_TRUE)}};
  SLresult result =
      slCreateEngine(engine_object_.Receive(), 1, option, 0, NULL, NULL);
  if (result != SL_RESULT_SUCCESS) {
    RTC_LOG(LS_ERROR) << "slCreateEngine() failed: "
                      << GetSLErrorString(result);
    engine_object_.Reset();
    return nullptr;
  }
  // Realize the SL Engine in synchronous mode.
  result = engine_object_->Realize(engine_object_.Get(), SL_BOOLEAN_FALSE);
  if (result != SL_RESULT_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Realize() failed: " << GetSLErrorString(result);
    engine_object_.Reset();
    return nullptr;
  }
  // Finally return the SLObjectItf interface of the engine object.
  return engine_object_.Get();
}

bool AudioManager::Init() {
  RTC_LOG(INFO) << "Init";
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  RTC_DCHECK(!initialized_);
  RTC_DCHECK_NE(audio_layer_, AudioDeviceModule::kPlatformDefaultAudio);
  if (!j_audio_manager_->Init()) {
    RTC_LOG(LS_ERROR) << "Init() failed";
    return false;
  }
  initialized_ = true;
  return true;
}

bool AudioManager::Close() {
  RTC_LOG(INFO) << "Close";
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  if (!initialized_)
    return true;
  j_audio_manager_->Close();
  initialized_ = false;
  return true;
}

bool AudioManager::IsCommunicationModeEnabled() const {
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  return j_audio_manager_->IsCommunicationModeEnabled();
}

bool AudioManager::IsAcousticEchoCancelerSupported() const {
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  return hardware_aec_;
}

bool AudioManager::IsAutomaticGainControlSupported() const {
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  return hardware_agc_;
}

bool AudioManager::IsNoiseSuppressorSupported() const {
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  return hardware_ns_;
}

bool AudioManager::IsLowLatencyPlayoutSupported() const {
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  // Some devices are blacklisted for usage of OpenSL ES even if they report
  // that low-latency playout is supported. See b/21485703 for details.
  return j_audio_manager_->IsDeviceBlacklistedForOpenSLESUsage()
             ? false
             : low_latency_playout_;
}

bool AudioManager::IsLowLatencyRecordSupported() const {
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  return low_latency_record_;
}

bool AudioManager::IsProAudioSupported() const {
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  // TODO(henrika): return the state independently of if OpenSL ES is
  // blacklisted or not for now. We could use the same approach as in
  // IsLowLatencyPlayoutSupported() but I can't see the need for it yet.
  return pro_audio_;
}

// TODO(henrika): improve comments...
bool AudioManager::IsAAudioSupported() const {
#if defined(AUDIO_DEVICE_INCLUDE_ANDROID_AAUDIO)
  return a_audio_;
#else
  return false;
#endif
}

bool AudioManager::IsStereoPlayoutSupported() const {
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  return (playout_parameters_.channels() == 2);
}

bool AudioManager::IsStereoRecordSupported() const {
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  return (record_parameters_.channels() == 2);
}

int AudioManager::GetDelayEstimateInMilliseconds() const {
  return delay_estimate_in_milliseconds_;
}

void AudioManager::CacheAudioParameters(JNIEnv* env,
                                        const JavaParamRef<jobject>& j_caller,
                                        jint sample_rate,
                                        jint output_channels,
                                        jint input_channels,
                                        jboolean hardware_aec,
                                        jboolean hardware_agc,
                                        jboolean hardware_ns,
                                        jboolean low_latency_output,
                                        jboolean low_latency_input,
                                        jboolean pro_audio,
                                        jboolean a_audio,
                                        jint output_buffer_size,
                                        jint input_buffer_size) {
  RTC_LOG(INFO)
      << "OnCacheAudioParameters: "
      << "hardware_aec: " << static_cast<bool>(hardware_aec)
      << ", hardware_agc: " << static_cast<bool>(hardware_agc)
      << ", hardware_ns: " << static_cast<bool>(hardware_ns)
      << ", low_latency_output: " << static_cast<bool>(low_latency_output)
      << ", low_latency_input: " << static_cast<bool>(low_latency_input)
      << ", pro_audio: " << static_cast<bool>(pro_audio)
      << ", a_audio: " << static_cast<bool>(a_audio)
      << ", sample_rate: " << static_cast<int>(sample_rate)
      << ", output_channels: " << static_cast<int>(output_channels)
      << ", input_channels: " << static_cast<int>(input_channels)
      << ", output_buffer_size: " << static_cast<int>(output_buffer_size)
      << ", input_buffer_size: " << static_cast<int>(input_buffer_size);
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  hardware_aec_ = hardware_aec;
  hardware_agc_ = hardware_agc;
  hardware_ns_ = hardware_ns;
  low_latency_playout_ = low_latency_output;
  low_latency_record_ = low_latency_input;
  pro_audio_ = pro_audio;
  a_audio_ = a_audio;
  playout_parameters_.reset(sample_rate, static_cast<size_t>(output_channels),
                            static_cast<size_t>(output_buffer_size));
  record_parameters_.reset(sample_rate, static_cast<size_t>(input_channels),
                           static_cast<size_t>(input_buffer_size));
}

const AudioParameters& AudioManager::GetPlayoutAudioParameters() {
  RTC_CHECK(playout_parameters_.is_valid());
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  return playout_parameters_;
}

const AudioParameters& AudioManager::GetRecordAudioParameters() {
  RTC_CHECK(record_parameters_.is_valid());
  RTC_DCHECK(thread_checker_.CalledOnValidThread());
  return record_parameters_;
}

}  // namespace android_adm

}  // namespace webrtc
