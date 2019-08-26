/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/voice_detection_impl.h"

#include "api/audio/audio_frame.h"
#include "common_audio/vad/include/webrtc_vad.h"
#include "modules/audio_processing/audio_buffer.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {
class VoiceDetectionImpl::Vad {
 public:
  Vad() {
    state_ = WebRtcVad_Create();
    RTC_CHECK(state_);
    int error = WebRtcVad_Init(state_);
    RTC_DCHECK_EQ(0, error);
  }
  ~Vad() { WebRtcVad_Free(state_); }
  VadInst* state() { return state_; }

 private:
  VadInst* state_ = nullptr;
  RTC_DISALLOW_COPY_AND_ASSIGN(Vad);
};

VoiceDetectionImpl::VoiceDetectionImpl(rtc::CriticalSection* crit)
    : crit_(crit) {
  RTC_DCHECK(crit);
}

VoiceDetectionImpl::~VoiceDetectionImpl() {}

void VoiceDetectionImpl::Initialize(int sample_rate_hz) {
  rtc::CritScope cs(crit_);
  sample_rate_hz_ = sample_rate_hz;
  std::unique_ptr<Vad> new_vad;
  if (enabled_) {
    new_vad.reset(new Vad());
  }
  vad_.swap(new_vad);
  using_external_vad_ = false;
  frame_size_samples_ =
      static_cast<size_t>(frame_size_ms_ * sample_rate_hz_) / 1000;
  set_likelihood(likelihood_);
}

bool VoiceDetectionImpl::ProcessCaptureAudio(AudioBuffer* audio) {
  rtc::CritScope cs(crit_);
  RTC_DCHECK(enabled_);

  RTC_DCHECK_GE(AudioBuffer::kMaxSplitFrameLength,
                audio->num_frames_per_band());
  std::array<int16_t, AudioBuffer::kMaxSplitFrameLength> mixed_low_pass_data;
  rtc::ArrayView<const int16_t> mixed_low_pass(mixed_low_pass_data.data(),
                                               audio->num_frames_per_band());
  if (audio->num_proc_channels() == 1) {
    FloatS16ToS16(audio->split_bands_const_f(0)[kBand0To8kHz],
                  audio->num_frames_per_band(), mixed_low_pass_data.data());
  } else {
    const int num_channels = static_cast<int>(audio->num_channels());
    for (size_t i = 0; i < audio->num_frames_per_band(); ++i) {
      int32_t value =
          FloatS16ToS16(audio->split_channels_const_f(kBand0To8kHz)[0][i]);
      for (int j = 1; j < num_channels; ++j) {
        value +=
            FloatS16ToS16(audio->split_channels_const_f(kBand0To8kHz)[j][i]);
      }
      mixed_low_pass_data[i] = value / num_channels;
    }
  }

  int vad_ret = WebRtcVad_Process(vad_->state(), sample_rate_hz_,
                                  mixed_low_pass.data(), frame_size_samples_);
  if (vad_ret == 0) {
    stream_has_voice_ = false;
    return false;
  } else if (vad_ret == 1) {
    stream_has_voice_ = true;
  } else {
    RTC_NOTREACHED();
  }

  return stream_has_voice_;
}

int VoiceDetectionImpl::Enable(bool enable) {
  rtc::CritScope cs(crit_);
  if (enabled_ != enable) {
    enabled_ = enable;
    Initialize(sample_rate_hz_);
  }
  return AudioProcessing::kNoError;
}

bool VoiceDetectionImpl::is_enabled() const {
  rtc::CritScope cs(crit_);
  return enabled_;
}

int VoiceDetectionImpl::set_stream_has_voice(bool has_voice) {
  rtc::CritScope cs(crit_);
  using_external_vad_ = true;
  stream_has_voice_ = has_voice;
  return AudioProcessing::kNoError;
}

bool VoiceDetectionImpl::stream_has_voice() const {
  rtc::CritScope cs(crit_);
  // TODO(ajm): enable this assertion?
  // RTC_DCHECK(using_external_vad_ || is_component_enabled());
  return stream_has_voice_;
}

int VoiceDetectionImpl::set_likelihood(VoiceDetection::Likelihood likelihood) {
  rtc::CritScope cs(crit_);
  likelihood_ = likelihood;
  if (enabled_) {
    int mode = 2;
    switch (likelihood) {
      case VoiceDetection::kVeryLowLikelihood:
        mode = 3;
        break;
      case VoiceDetection::kLowLikelihood:
        mode = 2;
        break;
      case VoiceDetection::kModerateLikelihood:
        mode = 1;
        break;
      case VoiceDetection::kHighLikelihood:
        mode = 0;
        break;
      default:
        RTC_NOTREACHED();
        break;
    }
    int error = WebRtcVad_set_mode(vad_->state(), mode);
    RTC_DCHECK_EQ(0, error);
  }
  return AudioProcessing::kNoError;
}

VoiceDetection::Likelihood VoiceDetectionImpl::likelihood() const {
  rtc::CritScope cs(crit_);
  return likelihood_;
}

int VoiceDetectionImpl::set_frame_size_ms(int size) {
  rtc::CritScope cs(crit_);
  RTC_DCHECK_EQ(10, size);  // TODO(ajm): remove when supported.
  frame_size_ms_ = size;
  Initialize(sample_rate_hz_);
  return AudioProcessing::kNoError;
}

int VoiceDetectionImpl::frame_size_ms() const {
  rtc::CritScope cs(crit_);
  return frame_size_ms_;
}
}  // namespace webrtc
