/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/audio/audio_frame.h"

#include "rtc_base/checks.h"
#include "rtc_base/timeutils.h"

namespace webrtc {

AudioFrame::AudioFrame() {
  // Visual Studio doesn't like this in the class definition.
  static_assert(sizeof(data_) == kMaxDataSizeBytes, "kMaxDataSizeBytes");
}

void AudioFrame::Reset() {
  ResetWithoutMuting();
  // It is a requirement that the buffer be reinitialized.
  muted_ = kUninitialized;
  // Should this ^^^^ be done in ResetWithoutMuting or here?
  // Should Mute() be called here?
}

void AudioFrame::ResetWithoutMuting() {
  // TODO(wu): Zero is a valid value for |timestamp_|. We should initialize
  // to an invalid value, or add a new member to indicate invalidity.
  timestamp_ = 0;
  elapsed_time_ms_ = -1;
  ntp_time_ms_ = -1;
  samples_per_channel_ = 0;
  sample_rate_hz_ = 0;
  num_channels_ = 0;
  speech_type_ = kUndefined;
  vad_activity_ = kVadUnknown;
  profile_timestamp_ms_ = 0;
}

void AudioFrame::UpdateFrame(uint32_t timestamp,
                             const int16_t* data,
                             size_t samples_per_channel,
                             int sample_rate_hz,
                             SpeechType speech_type,
                             VADActivity vad_activity,
                             size_t num_channels) {
  timestamp_ = timestamp;
  samples_per_channel_ = samples_per_channel;
  sample_rate_hz_ = sample_rate_hz;
  speech_type_ = speech_type;
  vad_activity_ = vad_activity;
  num_channels_ = num_channels;

  const size_t length = samples_per_channel * num_channels;
  RTC_CHECK_LE(length, kMaxDataSizeSamples);
  if (data != nullptr) {
    memcpy(data_, data, sizeof(int16_t) * length);
    muted_ = kNotMuted;
  } else {
    Mute();
  }
}

void AudioFrame::CopyFrom(const AudioFrame& src) {
  if (this == &src) return;

  timestamp_ = src.timestamp_;
  elapsed_time_ms_ = src.elapsed_time_ms_;
  ntp_time_ms_ = src.ntp_time_ms_;
  samples_per_channel_ = src.samples_per_channel_;
  sample_rate_hz_ = src.sample_rate_hz_;
  speech_type_ = src.speech_type_;
  vad_activity_ = src.vad_activity_;
  num_channels_ = src.num_channels_;

  const size_t length = samples_per_channel_ * num_channels_;
  RTC_CHECK_LE(length, kMaxDataSizeSamples);
  if (src.muted_ == kNotMuted) {
    memcpy(data_, src.data(), sizeof(int16_t) * length);
    muted_ = kNotMuted;
  } else if (src.muted_ == kMuted) {
    Mute();
  } else {
    RTC_DCHECK_EQ(kUninitialized, src.muted_);
    muted_ = kUninitialized;
  }
}

void AudioFrame::UpdateProfileTimeStamp() {
  profile_timestamp_ms_ = rtc::TimeMillis();
}

int64_t AudioFrame::ElapsedProfileTimeMs() const {
  if (profile_timestamp_ms_ == 0) {
    // Profiling has not been activated.
    return -1;
  }
  return rtc::TimeSince(profile_timestamp_ms_);
}

const int16_t* AudioFrame::data() const {
  RTC_DCHECK_NE(kUninitialized, muted_);
  return data_;
}

int16_t* AudioFrame::mutable_data() {
  if (muted_ == kUninitialized) {
    Mute();  // Zero out the buffer on first call.
    muted_ = kNotMuted;  // Now mark as not muted.
  }
  return data_;
}

void AudioFrame::Mute() {
  if (muted_ == kMuted)
    return;

  muted_ = kMuted;
  memset(data_, 0, kMaxDataSizeBytes);
}

bool AudioFrame::muted() const {
  // Note that we allow querying the muted_ state even if it might be
  // kUninitialized, which here is equivalent to kMuted.
  // The important thing to guard is that the data itself is never read when the
  // muted state is kUninitialized.
  return muted_ != kNotMuted;
}

AudioFrame& AudioFrame::operator>>=(const int rhs) {
  RTC_CHECK_GT(num_channels_, 0);
  RTC_CHECK_LT(num_channels_, 3);
  if ((num_channels_ > 2) || (num_channels_ < 1)) return *this;
  if (muted())
    return *this;

  for (size_t i = 0; i < samples_per_channel_ * num_channels_; i++) {
    data_[i] = static_cast<int16_t>(data_[i] >> rhs);
  }
  return *this;
}

AudioFrame& AudioFrame::operator+=(const AudioFrame& rhs) {
  // Sanity check
  RTC_CHECK_GT(num_channels_, 0);
  RTC_CHECK_LT(num_channels_, 3);
  if ((num_channels_ > 2) || (num_channels_ < 1)) return *this;
  if (num_channels_ != rhs.num_channels_) return *this;

  bool noPrevData = (muted_ != kNotMuted);
  if (samples_per_channel_ != rhs.samples_per_channel_) {
    if (samples_per_channel_ == 0) {
      // special case we have no data to start with
      samples_per_channel_ = rhs.samples_per_channel_;
      noPrevData = true;
    } else {
      return *this;
    }
  }

  if ((vad_activity_ == kVadActive) || rhs.vad_activity_ == kVadActive) {
    vad_activity_ = kVadActive;
  } else if (vad_activity_ == kVadUnknown || rhs.vad_activity_ == kVadUnknown) {
    vad_activity_ = kVadUnknown;
  }

  if (speech_type_ != rhs.speech_type_) speech_type_ = kUndefined;

  if (!rhs.muted()) {
    muted_ = kNotMuted;
    if (noPrevData) {
      memcpy(data_, rhs.data(),
             sizeof(int16_t) * rhs.samples_per_channel_ * num_channels_);
    } else {
      // IMPROVEMENT this can be done very fast in assembly
      for (size_t i = 0; i < samples_per_channel_ * num_channels_; i++) {
        int32_t wrap_guard =
            static_cast<int32_t>(data_[i]) + static_cast<int32_t>(rhs.data_[i]);
        data_[i] = rtc::saturated_cast<int16_t>(wrap_guard);
      }
    }
  }

  return *this;
}

}  // namespace webrtc
