/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/fixed_gain_controller.h"

#include <algorithm>
#include <cmath>

#include "api/array_view.h"
#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/agc2/interpolated_gain_curve.h"
#include "modules/audio_processing/agc2/level_estimator.h"
#include "rtc_base/checks.h"

namespace webrtc {
namespace {
float DbToLinear(float x) {
  return std::pow(10.0, x / 20.0);
}
}  // namespace

int FixedGainController::instance_count_ = 0;

FixedGainController::FixedGainController(float gain_to_apply_db,
                                         size_t sample_rate_hz,
                                         bool enable_limiter)
    : gain_to_apply_(DbToLinear(gain_to_apply_db)),
      target_gain_to_apply_(gain_to_apply_),
      apm_data_dumper_(instance_count_) {
  if (enable_limiter) {
    interp_gain_curve_.emplace(&apm_data_dumper_);
    level_estimator_.emplace(sample_rate_hz, &apm_data_dumper_);
    gain_curve_applier_.emplace(&*interp_gain_curve_, &*level_estimator_,
                                &apm_data_dumper_);
  }
  RTC_DCHECK_LT(0.f, gain_to_apply_);
  ++instance_count_;
}

FixedGainController::~FixedGainController() = default;

void FixedGainController::SetGain(float gain_to_apply_db) {
  RTC_DCHECK_RUNS_SERIALIZED(&race_checker_);
  target_gain_to_apply_ = DbToLinear(gain_to_apply_db);
}

void FixedGainController::SetRate(size_t sample_rate_hz) {
  if (level_estimator_) {
    level_estimator_->SetSampleRate(sample_rate_hz);
  }
}

void FixedGainController::SetAttackMs(float duration_ms) {
  if (level_estimator_) {
    level_estimator_->SetAttackMs(duration_ms);
  }
}

void FixedGainController::SetDecayMs(float duration_ms) {
  if (level_estimator_) {
    level_estimator_->SetDecayMs(duration_ms);
  }
}

void FixedGainController::Process(MutableFloatAudioFrame signal) {
  RTC_DCHECK_RUNS_SERIALIZED(&race_checker_);
  // Apply fixed digital gain; interpolate if necessary.
  if (gain_to_apply_ != 1.f && gain_to_apply_ == target_gain_to_apply_) {
    for (size_t k = 0; k < signal.num_channels(); ++k) {
      rtc::ArrayView<float> channel_view = signal.channel(k);
      for (auto& sample : channel_view) {
        sample *= gain_to_apply_;
      }
    }
  } else if (gain_to_apply_ != target_gain_to_apply_) {
    const float gain_delta =
        (target_gain_to_apply_ - gain_to_apply_) / signal.samples_per_channel();
    for (size_t k = 0; k < signal.num_channels(); ++k) {
      rtc::ArrayView<float> channel_view = signal.channel(k);
      for (size_t j = 0; j < channel_view.size(); ++j) {
        channel_view[j] *= (gain_to_apply_ + j * gain_delta);
      }
    }
    gain_to_apply_ = target_gain_to_apply_;
  }

  // Use the limiter (if injected).
  if (gain_curve_applier_) {
    gain_curve_applier_->Process(signal);

    // Dump data for debug.
    const auto channel_view = signal.channel(0);
    apm_data_dumper_.DumpRaw("agc2_fixed_digital_gain_curve_applier",
                             channel_view.size(), channel_view.data());
  }

  // Hard-clipping.
  for (size_t k = 0; k < signal.num_channels(); ++k) {
    rtc::ArrayView<float> channel_view = signal.channel(k);
    for (auto& sample : channel_view) {
      sample = std::max(kMinSampleValue, std::min(sample, kMaxSampleValue));
    }
  }
}

}  // namespace webrtc
