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
#include "common_audio/include/audio_util.h"
#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/safe_minmax.h"

namespace webrtc {
namespace {

// Returns true when the gain factor is so close to 1 that it would
// not affect int16 samples.
bool CloseToOne(float gain_factor) {
  return 1.f - 1.f / kMaxSampleValue <= gain_factor &&
         gain_factor <= 1.f + 1.f / kMaxSampleValue;
}
}  // namespace

FixedGainController::FixedGainController(ApmDataDumper* apm_data_dumper)
    : apm_data_dumper_(apm_data_dumper) {
  RTC_DCHECK_LT(0.f, gain_to_apply_);
  RTC_DLOG(LS_INFO) << "Gain to apply: " << gain_to_apply_;
}

void FixedGainController::SetGain(float gain_to_apply_db) {
  // Changes in gain_to_apply_ cause discontinuities. We assume
  // gain_to_apply_ is set in the beginning of the call. If it is
  // frequently changed, we should add interpolation between the
  // values.
  gain_to_apply_ = DbToRatio(gain_to_apply_db);
}

void FixedGainController::SetSampleRate(size_t sample_rate_hz) {
  // TODO(aleloi): propagate the new sample rate to the GainCurveApplier.
}

void FixedGainController::EnableLimiter(bool enable_limiter) {
  enable_limiter_ = enable_limiter;
}

void FixedGainController::Process(FloatAudioFrame signal) {
  // Apply fixed digital gain; interpolate if necessary. One of the
  // planned usages of the FGC is to only use the limiter. In that
  // case, the gain would be 1.0. Not doing the multiplications speeds
  // it up considerably. Hence the check.
  if (!CloseToOne(gain_to_apply_)) {
    for (size_t k = 0; k < signal.num_channels(); ++k) {
      rtc::ArrayView<float> channel_view = signal.channel(k);
      for (auto& sample : channel_view) {
        sample *= gain_to_apply_;
      }
    }
  }

  // Use the limiter (if configured to).
  if (enable_limiter_) {
    // TODO(aleloi): Process the signal with the
    // GainCurveApplier. This will be done in the upcoming CLs.

    // Dump data for debug.
    const auto channel_view = signal.channel(0);
    apm_data_dumper_->DumpRaw("agc2_fixed_digital_gain_curve_applier",
                              channel_view.size(), channel_view.data());
  }

  // Hard-clipping.
  for (size_t k = 0; k < signal.num_channels(); ++k) {
    rtc::ArrayView<float> channel_view = signal.channel(k);
    for (auto& sample : channel_view) {
      sample = rtc::SafeClamp(sample, kMinSampleValue, kMaxSampleValue);
    }
  }
}
}  // namespace webrtc
