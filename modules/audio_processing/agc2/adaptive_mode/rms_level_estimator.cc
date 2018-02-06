/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/adaptive_mode/rms_level_estimator.h"

#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"

namespace webrtc {

RmsLevelEstimator::RmsLevelEstimator(ApmDataDumper* apm_data_dumper)
    : apm_data_dumper_(apm_data_dumper) {
  RTC_DCHECK(apm_data_dumper_);
}

// TODO(aleloi): rewrite to match C++ syntax and variable names. Put
// comments inline.

// Level estimation algorithm.

// What happens here?
// Basically, we compute the average level of all recorded frame RMS values.
// We also have the access to the VAD probability, so we weight the RMS values
// by the vad probability. The formula then becomes

//   sum(rms[now-i]*vad_prob[now-i] for i in range(0, frames since start)) /
//   sum(vad_prob[now-i] for i in range(0, frames since start))

// We don't want the adaptation rate to decrease, so we weight older
// values less.  Every time step, the weight of a value decays with a
// factor (1-LEAK).  The formula then becomes

//    sum(rms[now-i]*vad_prob[now-i]*(1-LEAK)**i
//          for i in range(0, frames since start)) /
//    sum(vad_prob[now-i]*(1-LEAK)**i for i in range(0, frames since start))

// But we don't want to make a long silent period to reduce all weights to
// almost zero. If we do that, the level will jump to very close to the level of
// the next speech frame. We don't want large jumps, so we only decay when the
// vad probability is > self._config.confidence_threshold.

// The formula is suitable for a recursive computation.
// We save the numerator as self._last_speech_level and the denominator as
// self._confidence_sum.

// The LEAK factor is chosen so that the weights correspond to a buffer of size
// self._config.buffer_size_ms.
float RmsLevelEstimator::EstimateLevel(
    VadWithLevel::LevelAndProbability vad_data) {
  if (vad_data.speech_probability < kVadConfidenceThreshold) {
    DebugDumpEstimate();
    return last_estimate_with_offset_dbfs_ + saturation_protector_.LastMargin();
  }

  const bool buffer_is_full = buffer_size_ms_ > kFullBufferSizeMs;
  if (!buffer_is_full) {
    buffer_size_ms_ += kFrameDurationMs;
  }

  const float leak_factor = buffer_is_full ? kFullBufferLeakFactor : 1.f;

  const float numerator =
      last_estimate_with_offset_dbfs_ * sum_of_speech_probabilities_ *
          leak_factor +
      vad_data.speech_rms_dbfs * vad_data.speech_probability;
  const float denominator =
      sum_of_speech_probabilities_ + vad_data.speech_probability;

  last_estimate_with_offset_dbfs_ = numerator / denominator;
  sum_of_speech_probabilities_ = denominator;

  const float saturation_margin = saturation_protector_.CurrentMargin(
      vad_data, last_estimate_with_offset_dbfs_);
  DebugDumpEstimate();
  return last_estimate_with_offset_dbfs_ + saturation_margin;
}

void RmsLevelEstimator::DebugDumpEstimate() {
  apm_data_dumper_->DumpRaw("agc2_adaptive_level_estimate_with_offset_dbfs",
                            last_estimate_with_offset_dbfs_);
  apm_data_dumper_->DumpRaw("agc2_adaptive_saturation_margin_db",
                            saturation_protector_.LastMargin());
  apm_data_dumper_->DumpRaw(
      "agc2_adaptive_level_estimate_dbfs",
      last_estimate_with_offset_dbfs_ + saturation_protector_.LastMargin());
}

}  // namespace webrtc
