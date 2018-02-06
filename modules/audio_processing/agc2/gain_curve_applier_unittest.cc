/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/gain_curve_applier.h"

#include <string>
#include <utility>

#include "api/array_view.h"
#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/agc2/vector_float_frame.h"
#include "rtc_base/gunit.h"

namespace webrtc {
namespace {

std::string LevelEstimatorConfigToString(const std::vector<float>& config) {
  std::ostringstream ss;
  ss << "n_sf=" << config[0] << " a=" << config[1] << " d=" << config[2];
  return ss.str();
}

}  // namespace

TEST(GainCurveApplier, GainCurveApplierShouldConstructAndRun) {
  const int sample_rate_hz = 48000;
  ApmDataDumper apm_data_dumper(0);

  InterpolatedGainCurve igc(&apm_data_dumper);
  agc2::LevelEstimator le(sample_rate_hz, &apm_data_dumper);
  GainCurveApplier gain_curve_applier(&igc, &le, &apm_data_dumper);

  VectorFloatFrame vectors_with_float_frame(1, sample_rate_hz / 100,
                                            kMaxSampleValue);
  gain_curve_applier.Process(*vectors_with_float_frame.float_frame());
}

TEST(GainCurveApplier, OutputVolumeAboveThreshold) {
  const int sample_rate_hz = 48000;
  const float input_level =
      (kMaxSampleValue + DbfsToLinear(kLimiterMaxInputLevel)) / 2.f;
  ApmDataDumper apm_data_dumper(0);

  InterpolatedGainCurve igc(&apm_data_dumper);
  agc2::LevelEstimator le(sample_rate_hz, &apm_data_dumper);
  GainCurveApplier gain_curve_applier(&igc, &le, &apm_data_dumper);

  // Give the level estimator time to adapt.
  for (int i = 0; i < 5; ++i) {
    VectorFloatFrame vectors_with_float_frame(1, sample_rate_hz / 100,
                                              input_level);
    gain_curve_applier.Process(*vectors_with_float_frame.float_frame());
  }

  VectorFloatFrame vectors_with_float_frame(1, sample_rate_hz / 100,
                                            input_level);
  rtc::ArrayView<float> channel =
      vectors_with_float_frame.float_frame()->GetChannel(0);
  gain_curve_applier.Process(*vectors_with_float_frame.float_frame());

  for (const auto& sample : channel) {
    EXPECT_LT(0.9f * kInputLevelScaling, sample);
  }
}

}  // namespace webrtc
