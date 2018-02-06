/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_GAIN_CURVE_APPLIER_H_
#define MODULES_AUDIO_PROCESSING_AGC2_GAIN_CURVE_APPLIER_H_

#include <vector>

#include "modules/audio_processing/agc2/interpolated_gain_curve.h"
#include "modules/audio_processing/agc2/level_estimator.h"
#include "modules/audio_processing/include/float_audio_frame.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "rtc_base/constructormagic.h"

namespace webrtc {

class GainCurveApplier {
 public:
  GainCurveApplier(GainCurveApplier&&);
  GainCurveApplier(const InterpolatedGainCurve* interp_gain_curve,
                   agc2::LevelEstimator* estimator,
                   ApmDataDumper* apm_data_dumper);

  ~GainCurveApplier();

  void Process(MutableFloatAudioFrame signal);
  InterpolatedGainCurve::Stats GetGainCurveStats() const;

 private:
  const InterpolatedGainCurve& interp_gain_curve_;
  agc2::LevelEstimator& level_estimator_;
  // Work array containing the sub-frame scaling factors to be interpolated.
  ApmDataDumper* const apm_data_dumper_;
  std::array<float, kSubFramesInFrame + 1> scaling_factors_;
  std::array<float, kMaximalNumberOfSamplesPerChannel>
      per_sample_scaling_factors_;
  float last_scaling_factor_ = 1.f;

  RTC_DISALLOW_COPY_AND_ASSIGN(GainCurveApplier);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_GAIN_CURVE_APPLIER_H_
