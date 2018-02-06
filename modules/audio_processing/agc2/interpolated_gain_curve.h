/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_INTERPOLATED_GAIN_CURVE_H_
#define MODULES_AUDIO_PROCESSING_AGC2_INTERPOLATED_GAIN_CURVE_H_

#include <array>

#include "rtc_base/basictypes.h"
#include "rtc_base/constructormagic.h"

#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/agc2/limiter.h"

namespace webrtc {

class ApmDataDumper;

constexpr double kInputLevelScalingFactor = 32768.0;

constexpr size_t kInterpolatedGainCurveTotalPoints =
    kInterpolatedGainCurveKneePoints + kInterpolatedGainCurveBeyondKneePoints;

// Interpolated gain curve using under-approximation to avoid saturation.
//
// The goal of this class is allowing fast look up ops to get an accurate
// estimations of tha gain to apply given an estimated input level.
class InterpolatedGainCurve {
 public:
  struct Stats {
    // Region in which the output level equals the input one.
    size_t look_ups_identity_region = 0;
    // Smoothing between the identity and the limiter regions.
    size_t look_ups_knee_region = 0;
    // Limiter region in which the output and input levels are linearly related.
    size_t look_ups_limiter_region = 0;
    // Region in which saturation may occur since the input level is beyond the
    // maximum exptected by the limiter.
    size_t look_ups_saturation_region = 0;
    // True if stats have been populated.
    bool available = false;
  };

  InterpolatedGainCurve(InterpolatedGainCurve&&);
  explicit InterpolatedGainCurve(ApmDataDumper* apm_data_dumper);
  ~InterpolatedGainCurve();

  const Limiter& limiter() const { return limiter_; }
  Stats get_stats() const { return stats_; }

  // TODO(alessiob): Remove this getter once AGC2 fixed digital is finalized.
  std::array<float, kInterpolatedGainCurveTotalPoints> approx_params_x() const {
    return approximation_params_x_;
  }

  // Given a non-negative input level (linear scale), a scalar factor to apply
  // to a sub-frame is returned.
  // TODO(alessiob): Add details on |input_level| falling in the saturation
  // region once a final decision is taken.
  float LookUpGainToApply(float input_level) const;

 private:
  void UpdateStats(float input_level) const;

  // Computes the params for a piece-wise linear interpolation with which the
  // gain curve encoded in the limiter is approximated.
  void Init();
  void PrecomputeKneeApproxParams();
  void PrecomputeBeyondKneeApproxParams();

  const Limiter limiter_;
  // The saturation gain is defined in order to let hard-clipping occur for
  // those samples having a level that falls in the saturation region. It is an
  // upper bound of the actual gain to apply - i.e., that returned by the
  // limiter.
  const float saturation_gain_;

  ApmDataDumper* const apm_data_dumper_;

  // Knee and beyond-knee regions approximation parameters.
  // The gain curve is approximated as a piece-wise linear function.
  // |approx_params_x_| are the boundaries between adjacent linear pieces,
  // |approx_params_m_| and |approx_params_q_| are the slope and the y-intercept
  // values of each piece.
  std::array<float, kInterpolatedGainCurveTotalPoints> approximation_params_x_;
  std::array<float, kInterpolatedGainCurveTotalPoints> approximation_params_m_;
  std::array<float, kInterpolatedGainCurveTotalPoints> approximation_params_q_;

  // Stats.
  mutable Stats stats_;

  RTC_DISALLOW_COPY_AND_ASSIGN(InterpolatedGainCurve);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_INTERPOLATED_GAIN_CURVE_H_
