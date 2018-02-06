/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_FIXED_GAIN_CONTROLLER_H_
#define MODULES_AUDIO_PROCESSING_AGC2_FIXED_GAIN_CONTROLLER_H_

#include "modules/audio_processing/agc2/gain_curve_applier.h"
#include "modules/audio_processing/agc2/interpolated_gain_curve.h"
#include "modules/audio_processing/agc2/level_estimator.h"
#include "modules/audio_processing/include/float_audio_frame.h"

#include "api/optional.h"
#include "rtc_base/race_checker.h"

namespace webrtc {

class FixedGainController {
 public:
  // TODO(alessiob): Once the params become hard-coded, switch to a single ctor
  // with two parameters: gain to apply and flag to enable the limiter.
  FixedGainController(float gain_to_apply_db,
                      size_t sample_rate_hz,
                      bool enable_limiter);
  ~FixedGainController();
  void Process(MutableFloatAudioFrame signal);

  // Rate and gain may be changed at any time from the value passed to
  // the constructor.
  void SetGain(float gain_to_apply_db);
  void SetRate(size_t sample_rate_hz);

  void SetAttackMs(float duration_ms);
  void SetDecayMs(float duration_ms);

 private:
  static int instance_count_;

  rtc::RaceChecker race_checker_;

  float gain_to_apply_ RTC_GUARDED_BY(race_checker_);
  // If the gain changes, interpolate between the gains.
  float target_gain_to_apply_ RTC_GUARDED_BY(race_checker_);

  mutable ApmDataDumper apm_data_dumper_;
  rtc::Optional<InterpolatedGainCurve> interp_gain_curve_;
  rtc::Optional<agc2::LevelEstimator> level_estimator_;
  rtc::Optional<GainCurveApplier> gain_curve_applier_;

  RTC_DISALLOW_COPY_AND_ASSIGN(FixedGainController);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_FIXED_GAIN_CONTROLLER_H_
