/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_RMS_LEVEL_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_AGC2_RMS_LEVEL_ESTIMATOR_H_

#include "modules/audio_processing/agc2/adaptive_mode_level_estimator.h"
#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/agc2/saturation_protector.h"

// #include "rtc_base/gtest_prod_util.h"

namespace webrtc {
class ApmDataDumper;

class RmsLevelEstimator : public AdaptiveModeLevelEstimator {
 public:
  // TODO(aleloi): Possibly set some config.
  explicit RmsLevelEstimator(ApmDataDumper* apm_data_dumper);
  void EstimateLevel(VadWithLevel::LevelAndProbability vad_data) override;
  float LatestLevelEstimate() const override;

 private:
  void DebugDumpEstimate();

  int buffer_size_ms_ = 0;
  float last_estimate_with_offset_dbfs_ = kInitialSpeechLevelEstimateDbfs;
  float sum_of_speech_probabilities_ = 0.f;
  SaturationProtector saturation_protector_;
  ApmDataDumper* const apm_data_dumper_;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_RMS_LEVEL_ESTIMATOR_H_
