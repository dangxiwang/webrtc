/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_RMS_LEVEL_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_RMS_LEVEL_ESTIMATOR_H_

#include "modules/audio_processing/agc2/adaptive_mode/adaptive_mode_level_estimator.h"
#include "modules/audio_processing/agc2/adaptive_mode/saturation_protector.h"

namespace webrtc {
class ApmDataDumper;

class RmsLevelEstimator : public AdaptiveModeLevelEstimator {
 public:
  // Threshold 0.9 gave the most stable results with all history
  // window sizes when using APM-VAD. With RNN-VAD, we should use 0.4
  // instead.
  constexpr static float kVadConfidenceThreshold = 0.9;

  // When using APM at confidence 0.9, this gives a target history of
  // ~4 seconds (meaning it takes ~4 seconds of real input audio to
  // fully adjust the level). If we use RNN(confidence = 0.4), this
  // should be ~2500 instead for the same target history window size.
  constexpr static float kFullBufferSizeMs = 1000;

  constexpr static float kFullBufferLeakFactor = 1.f - 1.f / kFullBufferSizeMs;

  constexpr static float kInitialSpeechLevelEstimateDbfs = -30.f;

  // TODO(aleloi): Possibly set some config.
  explicit RmsLevelEstimator(ApmDataDumper* apm_data_dumper);
  float EstimateLevel(VadWithLevel::LevelAndProbability vad_data) override;

 private:
  void DebugDumpEstimate();

  int buffer_size_ms_ = 0;
  float last_estimate_with_offset_dbfs_ = kInitialSpeechLevelEstimateDbfs;
  float sum_of_speech_probabilities_ = 0.f;
  SaturationProtector saturation_protector_;
  ApmDataDumper* const apm_data_dumper_;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_RMS_LEVEL_ESTIMATOR_H_
