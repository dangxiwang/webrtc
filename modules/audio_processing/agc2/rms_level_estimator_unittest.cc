/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/rms_level_estimator.h"

#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "modules/audio_processing/vad/vad_with_level.h"

// #include "modules/audio_processing/agc2/fixed_digital_level_estimator.h"

// #include <iostream>

// #include "common_audio/include/audio_util.h"

// #include "modules/audio_processing/agc2/agc2_testing_common.h"
// #include "modules/audio_processing/agc2/vector_float_frame.h"
#include "rtc_base/gunit.h"
// #include "test/gtest.h"

namespace webrtc {
namespace {
void RunOnConstantLevel(int num_iterations,
                        VadWithLevel::LevelAndProbability vad_data,
                        RmsLevelEstimator* level_estimator) {
  // Give the estimator time to adapt (1 second).
  for (int i = 0; i < num_iterations; ++i) {
    level_estimator->EstimateLevel(vad_data);  // By copy
  }
}
}  // namespace

TEST(AutomaticGainController2RmsLevelEstimator, EstimatorShouldNotCrash) {
  ApmDataDumper apm_data_dumper(0);
  RmsLevelEstimator level_estimator(&apm_data_dumper);

  VadWithLevel::LevelAndProbability vad_data(1.f, -20.f, -10.f);
  level_estimator.EstimateLevel(vad_data);
  static_cast<void>(level_estimator.LatestLevelEstimate());
}

TEST(AutomaticGainController2RmsLevelEstimator, LevelShouldStabilize) {
  ApmDataDumper apm_data_dumper(0);
  RmsLevelEstimator level_estimator(&apm_data_dumper);

  constexpr float kSpeechRmsDbfs = -15.f;
  RunOnConstantLevel(
      100,
      VadWithLevel::LevelAndProbability(
          1.f, kSpeechRmsDbfs - kInitialSaturationMarginDb, kSpeechRmsDbfs),
      &level_estimator);

  EXPECT_NEAR(level_estimator.LatestLevelEstimate(), kSpeechRmsDbfs, 0.1f);
}

TEST(AutomaticGainController2RmsLevelEstimator,
     EstimatorIgnoresZeroProbabilityFrames) {
  ApmDataDumper apm_data_dumper(0);
  RmsLevelEstimator level_estimator(&apm_data_dumper);

  // Run for one second of fake audio.
  constexpr float kSpeechRmsDbfs = -25.f;
  RunOnConstantLevel(
      100,
      VadWithLevel::LevelAndProbability(
          1.f, kSpeechRmsDbfs - kInitialSaturationMarginDb, kSpeechRmsDbfs),
      &level_estimator);

  // Run for one more second, but mark as not speech.
  constexpr float kNoiseRmsDbfs = 0.f;
  RunOnConstantLevel(
      100, VadWithLevel::LevelAndProbability(0.f, kNoiseRmsDbfs, kNoiseRmsDbfs),
      &level_estimator);

  // Level should not have changed.
  EXPECT_NEAR(level_estimator.LatestLevelEstimate(), kSpeechRmsDbfs, 0.1f);
}

TEST(AutomaticGainController2RmsLevelEstimator, TimeToAdapt) {
  ApmDataDumper apm_data_dumper(0);
  RmsLevelEstimator level_estimator(&apm_data_dumper);

  // Run for one 'window size' interval
  constexpr float kInitialSpeechRmsDbfs = -30.f;
  RunOnConstantLevel(
      kFullBufferSizeMs / kFrameDurationMs,
      VadWithLevel::LevelAndProbability(
          1.f, kInitialSpeechRmsDbfs - kInitialSaturationMarginDb,
          kInitialSpeechRmsDbfs),
      &level_estimator);

  // Run for one half 'window size' interval. This should not be enough to
  // adapt.
  constexpr float kDifferentSpeechRmsDbfs = -10.f;
  RunOnConstantLevel(
      static_cast<int>(kFullBufferSizeMs / kFrameDurationMs / 2),
      VadWithLevel::LevelAndProbability(
          1.f, kDifferentSpeechRmsDbfs - kInitialSaturationMarginDb,
          kDifferentSpeechRmsDbfs),
      &level_estimator);
  EXPECT_GT(
      std::abs(kDifferentSpeechRmsDbfs - level_estimator.LatestLevelEstimate()),
      2.f);

  // Run for another half 'window size' interval. Now we should have adapted.
  RunOnConstantLevel(
      static_cast<int>(kFullBufferSizeMs / kFrameDurationMs / 2),
      VadWithLevel::LevelAndProbability(
          1.f, kDifferentSpeechRmsDbfs - kInitialSaturationMarginDb,
          kDifferentSpeechRmsDbfs),
      &level_estimator);
  EXPECT_NEAR(level_estimator.LatestLevelEstimate(), kDifferentSpeechRmsDbfs,
              2.f);
}

}  // namespace webrtc
