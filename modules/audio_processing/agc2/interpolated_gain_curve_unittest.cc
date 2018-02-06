/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <array>
#include <iostream>  // TODO(alessiob): Remove when PlotData test removed.
#include <sstream>
#include <vector>

#include "api/array_view.h"
#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/agc2/interpolated_gain_curve.h"
#include "modules/audio_processing/agc2/limiter.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "rtc_base/checks.h"
#include "rtc_base/gunit.h"

namespace webrtc {
namespace {

constexpr double kLevelEpsilon = 1e-2 * kInputLevelScaling;
constexpr float kInterpolatedGainCurveTolerance = 1.f / 32768.f;

}  // namespace

TEST(AutomaticGainController2InterpolatedGainCurve, CreateUse) {
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();

  const auto levels =
      LinSpace(kLevelEpsilon, DbfsToLinear(l.max_input_level_db() + 1), 500);
  for (const auto level : levels) {
    EXPECT_GE(igc.LookUpGainToApply(level), 0.0f);
  }
}

TEST(AutomaticGainController2InterpolatedGainCurve, CheckValidOutput) {
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();

  const auto levels =
      LinSpace(kLevelEpsilon, l.max_input_level_linear() * 2.0, 500);
  for (const auto level : levels) {
    SCOPED_TRACE(std::to_string(level));
    const float gain = igc.LookUpGainToApply(level);
    EXPECT_LE(0.0f, gain);
    EXPECT_LE(gain, 1.0f);
  }
}

TEST(AutomaticGainController2InterpolatedGainCurve, CheckMonotonicity) {
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();

  const auto levels = LinSpace(
      kLevelEpsilon, l.max_input_level_linear() + kLevelEpsilon + 0.5, 500);
  float prev_gain = igc.LookUpGainToApply(0.0f);
  for (const auto level : levels) {
    const float gain = igc.LookUpGainToApply(level);
    EXPECT_GE(prev_gain, gain);
    prev_gain = gain;
  }
}

TEST(AutomaticGainController2InterpolatedGainCurve, CheckApproximation) {
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();

  const auto levels =
      LinSpace(kLevelEpsilon, l.max_input_level_linear() - kLevelEpsilon, 500);
  for (const auto level : levels) {
    SCOPED_TRACE(std::to_string(level));
    EXPECT_LT(std::fabs(l.GetGainLinear(level) - igc.LookUpGainToApply(level)),
              kInterpolatedGainCurveTolerance);
  }
}

TEST(AutomaticGainController2InterpolatedGainCurve, CheckRegionBoundaries) {
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();

  const std::vector<double> levels{
      {kLevelEpsilon, l.knee_start_linear() + kLevelEpsilon,
       l.limiter_start_linear() + kLevelEpsilon,
       l.max_input_level_linear() + kLevelEpsilon}};
  for (const auto level : levels) {
    igc.LookUpGainToApply(level);
  }

  const auto stats = igc.get_stats();
  EXPECT_EQ(1, stats.look_ups_identity_region);
  EXPECT_EQ(1, stats.look_ups_knee_region);
  EXPECT_EQ(1, stats.look_ups_limiter_region);
  EXPECT_EQ(1, stats.look_ups_saturation_region);
}

TEST(AutomaticGainController2InterpolatedGainCurve, CheckIdentityRegion) {
  constexpr size_t kNumSteps = 10;
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();

  const auto levels = LinSpace(kLevelEpsilon, l.knee_start_linear(), kNumSteps);
  for (const auto level : levels) {
    SCOPED_TRACE(std::to_string(level));
    EXPECT_EQ(1.0f, igc.LookUpGainToApply(level));
  }

  const auto stats = igc.get_stats();
  EXPECT_EQ(kNumSteps - 1, stats.look_ups_identity_region);
  EXPECT_EQ(1, stats.look_ups_knee_region);
  EXPECT_EQ(0, stats.look_ups_limiter_region);
  EXPECT_EQ(0, stats.look_ups_saturation_region);
}

TEST(AutomaticGainController2InterpolatedGainCurve,
     CheckNoOverApproximationKnee) {
  constexpr size_t kNumSteps = 10;
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();

  const auto levels = LinSpace(l.knee_start_linear() + kLevelEpsilon,
                               l.limiter_start_linear(), kNumSteps);
  for (const auto level : levels) {
    SCOPED_TRACE(std::to_string(level));
    // Small tolerance added (needed because comparing a float with a double).
    EXPECT_LE(igc.LookUpGainToApply(level), l.GetGainLinear(level) + 1e-7);
  }

  const auto stats = igc.get_stats();
  EXPECT_EQ(0, stats.look_ups_identity_region);
  EXPECT_EQ(kNumSteps - 1, stats.look_ups_knee_region);
  EXPECT_EQ(1, stats.look_ups_limiter_region);
  EXPECT_EQ(0, stats.look_ups_saturation_region);
}

TEST(AutomaticGainController2InterpolatedGainCurve,
     CheckNoOverApproximationBeyondKnee) {
  constexpr size_t kNumSteps = 10;
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();

  const auto levels =
      LinSpace(l.limiter_start_linear() + kLevelEpsilon,
               l.max_input_level_linear() - kLevelEpsilon, kNumSteps);
  for (const auto level : levels) {
    SCOPED_TRACE(std::to_string(level));
    // Small tolerance added (needed because comparing a float with a double).
    EXPECT_LE(igc.LookUpGainToApply(level), l.GetGainLinear(level) + 1e-7);
  }

  const auto stats = igc.get_stats();
  EXPECT_EQ(0, stats.look_ups_identity_region);
  EXPECT_EQ(0, stats.look_ups_knee_region);
  EXPECT_EQ(kNumSteps, stats.look_ups_limiter_region);
  EXPECT_EQ(0, stats.look_ups_saturation_region);
}

TEST(AutomaticGainController2InterpolatedGainCurve,
     CheckNoOverApproximationWithSaturation) {
  constexpr size_t kNumSteps = 3;
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();

  const auto levels =
      LinSpace(l.max_input_level_linear() + kLevelEpsilon,
               l.max_input_level_linear() + kLevelEpsilon + 0.5, kNumSteps);
  for (const auto level : levels) {
    SCOPED_TRACE(std::to_string(level));
    EXPECT_LE(igc.LookUpGainToApply(level), l.GetGainLinear(level));
  }

  const auto stats = igc.get_stats();
  EXPECT_EQ(0, stats.look_ups_identity_region);
  EXPECT_EQ(0, stats.look_ups_knee_region);
  EXPECT_EQ(0, stats.look_ups_limiter_region);
  EXPECT_EQ(kNumSteps, stats.look_ups_saturation_region);
}

// Only activate this test to dump data to plot.
// TODO(alessiob): Remove this test once AGC2 fixed digital is finalized and
// replace it inside the InterpolatedGainCurve class by using ApmDataDump.
TEST(AutomaticGainController2InterpolatedGainCurve, DISABLED_PlotData) {
  ApmDataDumper apm_data_dumper(0);
  InterpolatedGainCurve igc(&apm_data_dumper);
  const auto& l = igc.limiter();
  const auto levels = LinSpace(kLevelEpsilon, DbfsToLinear(12), 1000);

  // Input levels, limiter gains and interpolated gains.
  for (const double& level : levels) {
    std::cout << " " << level;
  }
  std::cout << std::endl;
  for (const double& level : levels) {
    std::cout << " " << l.GetGainLinear(level);
  }
  std::cout << std::endl;
  for (const double& level : levels) {
    std::cout << " " << igc.LookUpGainToApply(level);
  }
  std::cout << std::endl;

  // Boundaries.
  const std::array<double, 3> boundaries{{l.knee_start_linear(),
                                          l.limiter_start_linear(),
                                          l.max_input_level_linear()}};
  for (const double& level : boundaries) {
    std::cout << " " << level;
  }
  std::cout << std::endl;
  for (const double& level : boundaries) {
    std::cout << " " << l.GetGainLinear(level);
  }
  std::cout << std::endl;

  // Interpolation params.
  const auto approx_params_x = igc.approx_params_x();
  for (const double& level : approx_params_x) {
    std::cout << " " << level;
  }
  std::cout << std::endl;
  for (const double& level : approx_params_x) {
    std::cout << " " << igc.LookUpGainToApply(level);
  }
  std::cout << std::endl;
}

}  // namespace webrtc
