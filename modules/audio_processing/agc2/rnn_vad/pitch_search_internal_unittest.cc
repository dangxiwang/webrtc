/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/rnn_vad/pitch_search_internal.h"

#include <array>
#include <tuple>

#include "modules/audio_processing/agc2/rnn_vad/test_utils.h"
// TODO(bugs.webrtc.org/8948): Add when the issue is fixed.
// #include "test/fpe_observer.h"
#include "test/gtest.h"

namespace webrtc {
namespace rnn_vad {
namespace test {
namespace {

constexpr int kTestPitchPeriodsLow = 3 * kMinPitch48kHz / 2;
constexpr int kTestPitchPeriodsHigh = (3 * kMinPitch48kHz + kMaxPitch48kHz) / 2;

constexpr float kTestPitchGainsLow = 0.35f;
constexpr float kTestPitchGainsHigh = 0.75f;

}  // namespace

// Checks that the frame-wise sliding square energy function produces output
// within tolerance given test input data.
TEST(RnnVadTest, ComputeSlidingFrameSquareEnergiesWithinTolerance) {
  PitchTestData test_data;
  std::array<float, kNumPitchBufSquareEnergies> computed_output;
  {
    // TODO(bugs.webrtc.org/8948): Add when the issue is fixed.
    // FloatingPointExceptionObserver fpe_observer;
    ComputeSlidingFrameSquareEnergies(test_data.GetPitchBufView(),
                                      computed_output);
  }
  auto square_energies_view = test_data.GetPitchBufSquareEnergiesView();
  ExpectNearAbsolute({square_energies_view.data(), square_energies_view.size()},
                     computed_output, 3e-2f);
}

// Checks that the estimated pitch period is bit-exact given test input data.
TEST(RnnVadTest, FindBestPitchPeriodsBitExactness) {
  PitchTestData test_data;
  std::array<float, kBufSize12kHz> pitch_buf_decimated;
  Decimate2x(test_data.GetPitchBufView(), pitch_buf_decimated);
  CandidatePitchPeriods pitch_candidates;
  {
    // TODO(bugs.webrtc.org/8948): Add when the issue is fixed.
    // FloatingPointExceptionObserver fpe_observer;
    auto auto_corr_view = test_data.GetPitchBufAutoCorrCoeffsView();
    pitch_candidates =
        FindBestPitchPeriods12kHz(auto_corr_view, pitch_buf_decimated);
  }
  EXPECT_EQ(pitch_candidates.best, 140);
  EXPECT_EQ(pitch_candidates.second_best, 142);
}

// Checks that the refined pitch period is bit-exact given test input data.
TEST(RnnVadTest, RefinePitchPeriod48kHzBitExactness) {
  PitchTestData test_data;
  // TODO(bugs.webrtc.org/8948): Add when the issue is fixed.
  // FloatingPointExceptionObserver fpe_observer;
  EXPECT_EQ(RefinePitchPeriod48kHz(test_data.GetPitchBufView(),
                                   /*pitch_candidates=*/{280, 284}),
            560);
  EXPECT_EQ(RefinePitchPeriod48kHz(test_data.GetPitchBufView(),
                                   /*pitch_candidates=*/{260, 284}),
            568);
}

class CheckLowerPitchPeriodsAndComputePitchGainTest
    : public ::testing::Test,
      public ::testing::WithParamInterface<std::tuple<
          /*initial_pitch_period=*/int,
          /*prev_pitch_period=*/int,
          /*prev_pitch_gain=*/float,
          /*expected_pitch_period=*/int,
          /*expected_pitch_gain=*/float>> {};

// Checks that the computed pitch period is bit-exact and that the computed
// pitch gain is within tolerance given test input data.
TEST_P(CheckLowerPitchPeriodsAndComputePitchGainTest,
       PeriodBitExactnessGainWithinTolerance) {
  const auto params = GetParam();
  const int initial_pitch_period = std::get<0>(params);
  const int prev_pitch_period = std::get<1>(params);
  const float prev_pitch_gain = std::get<2>(params);
  const int expected_pitch_period = std::get<3>(params);
  const float expected_pitch_gain = std::get<4>(params);
  PitchTestData test_data;
  {
    // TODO(bugs.webrtc.org/8948): Add when the issue is fixed.
    // FloatingPointExceptionObserver fpe_observer;
    const auto computed_output = CheckLowerPitchPeriodsAndComputePitchGain(
        test_data.GetPitchBufView(), initial_pitch_period,
        {prev_pitch_period, prev_pitch_gain});
    EXPECT_EQ(expected_pitch_period, computed_output.period);
    EXPECT_NEAR(expected_pitch_gain, computed_output.gain, 1e-6f);
  }
}

INSTANTIATE_TEST_SUITE_P(
    RnnVadTest,
    CheckLowerPitchPeriodsAndComputePitchGainTest,
    ::testing::Values(std::make_tuple(kTestPitchPeriodsLow,
                                      kTestPitchPeriodsLow,
                                      kTestPitchGainsLow,
                                      91,
                                      -0.0188608f),
                      std::make_tuple(kTestPitchPeriodsLow,
                                      kTestPitchPeriodsLow,
                                      kTestPitchGainsHigh,
                                      91,
                                      -0.0188608f),
                      std::make_tuple(kTestPitchPeriodsLow,
                                      kTestPitchPeriodsHigh,
                                      kTestPitchGainsLow,
                                      91,
                                      -0.0188608f),
                      std::make_tuple(kTestPitchPeriodsLow,
                                      kTestPitchPeriodsHigh,
                                      kTestPitchGainsHigh,
                                      91,
                                      -0.0188608f),
                      std::make_tuple(kTestPitchPeriodsHigh,
                                      kTestPitchPeriodsLow,
                                      kTestPitchGainsLow,
                                      475,
                                      -0.0904344f),
                      std::make_tuple(kTestPitchPeriodsHigh,
                                      kTestPitchPeriodsLow,
                                      kTestPitchGainsHigh,
                                      475,
                                      -0.0904344f),
                      std::make_tuple(kTestPitchPeriodsHigh,
                                      kTestPitchPeriodsHigh,
                                      kTestPitchGainsLow,
                                      475,
                                      -0.0904344f),
                      std::make_tuple(kTestPitchPeriodsHigh,
                                      kTestPitchPeriodsHigh,
                                      kTestPitchGainsHigh,
                                      475,
                                      -0.0904344f)));

}  // namespace test
}  // namespace rnn_vad
}  // namespace webrtc
