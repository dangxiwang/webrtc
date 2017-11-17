/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
  */

#include "modules/audio_processing/aec3/matched_filter_lag_aggregator.h"

#include <sstream>
#include <string>
#include <vector>

#include "api/array_view.h"
#include "modules/audio_processing/aec3/aec3_common.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "test/gtest.h"

namespace webrtc {
namespace {

constexpr size_t kNumLagsBeforeDetection = 25;

}  // namespace

// Verifies that the most accurate lag estimate is chosen.
TEST(MatchedFilterLagAggregator, MostAccurateLagChosen) {
  constexpr size_t kLag1 = 5;
  constexpr size_t kLag2 = 10;
  ApmDataDumper data_dumper(0);
  std::vector<MatchedFilter::LagEstimate> lag_estimates(2);
  MatchedFilterLagAggregator aggregator(&data_dumper, std::max(kLag1, kLag2));
  lag_estimates[0] = MatchedFilter::LagEstimate(1.f, true, kLag1, true);
  lag_estimates[1] = MatchedFilter::LagEstimate(0.5f, true, kLag2, true);

  for (size_t k = 0; k < kNumLagsBeforeDetection; ++k) {
    EXPECT_FALSE(aggregator.Aggregate(lag_estimates));
  }

  rtc::Optional<size_t> aggregated_lag = aggregator.Aggregate(lag_estimates);
  EXPECT_TRUE(aggregated_lag);
  EXPECT_EQ(kLag1, *aggregated_lag);

  lag_estimates[0] = MatchedFilter::LagEstimate(0.5f, true, kLag1, true);
  lag_estimates[1] = MatchedFilter::LagEstimate(1.f, true, kLag2, true);

  for (size_t k = 0; k < kNumLagsBeforeDetection; ++k) {
    aggregated_lag = aggregator.Aggregate(lag_estimates);
    EXPECT_TRUE(aggregated_lag);
    EXPECT_EQ(kLag1, *aggregated_lag);
  }

  aggregated_lag = aggregator.Aggregate(lag_estimates);
  aggregated_lag = aggregator.Aggregate(lag_estimates);
  EXPECT_TRUE(aggregated_lag);
  EXPECT_EQ(kLag2, *aggregated_lag);
}

// Verifies that varying lag estimates causes lag estimates to not be deemed
// reliable.
TEST(MatchedFilterLagAggregator,
     LagEstimateInvarianceRequiredForAggregatedLag) {
  ApmDataDumper data_dumper(0);
  std::vector<MatchedFilter::LagEstimate> lag_estimates(1);
  MatchedFilterLagAggregator aggregator(&data_dumper, 100);
  for (size_t k = 0; k < kNumLagsBeforeDetection * 100; ++k) {
    lag_estimates[0] = MatchedFilter::LagEstimate(1.f, true, k % 100, true);
    rtc::Optional<size_t> aggregated_lag = aggregator.Aggregate(lag_estimates);
    EXPECT_FALSE(aggregated_lag);
  }
}

// Verifies that lag estimate updates are required to produce an updated lag
// aggregate.
TEST(MatchedFilterLagAggregator,
     DISABLED_LagEstimateUpdatesRequiredForAggregatedLag) {
  constexpr size_t kLag = 5;
  ApmDataDumper data_dumper(0);
  std::vector<MatchedFilter::LagEstimate> lag_estimates(1);
  MatchedFilterLagAggregator aggregator(&data_dumper, kLag);
  for (size_t k = 0; k < kNumLagsBeforeDetection * 10; ++k) {
    lag_estimates[0] = MatchedFilter::LagEstimate(1.f, true, kLag, false);
    rtc::Optional<size_t> aggregated_lag = aggregator.Aggregate(lag_estimates);
    EXPECT_FALSE(aggregated_lag);
    EXPECT_EQ(kLag, *aggregated_lag);
  }
}

// Verifies that an aggregated lag is persistent if the lag estimates do not
// change and that an aggregated lag is not produced without gaining lag
// estimate confidence.
TEST(MatchedFilterLagAggregator, DISABLED_PersistentAggregatedLag) {
  constexpr size_t kLag1 = 5;
  constexpr size_t kLag2 = 10;
  ApmDataDumper data_dumper(0);
  std::vector<MatchedFilter::LagEstimate> lag_estimates(1);
  MatchedFilterLagAggregator aggregator(&data_dumper, std::max(kLag1, kLag2));
  rtc::Optional<size_t> aggregated_lag;
  for (size_t k = 0; k < kNumLagsBeforeDetection; ++k) {
    lag_estimates[0] = MatchedFilter::LagEstimate(1.f, true, kLag1, true);
    aggregated_lag = aggregator.Aggregate(lag_estimates);
  }
  EXPECT_TRUE(aggregated_lag);
  EXPECT_EQ(kLag1, *aggregated_lag);

  for (size_t k = 0; k < kNumLagsBeforeDetection * 40; ++k) {
    lag_estimates[0] = MatchedFilter::LagEstimate(1.f, false, kLag2, true);
    aggregated_lag = aggregator.Aggregate(lag_estimates);
    EXPECT_TRUE(aggregated_lag);
    EXPECT_EQ(kLag1, *aggregated_lag);
  }
}

#if RTC_DCHECK_IS_ON && GTEST_HAS_DEATH_TEST && !defined(WEBRTC_ANDROID)

// Verifies the check for non-null data dumper.
TEST(MatchedFilterLagAggregator, NullDataDumper) {
  EXPECT_DEATH(MatchedFilterLagAggregator(nullptr, 10), "");
}

#endif

}  // namespace webrtc
