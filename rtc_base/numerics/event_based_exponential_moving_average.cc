/*
 *  Copyright 2019 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/numerics/event_based_exponential_moving_average.h"

#include <cmath>

#include "rtc_base/checks.h"

namespace {

// For a normal distributed value, the 95% double sided confidence interval is
// is 1.96 * stddev.
constexpr double nintyfive_percent_confidence = 1.96;

}  // namespace

namespace rtc {

EventBasedExponentialMovingAverage::EventBasedExponentialMovingAverage(
    int half_time)
    : tau_(static_cast<double>(half_time) / log(2)) {}

void EventBasedExponentialMovingAverage::AddSample(int64_t now, int sample) {
  if (!last_observation_timestamp_.has_value()) {
    value_ = sample;
  } else {
    RTC_DCHECK(now > *last_observation_timestamp_);
    // Variance gets computed after second sample.
    int64_t age = now - *last_observation_timestamp_;
    double e = exp(-age / tau_);
    double alpha = e / (1 + e);
    double one_minus_alpha = 1 - alpha;
    double sample_diff = sample - value_;
    double new_value = one_minus_alpha * value_ + alpha * sample;
    double new_variance =
        (sample_variance_ == std::numeric_limits<double>::infinity())
            ?
            // First variance
            (sample_diff * sample_diff)
            :
            // Subsequent variance
            one_minus_alpha * sample_variance_ +
                alpha * sample_diff * sample_diff;
    double new_estimator_variance =
        (one_minus_alpha * one_minus_alpha) * estimator_variance_ +
        (alpha * alpha);
    value_ = new_value;
    sample_variance_ = new_variance;
    estimator_variance_ = new_estimator_variance;
  }
  last_observation_timestamp_ = now;
}

double EventBasedExponentialMovingAverage::GetConfidenceInterval() const {
  return nintyfive_percent_confidence *
         sqrt(sample_variance_ * estimator_variance_);
}

}  // namespace rtc
