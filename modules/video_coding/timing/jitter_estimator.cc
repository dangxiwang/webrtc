/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/timing/jitter_estimator.h"

#include <math.h>
#include <string.h>

#include <algorithm>
#include <cstdint>

#include "absl/types/optional.h"
#include "api/field_trials_view.h"
#include "api/units/data_size.h"
#include "api/units/frequency.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/video_coding/timing/rtt_filter.h"
#include "rtc_base/checks.h"
#include "rtc_base/numerics/safe_conversions.h"
#include "system_wrappers/include/clock.h"

namespace webrtc {
namespace {

// Number of frames to wait for before post processing estimate. Also used in
// the frame rate estimator ramp-up.
constexpr size_t kFrameProcessingStartupCount = 30;

// Number of frames to wait for before enabling the frame size filters.
constexpr size_t kFramesUntilSizeFiltering = 5;

// Initial value for frame size filters.
constexpr double kInitialAvgAndMaxFrameSizeBytes = 500.0;

// Time constant for average frame size filter.
constexpr double kPhi = 0.97;
// Time constant for max frame size filter.
constexpr double kPsi = 0.9999;
// Default constants for percentile frame size filter.
constexpr double kAvgFrameSizeMedianPercentile = 0.5;
constexpr double kDefaultMaxFrameSizePercentile = 0.95;
constexpr int kDefaultFrameSizeWindow = 30 * 10;

// Outlier rejection constants.
constexpr double kDefaultMaxTimestampDeviationInSigmas = 3.5;
constexpr double kNumStdDevDelayOutlier = 15.0;
constexpr double kNumStdDevSizeOutlier = 3.0;
constexpr double kCongestionRejectionFactor = -0.25;

// Rampup constant for deviation noise filters.
constexpr size_t kAlphaCountMax = 400;

// Noise threshold constants.
// ~Less than 1% chance (look up in normal distribution table)...
constexpr double kNoiseStdDevs = 2.33;
// ...of getting 30 ms freezes
constexpr double kNoiseStdDevOffset = 30.0;

// Jitter estimate clamping limits.
constexpr TimeDelta kMinJitterEstimate = TimeDelta::Millis(1);
constexpr TimeDelta kMaxJitterEstimate = TimeDelta::Seconds(10);

// A constant describing the delay from the jitter buffer to the delay on the
// receiving side which is not accounted for by the jitter buffer nor the
// decoding delay estimate.
constexpr TimeDelta OPERATING_SYSTEM_JITTER = TimeDelta::Millis(10);

// Time constant for reseting the NACK count.
constexpr TimeDelta kNackCountTimeout = TimeDelta::Seconds(60);

// RTT mult activation.
constexpr size_t kNackLimit = 3;

// Frame rate estimate clamping limit.
constexpr Frequency kMaxFramerateEstimate = Frequency::Hertz(200);

}  // namespace

constexpr char JitterEstimator::Config::kFieldTrialsKey[];

JitterEstimator::JitterEstimator(Clock* clock,
                                 const FieldTrialsView& field_trials)
    : config_(Config::Parse(field_trials.Lookup(Config::kFieldTrialsKey))),
      avg_frame_size_median_bytes_(kAvgFrameSizeMedianPercentile),
      max_frame_size_bytes_percentile_(
          config_.max_frame_size_percentile.value_or(
              kDefaultMaxFrameSizePercentile)),
      fps_counter_(30),  // TODO(sprang): Use an estimator with limit based
                         // on time, rather than number of samples.
      clock_(clock) {
  Reset();
}

JitterEstimator::~JitterEstimator() = default;

// Resets the JitterEstimate.
void JitterEstimator::Reset() {
  avg_frame_size_bytes_ = kInitialAvgAndMaxFrameSizeBytes;
  max_frame_size_bytes_ = kInitialAvgAndMaxFrameSizeBytes;
  var_frame_size_bytes2_ = 100;
  avg_frame_size_median_bytes_.Reset();
  max_frame_size_bytes_percentile_.Reset();
  frame_sizes_in_percentile_filter_ = std::queue<int64_t>();
  last_update_time_ = absl::nullopt;
  prev_estimate_ = absl::nullopt;
  prev_frame_size_ = absl::nullopt;
  avg_noise_ms_ = 0.0;
  var_noise_ms2_ = 4.0;
  alpha_count_ = 1;
  filter_jitter_estimate_ = TimeDelta::Zero();
  latest_nack_ = Timestamp::Zero();
  nack_count_ = 0;
  startup_frame_size_sum_bytes_ = 0;
  startup_frame_size_count_ = 0;
  startup_count_ = 0;
  rtt_filter_.Reset();
  fps_counter_.Reset();

  kalman_filter_ = FrameDelayVariationKalmanFilter();
}

// Updates the estimates with the new measurements.
void JitterEstimator::UpdateEstimate(TimeDelta frame_delay,
                                     DataSize frame_size) {
  if (frame_size.IsZero()) {
    return;
  }
  // Can't use DataSize since this can be negative.
  double delta_frame_bytes =
      frame_size.bytes() - prev_frame_size_.value_or(DataSize::Zero()).bytes();
  if (startup_frame_size_count_ < kFramesUntilSizeFiltering) {
    startup_frame_size_sum_bytes_ += frame_size.bytes();
    startup_frame_size_count_++;
  } else if (startup_frame_size_count_ == kFramesUntilSizeFiltering) {
    // Give the frame size filter.
    avg_frame_size_bytes_ = startup_frame_size_sum_bytes_ /
                            static_cast<double>(startup_frame_size_count_);
    startup_frame_size_count_++;
  }

  double avg_frame_size_bytes =
      kPhi * avg_frame_size_bytes_ + (1 - kPhi) * frame_size.bytes();
  double deviation_size_bytes = 2 * sqrt(var_frame_size_bytes2_);
  if (frame_size.bytes() < avg_frame_size_bytes_ + deviation_size_bytes) {
    // Only update the average frame size if this sample wasn't a key frame.
    avg_frame_size_bytes_ = avg_frame_size_bytes;
  }

  double delta_bytes = frame_size.bytes() - avg_frame_size_bytes;
  var_frame_size_bytes2_ = std::max(
      kPhi * var_frame_size_bytes2_ + (1 - kPhi) * (delta_bytes * delta_bytes),
      1.0);

  // Update non-linear IIR estimate of max frame size.
  max_frame_size_bytes_ =
      std::max<double>(kPsi * max_frame_size_bytes_, frame_size.bytes());

  // Maybe update percentile estimates of frame sizes.
  if (config_.avg_frame_size_median ||
      config_.MaxFrameSizePercentileEnabled()) {
    frame_sizes_in_percentile_filter_.push(frame_size.bytes());
    if (frame_sizes_in_percentile_filter_.size() >
        static_cast<size_t>(
            config_.frame_size_window.value_or(kDefaultFrameSizeWindow))) {
      double front = frame_sizes_in_percentile_filter_.front();
      avg_frame_size_median_bytes_.Erase(front);
      max_frame_size_bytes_percentile_.Erase(front);
      frame_sizes_in_percentile_filter_.pop();
    }
    if (config_.avg_frame_size_median) {
      avg_frame_size_median_bytes_.Insert(frame_size.bytes());
    }
    if (config_.MaxFrameSizePercentileEnabled()) {
      max_frame_size_bytes_percentile_.Insert(frame_size.bytes());
    }
  }

  if (!prev_frame_size_) {
    prev_frame_size_ = frame_size;
    return;
  }
  prev_frame_size_ = frame_size;

  // Cap frame_delay based on the current time deviation noise.
  TimeDelta max_time_deviation = TimeDelta::Millis(
      kDefaultMaxTimestampDeviationInSigmas * sqrt(var_noise_ms2_) + 0.5);
  frame_delay.Clamp(-max_time_deviation, max_time_deviation);

  double frame_delay_ms = ToMillis(frame_delay);
  double delay_deviation_ms =
      frame_delay_ms -
      kalman_filter_.GetFrameDelayVariationEstimateTotal(delta_frame_bytes);

  // Outlier rejection.
  double num_stddev_delay_outlier = GetNumStddevDelayOutlier();
  bool abs_delay_is_not_outlier =
      fabs(delay_deviation_ms) <
      num_stddev_delay_outlier * sqrt(var_noise_ms2_);
  bool size_is_positive_outlier =
      frame_size.bytes() >
      avg_frame_size_bytes_ +
          GetNumStddevSizeOutlier() * sqrt(var_frame_size_bytes2_);

  // Only update the Kalman filter if the sample is not considered an extreme
  // outlier. Even if it is an extreme outlier from a delay point of view, if
  // the frame size also is large the deviation is probably due to an incorrect
  // line slope.
  if (abs_delay_is_not_outlier || size_is_positive_outlier) {
    // Update the variance of the deviation from the line given by the Kalman
    // filter.
    EstimateRandomJitter(delay_deviation_ms);
    // Prevent updating with frames which have been congested by a large frame,
    // and therefore arrives almost at the same time as that frame.
    // This can occur when we receive a large frame (key frame) which has been
    // delayed. The next frame is of normal size (delta frame), and thus deltaFS
    // will be << 0. This removes all frame samples which arrives after a key
    // frame.
    double max_frame_size_bytes = GetMaxFrameSizeEstimateBytes();
    if (delta_frame_bytes >
        GetCongestionRejectionFactor() * max_frame_size_bytes) {
      // Update the Kalman filter with the new data
      kalman_filter_.PredictAndUpdate(frame_delay_ms, delta_frame_bytes,
                                      max_frame_size_bytes, var_noise_ms2_);
    }
  } else {
    double num_stddev = (delay_deviation_ms >= 0) ? num_stddev_delay_outlier
                                                  : -num_stddev_delay_outlier;
    EstimateRandomJitter(num_stddev * sqrt(var_noise_ms2_));
  }
  // Post process the total estimated jitter
  if (startup_count_ >= kFrameProcessingStartupCount) {
    PostProcessEstimate();
  } else {
    startup_count_++;
  }
}

// Updates the nack/packet ratio.
void JitterEstimator::FrameNacked() {
  if (nack_count_ < kNackLimit) {
    nack_count_++;
  }
  latest_nack_ = clock_->CurrentTime();
}

void JitterEstimator::UpdateRtt(TimeDelta rtt) {
  rtt_filter_.Update(rtt);
}

JitterEstimator::Config JitterEstimator::GetConfigForTest() const {
  return config_;
}

double JitterEstimator::ToMillis(const TimeDelta& time_delta) const {
  if (config_.microsecond_granularity) {
    return time_delta.us() / 1000.0;
  }
  return time_delta.ms();
}

TimeDelta JitterEstimator::FromMillis(double millis) const {
  if (config_.microsecond_granularity) {
    return TimeDelta::Micros(1000.0 * millis);
  }
  return TimeDelta::Millis(millis);
}

double JitterEstimator::GetAvgFrameSizeEstimateBytes() const {
  if (config_.avg_frame_size_median) {
    RTC_DCHECK_GT(frame_sizes_in_percentile_filter_.size(), 1u);
    RTC_DCHECK_LE(frame_sizes_in_percentile_filter_.size(),
                  config_.frame_size_window.value_or(kDefaultFrameSizeWindow));
    return avg_frame_size_median_bytes_.GetPercentileValue();
  }
  return avg_frame_size_bytes_;
}

double JitterEstimator::GetMaxFrameSizeEstimateBytes() const {
  if (config_.MaxFrameSizePercentileEnabled()) {
    RTC_DCHECK_GT(frame_sizes_in_percentile_filter_.size(), 1u);
    RTC_DCHECK_LE(frame_sizes_in_percentile_filter_.size(),
                  config_.frame_size_window.value_or(kDefaultFrameSizeWindow));
    return max_frame_size_bytes_percentile_.GetPercentileValue();
  }
  return max_frame_size_bytes_;
}

double JitterEstimator::GetNumStddevDelayOutlier() const {
  return config_.num_stddev_delay_outlier.value_or(kNumStdDevDelayOutlier);
}

double JitterEstimator::GetNumStddevSizeOutlier() const {
  return config_.num_stddev_size_outlier.value_or(kNumStdDevSizeOutlier);
}

double JitterEstimator::GetCongestionRejectionFactor() const {
  return config_.congestion_rejection_factor.value_or(
      kCongestionRejectionFactor);
}

// Estimates the random jitter by calculating the variance of the sample
// distance from the line given by the Kalman filter.
void JitterEstimator::EstimateRandomJitter(double d_dT) {
  Timestamp now = clock_->CurrentTime();
  if (last_update_time_.has_value()) {
    fps_counter_.AddSample((now - *last_update_time_).us());
  }
  last_update_time_ = now;

  if (alpha_count_ == 0) {
    RTC_DCHECK_NOTREACHED();
    return;
  }
  double alpha =
      static_cast<double>(alpha_count_ - 1) / static_cast<double>(alpha_count_);
  alpha_count_++;
  if (alpha_count_ > kAlphaCountMax)
    alpha_count_ = kAlphaCountMax;

  // In order to avoid a low frame rate stream to react slower to changes,
  // scale the alpha weight relative a 30 fps stream.
  Frequency fps = GetFrameRate();
  if (fps > Frequency::Zero()) {
    constexpr Frequency k30Fps = Frequency::Hertz(30);
    double rate_scale = k30Fps / fps;
    // At startup, there can be a lot of noise in the fps estimate.
    // Interpolate rate_scale linearly, from 1.0 at sample #1, to 30.0 / fps
    // at sample #kFrameProcessingStartupCount.
    if (alpha_count_ < kFrameProcessingStartupCount) {
      rate_scale = (alpha_count_ * rate_scale +
                    (kFrameProcessingStartupCount - alpha_count_)) /
                   kFrameProcessingStartupCount;
    }
    alpha = pow(alpha, rate_scale);
  }

  double avg_noise_ms = alpha * avg_noise_ms_ + (1 - alpha) * d_dT;
  double var_noise_ms2 = alpha * var_noise_ms2_ + (1 - alpha) *
                                                      (d_dT - avg_noise_ms_) *
                                                      (d_dT - avg_noise_ms_);
  avg_noise_ms_ = avg_noise_ms;
  var_noise_ms2_ = var_noise_ms2;
  if (var_noise_ms2_ < 1.0) {
    // The variance should never be zero, since we might get stuck and consider
    // all samples as outliers.
    var_noise_ms2_ = 1.0;
  }
}

double JitterEstimator::NoiseThreshold() const {
  double noise_threshold_ms =
      kNoiseStdDevs * sqrt(var_noise_ms2_) - kNoiseStdDevOffset;
  if (noise_threshold_ms < 1.0) {
    noise_threshold_ms = 1.0;
  }
  return noise_threshold_ms;
}

// Calculates the current jitter estimate from the filtered estimates.
TimeDelta JitterEstimator::CalculateEstimate() {
  double worst_case_frame_size_deviation_bytes =
      GetMaxFrameSizeEstimateBytes() - GetAvgFrameSizeEstimateBytes();
  double ret_ms = kalman_filter_.GetFrameDelayVariationEstimateSizeBased(
                      worst_case_frame_size_deviation_bytes) +
                  NoiseThreshold();
  TimeDelta ret = FromMillis(ret_ms);

  // A very low estimate (or negative) is neglected.
  if (ret < kMinJitterEstimate) {
    ret = prev_estimate_.value_or(kMinJitterEstimate);
    // Sanity check to make sure that no other method has set `prev_estimate_`
    // to a value lower than `kMinJitterEstimate`.
    RTC_DCHECK_GE(ret, kMinJitterEstimate);
  } else if (ret > kMaxJitterEstimate) {  // Sanity
    ret = kMaxJitterEstimate;
  }
  prev_estimate_ = ret;
  return ret;
}

void JitterEstimator::PostProcessEstimate() {
  filter_jitter_estimate_ = CalculateEstimate();
}

// Returns the current filtered estimate if available,
// otherwise tries to calculate an estimate.
TimeDelta JitterEstimator::GetJitterEstimate(
    double rtt_multiplier,
    absl::optional<TimeDelta> rtt_mult_add_cap) {
  TimeDelta jitter = CalculateEstimate() + OPERATING_SYSTEM_JITTER;
  Timestamp now = clock_->CurrentTime();

  if (now - latest_nack_ > kNackCountTimeout)
    nack_count_ = 0;

  if (filter_jitter_estimate_ > jitter)
    jitter = filter_jitter_estimate_;
  if (nack_count_ >= kNackLimit) {
    if (rtt_mult_add_cap.has_value()) {
      jitter += std::min(rtt_filter_.Rtt() * rtt_multiplier,
                         rtt_mult_add_cap.value());
    } else {
      jitter += rtt_filter_.Rtt() * rtt_multiplier;
    }
  }

  static const Frequency kJitterScaleLowThreshold = Frequency::Hertz(5);
  static const Frequency kJitterScaleHighThreshold = Frequency::Hertz(10);
  Frequency fps = GetFrameRate();
  // Ignore jitter for very low fps streams.
  if (fps < kJitterScaleLowThreshold) {
    if (fps.IsZero()) {
      return std::max(TimeDelta::Zero(), jitter);
    }
    return TimeDelta::Zero();
  }

  // Semi-low frame rate; scale by factor linearly interpolated from 0.0 at
  // kJitterScaleLowThreshold to 1.0 at kJitterScaleHighThreshold.
  if (fps < kJitterScaleHighThreshold) {
    jitter = (1.0 / (kJitterScaleHighThreshold - kJitterScaleLowThreshold)) *
             (fps - kJitterScaleLowThreshold) * jitter;
  }

  return std::max(TimeDelta::Zero(), jitter);
}

Frequency JitterEstimator::GetFrameRate() const {
  TimeDelta mean_frame_period = TimeDelta::Micros(fps_counter_.ComputeMean());
  if (mean_frame_period <= TimeDelta::Zero())
    return Frequency::Zero();

  Frequency fps = 1 / mean_frame_period;
  // Sanity check.
  RTC_DCHECK_GE(fps, Frequency::Zero());
  return std::min(fps, kMaxFramerateEstimate);
}
}  // namespace webrtc
