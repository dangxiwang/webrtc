/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "system_wrappers/include/rtp_to_ntp_estimator.h"

#include <stddef.h>
#include <cmath>

#include <vector>

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {
namespace {
// Maximum number of RTCP SR reports to use to map between RTP and NTP.
const size_t kNumRtcpReportsToUse = 20;
// Don't allow NTP timestamps to jump more than 1 hour. Chosen arbitrary as big
// enough to not affect normal use-cases. Yet it is smaller than RTP wrap-around
// half-period (90khz RTP clock wrap-arounds every 13.25 hours). After half of
// wrap-around period it is impossible to unwrap RTP timestamps correctly.
const int kMaxAllowedRtcpNtpIntervalMs = 60 * 60 * 1000;

bool Contains(const std::list<RtpToNtpEstimator::RtcpMeasurement>& measurements,
              const RtpToNtpEstimator::RtcpMeasurement& other) {
  for (const auto& measurement : measurements) {
    if (measurement.IsEqual(other))
      return true;
  }
  return false;
}

// Given x[] and y[] writes out such k and b that line y=k*x+b approximates
// given points in the best way (Least Squares Method).
bool LinearRegression(std::vector<double>& x,
                      std::vector<double>& y,
                      double* k,
                      double* b) {
  size_t n = x.size();
  if (n == 0)
    return false;

  long double sum_xx = 0;
  long double sum_x = 0;
  long double sum_y = 0;
  long double sum_xy = 0;
  for (size_t i = 0; i < n; ++i) {
    sum_x += x[i];
    sum_y += y[i];
    sum_xx += static_cast<long double>(x[i]) * x[i];
    sum_xy += static_cast<long double>(x[i]) * y[i];
  }
  long double d_x = sum_xx * n - sum_x * sum_x;
  if (std::fabs(d_x) < 1e-8)
    return false;
  long double cov_xy = sum_xy * n - sum_x * sum_y;

  *k = static_cast<double>((cov_xy) / (d_x));
  *b = static_cast<double>((sum_y - (*k) * sum_x) / n);

  return true;
}

}  // namespace

RtpToNtpEstimator::RtcpMeasurement::RtcpMeasurement(uint32_t ntp_secs,
                                                    uint32_t ntp_frac,
                                                    int64_t unwrapped_timestamp)
    : ntp_time(ntp_secs, ntp_frac),
      unwrapped_rtp_timestamp(unwrapped_timestamp) {}

bool RtpToNtpEstimator::RtcpMeasurement::IsEqual(
    const RtcpMeasurement& other) const {
  // Use || since two equal timestamps will result in zero frequency and in
  // RtpToNtpMs, |rtp_timestamp_ms| is estimated by dividing by the frequency.
  return (ntp_time == other.ntp_time) ||
         (unwrapped_rtp_timestamp == other.unwrapped_rtp_timestamp);
}

// Class for converting an RTP timestamp to the NTP domain.
RtpToNtpEstimator::RtpToNtpEstimator()
    : consecutive_invalid_samples_(0),
      params_calculated_(false) {}

RtpToNtpEstimator::~RtpToNtpEstimator() {}

void RtpToNtpEstimator::UpdateParameters() {
  if (measurements_.size() < 2)
    return;

  std::vector<double> x;
  std::vector<double> y;
  int64_t base_rtp_ts = measurements_.front().unwrapped_rtp_timestamp;
  int64_t base_ntp_ts = measurements_.front().ntp_time.ToMs();
  for (auto it = measurements_.begin(); it != measurements_.end(); ++it) {
    x.push_back(it->unwrapped_rtp_timestamp - base_rtp_ts);
    y.push_back(it->ntp_time.ToMs() - base_ntp_ts);
  }
  double slope, offset;

  if (!LinearRegression(x, y, &slope, &offset)) {
    return;
  }

  params_.frequency_khz = 1 / slope;
  params_.offset_ms = offset + base_ntp_ts - slope * base_rtp_ts;
  params_calculated_ = true;
}

bool RtpToNtpEstimator::UpdateMeasurements(uint32_t ntp_secs,
                                           uint32_t ntp_frac,
                                           uint32_t rtp_timestamp,
                                           bool* new_rtcp_sr) {
  *new_rtcp_sr = false;

  int64_t unwrapped_rtp_timestamp = unwrapper_.Unwrap(rtp_timestamp);

  RtcpMeasurement new_measurement(ntp_secs, ntp_frac, unwrapped_rtp_timestamp);

  if (Contains(measurements_, new_measurement)) {
    // RTCP SR report already added.
    return true;
  }

  if (!new_measurement.ntp_time.Valid())
    return false;

  int64_t ntp_ms_new = new_measurement.ntp_time.ToMs();
  bool invalid_sample = false;
  if (!measurements_.empty()) {
    int64_t old_rtp_timestamp = measurements_.front().unwrapped_rtp_timestamp;
    int64_t old_ntp_ms = measurements_.front().ntp_time.ToMs();
    if (ntp_ms_new <= old_ntp_ms ||
        ntp_ms_new > old_ntp_ms + kMaxAllowedRtcpNtpIntervalMs) {
      invalid_sample = true;
    } else if (unwrapped_rtp_timestamp <= old_rtp_timestamp) {
      RTC_LOG(LS_WARNING)
          << "Newer RTCP SR report with older RTP timestamp, dropping";
      invalid_sample = true;
    } else if (unwrapped_rtp_timestamp - old_rtp_timestamp > (1 << 25)) {
      // Sanity check. No jumps too far into the future in rtp.
      invalid_sample = true;
    }
  }

  if (invalid_sample) {
    ++consecutive_invalid_samples_;
    if (consecutive_invalid_samples_ < kMaxInvalidSamples) {
      return false;
    }
    RTC_LOG(LS_WARNING) << "Multiple consecutively invalid RTCP SR reports, "
                           "clearing measurements.";
    measurements_.clear();
    params_calculated_ = false;
  }
  consecutive_invalid_samples_ = 0;

  // Insert new RTCP SR report.
  if (measurements_.size() == kNumRtcpReportsToUse)
    measurements_.pop_back();

  measurements_.push_front(new_measurement);
  *new_rtcp_sr = true;

  // List updated, calculate new parameters.
  UpdateParameters();
  return true;
}

bool RtpToNtpEstimator::Estimate(int64_t rtp_timestamp,
                                 int64_t* ntp_timestamp_ms) const {
  if (!params_calculated_)
    return false;

  int64_t rtp_timestamp_unwrapped = unwrapper_.Unwrap(rtp_timestamp);

  // params_calculated_ should not be true unless ms params.frequency_khz has
  // been calculated to something non zero.
  RTC_DCHECK_NE(params_.frequency_khz, 0.0);
  double rtp_ms =
      static_cast<double>(rtp_timestamp_unwrapped) / params_.frequency_khz +
      params_.offset_ms + 0.5f;

  if (rtp_ms < 0)
    return false;

  *ntp_timestamp_ms = rtp_ms;

  return true;
}

const absl::optional<RtpToNtpEstimator::Parameters> RtpToNtpEstimator::params()
    const {
  if (!params_calculated_) {
    return absl::nullopt;
  }
  return params_;
}
}  // namespace webrtc
