/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/compute_interpolated_gain_curve.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {
namespace {

std::pair<double, double> ComputeLinearApproximationParams(
    const double x,
    const Limiter* limiter) {
  const double m = limiter->GetGainFirstDerivativeLinear(x);
  const double q = limiter->GetGainLinear(x) - m * x;
  return {m, q};
}

double ComputeAreaUnderPiecewiseLinearApproximation(const double x0,
                                                    const double x1,
                                                    const Limiter* limiter) {
  RTC_CHECK_LT(x0, x1);

  // Linear approximation in x0 and x1.
  double m0, q0, m1, q1;
  std::tie(m0, q0) = ComputeLinearApproximationParams(x0, limiter);
  std::tie(m1, q1) = ComputeLinearApproximationParams(x1, limiter);

  // Intersection point between two adjacent linear pieces.
  const double x_split = (q0 - q1) / (m1 - m0);
  RTC_CHECK_LT(x0, x_split);
  RTC_CHECK_LT(x_split, x1);

  auto area_under_linear_piece = [](double x_l, double x_r, double m,
                                    double q) {
    return x_r * (m * x_r / 2.0 + q) - x_l * (m * x_l / 2.0 + q);
  };
  return area_under_linear_piece(x0, x_split, m0, q0) +
         area_under_linear_piece(x_split, x1, m1, q1);
}

// Computes the approximation error in the limiter region for a given interval.
// The error is computed as the difference between the areas beneath the limiter
// curve to approximate and its linear under-approximation.
double LimiterUnderApproximationNegativeError(const Limiter* limiter,
                                              const double x0,
                                              const double x1) {
  const double area_limiter = limiter->GetGainIntegralLinear(x0, x1);
  const double area_interpolated_curve =
      ComputeAreaUnderPiecewiseLinearApproximation(x0, x1, limiter);
  RTC_CHECK_GE(area_limiter, area_interpolated_curve);
  return area_limiter - area_interpolated_curve;
}

// Automatically finds where to sample the beyond-knee region of a limiter using
// a greedy optimization algorithm that iteratively decreases the approximation
// error.
// The solution is sub-optimal because the algorithm is greedy and the points
// are assigned by halving intervals (starting with the whole beyond-knee region
// as a single interval). However, even if sub-optimal, this algorithm works
// well in practice and it is efficiently implemented using priority queues.
std::vector<double> SampleLimiterRegion(const Limiter* limiter) {
  RTC_CHECK(kInterpolatedGainCurveBeyondKneePoints > 2);

  struct Interval {
    Interval() = default;  // Ctor required by std::priority_queue.
    Interval(double l, double r, double e) : x0(l), x1(r), error(e) {
      RTC_CHECK(x0 < x1);
    }
    double x0;
    double x1;
    double error;
  };

  struct IntervalComparator {
    bool operator()(const Interval& a, const Interval& b) const {
      return a.error < b.error;
    }
  };

  // Init.
  std::priority_queue<Interval, std::vector<Interval>, IntervalComparator> q;
  q.emplace(limiter->limiter_start_linear(), limiter->max_input_level_linear(),
            LimiterUnderApproximationNegativeError(
                limiter, limiter->limiter_start_linear(),
                limiter->max_input_level_linear()));

  // Iteratively find points by halving the interval with greatest error.
  for (size_t i = 0; i < kInterpolatedGainCurveBeyondKneePoints - 1; ++i) {
    // Get the interval with highest error.
    const auto interval = q.top();
    q.pop();

    // Split |interval| and enqueue.
    double x_split = (interval.x0 + interval.x1) / 2.0;
    q.emplace(interval.x0, x_split,
              LimiterUnderApproximationNegativeError(limiter, interval.x0,
                                                     x_split));  // Left.
    q.emplace(x_split, interval.x1,
              LimiterUnderApproximationNegativeError(limiter, x_split,
                                                     interval.x1));  // Right.
  }

  // Copy x1 values and sort them.
  RTC_CHECK_EQ(q.size(), kInterpolatedGainCurveBeyondKneePoints);
  std::vector<double> samples(kInterpolatedGainCurveBeyondKneePoints);
  for (size_t i = 0; i < kInterpolatedGainCurveBeyondKneePoints; ++i) {
    const auto interval = q.top();
    q.pop();
    samples[i] = interval.x1;
  }
  RTC_CHECK(q.empty());
  std::sort(samples.begin(), samples.end());

  return samples;
}

}  // namespace

ComputeInterpolatedGainCurveCoefficients::
    ComputeInterpolatedGainCurveCoefficients() {
  // TODO(alessiob): When switching to C++14, make sure that the next 3 lines
  // are only executed at compile time (this will required changes in this class
  // as well as in Limiter).
  approximation_params_x_.fill(0.0f);
  PrecomputeKneeApproxParams();
  PrecomputeBeyondKneeApproxParams();
}

// Compute the parameters to over-approximate the knee region via linear
// interpolation. Over-approximating is saturation-safe since the knee region is
// convex.
void ComputeInterpolatedGainCurveCoefficients::PrecomputeKneeApproxParams() {
  RTC_CHECK(kInterpolatedGainCurveKneePoints > 2);
  // Get |kInterpolatedGainCurveKneePoints| - 1 equally spaced points.
  const std::vector<double> points =
      LinSpace(limiter_.knee_start_linear(), limiter_.limiter_start_linear(),
               kInterpolatedGainCurveKneePoints - 1);

  // Set the first two points. The second is computed to help with the beginning
  // of the knee region, which has high curvature.
  approximation_params_x_[0] = points[0];
  approximation_params_x_[1] = (points[0] + points[1]) / 2.0;
  // Copy the remaining points.
  std::copy(std::begin(points) + 1, std::end(points),
            std::begin(approximation_params_x_) + 2);

  // Compute (m, q) pairs for each linear piece y = mx + q.
  for (size_t i = 0; i < kInterpolatedGainCurveKneePoints - 1; ++i) {
    const double x0 = approximation_params_x_[i];
    const double x1 = approximation_params_x_[i + 1];
    const double y0 = limiter_.GetGainLinear(x0);
    const double y1 = limiter_.GetGainLinear(x1);
    approximation_params_m_[i] = (y1 - y0) / (x1 - x0);
    approximation_params_q_[i] = y0 - approximation_params_m_[i] * x0;
  }
}

// Compute the parameters to under-approximate the beyond-knee region via linear
// interpolation and greedy sampling. Under-approximating is saturation-safe
// since the beyond-knee region is concave.
void ComputeInterpolatedGainCurveCoefficients::
    PrecomputeBeyondKneeApproxParams() {
  // Find points on which the linear pieces are tangent to the gain curve.
  const auto samples = SampleLimiterRegion(&limiter_);

  // Parametrize each linear piece.
  double m, q;
  std::tie(m, q) = ComputeLinearApproximationParams(
      approximation_params_x_[kInterpolatedGainCurveKneePoints - 1], &limiter_);
  approximation_params_m_[kInterpolatedGainCurveKneePoints - 1] = m;
  approximation_params_q_[kInterpolatedGainCurveKneePoints - 1] = q;
  for (size_t i = 0; i < samples.size(); ++i) {
    std::tie(m, q) = ComputeLinearApproximationParams(samples[i], &limiter_);
    approximation_params_m_[i + kInterpolatedGainCurveKneePoints] = m;
    approximation_params_q_[i + kInterpolatedGainCurveKneePoints] = q;
  }

  // Find the point of intersection between adjacent linear pieces. They will be
  // used as boundaries between adjacent linear pieces.
  for (size_t i = kInterpolatedGainCurveKneePoints;
       i < kInterpolatedGainCurveKneePoints +
               kInterpolatedGainCurveBeyondKneePoints;
       ++i) {
    approximation_params_x_[i] =
        (  // Formula: (q0 - q1) / (m1 - m0).
            approximation_params_q_[i - 1] - approximation_params_q_[i]) /
        (approximation_params_m_[i] - approximation_params_m_[i - 1]);
  }
}

}  // namespace webrtc
