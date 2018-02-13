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
#include "modules/audio_processing/agc2/limiter.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {
namespace {
static const Limiter limiter;

std::pair<double, double> ComputeLinearApproximationParams(const double x) {
  const double m = limiter.GetGainFirstDerivativeLinear(x);
  const double q = limiter.GetGainLinear(x) - m * x;
  return {m, q};
}

double ComputeAreaUnderPiecewiseLinearApproximation(const double x0,
                                                    const double x1) {
  RTC_CHECK_LT(x0, x1);

  // Linear approximation in x0 and x1.
  double m0, q0, m1, q1;
  std::tie(m0, q0) = ComputeLinearApproximationParams(x0);
  std::tie(m1, q1) = ComputeLinearApproximationParams(x1);

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
double LimiterUnderApproximationNegativeError(const double x0,
                                              const double x1) {
  const double area_limiter = limiter.GetGainIntegralLinear(x0, x1);
  const double area_interpolated_curve =
      ComputeAreaUnderPiecewiseLinearApproximation(x0, x1);
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
std::vector<double> SampleLimiterRegion() {
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
  q.emplace(
      limiter.limiter_start_linear(), limiter.max_input_level_linear(),
      LimiterUnderApproximationNegativeError(limiter.limiter_start_linear(),
                                             limiter.max_input_level_linear()));

  // Iteratively find points by halving the interval with greatest error.
  for (size_t i = 0; i < kInterpolatedGainCurveBeyondKneePoints - 1; ++i) {
    // Get the interval with highest error.
    const auto interval = q.top();
    q.pop();

    // Split |interval| and enqueue.
    double x_split = (interval.x0 + interval.x1) / 2.0;
    q.emplace(interval.x0, x_split,
              LimiterUnderApproximationNegativeError(interval.x0,
                                                     x_split));  // Left.
    q.emplace(x_split, interval.x1,
              LimiterUnderApproximationNegativeError(x_split,
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

// Compute the parameters to over-approximate the knee region via linear
// interpolation. Over-approximating is saturation-safe since the knee region is
// convex.
void PrecomputeKneeApproxParams() {
  RTC_CHECK(kInterpolatedGainCurveKneePoints > 2);
  // Get |kInterpolatedGainCurveKneePoints| - 1 equally spaced points.
  const std::vector<double> points =
      LinSpace(limiter.knee_start_linear(), limiter.limiter_start_linear(),
               kInterpolatedGainCurveKneePoints - 1);

  // Set the first two points. The second is computed to help with the beginning
  // of the knee region, which has high curvature.
  test::computed_approximation_params_x[0] = points[0];
  test::computed_approximation_params_x[1] = (points[0] + points[1]) / 2.0;
  // Copy the remaining points.
  std::copy(std::begin(points) + 1, std::end(points),
            std::begin(test::computed_approximation_params_x) + 2);

  // Compute (m, q) pairs for each linear piece y = mx + q.
  for (size_t i = 0; i < kInterpolatedGainCurveKneePoints - 1; ++i) {
    const double x0 = test::computed_approximation_params_x[i];
    const double x1 = test::computed_approximation_params_x[i + 1];
    const double y0 = limiter.GetGainLinear(x0);
    const double y1 = limiter.GetGainLinear(x1);
    test::computed_approximation_params_m[i] = (y1 - y0) / (x1 - x0);
    test::computed_approximation_params_q[i] =
        y0 - test::computed_approximation_params_m[i] * x0;
  }
}

// Compute the parameters to under-approximate the beyond-knee region via linear
// interpolation and greedy sampling. Under-approximating is saturation-safe
// since the beyond-knee region is concave.
void PrecomputeBeyondKneeApproxParams() {
  // Find points on which the linear pieces are tangent to the gain curve.
  const auto samples = SampleLimiterRegion();

  // Parametrize each linear piece.
  double m, q;
  std::tie(m, q) = ComputeLinearApproximationParams(
      test::computed_approximation_params_x[kInterpolatedGainCurveKneePoints -
                                            1]);
  test::computed_approximation_params_m[kInterpolatedGainCurveKneePoints - 1] =
      m;
  test::computed_approximation_params_q[kInterpolatedGainCurveKneePoints - 1] =
      q;
  for (size_t i = 0; i < samples.size(); ++i) {
    std::tie(m, q) = ComputeLinearApproximationParams(samples[i]);
    test::computed_approximation_params_m[i +
                                          kInterpolatedGainCurveKneePoints] = m;
    test::computed_approximation_params_q[i +
                                          kInterpolatedGainCurveKneePoints] = q;
  }

  // Find the point of intersection between adjacent linear pieces. They will be
  // used as boundaries between adjacent linear pieces.
  for (size_t i = kInterpolatedGainCurveKneePoints;
       i < kInterpolatedGainCurveKneePoints +
               kInterpolatedGainCurveBeyondKneePoints;
       ++i) {
    test::computed_approximation_params_x[i] =
        (  // Formula: (q0 - q1) / (m1 - m0).
            test::computed_approximation_params_q[i - 1] -
            test::computed_approximation_params_q[i]) /
        (test::computed_approximation_params_m[i] -
         test::computed_approximation_params_m[i - 1]);
  }
}

}  // namespace

namespace test {

std::array<float, kInterpolatedGainCurveTotalPoints>
    computed_approximation_params_x;
std::array<float, kInterpolatedGainCurveTotalPoints>
    computed_approximation_params_m;
std::array<float, kInterpolatedGainCurveTotalPoints>
    computed_approximation_params_q;

void ComputeInterpolatedGainCurveApproximationParams() {
  // TODO(alessiob): When switching to C++14, make sure that the next 3 lines
  // are only executed at compile time (this will required changes in this class
  // as well as in Limiter).
  test::computed_approximation_params_x.fill(0.0f);
  test::computed_approximation_params_m.fill(0.0f);
  test::computed_approximation_params_q.fill(0.0f);
  PrecomputeKneeApproxParams();
  PrecomputeBeyondKneeApproxParams();
}
}  // namespace test
}  // namespace webrtc
