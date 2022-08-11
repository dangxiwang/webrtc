/*
 *  Copyright (c) 2022 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_VIDEO_CODING_TIMING_FRAME_DELAY_DELTA_KALMAN_FILTER_H_
#define MODULES_VIDEO_CODING_TIMING_FRAME_DELAY_DELTA_KALMAN_FILTER_H_

#include "api/units/data_size.h"
#include "api/units/time_delta.h"

namespace webrtc {

// This class uses a linear Kalman filter (see
// https://en.wikipedia.org/wiki/Kalman_filter) to estimate the frame delay
// delta (i.e., the difference in transmission time between a frame and the
// prior frame) for a frame, given its size delta in bytes (i.e., the difference
// in size between a frame and the prior frame). The idea is that, given a
// fixed link bandwidth, a larger frame (in bytes) would take proportionally
// longer to arrive than a correspondingly smaller frame. Using the variations
// of frame delay deltas and frame size deltas, the underlying bandwidth and
// queuing delay delta of the network link can be estimated.
//
// The filter takes as input the frame delay delta, the difference between the
// actual inter-frame arrival time and the expected inter-frame arrival time
// (based on RTP timestamp), and frame size delta, the inter-frame size delta,
// for a single frame. The frame delay delta is seen as the measurement, and the
// frame size delta is used in the observation model. The hidden state of the
// filter is the link bandwidth and queuing delay delta. The estimated state
// can be used to get the expected frame delay delta for a frame, given it's
// frame size delta. This information can then be used to estimate the frame
// delay variation coming from network jitter.
//
// Mathematical details:
//  * The state (`x` in Wikipedia notation) is a 2x1 vector comprising the
//    reciprocal of link bandwidth [1 / bytes per ms] and the
//    link queuing delay delta [ms].
//  * The state transition matrix (`F`) is the 2x2 identity matrix, meaning that
//    link bandwidth and link queuing delay delta are modeled as independent.
//  * The measurement (`z`) is the (scalar) frame delay delta [ms].
//  * The observation matrix (`H`) is a 1x2 vector set as
//    `{frame_size_delta [bytes], 1.0}`.
//  * The process noise covariance (`Q`) is a constant 2x2 diagonal matrix
//    [(1 / bytes per ms)^2, ms^2].
//  * The observation noise covariance (`r`) is a scalar [ms^2] that is
//    determined externally to this class.
class FrameDelayDeltaKalmanFilter {
 public:
  FrameDelayDeltaKalmanFilter();
  ~FrameDelayDeltaKalmanFilter() = default;

  // Predicts and updates the filter, given a new pair of frame delay delta and
  // frame size delta.
  //
  // Inputs:
  // `frame_delay_delta`:
  //    Frame delay delta as calculated by the `InterFrameDelay` estimator.
  //
  // `frame_size_delta_bytes`:
  //    Frame size delta, i.e., the current frame size minus the previous
  //    frame size (in bytes). Note that this quantity may be negative.
  //
  // `max_frame_size`:
  //    Filtered largest frame size received since the last reset.
  //
  // `var_noise`:
  //    Variance of the estimated random jitter.
  void PredictAndUpdate(TimeDelta frame_delay_delta,
                        double frame_size_delta_bytes,
                        DataSize max_frame_size,
                        double var_noise);

  // Given a frame size delta, returns the estimated frame delay delta explained
  // by the link bandwidth alone.
  double GetFrameDelayDeltaEstimateSizeBased(
      double frame_size_delta_bytes) const;

  // Given a frame size delta, returns the estimated frame delay delta explained
  // by both link bandwidth and link queuing delay delta.
  double GetFrameDelayDeltaEstimateTotal(double frame_size_delta_bytes) const;

 private:
  // State estimate (bandwidth [1 / bytes per ms], queue buildup [ms]).
  double estimate_[2];
  double estimate_cov_[2][2];  // Estimate covariance.

  double process_noise_cov_[2][2];  // Process noise covariance.
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_TIMING_FRAME_DELAY_DELTA_KALMAN_FILTER_H_
