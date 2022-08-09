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

// This class uses a linear Kalman filter (*) to estimate the frame delay
// delta (i.e., the difference in transmission time between a frame and the
// prior frame) for a frame, given its size delta in bytes (i.e., the difference
// in size between a frame and the prior frame). The idea is that, given a
// fixed link bandwidth, a larger frame (in bytes) would take proportionally
// longer to arrive than a correspondingly smaller frame. Using the variations
// of frame delay deltas and frame size deltas, the underlying bandwidth and
// propagation delay of the network link can be estimated.
//
// The filter takes as input the frame delay delta and frame size delta, for a
// single frame. The hidden state is the link bandwidth and propagation
// delay. The estimated state can be used to get the expected frame delay delta
// for a frame, given it's frame size delta. This information can then be used
// to estimate the frame delay variation coming from network jitter.
//
// Mathematical details:
//  * The state transition matrix `F` is the identity, meaning that the modeled
//    system is constant.
//  * The observation matrix `H` is [frame_size_delta_bytes, 1.0].
//  * The process noise covariance `Q` is diagonal and constant.
//  * The observation noise covariance `R` is xyz.
//
// (*) https://en.wikipedia.org/wiki/Kalman_filter
class FrameDelayDeltaKalmanFilter {
 public:
  FrameDelayDeltaKalmanFilter();
  ~FrameDelayDeltaKalmanFilter() = default;

  // Predicts and updates the state of the filter, given a new measurement pair.
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
  // by both link bandwidth and link propagation delay.
  double GetFrameDelayDeltaEstimateTotal(double frame_size_delta_bytes) const;

 private:
  // Type for keeping track of the current filter state.
  enum class LogicalState {
    kInitialized,
    kPredicted,
    kUpdated,
  };

  // Performs the predict step
  // (https://en.wikipedia.org/wiki/Kalman_filter#Predict). Should only be
  // called from `kInitialized` or `kUpdated`.
  void Predict();

  // Performs the update step
  // (https://en.wikipedia.org/wiki/Kalman_filter#Update). Should only be called
  // from `kPredicted`.
  void Update(TimeDelta frame_delay_delta,
              double frame_size_delta_bytes,
              DataSize max_frame_size,
              double var_noise);

  // Current logical state of the filter.
  LogicalState logical_state_ = LogicalState::kInitialized;

  // State vector estimate.
  double estimate_[2];  // Estimated line parameters:
                        //   (slope [1 / bytes per ms], offset [ms])

  // Estimate covariance. Contains the a priori estimate covariance when
  // in `kPredicted` and the a posteriori estimate covariance when in
  // `kUpdated`.
  double estimate_cov_[2][2];  // Estimate covariance.

  // Process noise covariance.
  double process_noise_cov_[2][2];  // Process noise covariance.
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_TIMING_FRAME_DELAY_DELTA_KALMAN_FILTER_H_
