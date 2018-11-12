/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_BITRATE_CONTROLLER_LOSS_BASED_BANDWIDTH_ESTIMATION_H_
#define MODULES_BITRATE_CONTROLLER_LOSS_BASED_BANDWIDTH_ESTIMATION_H_

#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "rtc_base/experiments/field_trial_parser.h"

namespace webrtc {

struct LossBasedControlConfig {
  LossBasedControlConfig();
  LossBasedControlConfig(const LossBasedControlConfig&);
  LossBasedControlConfig& operator=(const LossBasedControlConfig&) = default;
  ~LossBasedControlConfig();
  bool enabled;
  FieldTrialParameter<double> min_increase_factor;
  FieldTrialParameter<double> max_increase_factor;
  FieldTrialParameter<TimeDelta> increase_low_rtt;
  FieldTrialParameter<TimeDelta> increase_high_rtt;
  FieldTrialParameter<double> decrease_factor;
  FieldTrialParameter<double> smoothing_factor;
  FieldTrialParameter<double> loss_maxtrack_forgetting_factor;
  FieldTrialParameter<double> acknowledged_rate_maxtrack_forgetting_factor;
  FieldTrialParameter<DataRate> increase_offset;
  FieldTrialParameter<double> increase_threshold_scaler;
  FieldTrialParameter<double> decrease_threshold_scaler;
  FieldTrialParameter<bool> allow_resets;
  FieldTrialParameter<TimeDelta> decrease_interval;
  FieldTrialParameter<TimeDelta> loss_report_timeout;
};

class LossBasedBandwidthEstimation {
 public:
  LossBasedBandwidthEstimation();
  void Update(Timestamp at_time,
              DataRate min_bitrate,
              TimeDelta last_round_trip_time);
  void UpdateAcknowledgedBitrate(DataRate acknowledged_bitrate);
  void MaybeReset(DataRate bitrate) {
    if (config_.allow_resets)
      loss_based_bitrate_ = bitrate;
  }
  void SetInitialBitrate(DataRate bitrate) { loss_based_bitrate_ = bitrate; }
  bool Enabled() const { return config_.enabled; }
  void SetLoss(float last_fraction_loss, Timestamp at_time) {
    last_fraction_loss_ = last_fraction_loss;
    last_loss_packet_report_ = at_time;
    has_decreased_since_last_fraction_loss_ = false;
  }
  DataRate GetEstimate() const { return loss_based_bitrate_; }

 private:
  void UpdateLossStatistics(float loss);

  LossBasedControlConfig config_;
  float average_loss_;
  float average_loss_max_;
  DataRate loss_based_bitrate_;
  DataRate acknowledged_bitrate_;
  Timestamp time_last_decrease_;
  bool has_decreased_since_last_fraction_loss_;
  Timestamp last_loss_packet_report_;
  float last_fraction_loss_;
};

}  // namespace webrtc

#endif  // MODULES_BITRATE_CONTROLLER_LOSS_BASED_BANDWIDTH_ESTIMATION_H_
