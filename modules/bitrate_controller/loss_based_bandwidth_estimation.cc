/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/bitrate_controller/loss_based_bandwidth_estimation.h"

#include <algorithm>
#include <string>

#include "system_wrappers/include/field_trial.h"

namespace webrtc {
namespace {
using ::webrtc::field_trial::IsEnabled;
const char kBweLossBasedControl[] = "WebRTC-Bwe-LossBasedControl";

float GetIncreaseFactor(const LossBasedControlConfig& config, TimeDelta rtt) {
  // Increase slower when RTT is high.
  if (rtt > config.increase_high_rtt || rtt.ms() == 0) {
    return config.min_increase_factor;
  } else if (rtt < config.increase_low_rtt) {
    return config.max_increase_factor;
  }

  double slope =
      (config.min_increase_factor - config.max_increase_factor) /
      (config.increase_high_rtt->ms() - config.increase_low_rtt->ms());
  return config.max_increase_factor +
         slope * (rtt - config.increase_low_rtt).ms();
}

float loss_from_bitrate(DataRate bitrate, float factor) {
  return sqrt(factor / bitrate.kbps());
}

DataRate bitrate_from_loss(float loss, float factor) {
  return DataRate::kbps(factor / (loss * loss + 1e-8));
}
}  // namespace

LossBasedControlConfig::LossBasedControlConfig()
    : enabled(IsEnabled(kBweLossBasedControl)),
      min_increase_factor("min_incr", 1.02),
      max_increase_factor("max_incr", 1.08),
      increase_low_rtt("incr_low_rtt", TimeDelta::ms(200)),
      increase_high_rtt("incr_high_rtt", TimeDelta::ms(800)),
      decrease_factor("decr", 0.99),
      smoothing_factor("smoothing", 0.5),
      loss_maxtrack_forgetting_factor("loss_max_forget", 0.5),
      acknowledged_rate_maxtrack_forgetting_factor("ack_rate_max_forget", 0.5),
      increase_offset("incr_offset", DataRate::bps(1000)),
      increase_threshold_scaler("incr_scaler", 0.5),
      decrease_threshold_scaler("decr_scaler", 4),
      allow_resets("resets", false),
      decrease_interval("decr_intvl", TimeDelta::ms(300)),
      loss_report_timeout("timeout", TimeDelta::ms(6000)) {
  std::string trial_string = field_trial::FindFullName(kBweLossBasedControl);
  ParseFieldTrial(
      {&min_increase_factor, &max_increase_factor, &increase_low_rtt,
       &increase_high_rtt, &decrease_factor, &smoothing_factor,
       &loss_maxtrack_forgetting_factor,
       &acknowledged_rate_maxtrack_forgetting_factor, &increase_offset,
       &increase_threshold_scaler, &decrease_threshold_scaler, &allow_resets,
       &decrease_interval, &loss_report_timeout},
      trial_string);
}
LossBasedControlConfig::LossBasedControlConfig(const LossBasedControlConfig&) =
    default;
LossBasedControlConfig::~LossBasedControlConfig() = default;

void LossBasedBandwidthEstimation::UpdateLossStatistics(float loss) {
  average_loss_ += config_.smoothing_factor * (loss - average_loss_);
  if (average_loss_ > average_loss_max_) {
    average_loss_max_ = average_loss_;
  } else {
    average_loss_max_ += config_.loss_maxtrack_forgetting_factor *
                         (average_loss_ - average_loss_max_);
  }
}

LossBasedBandwidthEstimation::LossBasedBandwidthEstimation()
    : config_(LossBasedControlConfig()),
      average_loss_(0),
      average_loss_max_(0),
      loss_based_bitrate_(DataRate::Zero()),
      acknowledged_bitrate_(DataRate::Zero()),
      time_last_decrease_(Timestamp::MinusInfinity()),
      has_decreased_since_last_fraction_loss_(false),
      last_loss_packet_report_(Timestamp::MinusInfinity()),
      last_fraction_loss_(0) {}

void LossBasedBandwidthEstimation::UpdateAcknowledgedBitrate(
    DataRate acknowledged_bitrate) {
  if (acknowledged_bitrate > acknowledged_bitrate_) {
    acknowledged_bitrate_ = acknowledged_bitrate;
  } else {
    acknowledged_bitrate_ +=
        config_.acknowledged_rate_maxtrack_forgetting_factor *
        (acknowledged_bitrate - acknowledged_bitrate_);
  }
}

void LossBasedBandwidthEstimation::Update(Timestamp at_time,
                                          DataRate min_bitrate,
                                          TimeDelta last_round_trip_time) {
  TimeDelta time_since_loss_packet_report = at_time - last_loss_packet_report_;

  if (time_since_loss_packet_report < config_.loss_report_timeout) {
    float loss = last_fraction_loss_ / 256.0f;
    UpdateLossStatistics(loss);
    // Only increase if loss has been low for some time.
    float loss_increase_metric = average_loss_max_;
    // Avoid multiple decreases from averaging over one loss spike.
    float loss_decrease_metric = std::min(average_loss_, loss);

    float loss_increase_threshold = loss_from_bitrate(
        loss_based_bitrate_, config_.increase_threshold_scaler);
    float loss_decrease_threshold = loss_from_bitrate(
        loss_based_bitrate_, config_.decrease_threshold_scaler);
    bool allow_decrease = !has_decreased_since_last_fraction_loss_ &&
                          (at_time - time_last_decrease_ >=
                           last_round_trip_time + config_.decrease_interval);

    if (loss_increase_metric < loss_increase_threshold) {
      // Increase bitrate by RTT-adaptive ratio.
      DataRate new_bitrate_incr =
          DataRate::bps(0.5 +
                        min_bitrate.bps() *
                            GetIncreaseFactor(config_, last_round_trip_time)) +
          config_.increase_offset;
      // The bitrate that would make the loss "just high enough".
      DataRate new_bitrate_by_bound = bitrate_from_loss(
          loss_increase_metric, config_.increase_threshold_scaler);
      if (new_bitrate_by_bound < new_bitrate_incr) {
        new_bitrate_incr = new_bitrate_by_bound;
      }
      if (new_bitrate_incr > loss_based_bitrate_) {
        loss_based_bitrate_ = new_bitrate_incr;
      }
    } else if (loss_decrease_metric > loss_decrease_threshold &&
               allow_decrease) {
      DataRate new_bitrate_decr =
          config_.decrease_factor * acknowledged_bitrate_;
      // The bitrate that would make the loss "just acceptable".
      DataRate new_bitrate_by_bound = bitrate_from_loss(
          loss_decrease_metric, config_.decrease_threshold_scaler);
      if (new_bitrate_by_bound > new_bitrate_decr) {
        new_bitrate_decr = new_bitrate_by_bound;
      }
      if (new_bitrate_decr < loss_based_bitrate_) {
        time_last_decrease_ = at_time;
        has_decreased_since_last_fraction_loss_ = true;
        loss_based_bitrate_ = new_bitrate_decr;
      }
    }
  }
}

}  // namespace webrtc
