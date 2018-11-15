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
#include <vector>

#include "api/units/data_rate.h"
#include "system_wrappers/include/field_trial.h"

namespace webrtc {
namespace {
const char kBweLossBasedControl[] = "WebRTC-Bwe-LossBasedControl";

double GetIncreaseFactor(const LossBasedControlConfig& config, TimeDelta rtt) {
  // Increase slower when RTT is high.
  if (rtt > config.increase_high_rtt || rtt.ms() == 0) {
    return config.min_increase_factor;
  } else if (rtt < config.increase_low_rtt) {
    return config.max_increase_factor;
  }

  return config.max_increase_factor +
         config.increase_factor_slope * (rtt - config.increase_low_rtt).ms();
}

double loss_from_bitrate(DataRate bitrate,
                         double loss_bandwidth_balance,
                         double exponent) {
  const double epsilon = 1e-8;  // Avoid division by zero.
  return pow(loss_bandwidth_balance / (bitrate.kbps<double>() + epsilon),
             exponent);
}

DataRate bitrate_from_loss(double loss,
                           double loss_bandwidth_balance,
                           double exponent) {
  const double epsilon = 1e-8;  // Avoid division by zero.
  return DataRate::kbps(loss_bandwidth_balance /
                        pow(loss + epsilon, 1.0 / exponent));
}

double exponential_update(double forget_per_sec, TimeDelta interval) {
  return 1.0f - pow(forget_per_sec, interval.seconds<double>());
}

}  // namespace

LossBasedControlConfig::LossBasedControlConfig()
    : enabled(field_trial::IsEnabled(kBweLossBasedControl)),
      min_increase_factor("min_incr", 1.02),
      max_increase_factor("max_incr", 1.08),
      increase_low_rtt("incr_low_rtt", TimeDelta::ms(200)),
      increase_high_rtt("incr_high_rtt", TimeDelta::ms(800)),
      decrease_factor("decr", 0.99),
      loss_forget_per_sec("loss_forget", 0.3),
      loss_max_forget_per_sec("loss_max_forget", 0.3),
      acknowledged_rate_max_forget_per_sec("ack_max_forget", 0.3),
      increase_offset("incr_offset", DataRate::bps(1000)),
      loss_bandwidth_balance_increase("balance_incr", 0.5),
      loss_bandwidth_balance_decrease("balance_decr", 4),
      loss_bandwidth_balance_exponent("exponent", 0.5),
      allow_resets("resets", false),
      decrease_interval("decr_intvl", TimeDelta::ms(300)),
      loss_report_timeout("timeout", TimeDelta::ms(6000)) {
  std::string trial_string = field_trial::FindFullName(kBweLossBasedControl);
  ParseFieldTrial(
      {&min_increase_factor, &max_increase_factor, &increase_low_rtt,
       &increase_high_rtt, &decrease_factor, &loss_forget_per_sec,
       &loss_max_forget_per_sec, &acknowledged_rate_max_forget_per_sec,
       &increase_offset, &loss_bandwidth_balance_increase,
       &loss_bandwidth_balance_decrease, &loss_bandwidth_balance_exponent,
       &allow_resets, &decrease_interval, &loss_report_timeout},
      trial_string);
  increase_factor_slope = (min_increase_factor - max_increase_factor) /
                          (increase_high_rtt->ms() - increase_low_rtt->ms());
}
LossBasedControlConfig::LossBasedControlConfig(const LossBasedControlConfig&) =
    default;
LossBasedControlConfig::~LossBasedControlConfig() = default;

LossBasedBandwidthEstimation::LossBasedBandwidthEstimation()
    : config_(LossBasedControlConfig()),
      average_loss_(0),
      average_loss_max_(0),
      loss_based_bitrate_(DataRate::Zero()),
      acknowledged_bitrate_max_(DataRate::Zero()),
      acknowledged_bitrate_last_update_(Timestamp::MinusInfinity()),
      time_last_decrease_(Timestamp::MinusInfinity()),
      has_decreased_since_last_loss_report_(false),
      last_loss_packet_report_(Timestamp::MinusInfinity()),
      last_loss_ratio_(0) {}

void LossBasedBandwidthEstimation::UpdateLossStatistics(
    const std::vector<PacketResult>& packet_results,
    Timestamp at_time) {
  if (packet_results.empty())
    return;
  int loss_count = 0;
  for (auto pkt : packet_results) {
    loss_count += pkt.receive_time.IsInfinite() ? 1 : 0;
  }
  last_loss_ratio_ = static_cast<double>(loss_count) / packet_results.size();
  TimeDelta time_passed = last_loss_packet_report_.IsFinite()
                              ? at_time - last_loss_packet_report_
                              : TimeDelta::seconds(1);
  last_loss_packet_report_ = at_time;
  has_decreased_since_last_loss_report_ = false;

  average_loss_ +=
      exponential_update(config_.loss_forget_per_sec, time_passed) *
      (last_loss_ratio_ - average_loss_);
  if (average_loss_ > average_loss_max_) {
    average_loss_max_ = average_loss_;
  } else {
    average_loss_max_ +=
        exponential_update(config_.loss_max_forget_per_sec, time_passed) *
        (average_loss_ - average_loss_max_);
  }
}

void LossBasedBandwidthEstimation::UpdateAcknowledgedBitrate(
    DataRate acknowledged_bitrate,
    Timestamp at_time) {
  TimeDelta time_passed = acknowledged_bitrate_last_update_.IsFinite()
                              ? at_time - acknowledged_bitrate_last_update_
                              : TimeDelta::seconds(1);
  acknowledged_bitrate_last_update_ = at_time;
  if (acknowledged_bitrate > acknowledged_bitrate_max_) {
    acknowledged_bitrate_max_ = acknowledged_bitrate;
  } else {
    acknowledged_bitrate_max_ +=
        exponential_update(config_.acknowledged_rate_max_forget_per_sec,
                           time_passed) *
        (acknowledged_bitrate - acknowledged_bitrate_max_);
  }
}

void LossBasedBandwidthEstimation::Update(Timestamp at_time,
                                          DataRate min_bitrate,
                                          TimeDelta last_round_trip_time) {
  TimeDelta time_since_loss_packet_report = at_time - last_loss_packet_report_;

  if (time_since_loss_packet_report < config_.loss_report_timeout) {
    // Only increase if loss has been low for some time.
    double loss_increase_metric = average_loss_max_;
    // Avoid multiple decreases from averaging over one loss spike.
    double loss_decrease_metric = std::min(average_loss_, last_loss_ratio_);

    double loss_increase_threshold = loss_from_bitrate(
        loss_based_bitrate_, config_.loss_bandwidth_balance_increase,
        config_.loss_bandwidth_balance_exponent);
    double loss_decrease_threshold = loss_from_bitrate(
        loss_based_bitrate_, config_.loss_bandwidth_balance_decrease,
        config_.loss_bandwidth_balance_exponent);
    bool allow_decrease = !has_decreased_since_last_loss_report_ &&
                          (at_time - time_last_decrease_ >=
                           last_round_trip_time + config_.decrease_interval);

    if (loss_increase_metric < loss_increase_threshold) {
      // Increase bitrate by RTT-adaptive ratio.
      DataRate new_increased_bitrate =
          min_bitrate * GetIncreaseFactor(config_, last_round_trip_time) +
          config_.increase_offset;
      // The bitrate that would make the loss "just high enough".
      DataRate new_increased_bitrate_cap = bitrate_from_loss(
          loss_increase_metric, config_.loss_bandwidth_balance_increase,
          config_.loss_bandwidth_balance_exponent);
      if (new_increased_bitrate_cap < new_increased_bitrate) {
        new_increased_bitrate = new_increased_bitrate_cap;
      }
      if (new_increased_bitrate > loss_based_bitrate_) {
        loss_based_bitrate_ = new_increased_bitrate;
      }
    } else if (loss_decrease_metric > loss_decrease_threshold &&
               allow_decrease) {
      DataRate new_decreased_bitrate =
          config_.decrease_factor * acknowledged_bitrate_max_;
      // The bitrate that would make the loss "just acceptable".
      DataRate new_decreased_bitrate_floor = bitrate_from_loss(
          loss_decrease_metric, config_.loss_bandwidth_balance_decrease,
          config_.loss_bandwidth_balance_exponent);
      if (new_decreased_bitrate_floor > new_decreased_bitrate) {
        new_decreased_bitrate = new_decreased_bitrate_floor;
      }
      if (new_decreased_bitrate < loss_based_bitrate_) {
        time_last_decrease_ = at_time;
        has_decreased_since_last_loss_report_ = true;
        loss_based_bitrate_ = new_decreased_bitrate;
      }
    }
  }
}

}  // namespace webrtc
