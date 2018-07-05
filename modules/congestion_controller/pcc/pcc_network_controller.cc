/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "absl/memory/memory.h"
#include "modules/congestion_controller/pcc/pcc_network_controller.h"

namespace webrtc {
namespace pcc {
namespace {
constexpr int64_t kInitialRttMs = 200;
constexpr int64_t kInitialBandwidthKbps = 300;
constexpr double kMonitorIntervalDurationRatio = 1;
constexpr double kDefaultSamplingStep = 0.05;
constexpr double kTimeoutRatio = 4;
constexpr double kAlphaForRtt = 0.6;

// Bitrate controller constants.
constexpr double kInitialConversionFactor = 1;
constexpr double kInitialDynamicBoundary = 0.05;
constexpr double kDynamicBoundaryIncrement = 0.1;
// Utility function parameters.
constexpr double kRttGradientCoefficient = 900;
constexpr double kLossCoefficient = 11.35;
constexpr double kThroughputCoefficient = 0.9;
constexpr double kRttGradientThreshold = 0.01;
}  // namespace

PccNetworkController::PccNetworkController(NetworkControllerConfig config)
    : mode_(SLOW_START),
      default_bandwidth_(DataRate::kbps(kInitialBandwidthKbps)),
      bandwidth_estimate_(default_bandwidth_),
      rtt_tracker_(TimeDelta::ms(kInitialRttMs), kAlphaForRtt),
      monitor_interval_duration_ratio_(kMonitorIntervalDurationRatio),
      sampling_step_(kDefaultSamplingStep),
      monitor_interval_timeout_ratio_(kTimeoutRatio),
      bitrate_controller_(kInitialConversionFactor,
                          kInitialDynamicBoundary,
                          kDynamicBoundaryIncrement,
                          ConfidenceAmplifier(),
                          kRttGradientCoefficient,
                          kLossCoefficient,
                          kThroughputCoefficient,
                          kRttGradientThreshold),
      generator_(rd_()),
      distribution_(0, 1) {
  if (config.starting_bandwidth.IsFinite()) {
    default_bandwidth_ = config.starting_bandwidth;
    bandwidth_estimate_ = default_bandwidth_;
  }
}

PccNetworkController::~PccNetworkController() {}

NetworkControlUpdate PccNetworkController::CreateRateUpdate(
    Timestamp at_time) const {
  DataRate sending_rate = DataRate::Zero();
  if (monitor_block_) {
    sending_rate = monitor_block_->GetTargetBirtate();
  } else {
    sending_rate = bandwidth_estimate_;
  }
  // Set up config when sending rate is computed.
  NetworkControlUpdate update;

  // Set up target rate to encoder.
  TargetTransferRate target_rate_msg;
  target_rate_msg.network_estimate.at_time = at_time;
  target_rate_msg.network_estimate.round_trip_time = rtt_tracker_.GetRtt();
  target_rate_msg.network_estimate.bandwidth = bandwidth_estimate_;
  target_rate_msg.network_estimate.loss_rate_ratio = 0;
  target_rate_msg.network_estimate.bwe_period = TimeDelta::Zero();

  target_rate_msg.target_rate = sending_rate;
  update.target_rate = target_rate_msg;

  // Set up pacing/padding target rate.
  PacerConfig pacer_config;
  pacer_config.at_time = at_time;
  pacer_config.time_window = TimeDelta::ms(1);
  pacer_config.data_window = sending_rate * pacer_config.time_window;
  pacer_config.pad_window = sending_rate * pacer_config.time_window;

  update.pacer_config = pacer_config;
  return update;
}

NetworkControlUpdate PccNetworkController::OnSentPacket(SentPacket msg) {
  if (monitor_block_) {
    monitor_block_->NotifyCurrentTime(msg.send_time);
    if (monitor_block_->IsTimeoutExpired()) {
      bandwidth_estimate_ = bandwidth_estimate_ * 0.5;
    }
  }
  if (!monitor_block_ || monitor_block_->IsFeedbackCollectingDone() ||
      monitor_block_->IsTimeoutExpired()) {
    monitor_block_ = std::unique_ptr<MonitorBlock>{new MonitorBlock(
        msg.send_time, rtt_tracker_.GetRtt() * monitor_interval_duration_ratio_,
        bandwidth_estimate_,
        rtt_tracker_.GetRtt() * monitor_interval_timeout_ratio_,
        ComputeMonitorntervalsBitrates())};
  }
  return CreateRateUpdate(msg.send_time);
}

std::vector<DataRate> PccNetworkController::ComputeMonitorntervalsBitrates() {
  if (IsSlowStart()) {
    return {2 * bandwidth_estimate_};
  }
  int64_t sign = 2 * distribution_(generator_) % 2 - 1;
  return {bandwidth_estimate_ * (1 + sign * sampling_step_),
          bandwidth_estimate_ * (1 - sign * sampling_step_)};
}

NetworkControlUpdate PccNetworkController::OnTransportPacketsFeedback(
    TransportPacketsFeedback msg) {
  if (monitor_block_) {
    rtt_tracker_.OnPacketsFeedback(msg.PacketsWithFeedback());
    monitor_block_->OnPacketsFeedback(msg.PacketsWithFeedback());
    if (monitor_block_->IsFeedbackCollectingDone() &&
        !NeedDoubleCheckMeasurments()) {
      UpdateSendingRate();
    }
  }
  return NetworkControlUpdate();
}

bool PccNetworkController::NeedDoubleCheckMeasurments() {
  if (!monitor_block_->IsFeedbackCollectingDone() || IsSlowStart()) {
    return false;
  }
  if (IsDoubleCheck()) {
    mode_ = SLOW_START;
    return false;
  }
  double first_loss_rate = monitor_block_->GetMonitorInterval(0)->GetLossRate();
  double second_loss_rate =
      monitor_block_->GetMonitorInterval(1)->GetLossRate();
  DataRate first_bitrate =
      monitor_block_->GetMonitorInterval(0)->GetTargetBitrate();
  DataRate second_bitrate =
      monitor_block_->GetMonitorInterval(1)->GetTargetBitrate();
  if ((first_bitrate.bps() - second_bitrate.bps()) *
          (first_loss_rate - second_loss_rate) <
      0) {
    mode_ = DOUBLE_CHECK;
    return true;
  }
  return false;
}

void PccNetworkController::UpdateSendingRate() {
  if (!monitor_block_ || !monitor_block_->IsFeedbackCollectingDone()) {
    return;
  }
  DataRate old_bandwidth_estimate = bandwidth_estimate_;
  bandwidth_estimate_ = bitrate_controller_.ComputeRateUpdate(
      *monitor_block_, bandwidth_estimate_);
  if (IsSlowStart() && (bandwidth_estimate_ <= old_bandwidth_estimate)) {
    mode_ = ONLINE_LEARNING;
  }
}

bool PccNetworkController::IsSlowStart() const {
  return mode_ == SLOW_START;
}

bool PccNetworkController::IsOnlineLearning() const {
  return mode_ == ONLINE_LEARNING;
}

bool PccNetworkController::IsDoubleCheck() const {
  return mode_ == DOUBLE_CHECK;
}

NetworkControlUpdate PccNetworkController::OnNetworkAvailability(
    NetworkAvailability msg) {
  return NetworkControlUpdate();
}

NetworkControlUpdate PccNetworkController::OnNetworkRouteChange(
    NetworkRouteChange msg) {
  return NetworkControlUpdate();
}

NetworkControlUpdate PccNetworkController::OnProcessInterval(
    ProcessInterval msg) {
  return CreateRateUpdate(msg.at_time);
}

NetworkControlUpdate PccNetworkController::OnTargetRateConstraints(
    TargetRateConstraints msg) {
  return NetworkControlUpdate();
}

NetworkControlUpdate PccNetworkController::OnRemoteBitrateReport(
    RemoteBitrateReport) {
  return NetworkControlUpdate();
}

NetworkControlUpdate PccNetworkController::OnRoundTripTimeUpdate(
    RoundTripTimeUpdate) {
  return NetworkControlUpdate();
}

NetworkControlUpdate PccNetworkController::OnTransportLossReport(
    TransportLossReport) {
  return NetworkControlUpdate();
}

NetworkControlUpdate PccNetworkController::OnStreamsConfig(StreamsConfig) {
  return NetworkControlUpdate();
}

}  // namespace pcc
}  // namespace webrtc
