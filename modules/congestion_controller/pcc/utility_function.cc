/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/pcc/utility_function.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace webrtc {
namespace pcc {

VivaceUtilityFunction::VivaceUtilityFunction(double rtt_gradient_coefficient,
                                             double loss_coefficient,
                                             double throughput_coefficient,
                                             double rtt_gradient_threshold)
    : rtt_gradient_coefficient_{rtt_gradient_coefficient},
      loss_coefficient_{loss_coefficient},
      throughput_power_{throughput_coefficient},
      rtt_gradient_threshold_{rtt_gradient_threshold} {}

double VivaceUtilityFunction::ComputeUtilityFunction(
    const MonitorInterval& monitor_interval) const {
  if (!monitor_interval.IsFeedbackCollectingDone()) {
    return 0;
  }
  double bitrate = monitor_interval.GetTargetBitrate().kbps();
  double loss_rate = monitor_interval.GetLossRate();
  double rtt_gradient = ComputeRttGradient(monitor_interval);
  return std::pow(bitrate, throughput_power_) -
         rtt_gradient_coefficient_ * bitrate * rtt_gradient -
         loss_coefficient_ * bitrate * loss_rate;
}

VivaceUtilityFunction::~VivaceUtilityFunction() = default;

double VivaceUtilityFunction::ComputeRttGradient(
    const MonitorInterval& monitor_interval) const {
  std::vector<Timestamp> packets_sent_times =
      monitor_interval.GetReceivedPacketsSentTime();
  std::vector<TimeDelta> packets_rtts =
      monitor_interval.GetReceivedPacketsRtt();
  if (packets_sent_times.empty() ||
      packets_sent_times.front() == packets_sent_times.back()) {
    return 0;
  }
  double sum_times{0}, sum_rtts{0}, sum_square_times{0}, sum_times_dot_rtts{0};
  for (size_t i = 0; i < packets_rtts.size(); ++i) {
    sum_times += (packets_sent_times[i] - packets_sent_times[0]).us();
    sum_rtts += packets_rtts[i].us();
    sum_square_times += (packets_sent_times[i] - packets_sent_times[0]).us() *
                        (packets_sent_times[i] - packets_sent_times[0]).us();
    sum_times_dot_rtts += packets_rtts[i].us() *
                          (packets_sent_times[i] - packets_sent_times[0]).us();
  }
  double rtt_gradient =
      1.0 / (packets_rtts.size() * sum_square_times - sum_times * sum_times) *
      (packets_rtts.size() * sum_times_dot_rtts - sum_rtts * sum_times);
  rtt_gradient =
      (std::abs(rtt_gradient) < rtt_gradient_threshold_) ? 0 : rtt_gradient;
  return rtt_gradient;
}

}  // namespace pcc
}  // namespace webrtc
