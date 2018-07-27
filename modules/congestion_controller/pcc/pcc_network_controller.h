/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_PCC_PCC_NETWORK_CONTROLLER_H_
#define MODULES_CONGESTION_CONTROLLER_PCC_PCC_NETWORK_CONTROLLER_H_

#include <vector>

#include "api/transport/network_control.h"
#include "api/transport/network_types.h"
#include "modules/congestion_controller/pcc/bitrate_controller.h"
#include "modules/congestion_controller/pcc/monitor_interval.h"
#include "modules/congestion_controller/pcc/rtt_tracker.h"
#include "rtc_base/random.h"

namespace webrtc {
namespace pcc {

// PCC (Performance-oriented Congestion Control) Vivace is a congestion
// control algorithm based on online (convex) optimization in machine learning.
// It divides time into consecutive Monitor Intervals (MI) to test sending
// rates r(1 + eps), r(1 - eps) for the current sending rate r.
// At the end of each MI it computes utility function to transform the
// performance statistics into a numerical value. Then it updates current
// sending rate using gradient ascent to maximize utility function.
// https://www.usenix.org/system/files/conference/nsdi18/nsdi18-dong.pdf
class PccNetworkController : public NetworkControllerInterface {
 public:
  enum Mode {
    // Slow start phase of pcc doubles sending rate each MI.
    SLOW_START,
    // After getting the first decrease in utility function pcc exits slow start
    // and enters the online learning phase.
    ONLINE_LEARNING,
    // If we got that sending with the lower rate resulted in higher packet
    // loss, then the measurements unreliable and need to double check them.
    DOUBLE_CHECK
  };

  enum MonitorIntervalLengthStrategy {
    // Monitor interval length adaptive when it is proportional to packets RTT.
    ADAPTIVE,
    // Monitor interval length is fixed when it is equal to the time of sending
    // special amount of packets (kMinPacketsNumberPerInterval).
    FIXED
  };

  explicit PccNetworkController(NetworkControllerConfig config);
  ~PccNetworkController() override;

  // NetworkControllerInterface
  NetworkControlUpdate OnNetworkAvailability(NetworkAvailability msg) override;
  NetworkControlUpdate OnNetworkRouteChange(NetworkRouteChange msg) override;
  NetworkControlUpdate OnProcessInterval(ProcessInterval msg) override;
  NetworkControlUpdate OnSentPacket(SentPacket msg) override;
  NetworkControlUpdate OnTargetRateConstraints(
      TargetRateConstraints msg) override;
  NetworkControlUpdate OnTransportPacketsFeedback(
      TransportPacketsFeedback msg) override;

  // Part of remote bitrate estimation api, not implemented for PCC
  NetworkControlUpdate OnStreamsConfig(StreamsConfig msg) override;
  NetworkControlUpdate OnRemoteBitrateReport(RemoteBitrateReport msg) override;
  NetworkControlUpdate OnRoundTripTimeUpdate(RoundTripTimeUpdate msg) override;
  NetworkControlUpdate OnTransportLossReport(TransportLossReport msg) override;

 private:
  void UpdateSendingRate();
  NetworkControlUpdate CreateRateUpdate(Timestamp at_time) const;
  std::vector<DataRate> ComputeMonitorntervalsBitrates();
  bool NeedDoubleCheckMeasurments();
  bool IsTimeoutExpired(Timestamp current_time) const;
  bool IsFeedbackCollectionDone() const;

  Timestamp last_sent_packet_time_;
  TimeDelta smoothed_packets_sending_interval_;
  Mode mode_;

  // Default value used for initializing bandwidth.
  DataRate default_bandwidth_;
  // Current estimate r.
  DataRate bandwidth_estimate_;

  RttTracker rtt_tracker_;
  TimeDelta monitor_interval_timeout_;
  // Monitor inteval duration = RTT * const, monitor_interval_length_ratio_ is
  // this const.
  MonitorIntervalLengthStrategy monitor_interval_length_strategy_;
  double monitor_interval_duration_ratio_;
  double sampling_step_;  // Epsilon.
  double monitor_interval_timeout_ratio_;

  PccBitrateController bitrate_controller_;

  std::vector<MonitorInterval> monitor_intervals_;
  std::vector<DataRate> monitor_intervals_bitrates_;
  TimeDelta monitor_intervals_duration_;
  size_t complete_feedback_mi_number_;

  webrtc::Random random_generator_;
};

}  // namespace pcc
}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_PCC_PCC_NETWORK_CONTROLLER_H_
