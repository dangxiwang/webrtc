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

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "api/transport/network_control.h"
#include "api/transport/network_types.h"
#include "modules/congestion_controller/pcc/bitrate_controller.h"
#include "modules/congestion_controller/pcc/monitor_interval.h"
#include "modules/congestion_controller/pcc/rtt_tracker.h"

namespace webrtc {
namespace pcc {

class PccNetworkController : public NetworkControllerInterface {
 public:
  enum Mode {
    // Slow start phase of pcc doubles sending rate each MI.
    SLOW_START,
    // After getting the first decrease in utility function pcc exits slow start
    // and enters the online learning phase.
    ONLINE_LEARNING,
    // If we got that sending with the lower rate resulted in higher packet
    // loss, then the measurments unreliable and need to double check them.
    DOUBLE_CHECK
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

  bool IsSlowStart() const;
  bool IsOnlineLearning() const;
  bool IsDoubleCheck() const;

  Mode mode_;

  // Default value used for initializing bandwidth.
  DataRate default_bandwidth_;
  // Current estimate r.
  DataRate bandwidth_estimate_;

  RttTracker rtt_tracker_;
  // Monitor inteval duration = RTT * const, monitor_interval_length_ratio_ is
  // this const.
  double monitor_interval_duration_ratio_;
  double sampling_step_;  // Epsilon.
  double monitor_interval_timeout_ratio_;

  BitrateController bitrate_controller_;
  std::unique_ptr<MonitorBlock> monitor_block_;

  std::random_device rd_;
  std::mt19937 generator_;
  std::uniform_int_distribution<> distribution_;
};

}  // namespace pcc
}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_PCC_PCC_NETWORK_CONTROLLER_H_
