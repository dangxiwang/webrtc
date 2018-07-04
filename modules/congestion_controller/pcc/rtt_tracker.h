/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_PCC_RTT_TRACKER_H_
#define MODULES_CONGESTION_CONTROLLER_PCC_RTT_TRACKER_H_

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "api/transport/network_control.h"
#include "api/transport/network_types.h"

namespace webrtc {
namespace pcc {

class RttTracker {
 public:
  RttTracker(TimeDelta initial_rtt, double alpha);
  // Updates RTT estimate.
  void OnPacketsFeedback(std::vector<PacketResult> packet_feedbacks);
  TimeDelta GetRtt() const;

 private:
  TimeDelta rtt_estimate_;
  double alpha_;
};

}  // namespace pcc
}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_PCC_RTT_TRACKER_H_
