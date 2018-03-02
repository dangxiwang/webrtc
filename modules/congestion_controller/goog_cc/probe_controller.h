/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_GOOG_CC_PROBE_CONTROLLER_H_
#define MODULES_CONGESTION_CONTROLLER_GOOG_CC_PROBE_CONTROLLER_H_

#include <stdint.h>

#include <initializer_list>

#include "api/optional.h"
#include "modules/congestion_controller/network_control/include/network_control.h"

namespace webrtc {

class Clock;

namespace webrtc_cc {

// This class controls initiation of probing to estimate initial channel
// capacity. There is also support for probing during a session when max
// bitrate is adjusted by an application.
class ProbeController {
 public:
  explicit ProbeController(NetworkControllerObserver* observer,
                           int64_t at_time_ms,
                           int64_t probing_start_bitrate_bps,
                           int64_t estimated_bitrate_bps,
                           int64_t max_bitrate_bps,
                           bool enable_alr_probing);
  ~ProbeController();

  void UpdateMaxBitrate(int64_t max_bitrate_bps, int64_t at_time_ms);

  void SetEstimatedBitrate(int64_t bitrate_bps, int64_t at_time_ms);

  void EnablePeriodicAlrProbing(bool enable);

  void SetAlrStartTimeMs(rtc::Optional<int64_t> alr_start_time);
  void SetAlrEndedTimeMs(int64_t alr_end_time);

  void RequestProbe(int64_t at_time_ms);

  void Process(int64_t at_time_ms);

 private:
  enum class State {
    // Initial state where no probing has been triggered yet.
    kInit,
    // Waiting for probing results to continue further probing.
    kWaitingForProbingResult,
    // Probing is complete.
    kProbingComplete,
  };

  void InitiateExponentialProbing(int64_t at_time_ms);
  void InitiateProbing(int64_t now_ms,
                       std::initializer_list<int64_t> bitrates_to_probe,
                       bool probe_further);

  NetworkControllerObserver* const observer_;

  State state_;
  int64_t min_bitrate_to_probe_further_bps_;
  int64_t time_last_probing_initiated_ms_;
  int64_t estimated_bitrate_bps_;
  int64_t start_bitrate_bps_;
  int64_t max_bitrate_bps_;
  int64_t last_bwe_drop_probing_time_ms_;
  rtc::Optional<int64_t> alr_start_time_ms_;
  rtc::Optional<int64_t> alr_end_time_ms_;
  bool enable_periodic_alr_probing_;
  int64_t time_of_last_large_drop_ms_;
  int64_t bitrate_before_last_large_drop_bps_;

  bool in_rapid_recovery_experiment_;
  // For WebRTC.BWE.MidCallProbing.* metric.
  bool mid_call_probing_waiting_for_result_;
  int64_t mid_call_probing_bitrate_bps_;
  int64_t mid_call_probing_succcess_threshold_;

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(ProbeController);
};

}  // namespace webrtc_cc
}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_GOOG_CC_PROBE_CONTROLLER_H_
