/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef LOGGING_RTC_EVENT_LOG_ICELOGGER_H_
#define LOGGING_RTC_EVENT_LOG_ICELOGGER_H_

#include <string>
#include <unordered_map>

#include "logging/rtc_event_log/events/rtc_event_ice_candidate_pair.h"
#include "logging/rtc_event_log/events/rtc_event_ice_candidate_pair_config.h"

namespace webrtc {

class RtcEventLog;

class IceEventLog {
 public:
  IceEventLog();
  void set_event_log(RtcEventLog* event_log) { event_log_ = event_log; }
  void LogCandidatePairConfig(
      IceCandidatePairConfigType type,
      uint32_t candidate_pair_id,
      const IceCandidatePairDescription& candidate_pair_desc);
  void LogCandidatePairEvent(IceCandidatePairEventType type,
                             uint32_t candidate_pair_id);
  ~IceEventLog() = default;

 private:
  RtcEventLog* event_log_;
};

}  // namespace webrtc

#endif  // LOGGING_RTC_EVENT_LOG_ICELOGGER_H_
