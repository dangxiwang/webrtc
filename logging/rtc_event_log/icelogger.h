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

#include "logging/rtc_event_log/events/rtc_event_ice_candidate_pair.h"
#include "logging/rtc_event_log/events/rtc_event_ice_candidate_pair_config.h"

namespace webrtc {

class RtcEventLog;

// IceEventLog wraps RtcEventLog and provides structural logging of ICE-specific
// events. The logged events are serialized with other RtcEvent's if protobuf is
// enabled in the build.
class IceEventLog {
 public:
  IceEventLog() = default;
  void set_event_log(RtcEventLog* event_log) { event_log_ = event_log; }
  void LogCandidatePairEvent(
      IceCandidatePairEventType type,
      uint32_t candidate_pair_id,
      const IceCandidatePairDescription& candidate_pair_desc);
  ~IceEventLog() = default;

 private:
  bool IsIceCandidatePairConfigEvent(IceCandidatePairEventType type);

  RtcEventLog* event_log_ = nullptr;
};

}  // namespace webrtc

#endif  // LOGGING_RTC_EVENT_LOG_ICELOGGER_H_
