/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef LOGGING_RTC_EVENT_LOG_EVENTS_RTC_EVENT_ICE_CANDIDATE_PAIR_H_
#define LOGGING_RTC_EVENT_LOG_EVENTS_RTC_EVENT_ICE_CANDIDATE_PAIR_H_

#include "logging/rtc_event_log/events/rtc_event.h"

#include <string>

namespace webrtc {

enum class IceCandidatePairStatus {
  kAdded,
  kCheckSent,
  kCheckReceived,
  kCheckResponseSent,
  kCheckResponseReceived,
  kSelected,
  kPruned,
};

class RtcEventIceCandidatePair final : public RtcEvent {
 public:
  RtcEventIceCandidatePair(const std::string& candidate_pair_desc,
                           IceCandidatePairStatus status);

  ~RtcEventIceCandidatePair() override;

  Type GetType() const override;

  bool IsConfigEvent() const override;

  const std::string candidate_pair_desc_;
  const IceCandidatePairStatus status_;
};

}  // namespace webrtc

#endif  // LOGGING_RTC_EVENT_LOG_EVENTS_RTC_EVENT_ICE_CANDIDATE_PAIR_H_
