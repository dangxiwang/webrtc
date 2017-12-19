/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef LOGGING_RTC_EVENT_LOG_EVENTS_RTC_EVENT_ICE_CP_STATE_UPDATE_H_
#define LOGGING_RTC_EVENT_LOG_EVENTS_RTC_EVENT_ICE_CP_STATE_UPDATE_H_

#include "logging/rtc_event_log/events/rtc_event.h"

#include <string>

namespace webrtc {

enum class IceLogCpState {
  kInactive,
  kCheckSent,
  kCheckReceived,
  kCheckResponseSent,
  kCheckResponseReceived,
  kSelected,
};

class RtcEventIceCpStateUpdate final : public RtcEvent {
 public:
  RtcEventIceCpStateUpdate(const std::string& ice_cp_desc,
                           IceLogCpState new_state);

  ~RtcEventIceCpStateUpdate() override;

  Type GetType() const override;

  bool IsConfigEvent() const override;

  const std::string ice_cp_desc_;
  const IceLogCpState new_state_;
};

}  // namespace webrtc

#endif  // LOGGING_RTC_EVENT_LOG_EVENTS_RTC_EVENT_ICE_CP_STATE_UPDATE_H_
