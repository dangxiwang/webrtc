/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "logging/rtc_event_log/events/rtc_event_ice_cp_state_update.h"

namespace webrtc {

RtcEventIceCpStateUpdate::RtcEventIceCpStateUpdate(
    const std::string& ice_cp_desc,
    IceLogCpState new_state)
    : ice_cp_desc_(ice_cp_desc), new_state_(new_state) {}

RtcEventIceCpStateUpdate::~RtcEventIceCpStateUpdate() = default;

RtcEvent::Type RtcEventIceCpStateUpdate::GetType() const {
  return RtcEvent::Type::IceCpStateUpdate;
}

bool RtcEventIceCpStateUpdate::IsConfigEvent() const {
  return false;
}

}  // namespace webrtc
