/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "logging/rtc_event_log/events/rtc_event_ice_candidate_pair.h"

namespace webrtc {

RtcEventIceCandidatePair::RtcEventIceCandidatePair(
    const std::string& candidate_pair_desc,
    IceCandidatePairStatus status)
    : candidate_pair_desc_(candidate_pair_desc), status_(status) {}

RtcEventIceCandidatePair::~RtcEventIceCandidatePair() = default;

RtcEvent::Type RtcEventIceCandidatePair::GetType() const {
  return RtcEvent::Type::IceCandidatePairEvent;
}

bool RtcEventIceCandidatePair::IsConfigEvent() const {
  return false;
}

}  // namespace webrtc
