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

IceCandidatePairDescription::IceCandidatePairDescription() {}

IceCandidatePairDescription::IceCandidatePairDescription(
    const IceCandidatePairDescription& other) {
  content_name = other.content_name;
  local_candidate_type = other.local_candidate_type;
  local_network_type = other.local_network_type;
  remote_candidate_type = other.remote_candidate_type;
  candidate_pair_protocol = other.candidate_pair_protocol;
  candidate_pair_address_family = other.candidate_pair_address_family;
}

IceCandidatePairDescription::~IceCandidatePairDescription() {}

RtcEventIceCandidatePair::RtcEventIceCandidatePair(
    IceCandidatePairEventType type,
    uint32_t candidate_pair_id,
    const IceCandidatePairDescription& candidate_pair_desc)
    : type_(type),
      candidate_pair_id_(candidate_pair_id),
      candidate_pair_desc_(candidate_pair_desc) {}

RtcEventIceCandidatePair::~RtcEventIceCandidatePair() = default;

RtcEvent::Type RtcEventIceCandidatePair::GetType() const {
  return RtcEvent::Type::IceCandidatePairEvent;
}

bool RtcEventIceCandidatePair::IsConfigEvent() const {
  return false;
}

}  // namespace webrtc
