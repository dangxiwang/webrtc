/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "logging/rtc_event_log/icelogger.h"

#include "logging/rtc_event_log/rtc_event_log.h"
#include "rtc_base/basictypes.h"
#include "rtc_base/logging.h"
#include "rtc_base/ptr_util.h"

namespace webrtc {

IceEventLog::IceEventLog() : event_log_(nullptr) {}

void IceEventLog::LogCandidatePairConfig(
    IceCandidatePairConfigType type,
    uint32_t candidate_pair_id,
    const IceCandidatePairDescription& candidate_pair_desc) {
  if (event_log_ == nullptr) {
    return;
  }
  event_log_->Log(rtc::MakeUnique<RtcEventIceCandidatePairConfig>(
      type, candidate_pair_id, candidate_pair_desc));
}

void IceEventLog::LogCandidatePairEvent(IceCandidatePairEventType type,
                                        uint32_t candidate_pair_id) {
  if (event_log_ == nullptr) {
    return;
  }
  event_log_->Log(
      rtc::MakeUnique<RtcEventIceCandidatePair>(type, candidate_pair_id));
}

}  // namespace webrtc
