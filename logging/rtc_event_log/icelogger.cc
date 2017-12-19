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

void IceEventLog::LogCandidatePairStateUpdate(const std::string& ice_cp_desc,
                                              IceLogCpState new_state) {
  RTC_LOG(INFO) << "LogCandidatePairStateUpdate";
  event_log_->Log(
      rtc::MakeUnique<RtcEventIceCpStateUpdate>(ice_cp_desc, new_state));
}

}  // namespace webrtc
