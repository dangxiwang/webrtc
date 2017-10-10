/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/source/rtcp_transceiver_config.h"

#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "rtc_base/logging.h"

namespace webrtc {

RtcpTransceiverConfig::RtcpTransceiverConfig() = default;
RtcpTransceiverConfig::RtcpTransceiverConfig(const RtcpTransceiverConfig&) =
    default;
RtcpTransceiverConfig& RtcpTransceiverConfig::operator=(
    const RtcpTransceiverConfig&) = default;
RtcpTransceiverConfig::~RtcpTransceiverConfig() = default;

bool RtcpTransceiverConfig::Valid() const {
  if (feedback_ssrc == 0)
    LOG(LS_WARNING)
        << debug_id
        << "Ssrc 0 may be treated by some implementation as invalid.";
  if (cname.size() > 255) {
    LOG(LS_ERROR) << debug_id << "cname can be maximum 255 characters.";
    return false;
  }
  if (max_packet_size < 100) {
    LOG(LS_ERROR) << debug_id << "max packet size " << max_packet_size
                  << " is too small.";
    return false;
  }
  if (max_packet_size > IP_PACKET_SIZE) {
    LOG(LS_ERROR) << debug_id << "max packet size " << max_packet_size
                  << " more than " << IP_PACKET_SIZE << " is unsupported.";
    return false;
  }
  if (outgoing_transport == nullptr) {
    LOG(LS_ERROR) << debug_id << "outgoing transport must be set";
    return false;
  }
  if (min_periodic_report_ms <= 0) {
    LOG(LS_ERROR) << debug_id << "period " << min_periodic_report_ms
                  << "ms between reports should be positive.";
    return false;
  }
  if (receive_statistics == nullptr)
    LOG(LS_WARNING)
        << debug_id
        << "receive statistic should be set to generate rtcp report blocks.";
  return true;
}

}  // namespace webrtc
