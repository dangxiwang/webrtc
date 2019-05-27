/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/rtp_packet_info.h"

#include <algorithm>
#include <utility>

namespace webrtc {

RtpPacketInfo::RtpPacketInfo(uint32_t ssrc,
                             std::vector<uint32_t> csrcs,
                             uint16_t sequence_number,
                             uint32_t rtp_timestamp,
                             absl::optional<uint8_t> audio_level,
                             int64_t receive_time_ms)
    : ssrc_(ssrc),
      csrcs_(std::move(csrcs)),
      sequence_number_(sequence_number),
      rtp_timestamp_(rtp_timestamp),
      audio_level_(audio_level),
      receive_time_ms_(receive_time_ms) {}

RtpPacketInfo::RtpPacketInfo(const RTPHeader& rtp_header,
                             int64_t receive_time_ms)
    : ssrc_(rtp_header.ssrc),
      sequence_number_(rtp_header.sequenceNumber),
      rtp_timestamp_(rtp_header.timestamp),
      receive_time_ms_(receive_time_ms) {
  const auto& extension = rtp_header.extension;
  const auto csrcs_count = std::min<size_t>(rtp_header.numCSRCs, kRtpCsrcSize);

  csrcs_.assign(&rtp_header.arrOfCSRCs[0], &rtp_header.arrOfCSRCs[csrcs_count]);

  if (extension.hasAudioLevel) {
    audio_level_ = extension.audioLevel;
  }
}

bool operator==(const RtpPacketInfo& lhs, const RtpPacketInfo& rhs) {
  return (lhs.ssrc() == rhs.ssrc()) && (lhs.csrcs() == rhs.csrcs()) &&
         (lhs.sequence_number() == rhs.sequence_number()) &&
         (lhs.rtp_timestamp() == rhs.rtp_timestamp()) &&
         (lhs.audio_level() == rhs.audio_level()) &&
         (lhs.receive_time_ms() == rhs.receive_time_ms());
}

bool operator!=(const RtpPacketInfo& lhs, const RtpPacketInfo& rhs) {
  return !(lhs == rhs);
}

}  // namespace webrtc
