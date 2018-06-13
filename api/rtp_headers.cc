/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/rtp_headers.h"

#include <algorithm>
#include <type_traits>

#include "absl/strings/ascii.h"
#include "absl/strings/string_view.h"
#include "rtc_base/checks.h"

namespace webrtc {

constexpr size_t StringRtpHeaderExtension::kMaxSize;

bool StringRtpHeaderExtension::IsLegalName(absl::string_view name) {
  return name.size() <= kMaxSize && name.size() > 0 &&
         std::all_of(name.begin(), name.end(), absl::ascii_isalnum);
}

void StringRtpHeaderExtension::Set(absl::string_view value) {
  // If |data| contains \0, the stream id size might become less than |size|.
  RTC_CHECK_LE(value.size(), kMaxSize);
  memcpy(value_, value.data(), value.size());
  if (value.size() < kMaxSize)
    value_[value.size()] = 0;
}

// StreamId is used as member of RTPHeader that is sometimes copied with memcpy
// and thus assume trivial destructibility.
static_assert(std::is_trivially_destructible<StreamId>::value, "");

RTPHeaderExtension::RTPHeaderExtension()
    : hasTransmissionTimeOffset(false),
      transmissionTimeOffset(0),
      hasAbsoluteSendTime(false),
      absoluteSendTime(0),
      hasTransportSequenceNumber(false),
      transportSequenceNumber(0),
      hasAudioLevel(false),
      voiceActivity(false),
      audioLevel(0),
      hasVideoRotation(false),
      videoRotation(kVideoRotation_0),
      hasVideoContentType(false),
      videoContentType(VideoContentType::UNSPECIFIED),
      has_video_timing(false) {}

RTPHeaderExtension::RTPHeaderExtension(const RTPHeaderExtension& other) =
    default;

RTPHeaderExtension& RTPHeaderExtension::operator=(
    const RTPHeaderExtension& other) = default;

RTPHeader::RTPHeader()
    : markerBit(false),
      payloadType(0),
      sequenceNumber(0),
      timestamp(0),
      ssrc(0),
      numCSRCs(0),
      arrOfCSRCs(),
      paddingLength(0),
      headerLength(0),
      payload_type_frequency(0),
      extension() {}

RTPHeader::RTPHeader(const RTPHeader& other) = default;

RTPHeader& RTPHeader::operator=(const RTPHeader& other) = default;

}  // namespace webrtc
