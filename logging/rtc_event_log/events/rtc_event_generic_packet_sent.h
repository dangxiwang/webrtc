/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef LOGGING_RTC_EVENT_LOG_EVENTS_RTC_EVENT_GENERIC_PACKET_SENT_H_
#define LOGGING_RTC_EVENT_LOG_EVENTS_RTC_EVENT_GENERIC_PACKET_SENT_H_

#include <memory>

#include "api/rtc_event_log/rtc_event.h"

namespace webrtc {

class RtcEventGenericPacketSent final : public RtcEvent {
 public:
  static constexpr Type kType = Type::GenericPacketSent;

  RtcEventGenericPacketSent(int64_t packet_number,
                            size_t overhead_length,
                            size_t payload_length,
                            size_t padding_length);
  ~RtcEventGenericPacketSent() override;

  std::unique_ptr<RtcEventGenericPacketSent> Copy() const;

  Type GetType() const override { return kType; }
  bool IsConfigEvent() const override { return false; }

  // An identifier of the packet.
  int64_t packet_number() const { return packet_number_; }

  // Total packet length, including all packetization overheads, but not
  // including ICE/TURN/IP overheads.
  size_t packet_length() const {
    return overhead_length_ + payload_length_ + padding_length_;
  }

  // The number of bytes in overhead, including framing overheads, acks if
  // present, etc.
  size_t overhead_length() const { return overhead_length_; }

  // Length of payload sent (size of raw audio/video/data), without
  // packetization overheads. This may still include serialization overheads.
  size_t payload_length() const { return payload_length_; }

  size_t padding_length() const { return padding_length_; }

 private:
  RtcEventGenericPacketSent(const RtcEventGenericPacketSent& packet);

  const int64_t packet_number_;
  const size_t overhead_length_;
  const size_t payload_length_;
  const size_t padding_length_;
};

struct LoggedGenericPacketSent {
  LoggedGenericPacketSent() = default;
  LoggedGenericPacketSent(int64_t timestamp_us,
                          int64_t packet_number,
                          size_t overhead_length,
                          size_t payload_length,
                          size_t padding_length)
      : timestamp_us(timestamp_us),
        packet_number(packet_number),
        overhead_length(overhead_length),
        payload_length(payload_length),
        padding_length(padding_length) {}

  int64_t log_time_us() const { return timestamp_us; }
  int64_t log_time_ms() const { return timestamp_us / 1000; }

  size_t packet_length() const {
    return payload_length + padding_length + overhead_length;
  }
  int64_t timestamp_us;
  int64_t packet_number;
  size_t overhead_length;
  size_t payload_length;
  size_t padding_length;
};
}  // namespace webrtc

#endif  // LOGGING_RTC_EVENT_LOG_EVENTS_RTC_EVENT_GENERIC_PACKET_SENT_H_
