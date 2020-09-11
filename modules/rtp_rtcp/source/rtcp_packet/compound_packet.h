/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 */

#ifndef MODULES_RTP_RTCP_SOURCE_RTCP_PACKET_COMPOUND_PACKET_H_
#define MODULES_RTP_RTCP_SOURCE_RTCP_PACKET_COMPOUND_PACKET_H_

#include <memory>
#include <vector>

#include "modules/rtp_rtcp/source/rtcp_packet.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {
namespace rtcp {

class CompoundPacket : public RtcpPacket {
 public:
  CompoundPacket();
  ~CompoundPacket() override;

  void Append(std::unique_ptr<RtcpPacket> packet);

  // Fallback for call-sites that have not yet migrated to passing a unique_ptr.
  // TODO(sprang): Remove when all usage is gone.
  template <typename T>
  void Append(T* packet) {
    Append(std::make_unique<T>(*packet));
  }

  // Size of this packet in bytes (i.e. total size of nested packets).
  size_t BlockLength() const override;
  // Returns true if all calls to Create succeeded.
  bool Create(uint8_t* packet,
              size_t* index,
              size_t max_length,
              PacketReadyCallback callback) const override;

 protected:
  std::vector<std::unique_ptr<RtcpPacket>> appended_packets_;

 private:
  RTC_DISALLOW_COPY_AND_ASSIGN(CompoundPacket);
};

}  // namespace rtcp
}  // namespace webrtc
#endif  // MODULES_RTP_RTCP_SOURCE_RTCP_PACKET_COMPOUND_PACKET_H_
