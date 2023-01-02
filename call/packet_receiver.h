/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef CALL_PACKET_RECEIVER_H_
#define CALL_PACKET_RECEIVER_H_

#include "absl/functional/any_invocable.h"
#include "api/media_types.h"
#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "rtc_base/copy_on_write_buffer.h"

namespace webrtc {

class PacketReceiver {
 public:
  enum DeliveryStatus {
    DELIVERY_OK,
    DELIVERY_UNKNOWN_SSRC,
    DELIVERY_PACKET_ERROR,
  };

  virtual DeliveryStatus DeliverPacket(MediaType media_type,
                                       rtc::CopyOnWriteBuffer packet,
                                       int64_t packet_time_us) = 0;

  // Invoked once when a packet packet is received that can not be demuxed.
  // If the method returns true, a new attempt is made to demux the packet.
  using OnUnDemuxablePacketHandler =
      absl::AnyInvocable<bool(const RtpPacketReceived& parsed_packet)>;

  virtual void DeliverRtpPacket(
      MediaType media_type,
      webrtc::RtpPacketReceived packet,
      OnUnDemuxablePacketHandler un_demuxable_packet_handler) {
    // TODO(perkj, https://bugs.webrtc.org/7135): Implement in FakeCall and
    // FakeNetworkPipe.
    RTC_CHECK_NOTREACHED();
  }

 protected:
  virtual ~PacketReceiver() {}
};

}  // namespace webrtc

#endif  // CALL_PACKET_RECEIVER_H_
