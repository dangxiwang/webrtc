/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_RTP_RTCP_SOURCE_RTCP_TRANSCEIVER_IMPL_H_
#define MODULES_RTP_RTCP_SOURCE_RTCP_TRANSCEIVER_IMPL_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "api/array_view.h"
#include "api/optional.h"
#include "modules/rtp_rtcp/source/rtcp_packet/common_header.h"
#include "modules/rtp_rtcp/source/rtcp_packet/dlrr.h"
#include "modules/rtp_rtcp/source/rtcp_packet/remb.h"
#include "modules/rtp_rtcp/source/rtcp_packet/report_block.h"
#include "modules/rtp_rtcp/source/rtcp_packet/target_bitrate.h"
#include "modules/rtp_rtcp/source/rtcp_transceiver_config.h"
#include "rtc_base/constructormagic.h"
#include "rtc_base/weak_ptr.h"
#include "system_wrappers/include/ntp_time.h"

namespace webrtc {
//
// Manage incoming and outgoing rtcp messages for multiple BUNDLED streams.
//
// This class is not thread-safe.
class RtcpTransceiverImpl {
 public:
  explicit RtcpTransceiverImpl(const RtcpTransceiverConfig& config);
  ~RtcpTransceiverImpl();

  void AddMediaReceiverRtcpObserver(uint32_t remote_ssrc,
                                    MediaReceiverRtcpObserver* observer);
  void RemoveMediaReceiverRtcpObserver(uint32_t remote_ssrc,
                                       MediaReceiverRtcpObserver* observer);

  void ReceivePacket(rtc::ArrayView<const uint8_t> packet, int64_t now_us);

  void SendCompoundPacket();

  void SetRemb(int64_t bitrate_bps, std::vector<uint32_t> ssrcs);
  void UnsetRemb();
  // Temporary helpers to send pre-built TransportFeedback rtcp packet.
  uint32_t sender_ssrc() const { return config_.feedback_ssrc; }
  void SendRawPacket(rtc::ArrayView<const uint8_t> packet);

  void SendNack(uint32_t ssrc, std::vector<uint16_t> sequence_numbers);

  void SendPictureLossIndication(uint32_t ssrc);
  void SendFullIntraRequest(rtc::ArrayView<const uint32_t> ssrcs);

 private:
  class PacketSender;
  struct RemoteSenderState;

  void HandleReceivedPacket(const rtcp::CommonHeader& rtcp_packet_header,
                            int64_t now_us);
  // Individual rtcp packet handlers.
  void HandleBye(const rtcp::CommonHeader& rtcp_packet_header);
  void HandleSenderReport(const rtcp::CommonHeader& rtcp_packet_header,
                          int64_t now_us);
  void HandleExtendedReports(const rtcp::CommonHeader& rtcp_packet_header,
                             int64_t now_us);
  // Extended Reports blocks handlers.
  void HandleDlrr(const rtcp::Dlrr& dlrr, int64_t now_us);
  void HandleTargetBitrate(const rtcp::TargetBitrate& target_bitrate,
                           uint32_t remote_ssrc);

  void ReschedulePeriodicCompoundPackets();
  void SchedulePeriodicCompoundPackets(int64_t delay_ms);
  // Creates compound RTCP packet, as defined in
  // https://tools.ietf.org/html/rfc5506#section-2
  void CreateCompoundPacket(PacketSender* sender);
  // Sends RTCP packets.
  void SendPeriodicCompoundPacket();
  void SendImmediateFeedback(const rtcp::RtcpPacket& rtcp_packet);
  // Generate Report Blocks to be send in Sender or Receiver Report.
  std::vector<rtcp::ReportBlock> CreateReportBlocks(int64_t now_us);

  const RtcpTransceiverConfig config_;

  rtc::Optional<rtcp::Remb> remb_;
  // TODO(danilchap): Remove entries from remote_senders_ that are no longer
  // needed.
  std::map<uint32_t, RemoteSenderState> remote_senders_;
  rtc::WeakPtrFactory<RtcpTransceiverImpl> ptr_factory_;

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(RtcpTransceiverImpl);
};

}  // namespace webrtc

#endif  // MODULES_RTP_RTCP_SOURCE_RTCP_TRANSCEIVER_IMPL_H_
