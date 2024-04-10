/*
 *  Copyright (c) 2024 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/transport_feedback_sender.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/rtp_rtcp/source/time_util.h"
#include "rtc_base/network/ecn_marking.h"

namespace webrtc {

constexpr TimeDelta kMinInterval = TimeDelta::Millis(50);
constexpr TimeDelta kMaxInterval = TimeDelta::Millis(250);
constexpr TimeDelta kDefaultInterval = TimeDelta::Millis(100);

TransportFeedbackSender::TransportFeedbackSender(Clock& clock,
                                                 RtcpSender rtcp_sender)
    : clock_(clock),
      rtcp_sender_(std::move(rtcp_sender)),
      send_interval_(kDefaultInterval) {}

void TransportFeedbackSender::OnReceivedPacket(
    const RtpPacketReceived& packet) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);

  PacketInfo info = {.sequence_number = packet.SequenceNumber(),
                     .unwrapped_sequence_number =
                         sequence_number_unwrappers_[packet.Ssrc()].Unwrap(
                             packet.SequenceNumber()),
                     .arrival_time = packet.arrival_time(),
                     .ecn = packet.ecn()};
  packets_.push_back(info);
}

TimeDelta TransportFeedbackSender::Process(Timestamp now) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  Timestamp next_process_time = last_process_time_ + send_interval_;
  if (now >= next_process_time) {
    last_process_time_ = now;
    MaybeSendFeedback();
    return send_interval_;
  }
  return next_process_time - now;
}

void TransportFeedbackSender::OnTargetBitrateChanged(DataRate bitrate) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  // Uplink target rate decides how often we send feedback.
  // The following algorithm is copied from
  // RemoteEstimatorProxy::OnBitrateChanged.
  // TODO() Consider updating

  // TwccReportSize = Ipv4(20B) + UDP(8B) + SRTP(10B) +
  // AverageTwccReport(30B)
  // TwccReport size at 50ms interval is 24 byte.
  // TwccReport size at 250ms interval is 36 byte.
  // AverageTwccReport = (TwccReport(50ms) + TwccReport(250ms)) / 2
  constexpr DataSize kTwccReportSize = DataSize::Bytes(20 + 8 + 10 + 30);
  constexpr DataRate kMinTwccRate = kTwccReportSize / kMaxInterval;

  // Let TWCC reports occupy 5% of total bandwidth.
  DataRate twcc_bitrate = 0.05 * bitrate;

  // Check upper send_interval bound by checking bitrate to avoid overflow when
  // dividing by small bitrate, in particular avoid dividing by zero bitrate.
  send_interval_ = twcc_bitrate <= kMinTwccRate
                       ? kMaxInterval
                       : std::max(kTwccReportSize / twcc_bitrate, kMinInterval);
}

void TransportFeedbackSender::SetTransportOverhead(
    DataSize overhead_per_packet) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
}

void TransportFeedbackSender::MaybeSendFeedback() {
  if (packets_.empty()) {
    return;
  }

  std::sort(packets_.begin(), packets_.end(),
            [](const PacketInfo& a, const PacketInfo& b) {
              if (a.ssrc == b.ssrc) {
                return a.unwrapped_sequence_number <
                       b.unwrapped_sequence_number;
              }
              return a.ssrc < b.ssrc;
            });
  Timestamp report_timestamp = clock_.CurrentTime();
  uint32_t compact_ntp =
      CompactNtp(clock_.ConvertTimestampToNtpTime(report_timestamp));
  std::vector<rtcp::TransportLayerFeedback::PacketInfo> rtcp_packet_info;
  rtcp_packet_info.reserve(packets_.size());

  absl::optional<uint32_t> previous_ssrc;
  absl::optional<int64_t> previous_seq_no;
  for (const PacketInfo packet : packets_) {
    if (previous_ssrc == packet.ssrc &&
        previous_seq_no == packet.unwrapped_sequence_number) {
      /*
      According to RFC 8888:
    If duplicate copies of a particular RTP packet are received, then the
    arrival time of the first copy to arrive MUST be reported. If any of the
    copies of the duplicated packet are ECN-CE marked, then an ECN-CE mark
   MUST be reported for that packet; otherwise, the ECN mark of the first copy
   to arrive is reported.
*/
      if (packet.ecn == rtc::EcnMarking::kCe) {
        rtcp_packet_info.back().ecn = packet.ecn;
      }
    } else {
      previous_ssrc = packet.ssrc;
      previous_seq_no = packet.unwrapped_sequence_number;
      rtcp_packet_info.push_back(
          {.ssrc = packet.ssrc,
           .sequence_number = packet.sequence_number,
           .arrival_time_offset = report_timestamp - packet.arrival_time,
           .ecn = packet.ecn});
    }
  }

  // Create and send the RTCP packet.
  std::vector<std::unique_ptr<rtcp::RtcpPacket>> rtcp_packets;
  rtcp_packets.push_back(std::make_unique<rtcp::TransportLayerFeedback>(
      std::move(rtcp_packet_info), compact_ntp));
  rtcp_sender_(std::move(rtcp_packets));
}

}  // namespace webrtc
