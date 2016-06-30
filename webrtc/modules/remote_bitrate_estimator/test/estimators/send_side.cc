/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "webrtc/modules/remote_bitrate_estimator/test/estimators/send_side.h"

#include "webrtc/base/logging.h"
#include "webrtc/modules/remote_bitrate_estimator/remote_bitrate_estimator_abs_send_time.h"
#include "webrtc/modules/remote_bitrate_estimator/test/bwe_test_logging.h"

namespace webrtc {
namespace testing {
namespace bwe {

const int kFeedbackIntervalMs = 50;

FullBweSender::FullBweSender(int kbps, BitrateObserver* observer, Clock* clock)
    : bitrate_controller_(
          BitrateController::CreateBitrateController(clock, observer)),
      rbe_(new RemoteBitrateEstimatorAbsSendTime(this)),
      feedback_observer_(bitrate_controller_->CreateRtcpBandwidthObserver()),
      clock_(clock),
      send_time_history_(clock_, 10000),
      has_received_ack_(false),
      last_acked_seq_num_(0) {
  assert(kbps >= kMinBitrateKbps);
  assert(kbps <= kMaxBitrateKbps);
  bitrate_controller_->SetStartBitrate(1000 * kbps);
  bitrate_controller_->SetMinMaxBitrate(1000 * kMinBitrateKbps,
                                        1000 * kMaxBitrateKbps);
  rbe_->SetMinBitrate(1000 * kMinBitrateKbps);
}

FullBweSender::~FullBweSender() {
}

int FullBweSender::GetFeedbackIntervalMs() const {
  return kFeedbackIntervalMs;
}

void FullBweSender::GiveFeedback(const FeedbackPacket& feedback) {
  const SendSideBweFeedback& fb =
      static_cast<const SendSideBweFeedback&>(feedback);
  if (fb.packet_feedback_vector().empty())
    return;
  std::vector<PacketInfo> packet_feedback_vector(fb.packet_feedback_vector());
  for (PacketInfo& packet_info : packet_feedback_vector) {
    if (!send_time_history_.GetInfo(&packet_info, true)) {
      LOG(LS_WARNING) << "Ack arrived too late.";
    }
  }

  int64_t rtt_ms =
      clock_->TimeInMilliseconds() - feedback.latest_send_time_ms();
  rbe_->OnRttUpdate(rtt_ms, rtt_ms);
  BWE_TEST_LOGGING_PLOT(1, "RTT", clock_->TimeInMilliseconds(), rtt_ms);

  rbe_->IncomingPacketFeedbackVector(packet_feedback_vector);
  if (has_received_ack_) {
    int expected_packets = fb.packet_feedback_vector().back().sequence_number -
                           last_acked_seq_num_;
    // Assuming no reordering for now.
    if (expected_packets > 0) {
      int lost_packets = expected_packets -
                         static_cast<int>(fb.packet_feedback_vector().size());
      report_block_.fractionLost = (lost_packets << 8) / expected_packets;
      report_block_.cumulativeLost += lost_packets;
      report_block_.extendedHighSeqNum =
          packet_feedback_vector.back().sequence_number;
      ReportBlockList report_blocks;
      report_blocks.push_back(report_block_);
      feedback_observer_->OnReceivedRtcpReceiverReport(
          report_blocks, rtt_ms, clock_->TimeInMilliseconds());
    }
    bitrate_controller_->Process();

    last_acked_seq_num_ = LatestSequenceNumber(
        packet_feedback_vector.back().sequence_number, last_acked_seq_num_);
  } else {
    last_acked_seq_num_ = packet_feedback_vector.back().sequence_number;
    has_received_ack_ = true;
  }
}

void FullBweSender::OnPacketsSent(const Packets& packets) {
  for (Packet* packet : packets) {
    if (packet->GetPacketType() == Packet::kMedia) {
      MediaPacket* media_packet = static_cast<MediaPacket*>(packet);
      send_time_history_.AddAndRemoveOld(media_packet->header().sequenceNumber,
                                         media_packet->payload_size());
      send_time_history_.OnSentPacket(media_packet->header().sequenceNumber,
                                      media_packet->sender_timestamp_ms());
    }
  }
}

void FullBweSender::OnReceiveBitrateChanged(const std::vector<uint32_t>& ssrcs,
                                            uint32_t bitrate) {
  feedback_observer_->OnReceivedEstimatedBitrate(bitrate);
}

int64_t FullBweSender::TimeUntilNextProcess() {
  return bitrate_controller_->TimeUntilNextProcess();
}

void FullBweSender::Process() {
  rbe_->Process();
  bitrate_controller_->Process();
}

SendSideBweReceiver::SendSideBweReceiver(int flow_id)
    : BweReceiver(flow_id), last_feedback_ms_(0) {
}

SendSideBweReceiver::~SendSideBweReceiver() {
}

void SendSideBweReceiver::ReceivePacket(int64_t arrival_time_ms,
                                        const MediaPacket& media_packet) {
  packet_feedback_vector_.push_back(PacketInfo(
      -1, arrival_time_ms, media_packet.sender_timestamp_ms(),
      media_packet.header().sequenceNumber, media_packet.payload_size()));

  // Log received packet information.
  BweReceiver::ReceivePacket(arrival_time_ms, media_packet);
}

FeedbackPacket* SendSideBweReceiver::GetFeedback(int64_t now_ms) {
  if (now_ms - last_feedback_ms_ < kFeedbackIntervalMs)
    return NULL;
  last_feedback_ms_ = now_ms;
  int64_t corrected_send_time_ms =
      packet_feedback_vector_.back().send_time_ms + now_ms -
      packet_feedback_vector_.back().arrival_time_ms;
  FeedbackPacket* fb = new SendSideBweFeedback(
      flow_id_, now_ms * 1000, corrected_send_time_ms, packet_feedback_vector_);
  packet_feedback_vector_.clear();
  return fb;
}

}  // namespace bwe
}  // namespace testing
}  // namespace webrtc
