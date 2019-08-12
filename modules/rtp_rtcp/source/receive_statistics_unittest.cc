/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/include/receive_statistics.h"

#include <memory>
#include <vector>

#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "rtc_base/random.h"
#include "system_wrappers/include/clock.h"
#include "test/gmock.h"
#include "test/gtest.h"

namespace webrtc {
namespace {

using ::testing::SizeIs;
using ::testing::UnorderedElementsAre;

const size_t kPacketSize1 = 100;
const size_t kPacketSize2 = 300;
const uint32_t kSsrc1 = 101;
const uint32_t kSsrc2 = 202;
const uint32_t kSsrc3 = 203;
const uint32_t kSsrc4 = 304;

RtpPacketReceived CreateRtpPacket(uint32_t ssrc,
                                  size_t header_size,
                                  size_t payload_size,
                                  size_t padding_size) {
  RtpPacketReceived packet;
  packet.SetSsrc(ssrc);
  packet.SetSequenceNumber(100);
  packet.set_payload_type_frequency(90000);
  RTC_CHECK_GE(header_size, 12);
  RTC_CHECK_EQ(header_size % 4, 0);
  if (header_size > 12) {
    // Insert csrcs to increase header size.
    const int num_csrcs = (header_size - 12) / 4;
    std::vector<uint32_t> csrcs(num_csrcs);
    packet.SetCsrcs(csrcs);
  }
  packet.SetPayloadSize(payload_size);
  packet.SetPadding(padding_size);
  return packet;
}

RtpPacketReceived CreateRtpPacket(uint32_t ssrc, size_t packet_size) {
  return CreateRtpPacket(ssrc, 12, packet_size - 12, 0);
}

void IncrementSequenceNumber(RtpPacketReceived* packet, uint16_t incr) {
  packet->SetSequenceNumber(packet->SequenceNumber() + incr);
}

void IncrementSequenceNumber(RtpPacketReceived* packet) {
  IncrementSequenceNumber(packet, 1);
}

class ReceiveStatisticsTest : public ::testing::Test {
 public:
  ReceiveStatisticsTest()
      : clock_(0),
        receive_statistics_(ReceiveStatistics::Create(&clock_, nullptr)) {
    packet1_ = CreateRtpPacket(kSsrc1, kPacketSize1);
    packet2_ = CreateRtpPacket(kSsrc2, kPacketSize2);
  }

 protected:
  SimulatedClock clock_;
  std::unique_ptr<ReceiveStatistics> receive_statistics_;
  RtpPacketReceived packet1_;
  RtpPacketReceived packet2_;
};

TEST_F(ReceiveStatisticsTest, TwoIncomingSsrcs) {
  receive_statistics_->OnRtpPacket(packet1_);
  IncrementSequenceNumber(&packet1_);
  receive_statistics_->OnRtpPacket(packet2_);
  IncrementSequenceNumber(&packet2_);
  clock_.AdvanceTimeMilliseconds(100);
  receive_statistics_->OnRtpPacket(packet1_);
  IncrementSequenceNumber(&packet1_);
  receive_statistics_->OnRtpPacket(packet2_);
  IncrementSequenceNumber(&packet2_);

  StreamStatistician* statistician =
      receive_statistics_->GetStatistician(kSsrc1);
  ASSERT_TRUE(statistician != NULL);
  EXPECT_GT(statistician->BitrateReceived(), 0u);
  StreamDataCounters counters = statistician->GetReceiveStreamDataCounters();
  EXPECT_EQ(176u, counters.transmitted.payload_bytes);
  EXPECT_EQ(24u, counters.transmitted.header_bytes);
  EXPECT_EQ(0u, counters.transmitted.padding_bytes);
  EXPECT_EQ(2u, counters.transmitted.packets);

  statistician = receive_statistics_->GetStatistician(kSsrc2);
  ASSERT_TRUE(statistician != NULL);
  EXPECT_GT(statistician->BitrateReceived(), 0u);
  counters = statistician->GetReceiveStreamDataCounters();
  EXPECT_EQ(576u, counters.transmitted.payload_bytes);
  EXPECT_EQ(24u, counters.transmitted.header_bytes);
  EXPECT_EQ(0u, counters.transmitted.padding_bytes);
  EXPECT_EQ(2u, counters.transmitted.packets);

  EXPECT_EQ(2u, receive_statistics_->RtcpReportBlocks(3).size());
  // Add more incoming packets and verify that they are registered in both
  // access methods.
  receive_statistics_->OnRtpPacket(packet1_);
  IncrementSequenceNumber(&packet1_);
  receive_statistics_->OnRtpPacket(packet2_);
  IncrementSequenceNumber(&packet2_);

  counters = receive_statistics_->GetStatistician(kSsrc1)
                 ->GetReceiveStreamDataCounters();
  EXPECT_EQ(264u, counters.transmitted.payload_bytes);
  EXPECT_EQ(36u, counters.transmitted.header_bytes);
  EXPECT_EQ(0u, counters.transmitted.padding_bytes);
  EXPECT_EQ(3u, counters.transmitted.packets);

  counters = receive_statistics_->GetStatistician(kSsrc2)
                 ->GetReceiveStreamDataCounters();
  EXPECT_EQ(864u, counters.transmitted.payload_bytes);
  EXPECT_EQ(36u, counters.transmitted.header_bytes);
  EXPECT_EQ(0u, counters.transmitted.padding_bytes);
  EXPECT_EQ(3u, counters.transmitted.packets);
}

TEST_F(ReceiveStatisticsTest,
       RtcpReportBlocksReturnsMaxBlocksWhenThereAreMoreStatisticians) {
  RtpPacketReceived packet1 = CreateRtpPacket(kSsrc1, kPacketSize1);
  RtpPacketReceived packet2 = CreateRtpPacket(kSsrc2, kPacketSize1);
  RtpPacketReceived packet3 = CreateRtpPacket(kSsrc3, kPacketSize1);
  receive_statistics_->OnRtpPacket(packet1);
  receive_statistics_->OnRtpPacket(packet2);
  receive_statistics_->OnRtpPacket(packet3);

  EXPECT_THAT(receive_statistics_->RtcpReportBlocks(2), SizeIs(2));
  EXPECT_THAT(receive_statistics_->RtcpReportBlocks(2), SizeIs(2));
  EXPECT_THAT(receive_statistics_->RtcpReportBlocks(2), SizeIs(2));
}

TEST_F(ReceiveStatisticsTest,
       RtcpReportBlocksReturnsAllObservedSsrcsWithMultipleCalls) {
  RtpPacketReceived packet1 = CreateRtpPacket(kSsrc1, kPacketSize1);
  RtpPacketReceived packet2 = CreateRtpPacket(kSsrc2, kPacketSize1);
  RtpPacketReceived packet3 = CreateRtpPacket(kSsrc3, kPacketSize1);
  RtpPacketReceived packet4 = CreateRtpPacket(kSsrc4, kPacketSize1);
  receive_statistics_->OnRtpPacket(packet1);
  receive_statistics_->OnRtpPacket(packet2);
  receive_statistics_->OnRtpPacket(packet3);
  receive_statistics_->OnRtpPacket(packet4);

  std::vector<uint32_t> observed_ssrcs;
  std::vector<rtcp::ReportBlock> report_blocks =
      receive_statistics_->RtcpReportBlocks(2);
  ASSERT_THAT(report_blocks, SizeIs(2));
  observed_ssrcs.push_back(report_blocks[0].source_ssrc());
  observed_ssrcs.push_back(report_blocks[1].source_ssrc());

  report_blocks = receive_statistics_->RtcpReportBlocks(2);
  ASSERT_THAT(report_blocks, SizeIs(2));
  observed_ssrcs.push_back(report_blocks[0].source_ssrc());
  observed_ssrcs.push_back(report_blocks[1].source_ssrc());

  EXPECT_THAT(observed_ssrcs,
              UnorderedElementsAre(kSsrc1, kSsrc2, kSsrc3, kSsrc4));
}

TEST_F(ReceiveStatisticsTest, ActiveStatisticians) {
  receive_statistics_->OnRtpPacket(packet1_);
  IncrementSequenceNumber(&packet1_);
  clock_.AdvanceTimeMilliseconds(1000);
  receive_statistics_->OnRtpPacket(packet2_);
  IncrementSequenceNumber(&packet2_);
  // Nothing should time out since only 1000 ms has passed since the first
  // packet came in.
  EXPECT_EQ(2u, receive_statistics_->RtcpReportBlocks(3).size());

  clock_.AdvanceTimeMilliseconds(7000);
  // kSsrc1 should have timed out.
  EXPECT_EQ(1u, receive_statistics_->RtcpReportBlocks(3).size());

  clock_.AdvanceTimeMilliseconds(1000);
  // kSsrc2 should have timed out.
  EXPECT_EQ(0u, receive_statistics_->RtcpReportBlocks(3).size());

  receive_statistics_->OnRtpPacket(packet1_);
  IncrementSequenceNumber(&packet1_);
  // kSsrc1 should be active again and the data counters should have survived.
  EXPECT_EQ(1u, receive_statistics_->RtcpReportBlocks(3).size());
  StreamStatistician* statistician =
      receive_statistics_->GetStatistician(kSsrc1);
  ASSERT_TRUE(statistician != NULL);
  StreamDataCounters counters = statistician->GetReceiveStreamDataCounters();
  EXPECT_EQ(176u, counters.transmitted.payload_bytes);
  EXPECT_EQ(24u, counters.transmitted.header_bytes);
  EXPECT_EQ(0u, counters.transmitted.padding_bytes);
  EXPECT_EQ(2u, counters.transmitted.packets);
}

TEST_F(ReceiveStatisticsTest,
       DoesntCreateRtcpReportBlockUntilFirstReceivedPacketForSsrc) {
  // Creates a statistician object for the ssrc.
  receive_statistics_->EnableRetransmitDetection(kSsrc1, true);
  EXPECT_TRUE(receive_statistics_->GetStatistician(kSsrc1) != nullptr);
  EXPECT_EQ(0u, receive_statistics_->RtcpReportBlocks(3).size());
  // Receive first packet
  receive_statistics_->OnRtpPacket(packet1_);
  EXPECT_EQ(1u, receive_statistics_->RtcpReportBlocks(3).size());
}

TEST_F(ReceiveStatisticsTest, GetReceiveStreamDataCounters) {
  receive_statistics_->OnRtpPacket(packet1_);
  StreamStatistician* statistician =
      receive_statistics_->GetStatistician(kSsrc1);
  ASSERT_TRUE(statistician != NULL);

  StreamDataCounters counters = statistician->GetReceiveStreamDataCounters();
  EXPECT_GT(counters.first_packet_time_ms, -1);
  EXPECT_EQ(1u, counters.transmitted.packets);

  receive_statistics_->OnRtpPacket(packet1_);
  counters = statistician->GetReceiveStreamDataCounters();
  EXPECT_GT(counters.first_packet_time_ms, -1);
  EXPECT_EQ(2u, counters.transmitted.packets);
}

TEST_F(ReceiveStatisticsTest, SimpleLossComputation) {
  packet1_.SetSequenceNumber(1);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(3);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(4);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(5);
  receive_statistics_->OnRtpPacket(packet1_);

  StreamStatistician* statistician =
      receive_statistics_->GetStatistician(kSsrc1);
  RtcpStatistics statistics;
  statistician->GetStatistics(&statistics, true);
  // 20% = 51/255.
  EXPECT_EQ(51u, statistics.fraction_lost);
  EXPECT_EQ(1, statistics.packets_lost);
  EXPECT_EQ(20, statistician->GetFractionLostInPercent());
}

TEST_F(ReceiveStatisticsTest, LossComputationWithReordering) {
  packet1_.SetSequenceNumber(1);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(3);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(2);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(5);
  receive_statistics_->OnRtpPacket(packet1_);

  StreamStatistician* statistician =
      receive_statistics_->GetStatistician(kSsrc1);
  RtcpStatistics statistics;
  statistician->GetStatistics(&statistics, true);
  // 20% = 51/255.
  EXPECT_EQ(51u, statistics.fraction_lost);
  EXPECT_EQ(1, statistics.packets_lost);
  EXPECT_EQ(20, statistician->GetFractionLostInPercent());
}

TEST_F(ReceiveStatisticsTest, LossComputationWithDuplicates) {
  // Lose 2 packets, but also receive 1 duplicate. Should actually count as
  // only 1 packet being lost.
  packet1_.SetSequenceNumber(1);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(4);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(4);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(5);
  receive_statistics_->OnRtpPacket(packet1_);

  StreamStatistician* statistician =
      receive_statistics_->GetStatistician(kSsrc1);
  RtcpStatistics statistics;
  statistician->GetStatistics(&statistics, true);
  // 20% = 51/255.
  EXPECT_EQ(51u, statistics.fraction_lost);
  EXPECT_EQ(1, statistics.packets_lost);
  EXPECT_EQ(20, statistician->GetFractionLostInPercent());
}

TEST_F(ReceiveStatisticsTest, LossComputationWithSequenceNumberWrapping) {
  // First, test loss computation over a period that included a sequence number
  // rollover.
  packet1_.SetSequenceNumber(0xfffd);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(0);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(0xfffe);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(1);
  receive_statistics_->OnRtpPacket(packet1_);

  // Only one packet was actually lost, 0xffff.
  StreamStatistician* statistician =
      receive_statistics_->GetStatistician(kSsrc1);
  RtcpStatistics statistics;
  statistician->GetStatistics(&statistics, true);
  // 20% = 51/255.
  EXPECT_EQ(51u, statistics.fraction_lost);
  EXPECT_EQ(1, statistics.packets_lost);
  EXPECT_EQ(20, statistician->GetFractionLostInPercent());

  // Now test losing one packet *after* the rollover.
  packet1_.SetSequenceNumber(3);
  receive_statistics_->OnRtpPacket(packet1_);
  statistician->GetStatistics(&statistics, true);
  // 50% = 127/255.
  EXPECT_EQ(127u, statistics.fraction_lost);
  EXPECT_EQ(2, statistics.packets_lost);
  // 2 packets lost, 7 expected
  EXPECT_EQ(28, statistician->GetFractionLostInPercent());
}

TEST_F(ReceiveStatisticsTest, StreamRestartDoesntCountAsLoss) {
  RtcpStatistics statistics;
  receive_statistics_->SetMaxReorderingThreshold(kSsrc1, 200);

  packet1_.SetSequenceNumber(0);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(1);
  receive_statistics_->OnRtpPacket(packet1_);

  packet1_.SetSequenceNumber(400);
  receive_statistics_->OnRtpPacket(packet1_);
  StreamStatistician* statistician =
      receive_statistics_->GetStatistician(kSsrc1);
  statistician->GetStatistics(&statistics, true);
  EXPECT_EQ(0, statistics.fraction_lost);
  EXPECT_EQ(0, statistics.packets_lost);
  EXPECT_EQ(0, statistician->GetFractionLostInPercent());

  packet1_.SetSequenceNumber(401);
  receive_statistics_->OnRtpPacket(packet1_);
  statistician->GetStatistics(&statistics, true);
  EXPECT_EQ(0, statistics.fraction_lost);
  EXPECT_EQ(0, statistics.packets_lost);
  EXPECT_EQ(0, statistician->GetFractionLostInPercent());
}

TEST_F(ReceiveStatisticsTest, CountsLossAfterStreamRestart) {
  RtcpStatistics statistics;
  receive_statistics_->SetMaxReorderingThreshold(kSsrc1, 200);

  packet1_.SetSequenceNumber(0);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(1);
  receive_statistics_->OnRtpPacket(packet1_);

  packet1_.SetSequenceNumber(400);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(401);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(403);
  receive_statistics_->OnRtpPacket(packet1_);

  StreamStatistician* statistician =
      receive_statistics_->GetStatistician(kSsrc1);

  statistician->GetStatistics(&statistics, true);
  EXPECT_EQ(1, statistics.packets_lost);
  // Is this reasonable? */
  EXPECT_EQ(0, statistician->GetFractionLostInPercent());
}

TEST_F(ReceiveStatisticsTest, StreamCanRestartAtSequenceNumberWrapAround) {
  RtcpStatistics statistics;
  receive_statistics_->SetMaxReorderingThreshold(kSsrc1, 200);

  packet1_.SetSequenceNumber(0xffff - 401);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(0xffff - 400);
  receive_statistics_->OnRtpPacket(packet1_);

  packet1_.SetSequenceNumber(0xffff);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(0);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(2);
  receive_statistics_->OnRtpPacket(packet1_);

  receive_statistics_->GetStatistician(kSsrc1)->GetStatistics(&statistics,
                                                              true);
  EXPECT_EQ(1, statistics.packets_lost);
}

TEST_F(ReceiveStatisticsTest, StreamRestartNeedsTwoConsecutivePackets) {
  RtcpStatistics statistics;
  receive_statistics_->SetMaxReorderingThreshold(kSsrc1, 200);

  packet1_.SetSequenceNumber(400);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(401);
  receive_statistics_->OnRtpPacket(packet1_);

  packet1_.SetSequenceNumber(1);
  receive_statistics_->OnRtpPacket(packet1_);
  packet1_.SetSequenceNumber(3);
  receive_statistics_->OnRtpPacket(packet1_);
  receive_statistics_->GetStatistician(kSsrc1)->GetStatistics(&statistics,
                                                              true);
  EXPECT_EQ(401u, statistics.extended_highest_sequence_number);

  packet1_.SetSequenceNumber(4);
  receive_statistics_->OnRtpPacket(packet1_);
  receive_statistics_->GetStatistician(kSsrc1)->GetStatistics(&statistics,
                                                              true);
  EXPECT_EQ(4u, statistics.extended_highest_sequence_number);
}

TEST_F(ReceiveStatisticsTest, WrapsAroundExtendedHighestSequenceNumber) {
  RtcpStatistics statistics;
  packet1_.SetSequenceNumber(0xffff);
  receive_statistics_->OnRtpPacket(packet1_);
  receive_statistics_->GetStatistician(kSsrc1)->GetStatistics(&statistics,
                                                              true);
  EXPECT_EQ(0xffffu, statistics.extended_highest_sequence_number);

  // Wrap around.
  packet1_.SetSequenceNumber(1);
  receive_statistics_->OnRtpPacket(packet1_);
  receive_statistics_->GetStatistician(kSsrc1)->GetStatistics(&statistics,
                                                              true);
  EXPECT_EQ(0x10001u, statistics.extended_highest_sequence_number);

  // Should be treated as out of order; shouldn't increment highest extended
  // sequence number.
  packet1_.SetSequenceNumber(0x10000 - 6);
  receive_statistics_->GetStatistician(kSsrc1)->GetStatistics(&statistics,
                                                              true);
  EXPECT_EQ(0x10001u, statistics.extended_highest_sequence_number);

  // Receive a couple packets then wrap around again.
  receive_statistics_->SetMaxReorderingThreshold(kSsrc1, 200);
  for (int i = 10; i < 0xffff; i += 150) {
    packet1_.SetSequenceNumber(i);
    receive_statistics_->OnRtpPacket(packet1_);
  }
  packet1_.SetSequenceNumber(1);
  receive_statistics_->OnRtpPacket(packet1_);
  receive_statistics_->GetStatistician(kSsrc1)->GetStatistics(&statistics,
                                                              true);
  EXPECT_EQ(0x20001u, statistics.extended_highest_sequence_number);
}

class RtpTestCallback : public StreamDataCountersCallback {
 public:
  RtpTestCallback()
      : StreamDataCountersCallback(), num_calls_(0), ssrc_(0), stats_() {}
  ~RtpTestCallback() override = default;

  void DataCountersUpdated(const StreamDataCounters& counters,
                           uint32_t ssrc) override {
    ssrc_ = ssrc;
    stats_ = counters;
    ++num_calls_;
  }

  void MatchPacketCounter(const RtpPacketCounter& expected,
                          const RtpPacketCounter& actual) {
    EXPECT_EQ(expected.payload_bytes, actual.payload_bytes);
    EXPECT_EQ(expected.header_bytes, actual.header_bytes);
    EXPECT_EQ(expected.padding_bytes, actual.padding_bytes);
    EXPECT_EQ(expected.packets, actual.packets);
  }

  void Matches(uint32_t num_calls,
               uint32_t ssrc,
               const StreamDataCounters& expected) {
    EXPECT_EQ(num_calls, num_calls_);
    EXPECT_EQ(ssrc, ssrc_);
    MatchPacketCounter(expected.transmitted, stats_.transmitted);
    MatchPacketCounter(expected.retransmitted, stats_.retransmitted);
    MatchPacketCounter(expected.fec, stats_.fec);
  }

  uint32_t num_calls_;
  uint32_t ssrc_;
  StreamDataCounters stats_;
};

TEST_F(ReceiveStatisticsTest, RtpCallbacks) {
  RtpTestCallback callback;
  receive_statistics_ = ReceiveStatistics::Create(&clock_, &callback);
  receive_statistics_->EnableRetransmitDetection(kSsrc1, true);

  const size_t kHeaderLength = 20;
  const size_t kPaddingLength = 9;

  // One packet with payload size kPacketSize1.
  RtpPacketReceived packet1 =
      CreateRtpPacket(kSsrc1, kHeaderLength, kPacketSize1, 0);
  receive_statistics_->OnRtpPacket(packet1);
  StreamDataCounters expected;
  expected.transmitted.payload_bytes = kPacketSize1;
  expected.transmitted.header_bytes = kHeaderLength;
  expected.transmitted.padding_bytes = 0;
  expected.transmitted.packets = 1;
  expected.retransmitted.payload_bytes = 0;
  expected.retransmitted.header_bytes = 0;
  expected.retransmitted.padding_bytes = 0;
  expected.retransmitted.packets = 0;
  expected.fec.packets = 0;
  callback.Matches(1, kSsrc1, expected);

  // Another packet of size kPacketSize1 with 9 bytes padding.
  RtpPacketReceived packet2 =
      CreateRtpPacket(kSsrc1, kHeaderLength, kPacketSize1, 9);
  packet2.SetSequenceNumber(packet1.SequenceNumber() + 1);
  clock_.AdvanceTimeMilliseconds(5);
  receive_statistics_->OnRtpPacket(packet2);
  expected.transmitted.payload_bytes = kPacketSize1 * 2;
  expected.transmitted.header_bytes = kHeaderLength * 2;
  expected.transmitted.padding_bytes = kPaddingLength;
  expected.transmitted.packets = 2;
  callback.Matches(2, kSsrc1, expected);

  clock_.AdvanceTimeMilliseconds(5);
  // Retransmit last packet.
  receive_statistics_->OnRtpPacket(packet2);
  expected.transmitted.payload_bytes = kPacketSize1 * 3;
  expected.transmitted.header_bytes = kHeaderLength * 3;
  expected.transmitted.padding_bytes = kPaddingLength * 2;
  expected.transmitted.packets = 3;
  expected.retransmitted.payload_bytes = kPacketSize1;
  expected.retransmitted.header_bytes = kHeaderLength;
  expected.retransmitted.padding_bytes = kPaddingLength;
  expected.retransmitted.packets = 1;
  callback.Matches(3, kSsrc1, expected);

  // One FEC packet.
  packet1.SetSequenceNumber(packet2.SequenceNumber() + 1);
  clock_.AdvanceTimeMilliseconds(5);
  receive_statistics_->OnRtpPacket(packet1);
  receive_statistics_->FecPacketReceived(packet1);
  expected.transmitted.payload_bytes = kPacketSize1 * 4;
  expected.transmitted.header_bytes = kHeaderLength * 4;
  expected.transmitted.packets = 4;
  expected.fec.payload_bytes = kPacketSize1;
  expected.fec.header_bytes = kHeaderLength;
  expected.fec.packets = 1;
  callback.Matches(5, kSsrc1, expected);
}

TEST_F(ReceiveStatisticsTest, LastPacketReceivedTimestamp) {
  RtpTestCallback callback;
  receive_statistics_ = ReceiveStatistics::Create(&clock_, &callback);

  clock_.AdvanceTimeMilliseconds(42);
  receive_statistics_->OnRtpPacket(packet1_);
  EXPECT_EQ(42, callback.stats_.last_packet_received_timestamp_ms);

  clock_.AdvanceTimeMilliseconds(3);
  receive_statistics_->OnRtpPacket(packet1_);
  EXPECT_EQ(45, callback.stats_.last_packet_received_timestamp_ms);
}

TEST_F(ReceiveStatisticsTest, RtpCallbacksFecFirst) {
  RtpTestCallback callback;
  receive_statistics_ = ReceiveStatistics::Create(&clock_, &callback);

  const uint32_t kHeaderLength = 20;
  RtpPacketReceived packet =
      CreateRtpPacket(kSsrc1, kHeaderLength, kPacketSize1, 0);
  // If first packet is FEC, ignore it.
  receive_statistics_->FecPacketReceived(packet);
  EXPECT_EQ(0u, callback.num_calls_);

  receive_statistics_->OnRtpPacket(packet);
  StreamDataCounters expected;
  expected.transmitted.payload_bytes = kPacketSize1;
  expected.transmitted.header_bytes = kHeaderLength;
  expected.transmitted.padding_bytes = 0;
  expected.transmitted.packets = 1;
  expected.fec.packets = 0;
  callback.Matches(1, kSsrc1, expected);

  receive_statistics_->FecPacketReceived(packet);
  expected.fec.payload_bytes = kPacketSize1;
  expected.fec.header_bytes = kHeaderLength;
  expected.fec.packets = 1;
  callback.Matches(2, kSsrc1, expected);
}

}  // namespace
}  // namespace webrtc
