/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator.h"

#include <utility>

#include "absl/memory/memory.h"
#include "api/transport/field_trial_based_config.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "rtc_base/fake_clock.h"
#include "test/gmock.h"
#include "test/gtest.h"

using ::testing::_;
using ::testing::InSequence;
using ::testing::NiceMock;
using ::testing::Return;

namespace webrtc {

namespace {

constexpr int64_t kFirstArrivalTimeMs = 10;
constexpr int64_t kFirstSendTimeMs = 10;
constexpr uint16_t kSequenceNumber = 1;
constexpr size_t kPayloadSize = 10;

class MockBitrateEstimator : public BitrateEstimator {
 public:
  using BitrateEstimator::BitrateEstimator;
  MOCK_METHOD3(Update, void(int64_t now_ms, int bytes, bool in_alr));
  MOCK_CONST_METHOD0(bitrate_bps, absl::optional<uint32_t>());
  MOCK_METHOD0(ExpectFastRateChange, void());
};

struct AcknowledgedBitrateEstimatorTestStates {
  FieldTrialBasedConfig field_trial_config;
  std::unique_ptr<AcknowledgedBitrateEstimator> acknowledged_bitrate_estimator;
  MockBitrateEstimator* mock_bitrate_estimator;
};

AcknowledgedBitrateEstimatorTestStates CreateTestStates() {
  AcknowledgedBitrateEstimatorTestStates states;
  auto mock_bitrate_estimator =
      absl::make_unique<MockBitrateEstimator>(&states.field_trial_config);
  states.mock_bitrate_estimator = mock_bitrate_estimator.get();
  states.acknowledged_bitrate_estimator =
      absl::make_unique<AcknowledgedBitrateEstimator>(
          &states.field_trial_config, std::move(mock_bitrate_estimator));
  return states;
}

std::vector<PacketFeedback> CreateFeedbackVector() {
  std::vector<PacketFeedback> packet_feedback_vector;
  const PacedPacketInfo pacing_info;
  packet_feedback_vector.push_back(
      PacketFeedback(kFirstArrivalTimeMs, kFirstSendTimeMs, kSequenceNumber,
                     kPayloadSize, pacing_info));
  packet_feedback_vector.push_back(
      PacketFeedback(kFirstArrivalTimeMs + 10, kFirstSendTimeMs + 10,
                     kSequenceNumber, kPayloadSize + 10, pacing_info));
  return packet_feedback_vector;
}

}  // anonymous namespace

TEST(TestAcknowledgedBitrateEstimator, DontAddPacketsWhichAreNotInSendHistory) {
  auto states = CreateTestStates();
  std::vector<PacketFeedback> packet_feedback_vector;
  packet_feedback_vector.push_back(
      PacketFeedback(kFirstArrivalTimeMs, kSequenceNumber));
  EXPECT_CALL(*states.mock_bitrate_estimator, Update(_, _, _)).Times(0);
  states.acknowledged_bitrate_estimator->IncomingPacketFeedbackVector(
      packet_feedback_vector);
}

TEST(TestAcknowledgedBitrateEstimator, UpdateBandwidth) {
  auto states = CreateTestStates();
  auto packet_feedback_vector = CreateFeedbackVector();
  {
    InSequence dummy;
    EXPECT_CALL(*states.mock_bitrate_estimator,
                Update(packet_feedback_vector[0].arrival_time_ms,
                       static_cast<int>(packet_feedback_vector[0].payload_size),
                       /*in_alr*/ false))
        .Times(1);
    EXPECT_CALL(*states.mock_bitrate_estimator,
                Update(packet_feedback_vector[1].arrival_time_ms,
                       static_cast<int>(packet_feedback_vector[1].payload_size),
                       /*in_alr*/ false))
        .Times(1);
  }
  states.acknowledged_bitrate_estimator->IncomingPacketFeedbackVector(
      packet_feedback_vector);
}

TEST(TestAcknowledgedBitrateEstimator, ExpectFastRateChangeWhenLeftAlr) {
  auto states = CreateTestStates();
  auto packet_feedback_vector = CreateFeedbackVector();
  {
    InSequence dummy;
    EXPECT_CALL(*states.mock_bitrate_estimator,
                Update(packet_feedback_vector[0].arrival_time_ms,
                       static_cast<int>(packet_feedback_vector[0].payload_size),
                       /*in_alr*/ false))
        .Times(1);
    EXPECT_CALL(*states.mock_bitrate_estimator, ExpectFastRateChange())
        .Times(1);
    EXPECT_CALL(*states.mock_bitrate_estimator,
                Update(packet_feedback_vector[1].arrival_time_ms,
                       static_cast<int>(packet_feedback_vector[1].payload_size),
                       /*in_alr*/ false))
        .Times(1);
  }
  states.acknowledged_bitrate_estimator->SetAlrEndedTimeMs(kFirstArrivalTimeMs +
                                                           1);
  states.acknowledged_bitrate_estimator->IncomingPacketFeedbackVector(
      packet_feedback_vector);
}

TEST(TestAcknowledgedBitrateEstimator, ReturnBitrate) {
  auto states = CreateTestStates();
  absl::optional<uint32_t> return_value(42);
  EXPECT_CALL(*states.mock_bitrate_estimator, bitrate_bps())
      .Times(1)
      .WillOnce(Return(return_value));
  EXPECT_EQ(return_value, states.acknowledged_bitrate_estimator->bitrate_bps());
}

}  // namespace webrtc*/
