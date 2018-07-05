/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <utility>

#include "modules/congestion_controller/pcc/bitrate_controller.h"
#include "modules/congestion_controller/pcc/monitor_interval.h"
#include "rtc_base/ptr_util.h"
#include "test/gmock.h"
#include "test/gtest.h"

namespace webrtc {
namespace pcc {
namespace test {
namespace {
constexpr double kInitialConversionFactor = 1;
constexpr double kInitialDynamicBoundary = 0.05;
constexpr double kDynamicBoundaryIncrement = 0.1;

constexpr double kRttGradientCoefficient = 900;
constexpr double kLossCoefficient = 11.35;
constexpr double kThroughputCoefficient = 0.9;
constexpr double kRttGradientThreshold = 0.01;

const DataRate kTargetSendingRate = DataRate::kbps(300);
const double kEpsilon = 0.05;
const Timestamp kStartTime = Timestamp::us(0);
const TimeDelta kPacketsDelta = TimeDelta::us(1);
const TimeDelta kIntervalDuration = TimeDelta::us(1000);
const TimeDelta kDefaultRtt = TimeDelta::us(1000);
const DataSize kDefaultDataSize = DataSize::bytes(100);
const TimeDelta kMiTimeout = kIntervalDuration * 4;

std::vector<PacketResult> CreatePacketResults(
    const std::vector<Timestamp>& packets_send_times,
    const std::vector<Timestamp>& packets_received_times = {},
    const std::vector<DataSize>& packets_sizes = {}) {
  std::vector<PacketResult> packet_results;
  PacketResult packet_result;
  SentPacket sent_packet;
  for (size_t i = 0; i < packets_send_times.size(); ++i) {
    sent_packet.send_time = packets_send_times[i];
    if (packets_sizes.empty()) {
      sent_packet.size = kDefaultDataSize;
    } else {
      sent_packet.size = packets_sizes[i];
    }
    packet_result.sent_packet = sent_packet;
    if (packets_received_times.empty()) {
      packet_result.receive_time = packets_send_times[i] + kDefaultRtt;
    } else {
      packet_result.receive_time = packets_received_times[i];
    }
    packet_results.push_back(packet_result);
  }
  return packet_results;
}

class MockUtilityFunction : public UtilityFunction {
 public:
  MOCK_CONST_METHOD1(ComputeUtilityFunction,
                     double(const MonitorInterval& monitor_interval));
};

}  // namespace

TEST(PccConfidenceAmplifierTest, ValueInZero) {
  ConfidenceAmplifier confidence_amplifier;
  EXPECT_EQ(1, confidence_amplifier(0));
}

TEST(PccConfidenceAmplifierTest, MonotonicallyNonDecreasingFunction) {
  ConfidenceAmplifier confidence_amplifier;
  for (int i = 0; i < 100; ++i) {
    EXPECT_LE(confidence_amplifier(i), confidence_amplifier(i + 1));
  }
}

TEST(PccStepSizeTest, InitialValue) {
  StepSize step_size(ConfidenceAmplifier{}, kInitialConversionFactor);
  EXPECT_EQ(step_size.ComputeStepSize(10), kInitialConversionFactor);
}

TEST(PccStepSizeTest, ChangeInGradientSign) {
  StepSize step_size(ConfidenceAmplifier{}, kInitialConversionFactor);
  EXPECT_EQ(step_size.ComputeStepSize(10), kInitialConversionFactor);
  EXPECT_EQ(step_size.ComputeStepSize(-10), kInitialConversionFactor);
  EXPECT_EQ(step_size.ComputeStepSize(10), kInitialConversionFactor);
}

TEST(PccStepSizeTest, StepSizeIncreaseWhenGradientDoesntChangeSign) {
  ConfidenceAmplifier confidence_amplifier;
  StepSize step_size(ConfidenceAmplifier{}, kInitialConversionFactor);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(step_size.ComputeStepSize(10),
              confidence_amplifier(i + 1) * kInitialConversionFactor);
  }
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(step_size.ComputeStepSize(-10),
              confidence_amplifier(i + 1) * kInitialConversionFactor);
  }
}

TEST(PccDynamicBoundaryTest, InitialBoundary) {
  DynamicBoundary dynamic_boundary{kInitialDynamicBoundary,
                                   kDynamicBoundaryIncrement};
  double change_rate = 1;
  const double kBitrate = 100;
  EXPECT_EQ(dynamic_boundary.Shrink(change_rate, kBitrate), change_rate);

  dynamic_boundary =
      DynamicBoundary{kInitialDynamicBoundary, kDynamicBoundaryIncrement};
  change_rate = 100;
  EXPECT_EQ(dynamic_boundary.Shrink(change_rate, kBitrate),
            kBitrate * kInitialDynamicBoundary);
}

TEST(PccDynamicBoundaryTest, VariousChangeRate) {
  DynamicBoundary dynamic_boundary{kInitialDynamicBoundary,
                                   kDynamicBoundaryIncrement};
  double change_rate = 100;
  const double kBitrate = 1;
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(
        dynamic_boundary.Shrink(change_rate, kBitrate),
        kBitrate * (kInitialDynamicBoundary + i * kDynamicBoundaryIncrement));
  }
  change_rate = kBitrate * kInitialDynamicBoundary;
  EXPECT_EQ(dynamic_boundary.Shrink(change_rate, kBitrate), change_rate);
  change_rate =
      kBitrate * (kInitialDynamicBoundary + 2 * kDynamicBoundaryIncrement);
  EXPECT_EQ(dynamic_boundary.Shrink(change_rate, kBitrate),
            kBitrate * (kInitialDynamicBoundary + kDynamicBoundaryIncrement));

  change_rate = -100;
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(dynamic_boundary.Shrink(change_rate, kBitrate),
              (-1) * kBitrate *
                  (kInitialDynamicBoundary + i * kDynamicBoundaryIncrement));
  }
  change_rate = 100;
  EXPECT_EQ(dynamic_boundary.Shrink(change_rate, kBitrate),
            kBitrate * kInitialDynamicBoundary);
}

TEST(PccBitrateControllerTest, IncreaseRateWhenNoChangesForTestBitrates) {
  BitrateController bitrate_controller(
      kInitialConversionFactor, kInitialDynamicBoundary,
      kDynamicBoundaryIncrement, ConfidenceAmplifier(), kRttGradientCoefficient,
      kLossCoefficient, kThroughputCoefficient, kRttGradientThreshold);
  VivaceUtilityFunction utility_function(
      kRttGradientCoefficient, kLossCoefficient, kThroughputCoefficient,
      kRttGradientThreshold);
  MonitorBlock monitor_block{kStartTime,
                             kIntervalDuration,
                             kTargetSendingRate,
                             kMiTimeout,
                             {kTargetSendingRate * (1 + kEpsilon),
                              kTargetSendingRate * (1 - kEpsilon)}};
  // To start second MI
  monitor_block.NotifyCurrentTime(kStartTime + kIntervalDuration);
  monitor_block.OnPacketsFeedback(
      CreatePacketResults({kStartTime + kPacketsDelta,
                           kStartTime + kIntervalDuration + kPacketsDelta,
                           kStartTime + 3 * kIntervalDuration},
                          {}, {}));
  EXPECT_GT(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .bps(),
      kTargetSendingRate.bps());
}

TEST(PccBitrateControllerTest, NoChangesWhenUtilityFunctionDoesntChange) {
  std::unique_ptr<MockUtilityFunction> mock_utility_function =
      rtc::MakeUnique<MockUtilityFunction>();
  EXPECT_CALL(*mock_utility_function, ComputeUtilityFunction(testing::_))
      .Times(2)
      .WillOnce(testing::Return(100))
      .WillOnce(testing::Return(100));

  BitrateController bitrate_controller(
      kInitialConversionFactor, kInitialDynamicBoundary,
      kDynamicBoundaryIncrement, ConfidenceAmplifier(),
      std::move(mock_utility_function));
  MonitorBlock monitor_block{kStartTime,
                             kIntervalDuration,
                             kTargetSendingRate,
                             kMiTimeout,
                             {kTargetSendingRate * (1 + kEpsilon),
                              kTargetSendingRate * (1 - kEpsilon)}};
  // To start second MI
  monitor_block.NotifyCurrentTime(kStartTime + kIntervalDuration);
  // To complete collecting feedback within monitor Block.
  monitor_block.OnPacketsFeedback(
      CreatePacketResults({kStartTime + 3 * kIntervalDuration}, {}, {}));

  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate),
      kTargetSendingRate);
}

TEST(PccBitrateControllerTest, NoBoundaryWhenSmallGradient) {
  std::unique_ptr<MockUtilityFunction> mock_utility_function =
      rtc::MakeUnique<MockUtilityFunction>();
  constexpr double kFirstMiUtilityFunction = 0;
  const double kSecondMiUtilityFunction =
      2 * kTargetSendingRate.kbps() * kEpsilon;

  EXPECT_CALL(*mock_utility_function, ComputeUtilityFunction(testing::_))
      .Times(2)
      .WillOnce(testing::Return(kFirstMiUtilityFunction))
      .WillOnce(testing::Return(kSecondMiUtilityFunction));

  BitrateController bitrate_controller(
      kInitialConversionFactor, kInitialDynamicBoundary,
      kDynamicBoundaryIncrement, ConfidenceAmplifier(),
      std::move(mock_utility_function));
  MonitorBlock monitor_block{kStartTime,
                             kIntervalDuration,
                             kTargetSendingRate,
                             kMiTimeout,
                             {kTargetSendingRate * (1 + kEpsilon),
                              kTargetSendingRate * (1 - kEpsilon)}};
  // To start second MI
  monitor_block.NotifyCurrentTime(kStartTime + kIntervalDuration);
  // To complete collecting feedback within monitor Block.
  monitor_block.OnPacketsFeedback(
      CreatePacketResults({kStartTime + 3 * kIntervalDuration}, {}, {}));

  double gradient = (kFirstMiUtilityFunction - kSecondMiUtilityFunction) /
                    (kTargetSendingRate.kbps() * 2 * kEpsilon);
  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .kbps(),
      kTargetSendingRate.kbps() + kInitialConversionFactor * gradient);
}

TEST(PccBitrateControllerTest, FaceBoundaryWhenLargeGradient) {
  std::unique_ptr<MockUtilityFunction> mock_utility_function =
      rtc::MakeUnique<MockUtilityFunction>();
  constexpr double kFirstMiUtilityFunction = 0;
  const double kSecondMiUtilityFunction = 10 * kInitialDynamicBoundary *
                                          kTargetSendingRate.bps() * 2 *
                                          kTargetSendingRate.bps() * kEpsilon;

  EXPECT_CALL(*mock_utility_function, ComputeUtilityFunction(testing::_))
      .Times(4)
      .WillOnce(testing::Return(kFirstMiUtilityFunction))
      .WillOnce(testing::Return(kSecondMiUtilityFunction))
      .WillOnce(testing::Return(kFirstMiUtilityFunction))
      .WillOnce(testing::Return(kSecondMiUtilityFunction));

  BitrateController bitrate_controller(
      kInitialConversionFactor, kInitialDynamicBoundary,
      kDynamicBoundaryIncrement, ConfidenceAmplifier(),
      std::move(mock_utility_function));
  MonitorBlock monitor_block{kStartTime,
                             kIntervalDuration,
                             kTargetSendingRate,
                             kMiTimeout,
                             {kTargetSendingRate * (1 + kEpsilon),
                              kTargetSendingRate * (1 - kEpsilon)}};
  // To start second MI
  monitor_block.NotifyCurrentTime(kStartTime + kIntervalDuration);
  // To complete collecting feedback within monitor Block.
  monitor_block.OnPacketsFeedback(
      CreatePacketResults({kStartTime + 3 * kIntervalDuration}, {}, {}));
  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .bps(),
      kTargetSendingRate.bps() * (1 - kInitialDynamicBoundary));
  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .bps(),
      kTargetSendingRate.bps() *
          (1 - kInitialDynamicBoundary - kDynamicBoundaryIncrement));
}

TEST(PccBitrateControllerTest, SlowStartMode) {
  std::unique_ptr<MockUtilityFunction> mock_utility_function =
      rtc::MakeUnique<MockUtilityFunction>();
  constexpr double kFirstUtilityFunction = 1000;
  EXPECT_CALL(*mock_utility_function, ComputeUtilityFunction(testing::_))
      .Times(4)
      .WillOnce(testing::Return(kFirstUtilityFunction))
      .WillOnce(testing::Return(kFirstUtilityFunction + 1))
      .WillOnce(testing::Return(kFirstUtilityFunction + 2))
      .WillOnce(testing::Return(kFirstUtilityFunction + 1));

  BitrateController bitrate_controller(
      kInitialConversionFactor, kInitialDynamicBoundary,
      kDynamicBoundaryIncrement, ConfidenceAmplifier(),
      std::move(mock_utility_function));
  MonitorBlock monitor_block{kStartTime,
                             kIntervalDuration,
                             kTargetSendingRate,
                             kMiTimeout,
                             {kTargetSendingRate}};
  // To complete collecting feedback within monitor Block.
  monitor_block.OnPacketsFeedback(
      CreatePacketResults({kStartTime + 3 * kIntervalDuration}, {}, {}));

  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .bps(),
      kTargetSendingRate.bps() * 2);
  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .bps(),
      kTargetSendingRate.bps() * 2);
  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .bps(),
      kTargetSendingRate.bps() * 2);
  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .bps(),
      kTargetSendingRate.bps());
}

TEST(PccBitrateControllerTest, StepSizeIncrease) {
  std::unique_ptr<MockUtilityFunction> mock_utility_function =
      rtc::MakeUnique<MockUtilityFunction>();
  constexpr double kFirstMiUtilityFunction = 0;
  const double kSecondMiUtilityFunction =
      2 * kTargetSendingRate.kbps() * kEpsilon;

  EXPECT_CALL(*mock_utility_function, ComputeUtilityFunction(testing::_))
      .Times(4)
      .WillOnce(testing::Return(kFirstMiUtilityFunction))
      .WillOnce(testing::Return(kSecondMiUtilityFunction))
      .WillOnce(testing::Return(kFirstMiUtilityFunction))
      .WillOnce(testing::Return(kSecondMiUtilityFunction));
  MonitorBlock monitor_block{kStartTime,
                             kIntervalDuration,
                             kTargetSendingRate,
                             kMiTimeout,
                             {kTargetSendingRate * (1 + kEpsilon),
                              kTargetSendingRate * (1 - kEpsilon)}};
  // To start second MI
  monitor_block.NotifyCurrentTime(kStartTime + kIntervalDuration);
  // To complete collecting feedback within monitor Block.
  monitor_block.OnPacketsFeedback(
      CreatePacketResults({kStartTime + 3 * kIntervalDuration}, {}, {}));

  double gradient = (kFirstMiUtilityFunction - kSecondMiUtilityFunction) /
                    (kTargetSendingRate.kbps() * 2 * kEpsilon);
  BitrateController bitrate_controller(
      kInitialConversionFactor, kInitialDynamicBoundary,
      kDynamicBoundaryIncrement, ConfidenceAmplifier(),
      std::move(mock_utility_function));
  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .kbps(),
      kTargetSendingRate.kbps() +
          ConfidenceAmplifier()(1) * kInitialConversionFactor * gradient);
  EXPECT_EQ(
      bitrate_controller.ComputeRateUpdate(monitor_block, kTargetSendingRate)
          .kbps(),
      kTargetSendingRate.kbps() +
          ConfidenceAmplifier()(2) * kInitialConversionFactor * gradient);
}

}  // namespace test
}  // namespace pcc
}  // namespace webrtc
