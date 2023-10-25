/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/source/absolute_capture_time_sender.h"

#include "system_wrappers/include/ntp_time.h"
#include "test/gmock.h"
#include "test/gtest.h"

namespace webrtc {

TEST(AbsoluteCaptureTimeSenderTest, GetSourceWithoutCsrcs) {
  constexpr uint32_t kSsrc = 12;

  EXPECT_EQ(AbsoluteCaptureTimeSender::DeprecatedGetSource(kSsrc, {}), kSsrc);
}

TEST(AbsoluteCaptureTimeSenderTest, GetSourceWithCsrcs) {
  constexpr uint32_t kSsrc = 12;
  constexpr uint32_t kCsrcs[] = {34, 56, 78, 90};

  EXPECT_EQ(AbsoluteCaptureTimeSender::DeprecatedGetSource(kSsrc, kCsrcs),
            kCsrcs[0]);
}

TEST(AbsoluteCaptureTimeSenderTest, InterpolateLaterPacketSentLater) {
  constexpr int kRtpClockFrequency = 64'000;
  constexpr uint32_t kRtpTimestamp0 = 1020300000;
  constexpr uint32_t kRtpTimestamp1 = kRtpTimestamp0 + 1280;
  constexpr uint32_t kRtpTimestamp2 = kRtpTimestamp0 + 2560;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {Int64MsToUQ32x32(9000 + 20),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension2 = {Int64MsToUQ32x32(9000 + 40),
                                          Int64MsToQ32x32(-350)};
  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset),
            absl::nullopt);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset),
            absl::nullopt);
}

TEST(AbsoluteCaptureTimeSenderTest, InterpolateEarlierPacketSentLater) {
  constexpr int kRtpClockFrequency = 64'000;
  constexpr uint32_t kRtpTimestamp0 = 1020300000;
  constexpr uint32_t kRtpTimestamp1 = kRtpTimestamp0 - 1280;
  constexpr uint32_t kRtpTimestamp2 = kRtpTimestamp0 - 2560;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {Int64MsToUQ32x32(9000 - 20),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension2 = {Int64MsToUQ32x32(9000 - 40),
                                          Int64MsToQ32x32(-350)};

  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset),
            absl::nullopt);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset),
            absl::nullopt);
}

TEST(AbsoluteCaptureTimeSenderTest,
     InterpolateLaterPacketSentLaterWithRtpTimestampWrapAround) {
  constexpr int kRtpClockFrequency = 64'000;
  constexpr uint32_t kRtpTimestamp0 = uint32_t{0} - 80;
  constexpr uint32_t kRtpTimestamp1 = 1280 - 80;
  constexpr uint32_t kRtpTimestamp2 = 2560 - 80;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {Int64MsToUQ32x32(9000 + 20),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension2 = {Int64MsToUQ32x32(9000 + 40),
                                          Int64MsToQ32x32(-350)};

  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset),
            absl::nullopt);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset),
            absl::nullopt);
}

TEST(AbsoluteCaptureTimeSenderTest,
     InterpolateEarlierPacketSentLaterWithRtpTimestampWrapAround) {
  constexpr int kRtpClockFrequency = 64'000;
  constexpr uint32_t kRtpTimestamp0 = 799;
  constexpr uint32_t kRtpTimestamp1 = kRtpTimestamp0 - 1280;
  constexpr uint32_t kRtpTimestamp2 = kRtpTimestamp0 - 2560;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {Int64MsToUQ32x32(9000 - 20),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension2 = {Int64MsToUQ32x32(9000 - 40),
                                          Int64MsToQ32x32(-350)};

  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset),
            absl::nullopt);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset),
            absl::nullopt);
}

TEST(AbsoluteCaptureTimeSenderTest, SkipInterpolateIfTooLate) {
  constexpr int kRtpClockFrequency = 64'000;
  constexpr uint32_t kRtpTimestamp0 = 1020300000;
  constexpr uint32_t kRtpTimestamp1 = kRtpTimestamp0 + 1280;
  constexpr uint32_t kRtpTimestamp2 = kRtpTimestamp0 + 2560;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {Int64MsToUQ32x32(9000 + 20),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension2 = {Int64MsToUQ32x32(9000 + 40),
                                          Int64MsToQ32x32(-350)};

  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  clock.AdvanceTime(AbsoluteCaptureTimeSender::kInterpolationMaxInterval);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset),
            absl::nullopt);

  clock.AdvanceTime(TimeDelta::Millis(1));

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset),
            extension2);
}

TEST(AbsoluteCaptureTimeSenderTest, SkipInterpolateWhenForced) {
  constexpr int kRtpClockFrequency = 64'000;
  constexpr uint32_t kRtpTimestamp0 = 1020300000;
  constexpr uint32_t kRtpTimestamp1 = kRtpTimestamp0 + 1280;
  constexpr uint32_t kRtpTimestamp2 = kRtpTimestamp0 + 2560;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {Int64MsToUQ32x32(9000 + 20),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension2 = {Int64MsToUQ32x32(9000 + 40),
                                          Int64MsToQ32x32(-350)};

  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset,
                                /*force=*/true),
            extension1);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset,
                                /*force=*/false),
            absl::nullopt);
}

TEST(AbsoluteCaptureTimeSenderTest, SkipInterpolateIfRtpClockFrequencyChanged) {
  constexpr int kRtpClockFrequency0 = 64'000;
  constexpr int kRtpClockFrequency1 = 32'000;
  constexpr int kRtpClockFrequency2 = 32'000;
  constexpr uint32_t kRtpTimestamp0 = 1020300000;
  constexpr uint32_t kRtpTimestamp1 = kRtpTimestamp0 + 640;
  constexpr uint32_t kRtpTimestamp2 = kRtpTimestamp0 + 1280;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {Int64MsToUQ32x32(9000 + 20),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension2 = {Int64MsToUQ32x32(9000 + 40),
                                          Int64MsToQ32x32(-350)};

  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency0,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency1,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset),
            extension1);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency2,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset),
            absl::nullopt);
}

TEST(AbsoluteCaptureTimeSenderTest,
     SkipInterpolateIfRtpClockFrequencyIsInvalid) {
  constexpr int kRtpClockFrequency0 = 0;
  constexpr int kRtpClockFrequency1 = 0;
  constexpr int kRtpClockFrequency2 = 0;
  constexpr uint32_t kRtpTimestamp0 = 1020300000;
  constexpr uint32_t kRtpTimestamp1 = kRtpTimestamp0;
  constexpr uint32_t kRtpTimestamp2 = kRtpTimestamp0;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {Int64MsToUQ32x32(9000 + 20),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension2 = {Int64MsToUQ32x32(9000 + 40),
                                          Int64MsToQ32x32(-350)};

  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency0,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency1,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset),
            extension1);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency2,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset),
            extension2);
}

TEST(AbsoluteCaptureTimeSenderTest,
     SkipInterpolateIfEstimatedCaptureClockOffsetChanged) {
  constexpr int kRtpClockFrequency = 64'000;
  constexpr uint32_t kRtpTimestamp0 = 1020300000;
  constexpr uint32_t kRtpTimestamp1 = kRtpTimestamp0 + 1280;
  constexpr uint32_t kRtpTimestamp2 = kRtpTimestamp0 + 2560;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {Int64MsToUQ32x32(9000 + 20),
                                          Int64MsToQ32x32(370)};
  const AbsoluteCaptureTime extension2 = {Int64MsToUQ32x32(9000 + 40),
                                          absl::nullopt};

  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset),
            extension1);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset),
            extension2);
}

TEST(AbsoluteCaptureTimeSenderTest,
     SkipInterpolateIfTooMuchInterpolationError) {
  constexpr int kRtpClockFrequency = 64'000;
  constexpr uint32_t kRtpTimestamp0 = 1020300000;
  constexpr uint32_t kRtpTimestamp1 = kRtpTimestamp0 + 1280;
  constexpr uint32_t kRtpTimestamp2 = kRtpTimestamp0 + 2560;
  const AbsoluteCaptureTime extension0 = {Int64MsToUQ32x32(9000),
                                          Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension1 = {
      Int64MsToUQ32x32(9000 + 20 +
                       AbsoluteCaptureTimeSender::kInterpolationMaxError.ms()),
      Int64MsToQ32x32(-350)};
  const AbsoluteCaptureTime extension2 = {
      Int64MsToUQ32x32(9000 + 40 +
                       AbsoluteCaptureTimeSender::kInterpolationMaxError.ms() +
                       1),
      Int64MsToQ32x32(-350)};

  SimulatedClock clock(0);
  AbsoluteCaptureTimeSender sender(&clock);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp0, kRtpClockFrequency,
                                NtpTime(extension0.absolute_capture_timestamp),
                                extension0.estimated_capture_clock_offset),
            extension0);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp1, kRtpClockFrequency,
                                NtpTime(extension1.absolute_capture_timestamp),
                                extension1.estimated_capture_clock_offset),
            absl::nullopt);

  EXPECT_EQ(sender.OnSendPacket(kRtpTimestamp2, kRtpClockFrequency,
                                NtpTime(extension2.absolute_capture_timestamp),
                                extension2.estimated_capture_clock_offset),
            extension2);
}

}  // namespace webrtc
