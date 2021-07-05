/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/source/rtp_util.h"

#include "test/gmock.h"

namespace webrtc {
namespace {

TEST(RtpUtil, IsRtpPacket) {
  constexpr uint8_t kMinimalisticRtpPacket[] = {0x80, 97, 0, 0,  //
                                                0,    0,  0, 0,  //
                                                0,    0,  0, 0};
  EXPECT_TRUE(IsRtpPacket(kMinimalisticRtpPacket));

  constexpr uint8_t kWrongRtpVersion[] = {0xc0, 97, 0, 0,  //
                                          0,    0,  0, 0,  //
                                          0,    0,  0, 0};
  EXPECT_FALSE(IsRtpPacket(kWrongRtpVersion));

  constexpr uint8_t kPacketWithPayloadForRtcp[] = {0x80, 200, 0, 0,  //
                                                   0,    0,   0, 0,  //
                                                   0,    0,   0, 0};
  EXPECT_FALSE(IsRtpPacket(kPacketWithPayloadForRtcp));

  constexpr uint8_t kTooSmallRtpPacket[] = {0x80, 97, 0, 0,  //
                                            0,    0,  0, 0,  //
                                            0,    0,  0};
  EXPECT_FALSE(IsRtpPacket(kTooSmallRtpPacket));

  EXPECT_FALSE(IsRtpPacket({}));
}

TEST(RtpUtil, IsRtcpPacket) {
  constexpr uint8_t kMinimalisticRtcpPacket[] = {0x80, 201, 0, 0,  //
                                                 0,    0,   0, 0};
  EXPECT_TRUE(IsRtcpPacket(kMinimalisticRtcpPacket));

  constexpr uint8_t kWrongRtpVersion[] = {0xc0, 201, 0, 0,  //
                                          0,    0,   0, 0};
  EXPECT_FALSE(IsRtcpPacket(kWrongRtpVersion));

  constexpr uint8_t kPacketWithPayloadForRtp[] = {0x80, 225, 0, 0,  //
                                                  0,    0,   0, 0};
  EXPECT_FALSE(IsRtcpPacket(kPacketWithPayloadForRtp));

  constexpr uint8_t kTooSmallRtcpPacket[] = {0x80, 201, 0, 0,  //
                                             0,    0,   0};
  EXPECT_FALSE(IsRtcpPacket(kTooSmallRtcpPacket));

  EXPECT_FALSE(IsRtcpPacket({}));
}

}  // namespace
}  // namespace webrtc
