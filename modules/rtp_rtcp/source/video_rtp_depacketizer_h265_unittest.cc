/*
 *  Copyright (c) 2023 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/source/video_rtp_depacketizer_h265.h"

#include <cstdint>
#include <vector>

#include "absl/types/optional.h"
#include "api/array_view.h"
#include "common_video/h265/h265_common.h"
#include "modules/rtp_rtcp/mocks/mock_rtp_rtcp.h"
#include "modules/rtp_rtcp/source/byte_io.h"
#include "rtc_base/copy_on_write_buffer.h"
#include "test/gmock.h"
#include "test/gtest.h"

namespace webrtc {
namespace {

using ::testing::Each;
using ::testing::ElementsAre;
using ::testing::ElementsAreArray;
using ::testing::Eq;
using ::testing::IsEmpty;
using ::testing::SizeIs;

/*constexpr size_t kNalHeaderSize = 2;
constexpr size_t kFuAHeaderSize = 3;

constexpr uint8_t kNaluTypeMask = 0x7E;*/

// Bit masks for FU headers.
constexpr uint8_t kH265SBit = 0x80;
constexpr uint8_t kH265EBit = 0x40;

TEST(VideoRtpDepacketizerH265Test, SingleNalu) {
  uint8_t packet[2] = {0x26,
                       0x02};  // F=0, Type=19 (kIdrWRadl), LayerId=0, TID=2.
  rtc::CopyOnWriteBuffer rtp_payload(packet);

  VideoRtpDepacketizerH265 depacketizer;
  absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed =
      depacketizer.Parse(rtp_payload);
  ASSERT_TRUE(parsed);

  EXPECT_EQ(parsed->video_payload, rtp_payload);
  EXPECT_EQ(parsed->video_header.frame_type, VideoFrameType::kVideoFrameKey);
  EXPECT_EQ(parsed->video_header.codec, kVideoCodecH265);
  EXPECT_TRUE(parsed->video_header.is_first_packet_in_frame);
  const RTPVideoHeaderH265& H265 =
      absl::get<RTPVideoHeaderH265>(parsed->video_header.video_type_header);
  EXPECT_EQ(H265.nalu_type, H265::kIdrWRadl);
}

TEST(VideoRtpDepacketizerH265Test, SingleNaluSpsWithResolution) {
  uint8_t packet[] = {0x42, 0x02, 0x01, 0x04, 0x08, 0x00, 0x00, 0x03,
                      0x00, 0x9d, 0x08, 0x00, 0x00, 0x03, 0x00, 0x00,
                      0x5d, 0xb0, 0x02, 0x80, 0x80, 0x2d, 0x16, 0x59,
                      0x59, 0xa4, 0x93, 0x2b, 0x80, 0x40, 0x00, 0x00,
                      0x03, 0x00, 0x40, 0x00, 0x00, 0x07, 0x82};
  rtc::CopyOnWriteBuffer rtp_payload(packet);

  VideoRtpDepacketizerH265 depacketizer;
  absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed =
      depacketizer.Parse(rtp_payload);
  ASSERT_TRUE(parsed);

  EXPECT_EQ(parsed->video_payload, rtp_payload);
  EXPECT_EQ(parsed->video_header.frame_type, VideoFrameType::kVideoFrameKey);
  EXPECT_EQ(parsed->video_header.codec, kVideoCodecH265);
  EXPECT_TRUE(parsed->video_header.is_first_packet_in_frame);
  EXPECT_EQ(parsed->video_header.width, 1280u);
  EXPECT_EQ(parsed->video_header.height, 720u);
}

TEST(VideoRtpDepacketizerH265Test, PACIPackets) {
  uint8_t packet[2] = {0x64, 0x02};  // F=0, Type=50 (kPACI), LayerId=0, TID=2.
  rtc::CopyOnWriteBuffer rtp_payload(packet);

  VideoRtpDepacketizerH265 depacketizer;
  absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed =
      depacketizer.Parse(rtp_payload);
  ASSERT_FALSE(parsed);
}

TEST(VideoRtpDepacketizerH265Test, APKey) {
  // clang-format off
  const H265NaluInfo kExpectedNalus[] = {
                                      // kH265Vps
                                      {32, 1, -1, -1},
                                      // kH265Sps
                                      {33, 0, 0, -1},
                                      // kH265Pps
                                      {34, -1, 1, 0},
                                      // kIdrWRadl
                                      {19, -1, -1, 0}};
  uint8_t packet[] = {
                      // F=0, Type=48.
                      0x60, 0x02,
                      // Length, nal header, payload.
                      // vps
                      0, 0x17, 0x40, 0x02,
                        0x1c, 0x01, 0xff, 0xff, 0x04, 0x08, 0x00, 0x00,
                        0x03, 0x00, 0x9d, 0x08, 0x00, 0x00, 0x03, 0x00,
                        0x00, 0x78, 0x95, 0x98, 0x09,
                      // sps
                      0, 0x27, 0x42, 0x02,
                        0x01, 0x04, 0x08, 0x00, 0x00, 0x03, 0x00, 0x9d,
                        0x08, 0x00, 0x00, 0x03, 0x00, 0x00, 0x5d, 0xb0,
                        0x02, 0x80, 0x80, 0x2d, 0x16, 0x59, 0x59, 0xa4,
                        0x93, 0x2b, 0x80, 0x40, 0x00, 0x00, 0x03, 0x00,
                        0x40, 0x00, 0x00, 0x07, 0x82,
                      // pps
                      0, 0x32, 0x44, 0x02,
                        0xa4, 0x04, 0x55, 0xa2, 0x6d, 0xce, 0xc0, 0xc3,
                        0xed, 0x0b, 0xac, 0xbc, 0x00, 0xc4, 0x44, 0x2e,
                        0xf7, 0x55, 0xfd, 0x05, 0x86, 0x92, 0x19, 0xdf,
                        0x58, 0xec, 0x38, 0x36, 0xb7, 0x7c, 0x00, 0x15,
                        0x33, 0x78, 0x03, 0x67, 0x26, 0x0f, 0x7b, 0x30,
                        0x1c, 0xd7, 0xd4, 0x3a, 0xec, 0xad, 0xef, 0x73,
                      // Idr
                      0, 0xa, 0x26, 0x02,
                        0xaf, 0x08, 0x4a, 0x31, 0x11, 0x15, 0xe5, 0xc0};
  // clang-format on
  rtc::CopyOnWriteBuffer rtp_payload(packet);

  VideoRtpDepacketizerH265 depacketizer;
  absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed =
      depacketizer.Parse(rtp_payload);
  ASSERT_TRUE(parsed);

  EXPECT_EQ(parsed->video_payload, rtp_payload);
  EXPECT_EQ(parsed->video_header.frame_type, VideoFrameType::kVideoFrameKey);
  EXPECT_EQ(parsed->video_header.codec, kVideoCodecH265);
  EXPECT_TRUE(parsed->video_header.is_first_packet_in_frame);
  const auto& H265 =
      absl::get<RTPVideoHeaderH265>(parsed->video_header.video_type_header);
  // NALU type for aggregated packets is the type of the first packet only.
  EXPECT_EQ(H265.nalu_type, H265::kVps);
  ASSERT_EQ(H265.nalus_length, 4u);
  for (size_t i = 0; i < H265.nalus_length; ++i) {
    EXPECT_EQ(H265.nalus[i].type, kExpectedNalus[i].type)
        << "Failed parsing nalu " << i;
    EXPECT_EQ(H265.nalus[i].vps_id, kExpectedNalus[i].vps_id)
        << "Failed parsing nalu " << i;
    EXPECT_EQ(H265.nalus[i].sps_id, kExpectedNalus[i].sps_id)
        << "Failed parsing nalu " << i;
    EXPECT_EQ(H265.nalus[i].pps_id, kExpectedNalus[i].pps_id)
        << "Failed parsing nalu " << i;
  }
}

TEST(VideoRtpDepacketizerH265Test, APNaluSpsWithResolution) {
  uint8_t packet[] = {
      0x60, 0x02,           // F=0, Type=48.
                            // Length, nal header, payload.
      0, 0x17, 0x40, 0x02,  // vps
      0x1c, 0x01, 0xff, 0xff, 0x04, 0x08, 0x00, 0x00, 0x03, 0x00, 0x9d, 0x08,
      0x00, 0x00, 0x03, 0x00, 0x00, 0x78, 0x95, 0x98, 0x09, 0, 0x27, 0x42,
      0x02,  // sps
      0x01, 0x04, 0x08, 0x00, 0x00, 0x03, 0x00, 0x9d, 0x08, 0x00, 0x00, 0x03,
      0x00, 0x00, 0x5d, 0xb0, 0x02, 0x80, 0x80, 0x2d, 0x16, 0x59, 0x59, 0xa4,
      0x93, 0x2b, 0x80, 0x40, 0x00, 0x00, 0x03, 0x00, 0x40, 0x00, 0x00, 0x07,
      0x82, 0, 0x32, 0x44, 0x02,  // pps
      0xa4, 0x04, 0x55, 0xa2, 0x6d, 0xce, 0xc0, 0xc3, 0xed, 0x0b, 0xac, 0xbc,
      0x00, 0xc4, 0x44, 0x2e, 0xf7, 0x55, 0xfd, 0x05, 0x86, 0x92, 0x19, 0xdf,
      0x58, 0xec, 0x38, 0x36, 0xb7, 0x7c, 0x00, 0x15, 0x33, 0x78, 0x03, 0x67,
      0x26, 0x0f, 0x7b, 0x30, 0x1c, 0xd7, 0xd4, 0x3a, 0xec, 0xad, 0xef, 0x73, 0,
      0xa, 0x26, 0x02,  // kIdrWRadl
      0xaf, 0x08, 0x4a, 0x31, 0x11, 0x15, 0xe5, 0xc0};
  rtc::CopyOnWriteBuffer rtp_payload(packet);

  VideoRtpDepacketizerH265 depacketizer;
  absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed =
      depacketizer.Parse(rtp_payload);
  ASSERT_TRUE(parsed);

  EXPECT_EQ(parsed->video_payload, rtp_payload);
  EXPECT_EQ(parsed->video_header.frame_type, VideoFrameType::kVideoFrameKey);
  EXPECT_EQ(parsed->video_header.codec, kVideoCodecH265);
  EXPECT_TRUE(parsed->video_header.is_first_packet_in_frame);
  EXPECT_EQ(parsed->video_header.width, 1280u);
  EXPECT_EQ(parsed->video_header.height, 720u);
}

TEST(VideoRtpDepacketizerH265Test, EmptyAPRejected) {
  uint8_t lone_empty_packet[] = {0x60, 0x02,  // F=0, Type=48 (kH265Ap).
                                 0x00, 0x00};
  uint8_t leading_empty_packet[] = {0x60, 0x02,  // F=0, Type=48 (kH265Ap).
                                    0x00, 0x00, 0x00, 0x05, 0x26,
                                    0x02, 0xFF, 0x00, 0x11};  // kIdrWRadl
  uint8_t middle_empty_packet[] = {0x60, 0x02,  // F=0, Type=48 (kH265Ap).
                                   0x00, 0x04, 0x26, 0x02, 0xFF,
                                   0x00, 0x00, 0x00, 0x00, 0x05,
                                   0x26, 0x02, 0xFF, 0x00, 0x11};  // kIdrWRadl
  uint8_t trailing_empty_packet[] = {0x60, 0x02,  // F=0, Type=48 (kH265Ap).
                                     0x00, 0x04, 0x26,
                                     0x02, 0xFF, 0x00,  // kIdrWRadl
                                     0x00, 0x00};

  VideoRtpDepacketizerH265 depacketizer;
  EXPECT_FALSE(depacketizer.Parse(rtc::CopyOnWriteBuffer(lone_empty_packet)));
  EXPECT_FALSE(
      depacketizer.Parse(rtc::CopyOnWriteBuffer(leading_empty_packet)));
  EXPECT_FALSE(depacketizer.Parse(rtc::CopyOnWriteBuffer(middle_empty_packet)));
  EXPECT_FALSE(
      depacketizer.Parse(rtc::CopyOnWriteBuffer(trailing_empty_packet)));
}

TEST(VideoRtpDepacketizerH265Test, APDelta) {
  uint8_t packet[20] = {0x60, 0x02,  // F=0, Type=48 (kH265Ap).
                                     // Length, nal header, payload.
                        0, 0x03, 0x02, 0x02, 0xFF,               // TrailR
                        0, 0x04, 0x02, 0x02, 0xFF, 0x00,         // TrailR
                        0, 0x05, 0x02, 0x02, 0xFF, 0x00, 0x11};  // TrailR
  rtc::CopyOnWriteBuffer rtp_payload(packet);

  VideoRtpDepacketizerH265 depacketizer;
  absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed =
      depacketizer.Parse(rtp_payload);
  ASSERT_TRUE(parsed);

  EXPECT_EQ(parsed->video_payload.size(), rtp_payload.size());
  EXPECT_EQ(parsed->video_payload.cdata(), rtp_payload.cdata());

  EXPECT_EQ(parsed->video_header.frame_type, VideoFrameType::kVideoFrameDelta);
  EXPECT_EQ(parsed->video_header.codec, kVideoCodecH265);
  EXPECT_TRUE(parsed->video_header.is_first_packet_in_frame);
  const RTPVideoHeaderH265& H265 =
      absl::get<RTPVideoHeaderH265>(parsed->video_header.video_type_header);
  // NALU type for aggregated packets is the type of the first packet only.
  EXPECT_EQ(H265.nalu_type, H265::kTrailR);
}

TEST(VideoRtpDepacketizerH265Test, FuA) {
  // clang-format off
  uint8_t packet1[] = {
      0x62, 0x02,  // F=0, Type=49 (kH265Fu).
      kH265SBit | H265::kIdrWRadl,  // FU header.
      0xaf, 0x08, 0x4a, 0x31, 0x11, 0x15, 0xe5, 0xc0  // Payload.
  };
  // clang-format on
  // F=0, Type=19, (kIdrWRadl), tid=1, nalu header: 00100110 00000010, which is
  // 0x26, 0x02
  const uint8_t kExpected1[] = {0x26, 0x02, 0xaf, 0x08, 0x4a,
                                0x31, 0x11, 0x15, 0xe5, 0xc0};

  uint8_t packet2[] = {
      0x62, 0x02,       // F=0, Type=49 (kH265Fu).
      H265::kIdrWRadl,  // FU header.
      0x02              // Payload.
  };
  const uint8_t kExpected2[] = {0x02};

  uint8_t packet3[] = {
      0x62, 0x02,                   // F=0, Type=49 (kH265Fu).
      kH265EBit | H265::kIdrWRadl,  // FU header.
      0x03                          // Payload.
  };
  const uint8_t kExpected3[] = {0x03};

  VideoRtpDepacketizerH265 depacketizer;
  absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed1 =
      depacketizer.Parse(rtc::CopyOnWriteBuffer(packet1));
  ASSERT_TRUE(parsed1);
  // We expect that the first packet is one byte shorter since the FU-A header
  // has been replaced by the original nal header.
  EXPECT_THAT(rtc::MakeArrayView(parsed1->video_payload.cdata(),
                                 parsed1->video_payload.size()),
              ElementsAreArray(kExpected1));
  EXPECT_EQ(parsed1->video_header.frame_type, VideoFrameType::kVideoFrameKey);
  EXPECT_EQ(parsed1->video_header.codec, kVideoCodecH265);
  EXPECT_TRUE(parsed1->video_header.is_first_packet_in_frame);
  {
    const RTPVideoHeaderH265& H265 =
        absl::get<RTPVideoHeaderH265>(parsed1->video_header.video_type_header);
    EXPECT_EQ(H265.nalu_type, H265::kIdrWRadl);
    ASSERT_EQ(H265.nalus_length, 1u);
    EXPECT_EQ(H265.nalus[0].type, static_cast<H265::NaluType>(H265::kIdrWRadl));
    EXPECT_EQ(H265.nalus[0].sps_id, -1);
    EXPECT_EQ(H265.nalus[0].pps_id, 0);
  }

  // Following packets will be 2 bytes shorter since they will only be appended
  // onto the first packet.
  auto parsed2 = depacketizer.Parse(rtc::CopyOnWriteBuffer(packet2));
  EXPECT_THAT(rtc::MakeArrayView(parsed2->video_payload.cdata(),
                                 parsed2->video_payload.size()),
              ElementsAreArray(kExpected2));
  EXPECT_FALSE(parsed2->video_header.is_first_packet_in_frame);
  EXPECT_EQ(parsed2->video_header.codec, kVideoCodecH265);
  {
    const RTPVideoHeaderH265& H265 =
        absl::get<RTPVideoHeaderH265>(parsed2->video_header.video_type_header);
    EXPECT_EQ(H265.nalu_type, H265::kIdrWRadl);
    // NALU info is only expected for the first FU-A packet.
    EXPECT_EQ(H265.nalus_length, 0u);
  }

  auto parsed3 = depacketizer.Parse(rtc::CopyOnWriteBuffer(packet3));
  EXPECT_THAT(rtc::MakeArrayView(parsed3->video_payload.cdata(),
                                 parsed3->video_payload.size()),
              ElementsAreArray(kExpected3));
  EXPECT_FALSE(parsed3->video_header.is_first_packet_in_frame);
  EXPECT_EQ(parsed3->video_header.codec, kVideoCodecH265);
  {
    const RTPVideoHeaderH265& H265 =
        absl::get<RTPVideoHeaderH265>(parsed3->video_header.video_type_header);
    EXPECT_EQ(H265.nalu_type, H265::kIdrWRadl);
    // NALU info is only expected for the first FU-A packet.
    ASSERT_EQ(H265.nalus_length, 0u);
  }
}

TEST(VideoRtpDepacketizerH265Test, EmptyPayload) {
  rtc::CopyOnWriteBuffer empty;
  VideoRtpDepacketizerH265 depacketizer;
  EXPECT_FALSE(depacketizer.Parse(empty));
}

TEST(VideoRtpDepacketizerH265Test, TruncatedFuaNalu) {
  const uint8_t kPayload[] = {0x62};
  VideoRtpDepacketizerH265 depacketizer;
  EXPECT_FALSE(depacketizer.Parse(rtc::CopyOnWriteBuffer(kPayload)));
}

TEST(VideoRtpDepacketizerH265Test, TruncatedSingleAPNalu) {
  const uint8_t kPayload[] = {0xe0, 0x02, 0x40};
  VideoRtpDepacketizerH265 depacketizer;
  EXPECT_FALSE(depacketizer.Parse(rtc::CopyOnWriteBuffer(kPayload)));
}

TEST(VideoRtpDepacketizerH265Test, APPacketWithTruncatedNalUnits) {
  const uint8_t kPayload[] = {0x60, 0x02, 0xED, 0xDF};
  VideoRtpDepacketizerH265 depacketizer;
  EXPECT_FALSE(depacketizer.Parse(rtc::CopyOnWriteBuffer(kPayload)));
}

TEST(VideoRtpDepacketizerH265Test, TruncationJustAfterSingleAPNalu) {
  const uint8_t kPayload[] = {0x60, 0x02, 0x40, 0x40};
  VideoRtpDepacketizerH265 depacketizer;
  EXPECT_FALSE(depacketizer.Parse(rtc::CopyOnWriteBuffer(kPayload)));
}

TEST(VideoRtpDepacketizerH265Test, ShortSpsPacket) {
  const uint8_t kPayload[] = {0x40, 0x80, 0x00};
  VideoRtpDepacketizerH265 depacketizer;
  EXPECT_TRUE(depacketizer.Parse(rtc::CopyOnWriteBuffer(kPayload)));
}

TEST(VideoRtpDepacketizerH265Test, SeiPacket) {
  const uint8_t kPayload[] = {
      0x4e, 0x02,             // F=0, Type=39 (kPrefixSei).
      0x03, 0x03, 0x03, 0x03  // Payload.
  };
  VideoRtpDepacketizerH265 depacketizer;
  auto parsed = depacketizer.Parse(rtc::CopyOnWriteBuffer(kPayload));
  ASSERT_TRUE(parsed);
  const RTPVideoHeaderH265& H265 =
      absl::get<RTPVideoHeaderH265>(parsed->video_header.video_type_header);
  EXPECT_EQ(parsed->video_header.frame_type, VideoFrameType::kVideoFrameDelta);
  EXPECT_EQ(H265.nalu_type, H265::kPrefixSei);
  ASSERT_EQ(H265.nalus_length, 1u);
  EXPECT_EQ(H265.nalus[0].type, static_cast<H265::NaluType>(H265::kPrefixSei));
  EXPECT_EQ(H265.nalus[0].vps_id, -1);
  EXPECT_EQ(H265.nalus[0].sps_id, -1);
  EXPECT_EQ(H265.nalus[0].pps_id, -1);
}

}  // namespace
}  // namespace webrtc
