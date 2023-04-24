/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/source/rtp_video_stream_receiver_frame_transformer_delegate.h"

#include <cstdio>
#include <memory>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "api/call/transport.h"
#include "api/test/mock_transformable_video_frame.h"
#include "api/units/timestamp.h"
#include "call/video_receive_stream.h"
#include "modules/rtp_rtcp/source/rtp_descriptor_authentication.h"
#include "rtc_base/event.h"
#include "test/gmock.h"
#include "test/gtest.h"
#include "test/mock_frame_transformer.h"

namespace webrtc {
namespace {

using ::testing::_;
using ::testing::ElementsAre;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::SaveArg;

std::unique_ptr<RtpFrameObject> CreateRtpFrameObject(
    const RTPVideoHeader& video_header,
    std::vector<uint32_t> csrcs) {
  RtpPacketInfo packet_info(/*ssrc=*/123, csrcs, /*rtc_timestamp=*/0,
                            /*receive_time=*/Timestamp::Seconds(123456));
  return std::make_unique<RtpFrameObject>(
      /*first_seq_num=*/1, /*last_seq_num=*/2, /*markerBit=*/true,
      /*times_nacked=*/3, /*first_packet_received_time=*/4,
      /*last_packet_received_time=*/5, /*rtp_timestamp=*/6, /*ntp_time_ms=*/7,
      VideoSendTiming(), /*payload_type=*/8, video_header.codec,
      kVideoRotation_0, VideoContentType::UNSPECIFIED, video_header,
      absl::nullopt, RtpPacketInfos({packet_info}),
      EncodedImageBuffer::Create(0));
}

std::unique_ptr<RtpFrameObject> CreateRtpFrameObject() {
  return CreateRtpFrameObject(RTPVideoHeader(), /*csrcs=*/{});
}

class TestRtpVideoFrameReceiver : public RtpVideoFrameReceiver {
 public:
  TestRtpVideoFrameReceiver() {}
  ~TestRtpVideoFrameReceiver() override = default;

  MOCK_METHOD(void,
              ManageFrame,
              (std::unique_ptr<RtpFrameObject> frame),
              (override));
};

TEST(RtpVideoStreamReceiverFrameTransformerDelegateTest,
     RegisterTransformedFrameCallbackSinkOnInit) {
  TestRtpVideoFrameReceiver receiver;
  auto frame_transformer(rtc::make_ref_counted<MockFrameTransformer>());
  auto delegate(
      rtc::make_ref_counted<RtpVideoStreamReceiverFrameTransformerDelegate>(
          &receiver, frame_transformer, rtc::Thread::Current(),
          /*remote_ssrc*/ 1111));
  EXPECT_CALL(*frame_transformer,
              RegisterTransformedFrameSinkCallback(testing::_, 1111));
  delegate->Init();
}

TEST(RtpVideoStreamReceiverFrameTransformerDelegateTest,
     UnregisterTransformedFrameSinkCallbackOnReset) {
  TestRtpVideoFrameReceiver receiver;
  auto frame_transformer(rtc::make_ref_counted<MockFrameTransformer>());
  auto delegate(
      rtc::make_ref_counted<RtpVideoStreamReceiverFrameTransformerDelegate>(
          &receiver, frame_transformer, rtc::Thread::Current(),
          /*remote_ssrc*/ 1111));
  EXPECT_CALL(*frame_transformer, UnregisterTransformedFrameSinkCallback(1111));
  delegate->Reset();
}

TEST(RtpVideoStreamReceiverFrameTransformerDelegateTest, TransformFrame) {
  TestRtpVideoFrameReceiver receiver;
  auto frame_transformer(
      rtc::make_ref_counted<testing::NiceMock<MockFrameTransformer>>());
  auto delegate(
      rtc::make_ref_counted<RtpVideoStreamReceiverFrameTransformerDelegate>(
          &receiver, frame_transformer, rtc::Thread::Current(),
          /*remote_ssrc*/ 1111));
  auto frame = CreateRtpFrameObject();
  EXPECT_CALL(*frame_transformer, Transform);
  delegate->TransformFrame(std::move(frame));
}

TEST(RtpVideoStreamReceiverFrameTransformerDelegateTest,
     ManageFrameOnTransformedFrame) {
  rtc::AutoThread main_thread_;
  TestRtpVideoFrameReceiver receiver;
  auto mock_frame_transformer(
      rtc::make_ref_counted<NiceMock<MockFrameTransformer>>());
  std::vector<uint32_t> csrcs = {234, 345, 456};
  auto delegate =
      rtc::make_ref_counted<RtpVideoStreamReceiverFrameTransformerDelegate>(
          &receiver, mock_frame_transformer, rtc::Thread::Current(),
          /*remote_ssrc*/ 1111);

  rtc::scoped_refptr<TransformedFrameCallback> callback;
  EXPECT_CALL(*mock_frame_transformer, RegisterTransformedFrameSinkCallback)
      .WillOnce(SaveArg<0>(&callback));
  delegate->Init();
  ASSERT_TRUE(callback);

  EXPECT_CALL(receiver, ManageFrame)
      .WillOnce([&](std::unique_ptr<RtpFrameObject> frame) {
        EXPECT_EQ(frame->Csrcs(), csrcs);
        EXPECT_EQ(frame->first_seq_num(), 1);
        EXPECT_EQ(frame->last_seq_num(), 2);
      });
  ON_CALL(*mock_frame_transformer, Transform)
      .WillByDefault(
          [&callback](std::unique_ptr<TransformableFrameInterface> frame) {
            callback->OnTransformedFrame(std::move(frame));
          });
  delegate->TransformFrame(CreateRtpFrameObject(RTPVideoHeader(), csrcs));
  rtc::ThreadManager::ProcessAllMessageQueuesForTesting();
}

TEST(RtpVideoStreamReceiverFrameTransformerDelegateTest,
     TransformableFrameMetadataHasCorrectValue) {
  TestRtpVideoFrameReceiver receiver;
  auto mock_frame_transformer =
      rtc::make_ref_counted<NiceMock<MockFrameTransformer>>();
  auto delegate =
      rtc::make_ref_counted<RtpVideoStreamReceiverFrameTransformerDelegate>(
          &receiver, mock_frame_transformer, rtc::Thread::Current(), 1111);
  delegate->Init();
  RTPVideoHeader video_header;
  video_header.width = 1280u;
  video_header.height = 720u;
  RTPVideoHeader::GenericDescriptorInfo& generic =
      video_header.generic.emplace();
  generic.frame_id = 10;
  generic.temporal_index = 3;
  generic.spatial_index = 2;
  generic.decode_target_indications = {DecodeTargetIndication::kSwitch};
  generic.dependencies = {5};

  std::vector<uint32_t> csrcs = {234, 345, 456};

  // Check that the transformable frame passed to the frame transformer has the
  // correct metadata.
  EXPECT_CALL(*mock_frame_transformer, Transform)
      .WillOnce([&](std::unique_ptr<TransformableFrameInterface>
                        transformable_frame) {
        auto frame =
            absl::WrapUnique(static_cast<TransformableVideoFrameInterface*>(
                transformable_frame.release()));
        ASSERT_TRUE(frame);
        auto metadata = frame->Metadata();
        EXPECT_EQ(metadata.GetWidth(), 1280u);
        EXPECT_EQ(metadata.GetHeight(), 720u);
        EXPECT_EQ(metadata.GetFrameId(), 10);
        EXPECT_EQ(metadata.GetTemporalIndex(), 3);
        EXPECT_EQ(metadata.GetSpatialIndex(), 2);
        EXPECT_THAT(metadata.GetFrameDependencies(), ElementsAre(5));
        EXPECT_THAT(metadata.GetDecodeTargetIndications(),
                    ElementsAre(DecodeTargetIndication::kSwitch));
        EXPECT_EQ(metadata.GetCsrcs(), csrcs);
      });
  // The delegate creates a transformable frame from the RtpFrameObject.
  delegate->TransformFrame(CreateRtpFrameObject(video_header, csrcs));
}

TEST(RtpVideoStreamReceiverFrameTransformerDelegateTest,
     SenderFramesAreConvertedToReceiverFrames) {
  rtc::AutoThread main_thread_;
  TestRtpVideoFrameReceiver receiver;
  auto mock_frame_transformer =
      rtc::make_ref_counted<NiceMock<MockFrameTransformer>>();
  auto delegate =
      rtc::make_ref_counted<RtpVideoStreamReceiverFrameTransformerDelegate>(
          &receiver, mock_frame_transformer, rtc::Thread::Current(),
          /*remote_ssrc*/ 1111);

  auto mock_sender_frame =
      std::make_unique<NiceMock<MockTransformableVideoFrame>>();
  ON_CALL(*mock_sender_frame, GetDirection)
      .WillByDefault(Return(TransformableFrameInterface::Direction::kSender));
  VideoFrameMetadata metadata;
  metadata.SetCodec(kVideoCodecVP8);
  metadata.SetRTPVideoHeaderCodecSpecifics(RTPVideoHeaderVP8());
  ON_CALL(*mock_sender_frame, Metadata).WillByDefault(Return(metadata));
  rtc::scoped_refptr<EncodedImageBufferInterface> buffer =
      EncodedImageBuffer::Create(1);
  ON_CALL(*mock_sender_frame, GetData)
      .WillByDefault(Return(rtc::ArrayView<const uint8_t>(*buffer)));

  rtc::scoped_refptr<TransformedFrameCallback> callback;
  EXPECT_CALL(*mock_frame_transformer, RegisterTransformedFrameSinkCallback)
      .WillOnce(SaveArg<0>(&callback));
  delegate->Init();
  ASSERT_TRUE(callback);

  EXPECT_CALL(receiver, ManageFrame)
      .WillOnce([&](std::unique_ptr<RtpFrameObject> frame) {
        EXPECT_EQ(frame->codec_type(), metadata.GetCodec());
      });
  callback->OnTransformedFrame(std::move(mock_sender_frame));
  rtc::ThreadManager::ProcessAllMessageQueuesForTesting();
}

TEST(RtpVideoStreamReceiverFrameTransformerDelegateTest,
     ManageFrameFromDifferentReceiver) {
  rtc::AutoThread main_thread_;
  std::vector<uint32_t> csrcs = {234, 345, 456};

  TestRtpVideoFrameReceiver receiver1;
  auto mock_frame_transformer1(
      rtc::make_ref_counted<NiceMock<MockFrameTransformer>>());
  auto delegate1 =
      rtc::make_ref_counted<RtpVideoStreamReceiverFrameTransformerDelegate>(
          &receiver1, mock_frame_transformer1, rtc::Thread::Current(),
          /*remote_ssrc*/ 1111);

  TestRtpVideoFrameReceiver receiver2;
  auto mock_frame_transformer2(
      rtc::make_ref_counted<NiceMock<MockFrameTransformer>>());
  auto delegate2 =
      rtc::make_ref_counted<RtpVideoStreamReceiverFrameTransformerDelegate>(
          &receiver2, mock_frame_transformer2, rtc::Thread::Current(),
          /*remote_ssrc*/ 1111);

  delegate1->Init();
  rtc::scoped_refptr<TransformedFrameCallback> callback_for_2;
  EXPECT_CALL(*mock_frame_transformer2, RegisterTransformedFrameSinkCallback)
      .WillOnce(SaveArg<0>(&callback_for_2));
  delegate2->Init();
  ASSERT_TRUE(callback_for_2);

  // Expect a call on receiver2's ManageFrame with sequence numbers overwritten
  // with the frame's ID.
  EXPECT_CALL(receiver2, ManageFrame)
      .WillOnce([&](std::unique_ptr<RtpFrameObject> frame) {
        EXPECT_EQ(frame->Csrcs(), csrcs);
        EXPECT_EQ(frame->first_seq_num(), 11);
        EXPECT_EQ(frame->last_seq_num(), 11);
      });
  // When the frame transformer for receiver 1 receives the frame to transform,
  // pipe it over to the callback for receiver 2.
  ON_CALL(*mock_frame_transformer1, Transform)
      .WillByDefault([&callback_for_2](
                         std::unique_ptr<TransformableFrameInterface> frame) {
        callback_for_2->OnTransformedFrame(std::move(frame));
      });
  std::unique_ptr<RtpFrameObject> untransformed_frame =
      CreateRtpFrameObject(RTPVideoHeader(), csrcs);
  untransformed_frame->SetId(11);
  delegate1->TransformFrame(std::move(untransformed_frame));
  rtc::ThreadManager::ProcessAllMessageQueuesForTesting();
}

}  // namespace
}  // namespace webrtc
