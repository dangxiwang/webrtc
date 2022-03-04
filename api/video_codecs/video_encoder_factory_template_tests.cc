/*
 *  Copyright (c) 2022 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

//#include "api/test/mock_video_encoder.h"
#include "api/video_codecs/video_encoder_factory_template.h"
#include "api/video_codecs/video_encoder_factory_template_adapters.h"
#include "test/gmock.h"
#include "test/gtest.h"

using ::testing::Each;
using ::testing::Eq;
using ::testing::Field;
using ::testing::Ge;
using ::testing::Ne;
using ::testing::SizeIs;
using ::testing::UnorderedElementsAre;

namespace webrtc {
namespace {
using CodecSupport = VideoEncoderFactory::CodecSupport;
// const SdpVideoFormat kFooSdp("Foo");
// const SdpVideoFormat kBarLowSdp("Bar", {{"profile", "low"}});
// const SdpVideoFormat kBarHighSdp("Bar", {{"profile", "high"}});

// struct FooEncoderTemplateAdapter {
//   static std::vector<SdpVideoFormat> SupportedFormats() {
//     return {SdpVideoFormat("Foo")};
//   }

//   static std::unique_ptr<VideoEncoder> CreateEncoder(
//       const SdpVideoFormat& format) {
//     return std::make_unique<testing::StrictMock<MockVideoEncoder>>();
//   }

//   static bool IsScalabilityModeSupported(
//       const absl::string_view scalability_mode) {
//     return scalability_mode == "L1T2" || scalability_mode == "L1T3";
//   }
// };

// struct BarEncoderTemplateAdapter {
//   static std::vector<SdpVideoFormat> SupportedFormats() {
//     return {SdpVideoFormat("Bar", {{"profile", "low"}}),
//             SdpVideoFormat("Bar", {{"profile", "high"}})};
//   }

//   static std::unique_ptr<VideoEncoder> CreateEncoder(
//       const SdpVideoFormat& format) {
//     return std::make_unique<testing::StrictMock<MockVideoEncoder>>();
//   }

//   static bool IsScalabilityModeSupported(
//       const absl::string_view scalability_mode) {
//     return scalability_mode == "L1T2" || scalability_mode == "L1T3" ||
//            scalability_mode == "S2T2" || scalability_mode == "S2T3";
//   }
// };

// TEST(VideoEncoderFactoryTemplate, OneTemplateAdapterCreateEncoder) {
//   VideoEncoderFactoryTemplate<FooEncoderTemplateAdapter> factory;
//   EXPECT_THAT(factory.GetSupportedFormats(), UnorderedElementsAre(kFooSdp));
//   EXPECT_THAT(factory.CreateVideoEncoder(kFooSdp), Ne(nullptr));
//   EXPECT_THAT(factory.CreateVideoEncoder(SdpVideoFormat("FooX")), Eq(nullptr));
// }

// TEST(VideoEncoderFactoryTemplate, OneTemplateAdapterCodecSupport) {
//   VideoEncoderFactoryTemplate<FooEncoderTemplateAdapter> factory;
//   EXPECT_THAT(factory.QueryCodecSupport(kFooSdp, absl::nullopt),
//               Field(&CodecSupport::is_supported, true));
//   EXPECT_THAT(factory.QueryCodecSupport(kFooSdp, "L1T2"),
//               Field(&CodecSupport::is_supported, true));
//   EXPECT_THAT(factory.QueryCodecSupport(kFooSdp, "S2T3"),
//               Field(&CodecSupport::is_supported, false));
//   EXPECT_THAT(factory.QueryCodecSupport(SdpVideoFormat("FooX"), absl::nullopt),
//               Field(&CodecSupport::is_supported, false));
// }

// TEST(VideoEncoderFactoryTemplate, TwoTemplateAdaptersCreateEncoders) {
//   VideoEncoderFactoryTemplate<FooEncoderTemplateAdapter,
//                               BarEncoderTemplateAdapter>
//       factory;
//   EXPECT_THAT(factory.GetSupportedFormats(),
//               UnorderedElementsAre(kFooSdp, kBarLowSdp, kBarHighSdp));
//   EXPECT_THAT(factory.CreateVideoEncoder(kFooSdp), Ne(nullptr));
//   EXPECT_THAT(factory.CreateVideoEncoder(kBarLowSdp), Ne(nullptr));
//   EXPECT_THAT(factory.CreateVideoEncoder(kBarHighSdp), Ne(nullptr));
//   EXPECT_THAT(factory.CreateVideoEncoder(SdpVideoFormat("FooX")), Eq(nullptr));
//   EXPECT_THAT(factory.CreateVideoEncoder(SdpVideoFormat("Bar")), Eq(nullptr));
// }

// TEST(VideoEncoderFactoryTemplate, TwoTemplateAdaptersCodecSupport) {
//   VideoEncoderFactoryTemplate<FooEncoderTemplateAdapter,
//                               BarEncoderTemplateAdapter>
//       factory;
//   EXPECT_THAT(factory.QueryCodecSupport(kFooSdp, absl::nullopt),
//               Field(&CodecSupport::is_supported, true));
//   EXPECT_THAT(factory.QueryCodecSupport(kFooSdp, "L1T2"),
//               Field(&CodecSupport::is_supported, true));
//   EXPECT_THAT(factory.QueryCodecSupport(kFooSdp, "S2T3"),
//               Field(&CodecSupport::is_supported, false));
//   EXPECT_THAT(factory.QueryCodecSupport(kBarLowSdp, absl::nullopt),
//               Field(&CodecSupport::is_supported, true));
//   EXPECT_THAT(factory.QueryCodecSupport(kBarHighSdp, absl::nullopt),
//               Field(&CodecSupport::is_supported, true));
//   EXPECT_THAT(factory.QueryCodecSupport(kBarLowSdp, "S2T2"),
//               Field(&CodecSupport::is_supported, true));
//   EXPECT_THAT(factory.QueryCodecSupport(kBarHighSdp, "S3T2"),
//               Field(&CodecSupport::is_supported, false));
// }

TEST(VideoEncoderFactoryTemplate, LibvpxVp8) {
  VideoEncoderFactoryTemplate<LibvpxVp8EncoderTemplateAdapter> factory;
  const SdpVideoFormat kVp8Sdp("VP8");
  EXPECT_THAT(factory.GetSupportedFormats(), UnorderedElementsAre(kVp8Sdp));
  EXPECT_THAT(factory.CreateVideoEncoder(kVp8Sdp), Ne(nullptr));
}

TEST(VideoEncoderFactoryTemplate, LibvpxVp9) {
  VideoEncoderFactoryTemplate<LibvpxVp9EncoderTemplateAdapter> factory;
  auto formats = factory.GetSupportedFormats();
  EXPECT_THAT(formats, SizeIs(Ge(1UL)));
  EXPECT_THAT(formats, Each(Field(&SdpVideoFormat::name, "VP9")));
  EXPECT_THAT(factory.CreateVideoEncoder(formats[0]), Ne(nullptr));
}

// TODO(bugs.webrtc.org/13573): When OpenH264 is no longer a conditional build
//                              target remove this #ifdef.
#if defined(WEBRTC_USE_H264)
TEST(VideoEncoderFactoryTemplate, OpenH264) {
  VideoEncoderFactoryTemplate<OpenH264EncoderTemplateAdapter> factory;
  auto formats = factory.GetSupportedFormats();
  EXPECT_THAT(formats, SizeIs(Ge(1UL)));
  EXPECT_THAT(formats, Each(Field(&SdpVideoFormat::name, "H264")));
  EXPECT_THAT(factory.CreateVideoEncoder(formats[0]), Ne(nullptr));
}
#endif  // defined(WEBRTC_USE_H264)

TEST(VideoEncoderFactoryTemplate, LibaomAv1) {
  VideoEncoderFactoryTemplate<LibaomAv1EncoderTemplateAdapter> factory;
  const SdpVideoFormat kAv1Sdp("AV1");
  EXPECT_THAT(factory.GetSupportedFormats(), UnorderedElementsAre(kAv1Sdp));
  EXPECT_THAT(factory.CreateVideoEncoder(kAv1Sdp), Ne(nullptr));
}

}  // namespace
}  // namespace webrtc
