/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/codecs/test/videoprocessor_integrationtest.h"

#include <vector>

#include "api/test/create_videoprocessor_integrationtest_fixture.h"
#include "media/base/mediaconstants.h"
#include "modules/video_coding/codecs/test/objc_codec_factory_helper.h"
#include "test/field_trial.h"
#include "test/testsupport/fileutils.h"

namespace webrtc {
namespace test {

namespace {

typedef VideoProcessorIntegrationTestFixtureInterface BaseTest;

const int kForemanNumFrames = 300;

static std::unique_ptr<BaseTest> CreateTestFixture() {
  auto decoder_factory = CreateObjCDecoderFactory();
  auto encoder_factory = CreateObjCEncoderFactory();
  auto fixture = CreateVideoProcessorIntegrationTestFixture(
      std::move(decoder_factory), std::move(encoder_factory));
  fixture->config.filename = "foreman_cif";
  fixture->config.filepath = ResourcePath(fixture->config.filename, "yuv");
  fixture->config.num_frames = kForemanNumFrames;
  fixture->config.hw_encoder = true;
  fixture->config.hw_decoder = true;
  fixture->config.encoded_frame_checker =
      new VideoProcessorIntegrationTest::H264KeyframeChecker();
  return fixture;
}
}  // namespace

// TODO(webrtc:9099): Disabled until the issue is fixed.
// HW codecs don't work on simulators. Only run these tests on device.
// #if TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR
// #define MAYBE_TEST TEST
// #else
#define MAYBE_TEST(s, name) TEST(s, DISABLED_##name)
// #endif

// TODO(kthelgason): Use RC Thresholds when the internal bitrateAdjuster is no
// longer in use.
MAYBE_TEST(VideoProcessorIntegrationTestVideoToolbox,
           ForemanCif500kbpsH264CBP) {
  auto fixture = CreateTestFixture();
  fixture->config.SetCodecSettings(cricket::kH264CodecName, 1, 1, 1, false,
                                   false, false, 352, 288);

  std::vector<RateProfile> rate_profiles = {{500, 30, kForemanNumFrames}};

  std::vector<QualityThresholds> quality_thresholds = {{33, 29, 0.9, 0.82}};

  fixture->ProcessFramesAndMaybeVerify(rate_profiles, nullptr,
                                       &quality_thresholds, nullptr, nullptr);
}

MAYBE_TEST(VideoProcessorIntegrationTestVideoToolbox,
           ForemanCif500kbpsH264CHP) {
  auto fixture = CreateTestFixture();
  ScopedFieldTrials override_field_trials("WebRTC-H264HighProfile/Enabled/");

  fixture->config.h264_codec_settings.profile = H264::kProfileConstrainedHigh;
  fixture->config.SetCodecSettings(cricket::kH264CodecName, 1, 1, 1, false,
                                   false, false, 352, 288);

  std::vector<RateProfile> rate_profiles = {{500, 30, kForemanNumFrames}};

  std::vector<QualityThresholds> quality_thresholds = {{33, 30, 0.91, 0.83}};

  fixture->ProcessFramesAndMaybeVerify(rate_profiles, nullptr,
                                       &quality_thresholds, nullptr, nullptr);
}

}  // namespace test
}  // namespace webrtc
