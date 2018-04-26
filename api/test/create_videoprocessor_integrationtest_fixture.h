/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_TEST_CREATE_VIDEOPROCESSOR_INTEGRATIONTEST_FIXTURE_H_
#define API_TEST_CREATE_VIDEOPROCESSOR_INTEGRATIONTEST_FIXTURE_H_

#include <memory>

#include "api/test/videoprocessor_integrationtest_fixture.h"
#include "api/video_codecs/video_decoder_factory.h"
#include "api/video_codecs/video_encoder_factory.h"

namespace webrtc {
namespace test {

std::unique_ptr<VideoProcessorIntegrationTestFixtureInterface>
CreateVideoProcessorIntegrationTestFixture();

std::unique_ptr<VideoProcessorIntegrationTestFixtureInterface>
CreateVideoProcessorIntegrationTestFixture(
    std::unique_ptr<VideoDecoderFactory> decoder_factory,
    std::unique_ptr<VideoEncoderFactory> encoder_factory);

}  // namespace test
}  // namespace webrtc

#endif  // API_TEST_CREATE_VIDEOPROCESSOR_INTEGRATIONTEST_FIXTURE_H_
