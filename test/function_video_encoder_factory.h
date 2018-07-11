/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef TEST_FUNCTION_VIDEO_ENCODER_FACTORY_H_
#define TEST_FUNCTION_VIDEO_ENCODER_FACTORY_H_

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_encoder_factory.h"
#include "rtc_base/checks.h"

namespace webrtc {
namespace test {

// An encoder factory producing encoders by calling a supplied create
// function.
class FunctionVideoEncoderFactory final : public VideoEncoderFactory {
 public:
  explicit FunctionVideoEncoderFactory(
      std::function<std::unique_ptr<VideoEncoder>()> create)
      : create_([create](const SdpVideoFormat&) { return create(); }) {}
  explicit FunctionVideoEncoderFactory(
      std::function<std::unique_ptr<VideoEncoder>(const SdpVideoFormat&)>
          create)
      : create_(std::move(create)) {}

  // Unused by tests.
  std::vector<SdpVideoFormat> GetSupportedFormats() const override {
    RTC_NOTREACHED();
    return {};
  }

  CodecInfo QueryVideoEncoder(
      const SdpVideoFormat& /* format */) const override {
    CodecInfo codec_info;
    codec_info.is_hardware_accelerated = false;
    codec_info.has_internal_source = false;
    return codec_info;
  }

  std::unique_ptr<VideoEncoder> CreateVideoEncoder(
      const SdpVideoFormat& format) override {
    return create_(format);
  }

 private:
  const std::function<std::unique_ptr<VideoEncoder>(const SdpVideoFormat&)>
      create_;
};

}  // namespace test
}  // namespace webrtc

#endif  // TEST_FUNCTION_VIDEO_ENCODER_FACTORY_H_
