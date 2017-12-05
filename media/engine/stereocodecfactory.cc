/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "media/engine/stereocodecfactory.h"

#include <utility>

#include "api/video_codecs/sdp_video_format.h"
#include "media/base/codec.h"
#include "media/base/mediaconstants.h"
#include "modules/video_coding/codecs/stereo/include/stereo_decoder_adapter.h"
#include "modules/video_coding/codecs/stereo/include/stereo_encoder_adapter.h"
#include "rtc_base/logging.h"

namespace {

bool IsStereoCodec(const cricket::VideoCodec& codec) {
  return cricket::CodecNamesEq(codec.name.c_str(), cricket::kStereoCodecName);
}

}  // anonymous namespace

namespace webrtc {

constexpr const char* kStereoAssociatedCodecName = cricket::kVp9CodecName;

StereoEncoderFactory::StereoEncoderFactory(
    std::unique_ptr<VideoEncoderFactory> factory)
    : factory_(std::move(factory)) {}

std::vector<SdpVideoFormat> StereoEncoderFactory::GetSupportedFormats() const {
  std::vector<SdpVideoFormat> formats = factory_->GetSupportedFormats();
  for (const auto& format : formats) {
    if (cricket::CodecNamesEq(format.name, kStereoAssociatedCodecName)) {
      formats.emplace_back(cricket::kStereoCodecName);
      break;
    }
  }
  return formats;
}

VideoEncoderFactory::CodecInfo StereoEncoderFactory::QueryVideoEncoder(
    const SdpVideoFormat& format) const {
  if (!IsStereoCodec(cricket::VideoCodec(format)))
    return factory_->QueryVideoEncoder(format);
  return factory_->QueryVideoEncoder(
      SdpVideoFormat(kStereoAssociatedCodecName));
}

std::unique_ptr<VideoEncoder> StereoEncoderFactory::CreateVideoEncoder(
    const SdpVideoFormat& format) {
  if (!IsStereoCodec(cricket::VideoCodec(format)))
    return factory_->CreateVideoEncoder(format);
  return std::unique_ptr<VideoEncoder>(new StereoEncoderAdapter(
      factory_.get(), SdpVideoFormat(kStereoAssociatedCodecName)));
}

StereoDecoderFactory::StereoDecoderFactory(
    std::unique_ptr<VideoDecoderFactory> factory)
    : factory_(std::move(factory)) {}

std::vector<SdpVideoFormat> StereoDecoderFactory::GetSupportedFormats() const {
  std::vector<SdpVideoFormat> formats = factory_->GetSupportedFormats();
  for (const auto& format : formats) {
    if (cricket::CodecNamesEq(format.name, kStereoAssociatedCodecName)) {
      formats.emplace_back(cricket::kStereoCodecName);
      break;
    }
  }
  return formats;
}

std::unique_ptr<VideoDecoder> StereoDecoderFactory::CreateVideoDecoder(
    const SdpVideoFormat& format) {
  if (!IsStereoCodec(cricket::VideoCodec(format)))
    return factory_->CreateVideoDecoder(format);
  return std::unique_ptr<VideoDecoder>(new StereoDecoderAdapter(
      factory_.get(), SdpVideoFormat(kStereoAssociatedCodecName)));
}

}  // namespace webrtc
