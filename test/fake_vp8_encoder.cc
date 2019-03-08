/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "test/fake_vp8_encoder.h"

#include <algorithm>

#include "absl/types/optional.h"
#include "api/video_codecs/vp8_temporal_layers.h"
#include "api/video_codecs/vp8_temporal_layers_factory.h"
#include "common_types.h"  // NOLINT(build/include)
#include "modules/video_coding/codecs/interface/common_constants.h"
#include "modules/video_coding/include/video_codec_interface.h"
#include "modules/video_coding/include/video_error_codes.h"
#include "modules/video_coding/utility/simulcast_utility.h"

namespace {

// Write width and height to the payload the same way as the real encoder does.
// It requires that |payload| has a size of at least kMinPayLoadHeaderLength.
void WriteFakeVp8(unsigned char* payload,
                  int width,
                  int height,
                  bool key_frame) {
  payload[0] = key_frame ? 0 : 0x01;

  if (key_frame) {
    payload[9] = (height & 0x3F00) >> 8;
    payload[8] = (height & 0x00FF);

    payload[7] = (width & 0x3F00) >> 8;
    payload[6] = (width & 0x00FF);
  }
}
}  // namespace

namespace webrtc {

namespace test {

FakeVP8Encoder::FakeVP8Encoder(Clock* clock) : FakeEncoder(clock) {
  sequence_checker_.Detach();
}

int32_t FakeVP8Encoder::InitEncode(const VideoCodec* config,
                                   int32_t number_of_cores,
                                   size_t max_payload_size) {
  RTC_DCHECK_CALLED_SEQUENTIALLY(&sequence_checker_);
  auto result =
      FakeEncoder::InitEncode(config, number_of_cores, max_payload_size);
  if (result != WEBRTC_VIDEO_CODEC_OK) {
    return result;
  }

  SetupTemporalLayers(*config);

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t FakeVP8Encoder::Release() {
  auto result = FakeEncoder::Release();
  sequence_checker_.Detach();
  return result;
}

// TODO: !!! 1. Single controller. 2. Remove .h files.
void FakeVP8Encoder::SetupTemporalLayers(const VideoCodec& codec) {
  RTC_DCHECK_CALLED_SEQUENTIALLY(&sequence_checker_);

  Vp8TemporalLayersFactory factory;

  int num_streams = SimulcastUtility::NumberOfSimulcastStreams(codec);
  for (int i = 0; i < num_streams; ++i) {
    temporal_layers_.push_back(factory.Create(codec));
  }
}

void FakeVP8Encoder::PopulateCodecSpecific(CodecSpecificInfo* codec_specific,
                                           size_t size_bytes,
                                           VideoFrameType frame_type,
                                           int stream_idx,
                                           uint32_t timestamp) {
  RTC_DCHECK_CALLED_SEQUENTIALLY(&sequence_checker_);
  codec_specific->codecType = kVideoCodecVP8;
  codec_specific->codecSpecific.VP8.keyIdx = kNoKeyIdx;
  codec_specific->codecSpecific.VP8.nonReference = false;
  temporal_layers_[stream_idx]->OnEncodeDone(stream_idx, timestamp, size_bytes,
                                             frame_type == kVideoFrameKey, -1,
                                             codec_specific);
}

std::unique_ptr<RTPFragmentationHeader> FakeVP8Encoder::EncodeHook(
    EncodedImage* encoded_image,
    CodecSpecificInfo* codec_specific) {
  RTC_DCHECK_CALLED_SEQUENTIALLY(&sequence_checker_);
  uint8_t stream_idx = encoded_image->SpatialIndex().value_or(0);
  temporal_layers_[stream_idx]->UpdateLayerConfig(stream_idx,
                                                  encoded_image->Timestamp());
  PopulateCodecSpecific(codec_specific, encoded_image->size(),
                        encoded_image->_frameType, stream_idx,
                        encoded_image->Timestamp());

  // Write width and height to the payload the same way as the real encoder
  // does.
  WriteFakeVp8(encoded_image->data(), encoded_image->_encodedWidth,
               encoded_image->_encodedHeight,
               encoded_image->_frameType == kVideoFrameKey);
  return nullptr;
}

VideoEncoder::EncoderInfo FakeVP8Encoder::GetEncoderInfo() const {
  EncoderInfo info;
  info.implementation_name = "FakeVp8Encoder";
  return info;
}

}  // namespace test
}  // namespace webrtc
