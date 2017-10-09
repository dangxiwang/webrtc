/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_VIDEO_CODING_CODECS_ALPHA_INCLUDE_ALPHA_ENCODER_ADAPTER_H_
#define WEBRTC_MODULES_VIDEO_CODING_CODECS_ALPHA_INCLUDE_ALPHA_ENCODER_ADAPTER_H_

#include <map>
#include <memory>
#include <vector>

#include "api/video_codecs/video_encoder.h"
#include "media/engine/scopedvideoencoder.h"
#include "media/engine/webrtcvideoencoderfactory.h"
#include "modules/video_coding/include/video_codec_interface.h"

namespace webrtc {

enum StereoCodecStream {
  kYUVStream = 0,
  kAXXStream = 1,
  kStereoCodecStreams = 2,
};

class StereoEncoderAdapter : public VideoEncoder {
 public:
  explicit StereoEncoderAdapter(cricket::WebRtcVideoEncoderFactory* factory);
  virtual ~StereoEncoderAdapter();

  // Implements VideoEncoder
  int InitEncode(const VideoCodec* inst,
                 int number_of_cores,
                 size_t max_payload_size) override;
  int Encode(const VideoFrame& input_image,
             const CodecSpecificInfo* codec_specific_info,
             const std::vector<FrameType>* frame_types) override;
  int RegisterEncodeCompleteCallback(EncodedImageCallback* callback) override;
  int SetChannelParameters(uint32_t packet_loss, int64_t rtt) override;
  int SetRateAllocation(const BitrateAllocation& bitrate,
                        uint32_t new_framerate) override;
  int Release() override;
  const char* ImplementationName() const override {
    return "StereoEncoderAdapter";
  }

  EncodedImageCallback::Result OnEncodedImage(
      StereoCodecStream stream_idx,
      const EncodedImage& encodedImage,
      const CodecSpecificInfo* codecSpecificInfo,
      const RTPFragmentationHeader* fragmentation);

 private:
  // Wrapper class that redirects OnEncodedImage() calls.
  class AdapterEncodedImageCallback;

  // Holds the encoded image output of a frame.
  struct EncodedImageData;

  cricket::WebRtcVideoEncoderFactory* const factory_;
  std::vector<std::unique_ptr<VideoEncoder>> encoders_;
  std::vector<std::unique_ptr<AdapterEncodedImageCallback>> adapter_callbacks_;
  EncodedImageCallback* encoded_complete_callback_;

  // Map timestamp.
  std::map<uint32_t /* timestamp */, int /*count*/> frame_count_;

  uint64_t picture_index_ = 0;
  std::vector<uint8_t> merged_image_buffer_;
  std::vector<uint8_t> stereo_dummy_planes_;
};

}  // namespace webrtc

#endif  // WEBRTC_MODULES_VIDEO_CODING_CODECS_ALPHA_INCLUDE_ALPHA_ENCODER_ADAPTER_H_
