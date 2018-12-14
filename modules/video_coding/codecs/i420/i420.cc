/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/codecs/i420/include/i420.h"

#include <limits>
#include <string>

#include "api/video/i420_buffer.h"
#include "common_video/libyuv/include/webrtc_libyuv.h"
#include "third_party/libyuv/include/libyuv.h"

namespace {
const size_t kI420HeaderSize = 4;
}

namespace webrtc {

I420Encoder::I420Encoder()
    : _inited(false), _encodedImage(), _encodedCompleteCallback(NULL) {}

I420Encoder::~I420Encoder() {
  _inited = false;
  delete[] _encodedImage._buffer;
}

int I420Encoder::Release() {
  // Should allocate an encoded frame and then release it here, for that we
  // actually need an init flag.
  if (_encodedImage._buffer != NULL) {
    delete[] _encodedImage._buffer;
    _encodedImage._buffer = NULL;
  }
  _inited = false;
  return WEBRTC_VIDEO_CODEC_OK;
}

int I420Encoder::InitEncode(const VideoCodec* codecSettings,
                            int /*numberOfCores*/,
                            size_t /*maxPayloadSize */) {
  if (codecSettings == NULL) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (codecSettings->width < 1 || codecSettings->height < 1) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  // Allocating encoded memory.
  if (_encodedImage._buffer != NULL) {
    delete[] _encodedImage._buffer;
    _encodedImage.set_buffer(NULL, 0);
  }
  const size_t newCapacity =
      CalcBufferSize(VideoType::kI420, codecSettings->width,
                     codecSettings->height) +
      kI420HeaderSize;
  uint8_t* newBuffer = new uint8_t[newCapacity];
  if (newBuffer == NULL) {
    return WEBRTC_VIDEO_CODEC_MEMORY;
  }
  _encodedImage.set_buffer(newBuffer, newCapacity);

  // If no memory allocation, no point to init.
  _inited = true;
  return WEBRTC_VIDEO_CODEC_OK;
}

int I420Encoder::Encode(const VideoFrame& inputImage,
                        const CodecSpecificInfo* /*codecSpecificInfo*/,
                        const std::vector<FrameType>* /*frame_types*/) {
  if (!_inited) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (_encodedCompleteCallback == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  _encodedImage._frameType = kVideoFrameKey;
  _encodedImage.SetTimestamp(inputImage.timestamp());
  _encodedImage._encodedHeight = inputImage.height();
  _encodedImage._encodedWidth = inputImage.width();

  int width = inputImage.width();
  if (width > std::numeric_limits<uint16_t>::max()) {
    return WEBRTC_VIDEO_CODEC_ERR_SIZE;
  }
  int height = inputImage.height();
  if (height > std::numeric_limits<uint16_t>::max()) {
    return WEBRTC_VIDEO_CODEC_ERR_SIZE;
  }

  size_t req_capacity = CalcBufferSize(VideoType::kI420, inputImage.width(),
                                       inputImage.height()) +
                        kI420HeaderSize;
  if (_encodedImage.capacity() < req_capacity) {
    // Reallocate buffer.
    delete[] _encodedImage._buffer;
    _encodedImage.set_buffer(new uint8_t[req_capacity], req_capacity);
  }

  uint8_t* buffer = _encodedImage._buffer;

  buffer = InsertHeader(buffer, width, height);

  int ret_length =
      ExtractBuffer(inputImage, req_capacity - kI420HeaderSize, buffer);
  if (ret_length < 0)
    return WEBRTC_VIDEO_CODEC_MEMORY;
  _encodedImage._length = ret_length + kI420HeaderSize;

  _encodedCompleteCallback->OnEncodedImage(_encodedImage, nullptr, nullptr);

  return WEBRTC_VIDEO_CODEC_OK;
}

uint8_t* I420Encoder::InsertHeader(uint8_t* buffer,
                                   uint16_t width,
                                   uint16_t height) {
  *buffer++ = static_cast<uint8_t>(width >> 8);
  *buffer++ = static_cast<uint8_t>(width & 0xFF);
  *buffer++ = static_cast<uint8_t>(height >> 8);
  *buffer++ = static_cast<uint8_t>(height & 0xFF);
  return buffer;
}

int I420Encoder::RegisterEncodeCompleteCallback(
    EncodedImageCallback* callback) {
  _encodedCompleteCallback = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

I420Decoder::I420Decoder() : _decodeCompleteCallback(NULL) {}

I420Decoder::~I420Decoder() {
  Release();
}

int I420Decoder::InitDecode(const VideoCodec* codecSettings,
                            int /*numberOfCores */) {
  return WEBRTC_VIDEO_CODEC_OK;
}

int I420Decoder::Decode(const EncodedImage& inputImage,
                        bool /*missingFrames*/,
                        const CodecSpecificInfo* /*codecSpecificInfo*/,
                        int64_t /*renderTimeMs*/) {
  if (inputImage._buffer == NULL) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (_decodeCompleteCallback == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (inputImage._length <= 0) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inputImage._completeFrame == false) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inputImage._length < kI420HeaderSize) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  const uint8_t* buffer = inputImage._buffer;
  uint16_t width, height;
  buffer = ExtractHeader(buffer, &width, &height);

  // Verify that the available length is sufficient:
  size_t req_capacity =
      CalcBufferSize(VideoType::kI420, width, height) + kI420HeaderSize;

  if (req_capacity > inputImage._length) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  // Set decoded image parameters.
  rtc::scoped_refptr<webrtc::I420Buffer> frame_buffer =
      I420Buffer::Create(width, height);

  // Converting from raw buffer I420Buffer.
  int y_stride = 16 * ((width + 15) / 16);
  int uv_stride = 16 * ((width + 31) / 32);
  int y_size = y_stride * height;
  int u_size = uv_stride * frame_buffer->ChromaHeight();
  int ret = libyuv::I420Copy(
      buffer, y_stride, buffer + y_size, uv_stride, buffer + y_size + u_size,
      uv_stride, frame_buffer.get()->MutableDataY(),
      frame_buffer.get()->StrideY(), frame_buffer.get()->MutableDataU(),
      frame_buffer.get()->StrideU(), frame_buffer.get()->MutableDataV(),
      frame_buffer.get()->StrideV(), width, height);
  if (ret < 0) {
    return WEBRTC_VIDEO_CODEC_MEMORY;
  }

  VideoFrame decoded_image =
                 VideoFrame::Builder().set_video_frame_buffer(frame_buffer),
             .set_timestamp_rtp(inputImage.Timestamp())
                 .set_timestamp_us(0)
                 .set_rotation(webrtc::kVideoRotation_0)
                 .build();
  _decodeCompleteCallback->Decoded(decoded_image);
  return WEBRTC_VIDEO_CODEC_OK;
}

const uint8_t* I420Decoder::ExtractHeader(const uint8_t* buffer,
                                          uint16_t* width,
                                          uint16_t* height) {
  *width = static_cast<uint16_t>(*buffer++) << 8;
  *width |= *buffer++;
  *height = static_cast<uint16_t>(*buffer++) << 8;
  *height |= *buffer++;

  return buffer;
}

int I420Decoder::RegisterDecodeCompleteCallback(
    DecodedImageCallback* callback) {
  _decodeCompleteCallback = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int I420Decoder::Release() {
  return WEBRTC_VIDEO_CODEC_OK;
}
}  // namespace webrtc
