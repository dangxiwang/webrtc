/*
 *  Copyright (c) 2023 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "api/video/i410_buffer.h"

#include <string.h>

#include <algorithm>
#include <utility>

#include "api/make_ref_counted.h"
#include "api/video/i420_buffer.h"
#include "rtc_base/checks.h"
#include "third_party/libyuv/include/libyuv/convert.h"
#include "third_party/libyuv/include/libyuv/planar_functions.h"
#include "third_party/libyuv/include/libyuv/scale.h"

// Aligning pointer to 64 bytes for improved performance, e.g. use SIMD.
static const int kBufferAlignment = 64;
static const int kBytesPerPixel = 2;

namespace webrtc {

namespace {

int I410DataSize(int height, int stride_y, int stride_u, int stride_v) {
  return kBytesPerPixel *
         (stride_y * height + stride_u * height + stride_v * height);
}

}  // namespace

I410Buffer::I410Buffer(int width, int height)
    : I410Buffer(width, height, width, (width), (width)) {}

I410Buffer::I410Buffer(int width,
                       int height,
                       int stride_y,
                       int stride_u,
                       int stride_v)
    : width_(width),
      height_(height),
      stride_y_(stride_y),
      stride_u_(stride_u),
      stride_v_(stride_v),
      data_(static_cast<uint16_t*>(
          AlignedMalloc(I410DataSize(height, stride_y, stride_u, stride_v),
                        kBufferAlignment))) {
  RTC_DCHECK_GT(width, 0);
  RTC_DCHECK_GT(height, 0);
  RTC_DCHECK_GE(stride_y, width);
  RTC_DCHECK_GE(stride_u, (width));
  RTC_DCHECK_GE(stride_v, (width));
}

I410Buffer::~I410Buffer() {}

// static
rtc::scoped_refptr<I410Buffer> I410Buffer::Create(int width, int height) {
  return rtc::make_ref_counted<I410Buffer>(width, height);
}

// static
rtc::scoped_refptr<I410Buffer> I410Buffer::Create(int width,
                                                  int height,
                                                  int stride_y,
                                                  int stride_u,
                                                  int stride_v) {
  return rtc::make_ref_counted<I410Buffer>(width, height, stride_y, stride_u,
                                           stride_v);
}

// static
rtc::scoped_refptr<I410Buffer> I410Buffer::Copy(
    const I410BufferInterface& source) {
  return Copy(source.width(), source.height(), source.DataY(), source.StrideY(),
              source.DataU(), source.StrideU(), source.DataV(),
              source.StrideV());
}

// static
rtc::scoped_refptr<I410Buffer> I410Buffer::Copy(int width,
                                                int height,
                                                const uint16_t* data_y,
                                                int stride_y,
                                                const uint16_t* data_u,
                                                int stride_u,
                                                const uint16_t* data_v,
                                                int stride_v) {
  // Note: May use different strides than the input data.
  rtc::scoped_refptr<I410Buffer> buffer = Create(width, height);
  RTC_CHECK_EQ(0, libyuv::I410Copy(data_y, stride_y, data_u, stride_u, data_v,
                                   stride_v, buffer->MutableDataY(),
                                   buffer->StrideY(), buffer->MutableDataU(),
                                   buffer->StrideU(), buffer->MutableDataV(),
                                   buffer->StrideV(), width, height));
  return buffer;
}

// static
rtc::scoped_refptr<I410Buffer> I410Buffer::Rotate(
    const I410BufferInterface& src,
    VideoRotation rotation) {
  RTC_CHECK(src.DataY());
  RTC_CHECK(src.DataU());
  RTC_CHECK(src.DataV());

  int rotated_width = src.width();
  int rotated_height = src.height();
  if (rotation == webrtc::kVideoRotation_90 ||
      rotation == webrtc::kVideoRotation_270) {
    std::swap(rotated_width, rotated_height);
  }

  rtc::scoped_refptr<webrtc::I410Buffer> buffer =
      I410Buffer::Create(rotated_width, rotated_height);

  RTC_CHECK_EQ(0,
               libyuv::I410Rotate(
                   src.DataY(), src.StrideY(), src.DataU(), src.StrideU(),
                   src.DataV(), src.StrideV(), buffer->MutableDataY(),
                   buffer->StrideY(), buffer->MutableDataU(), buffer->StrideU(),
                   buffer->MutableDataV(), buffer->StrideV(), src.width(),
                   src.height(), static_cast<libyuv::RotationMode>(rotation)));

  return buffer;
}

rtc::scoped_refptr<I420BufferInterface> I410Buffer::ToI420() {
  rtc::scoped_refptr<I420Buffer> i420_buffer =
      I420Buffer::Create(width(), height());
  libyuv::I410ToI420(DataY(), StrideY(), DataU(), StrideU(), DataV(), StrideV(),
                     i420_buffer->MutableDataY(), i420_buffer->StrideY(),
                     i420_buffer->MutableDataU(), i420_buffer->StrideU(),
                     i420_buffer->MutableDataV(), i420_buffer->StrideV(),
                     width(), height());
  return i420_buffer;
}

void I410Buffer::InitializeData() {
  memset(data_.get(), 0,
         I410DataSize(height_, stride_y_, stride_u_, stride_v_));
}

int I410Buffer::width() const {
  return width_;
}

int I410Buffer::height() const {
  return height_;
}

const uint16_t* I410Buffer::DataY() const {
  return data_.get();
}
const uint16_t* I410Buffer::DataU() const {
  return data_.get() + stride_y_ * height_;
}
const uint16_t* I410Buffer::DataV() const {
  return data_.get() + stride_y_ * height_ + stride_u_ * ((height_));
}

int I410Buffer::StrideY() const {
  return stride_y_;
}
int I410Buffer::StrideU() const {
  return stride_u_;
}
int I410Buffer::StrideV() const {
  return stride_v_;
}

uint16_t* I410Buffer::MutableDataY() {
  return const_cast<uint16_t*>(DataY());
}
uint16_t* I410Buffer::MutableDataU() {
  return const_cast<uint16_t*>(DataU());
}
uint16_t* I410Buffer::MutableDataV() {
  return const_cast<uint16_t*>(DataV());
}

void I410Buffer::CropAndScaleFrom(const I410BufferInterface& src,
                                  int offset_x,
                                  int offset_y,
                                  int crop_width,
                                  int crop_height) {
  RTC_CHECK_LE(crop_width, src.width());
  RTC_CHECK_LE(crop_height, src.height());
  RTC_CHECK_LE(crop_width + offset_x, src.width());
  RTC_CHECK_LE(crop_height + offset_y, src.height());
  RTC_CHECK_GE(offset_x, 0);
  RTC_CHECK_GE(offset_y, 0);

  const uint16_t* y_plane = src.DataY() + src.StrideY() * offset_y + offset_x;
  const uint16_t* u_plane = src.DataU() + src.StrideU() * offset_y + offset_x;
  const uint16_t* v_plane = src.DataV() + src.StrideV() * offset_y + offset_x;
  int res = libyuv::I444Scale_16(
      y_plane, src.StrideY(), u_plane, src.StrideU(), v_plane, src.StrideV(),
      crop_width, crop_height, MutableDataY(), StrideY(), MutableDataU(),
      StrideU(), MutableDataV(), StrideV(), width(), height(),
      libyuv::kFilterBox);

  RTC_DCHECK_EQ(res, 0);
}

void I410Buffer::ScaleFrom(const I410BufferInterface& src) {
  CropAndScaleFrom(src, 0, 0, src.width(), src.height());
}

}  // namespace webrtc
