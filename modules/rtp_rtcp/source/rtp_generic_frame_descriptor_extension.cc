/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/source/rtp_generic_frame_descriptor_extension.h"

#include "rtc_base/checks.h"

namespace webrtc {
namespace {

constexpr uint8_t kFlagBeginOfFrame = 0x80;
constexpr uint8_t kFlagEndOfFrame = 0x40;
constexpr uint8_t kFlagDependencies = 0x08;
constexpr uint8_t kMaskTemporalLayer = 0x07;

constexpr uint8_t kFlagMoreDependencies = 0x01;
constexpr uint8_t kFlageXtendedOffset = 0x02;

}  // namespace
//       0 1 2 3 4 5 6 7
//      +-+-+-+-+-+-+-+-+
//      |B|E|X| |D|  T  |
//      +-+-+-+-+-+-+-+-+
// B:   |       S       |
//      +-+-+-+-+-+-+-+-+
//      |               |
// B:   +      FID      +
//      |               |
//      +-+-+-+-+-+-+-+-+
//      |               |
//      +     Width     +
// B=1  |               |
// and  +-+-+-+-+-+-+-+-+
// D=0  |               |
//      +     Height    +
//      |               |
//      +-+-+-+-+-+-+-+-+
// D:   |    FDIFF  |X|M|
//      +---------------+
// X:   |      ...      |
//      +-+-+-+-+-+-+-+-+
// M:   |    FDIFF  |X|M|
//      +---------------+
//      |      ...      |
//      +-+-+-+-+-+-+-+-+
constexpr RTPExtensionType RtpGenericFrameDescriptorExtension::kId;
constexpr char RtpGenericFrameDescriptorExtension::kUri[];

bool RtpGenericFrameDescriptorExtension::Parse(
    rtc::ArrayView<const uint8_t> data,
    RtpGenericFrameDescriptor* descriptor) {
  if (data.empty()) {
    return false;
  }

  bool begins_frame = (data[0] & kFlagBeginOfFrame) != 0;
  descriptor->SetFirstPacketInFrame(begins_frame);
  descriptor->SetLastPacketInFrame((data[0] & kFlagEndOfFrame) != 0);

  // Parse Frame details provided in 1st packet of frame.
  if (!begins_frame) {
    return data.size() == 1;
  }
  if (data.size() < 4) {
    return false;
  }
  descriptor->SetTemporalLayer(data[0] & kMaskTemporalLayer);
  descriptor->SetSpatialLayersBitmask(data[1]);
  descriptor->SetFrameId(data[2] | (data[3] << 8));

  // Parse dependencies.
  descriptor->ClearFrameDependencies();
  size_t offset = 4;
  bool has_more_dependencies = (data[0] & kFlagDependencies) != 0;
  if (!has_more_dependencies && data.size() >= offset + 4) {
    uint16_t width = (data[offset] << 8) | data[offset + 1];
    uint16_t height = (data[offset + 2] << 8) | data[offset + 3];
    descriptor->SetResolution(width, height);
    offset += 4;
  }
  while (has_more_dependencies) {
    if (data.size() == offset)
      return false;
    has_more_dependencies = (data[offset] & kFlagMoreDependencies) != 0;
    bool extended = (data[offset] & kFlageXtendedOffset) != 0;
    uint16_t fdiff = data[offset] >> 2;
    offset++;
    if (extended) {
      if (data.size() == offset)
        return false;
      fdiff |= (data[offset] << 6);
      offset++;
    }
    if (!descriptor->AddFrameDependencyDiff(fdiff))
      return false;
  }
  return true;
}

size_t RtpGenericFrameDescriptorExtension::ValueSize(
    const RtpGenericFrameDescriptor& descriptor) {
  if (!descriptor.FirstPacketInFrame())
    return 1;

  size_t size = 4;
  for (uint16_t fdiff : descriptor.FrameDependenciesDiffs()) {
    size += (fdiff >= (1 << 6)) ? 2 : 1;
  }
  if (descriptor.FirstPacketInFrame() &&
      descriptor.FrameDependenciesDiffs().empty() && descriptor.Width() > 0 &&
      descriptor.Height() > 0) {
    size += 4;
  }
  return size;
}

bool RtpGenericFrameDescriptorExtension::Write(
    rtc::ArrayView<uint8_t> data,
    const RtpGenericFrameDescriptor& descriptor) {
  RTC_CHECK_EQ(data.size(), ValueSize(descriptor));
  uint8_t base_header =
      (descriptor.FirstPacketInFrame() ? kFlagBeginOfFrame : 0) |
      (descriptor.LastPacketInFrame() ? kFlagEndOfFrame : 0);
  if (!descriptor.FirstPacketInFrame()) {
    data[0] = base_header;
    return true;
  }
  data[0] =
      base_header |
      (descriptor.FrameDependenciesDiffs().empty() ? 0 : kFlagDependencies) |
      descriptor.TemporalLayer();
  data[1] = descriptor.SpatialLayersBitmask();
  uint16_t frame_id = descriptor.FrameId();
  data[2] = frame_id & 0xff;
  data[3] = frame_id >> 8;
  rtc::ArrayView<const uint16_t> fdiffs = descriptor.FrameDependenciesDiffs();
  size_t offset = 4;
  if (descriptor.FirstPacketInFrame() && fdiffs.empty() &&
      descriptor.Width() > 0 && descriptor.Height() > 0) {
    data[offset++] = (descriptor.Width() >> 8);
    data[offset++] = (descriptor.Width() & 0xFF);
    data[offset++] = (descriptor.Height() >> 8);
    data[offset++] = (descriptor.Height() & 0xFF);
  }
  for (size_t i = 0; i < fdiffs.size(); i++) {
    bool extended = fdiffs[i] >= (1 << 6);
    bool more = i < fdiffs.size() - 1;
    data[offset++] = ((fdiffs[i] & 0x3f) << 2) |
                     (extended ? kFlageXtendedOffset : 0) |
                     (more ? kFlagMoreDependencies : 0);
    if (extended) {
      data[offset++] = fdiffs[i] >> 6;
    }
  }
  return true;
}

}  // namespace webrtc
