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

namespace webrtc {
namespace {

constexpr uint8_t kFlagFirstSubframe = 0x80;
constexpr uint8_t kFlagLastSubframe = 0x40;
constexpr uint8_t kFlagBeginOfSubframe = 0x20;
constexpr uint8_t kFlagEndOfSubframe = 0x10;
constexpr uint8_t kFlagDependencies = 0x08;
constexpr uint8_t kMaskTemporalLayer = 0x07;

constexpr uint8_t kFlagMoreDependencies = 0x01;
constexpr uint8_t kFlageXtendedOffset = 0x02;

}  // namespace
//       0 1 2 3 4 5 6 7
//      +-+-+-+-+-+-+-+-+
//      |F|L|B|E|D|  T  |
//      +-+-+-+-+-+-+-+-+
// B:   |       S       |
//      +-+-+-+-+-+-+-+-+
//      |               |
// B:   +      FID      +
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

bool RtpGenericFrameDescriptorExtension::Parse(
    rtc::ArrayView<const uint8_t> data,
    RtpGenericFrameDescriptor* descriptor) {
  if (data.empty()) {
    return false;
  }

  bool begins_subframe = (data[0] & kFlagBeginOfSubframe) != 0;
  descriptor->SetFirstPacketInSubFrame(begins_subframe);
  descriptor->SetLastPacketInSubFrame((data[0] & kFlagEndOfSubframe) != 0);
  descriptor->SetFirstSubFrameInFrame((data[0] & kFlagFirstSubframe) != 0);
  descriptor->SetLastSubFrameInFrame((data[0] & kFlagLastSubframe) != 0);

  // Parse Subframe details provided in 1st packet of subframe.
  if (!begins_subframe) {
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
  bool has_dependencies = (data[0] & kFlagDependencies) != 0;
  if (!has_dependencies) {
    return data.size() == 4;
  }
  size_t offset = 4;
  while (true) {
    if (data.size() == offset)
      return false;
    bool more = (data[offset] & kFlagMoreDependencies) != 0;
    bool extended = (data[offset] & kFlageXtendedOffset) != 0;
    uint16_t fdiff = data[offset] >> 2;
    offset++;
    if (extended) {
      if (data.size() == offset)
        return false;
      fdiff |= (data[offset] << 6);
      offset++;
    }
    descriptor->AddFrameDependencyDiff(fdiff);
    if (!more) {
      return data.size() == offset;
    }
  }
}

size_t RtpGenericFrameDescriptorExtension::ValueSize(
    const RtpGenericFrameDescriptor& descriptor) {
  if (!descriptor.FirstPacketInSubFrame())
    return 1;

  size_t size = 4;
  for (uint16_t fdiff : descriptor.FrameDependenciesDiffs()) {
    size += (fdiff >= (1 << 6)) ? 2 : 1;
  }
  return size;
}

bool RtpGenericFrameDescriptorExtension::Write(
    uint8_t* data,
    const RtpGenericFrameDescriptor& descriptor) {
  data[0] = (descriptor.FirstPacketInSubFrame() ? kFlagBeginOfSubframe : 0) |
            (descriptor.LastPacketInSubFrame() ? kFlagEndOfSubframe : 0) |
            (descriptor.FirstSubFrameInFrame() ? kFlagFirstSubframe : 0) |
            (descriptor.LastSubFrameInFrame() ? kFlagLastSubframe : 0);
  if (!descriptor.FirstPacketInSubFrame()) {
    return true;
  }
  data[0] |=
      (descriptor.FrameDependenciesDiffs().empty() ? 0 : kFlagDependencies) |
      descriptor.TemporalLayer();
  data[1] = descriptor.SpatialLayersBitmask();
  uint16_t frame_id = descriptor.FrameId();
  data[2] = frame_id & 0xff;
  data[3] = frame_id >> 8;
  rtc::ArrayView<const uint16_t> fdiffs = descriptor.FrameDependenciesDiffs();
  size_t offset = 4;
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
