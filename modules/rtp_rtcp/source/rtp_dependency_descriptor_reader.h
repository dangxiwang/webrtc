/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef MODULES_RTP_RTCP_SOURCE_RTP_DEPENDENCY_DESCRIPTOR_READER_H_
#define MODULES_RTP_RTCP_SOURCE_RTP_DEPENDENCY_DESCRIPTOR_READER_H_

#include <cstdint>

#include "api/array_view.h"
#include "common_video/generic_frame_descriptor/generic_frame_info.h"
#include "rtc_base/bit_buffer.h"

namespace webrtc {
// Keeps and updates state required to deserialize DependencyDescriptor
// rtp header extension.
class RtpDependencyDescriptorReader {
 public:
  RtpDependencyDescriptorReader() = default;
  RtpDependencyDescriptorReader(const RtpDependencyDescriptorReader&) = delete;
  RtpDependencyDescriptorReader& operator=(
      const RtpDependencyDescriptorReader&) = delete;

  // Parses the dependency descriptor. Returns false on failure.
  // value of |descriptor| might be in inconsistent state if parsed failed.
  bool Parse(rtc::ArrayView<const uint8_t> raw_data,
             const FrameDependencyStructure* latest_structure,
             DependencyDescriptor* descriptor);

 private:
  bool ReadMandatoryFields(rtc::BitBuffer* buffer,
                           DependencyDescriptor* descriptor);

  bool ReadExtendedFields(rtc::BitBuffer* buffer,
                          DependencyDescriptor* descriptor);
  bool ReadFrameDependencyDefinition(rtc::BitBuffer* buffer,
                                     DependencyDescriptor* descriptor);

  // Values that are needed while reading the descriptor, but can be discarded
  // when reading is complete.
  int frame_dependency_template_id_ = 0;
  bool custom_dtis_flag_ = false;
  bool custom_fdiffs_flag_ = false;
  bool custom_chains_flag_ = false;
  const FrameDependencyStructure* structure_ = nullptr;
};

}  // namespace webrtc

#endif  // MODULES_RTP_RTCP_SOURCE_RTP_DEPENDENCY_DESCRIPTOR_READER_H_
