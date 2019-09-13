/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "modules/rtp_rtcp/source/rtp_dependency_descriptor_reader.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "rtc_base/bit_buffer.h"
#include "rtc_base/checks.h"

namespace webrtc {
namespace {

constexpr int kMaxTemporalId = 7;
constexpr int kMaxSpatialId = 3;
constexpr int kMaxTemplates = 63;
constexpr int kMaxTemplateId = kMaxTemplates - 1;
constexpr int kExtendedFieldsIndicator = kMaxTemplates;

bool ReadFrameDtis(rtc::BitBuffer* buffer,
                   int num_decode_targets,
                   FrameDependencyTemplate* frame) {
  if (static_cast<int>(buffer->RemainingBitCount()) < num_decode_targets * 2) {
    return false;
  }
  frame->decode_target_indications.resize(num_decode_targets);
  for (int i = 0; i < num_decode_targets; ++i) {
    uint32_t dti;
    bool ok = buffer->ReadBits(&dti, 2);
    RTC_DCHECK(ok);
    frame->decode_target_indications[i] =
        static_cast<DecodeTargetIndication>(dti);
  }
  return true;
}

bool ReadFrameFdiffs(rtc::BitBuffer* buffer, FrameDependencyTemplate* frame) {
  bool ok = true;
  frame->frame_diffs.clear();
  uint32_t next_fdiff_size;
  ok = ok && buffer->ReadBits(&next_fdiff_size, 2);
  while (ok && next_fdiff_size > 0) {
    uint32_t fdiff_minus_one = 0;
    ok = ok && buffer->ReadBits(&fdiff_minus_one, 4 * next_fdiff_size);
    frame->frame_diffs.push_back(fdiff_minus_one + 1);
    ok = ok && buffer->ReadBits(&next_fdiff_size, 2);
  }
  return ok;
}

bool ReadTemplateLayers(rtc::BitBuffer* buffer,
                        std::vector<FrameDependencyTemplate>* templates) {
  RTC_DCHECK(templates);
  RTC_DCHECK(templates->empty());
  int temporal_id = 0;
  int spatial_id = 0;
  while (templates->size() < kMaxTemplates) {
    templates->emplace_back();
    FrameDependencyTemplate& last_template = templates->back();
    last_template.temporal_id = temporal_id;
    last_template.spatial_id = spatial_id;

    uint32_t next_layer_idc;
    if (!buffer->ReadBits(&next_layer_idc, 2))
      return false;
    switch (next_layer_idc) {
      case 0:
        break;
      case 1:
        temporal_id++;
        if (temporal_id > kMaxTemporalId)
          return false;
        break;
      case 2:
        temporal_id = 0;
        spatial_id++;
        if (spatial_id > kMaxSpatialId)
          return false;
        break;
      case 3:
        return true;
    }
  }

  return false;
}

bool ReadTemplateDtis(rtc::BitBuffer* buffer,
                      FrameDependencyStructure* structure) {
  bool ok = true;
  for (FrameDependencyTemplate& current_template : structure->templates) {
    ok = ok && ReadFrameDtis(buffer, structure->num_decode_targets,
                             &current_template);
  }
  return ok;
}

bool ReadTemplateFdiffs(rtc::BitBuffer* buffer,
                        FrameDependencyStructure* structure) {
  bool ok = true;
  for (FrameDependencyTemplate& current_template : structure->templates) {
    uint32_t fdiff_follows;
    ok = ok && buffer->ReadBits(&fdiff_follows, 1);
    while (ok && fdiff_follows) {
      uint32_t fdiff_minus_one = 0;
      ok = ok && buffer->ReadBits(&fdiff_minus_one, 4);
      current_template.frame_diffs.push_back(fdiff_minus_one + 1);
      ok = ok && buffer->ReadBits(&fdiff_follows, 1);
    }
  }
  return ok;
}

bool ReadTemplateChains(rtc::BitBuffer* buffer,
                        FrameDependencyStructure* structure) {
  bool ok = true;
  uint32_t value = 0;
  ok =
      ok && buffer->ReadNonSymmetric(&value, structure->num_decode_targets + 1);
  structure->num_chains = value;
  if (structure->num_chains == 0) {
    return ok;
  }
  RTC_DCHECK(structure->decode_target_protected_by_chain.empty());
  for (int i = 0; i < structure->num_decode_targets; ++i) {
    ok = ok && buffer->ReadNonSymmetric(&value, structure->num_chains + 1);
    structure->decode_target_protected_by_chain.push_back(value);
  }
  for (FrameDependencyTemplate& frame_template : structure->templates) {
    RTC_DCHECK(frame_template.chain_diffs.empty());
    for (int chain_id = 0; chain_id < structure->num_chains; ++chain_id) {
      ok = ok && buffer->ReadBits(&value, 4);
      frame_template.chain_diffs.push_back(value);
    }
  }
  return ok;
}

bool ReadResolutions(rtc::BitBuffer* buffer,
                     FrameDependencyStructure* structure) {
  // The way templates are bitpacked, they are always ordered by spatial_id.
  int spatial_layers = structure->templates.back().spatial_id + 1;
  if (static_cast<int>(buffer->RemainingBitCount()) < spatial_layers * 32) {
    return false;
  }
  structure->resolutions.reserve(spatial_layers);
  for (int sid = 0; sid < spatial_layers; ++sid) {
    uint16_t width_minus_1;
    uint16_t height_minus_1;
    bool ok = buffer->ReadUInt16(&width_minus_1) &&
              buffer->ReadUInt16(&height_minus_1);
    // Do not expect read failure other than remaining buffer size it too small.
    // Remaining buffer size is checked in bulk at the start of the function.
    RTC_DCHECK(ok);
    structure->resolutions.emplace_back(width_minus_1 + 1, height_minus_1 + 1);
  }
  return true;
}

std::unique_ptr<FrameDependencyStructure> ReadTemplateDependencyStructure(
    rtc::BitBuffer* buffer) {
  bool ok = true;
  auto structure = absl::make_unique<FrameDependencyStructure>();
  uint32_t template_id_offset = 0;
  ok = ok && buffer->ReadBits(&template_id_offset, 6);
  structure->structure_id = template_id_offset;

  uint32_t num_decode_targets_minus_1 = 0;
  ok = ok && buffer->ReadBits(&num_decode_targets_minus_1, 5);
  structure->num_decode_targets = num_decode_targets_minus_1 + 1;

  ok = ok && ReadTemplateLayers(buffer, &structure->templates);
  ok = ok && ReadTemplateDtis(buffer, structure.get());
  ok = ok && ReadTemplateFdiffs(buffer, structure.get());

  ok = ok && ReadTemplateChains(buffer, structure.get());
  uint32_t has_resolutions = 0;
  ok = ok && buffer->ReadBits(&has_resolutions, 1);
  if (has_resolutions) {
    ok = ok && ReadResolutions(buffer, structure.get());
  }
  return ok ? std::move(structure) : nullptr;
}

}  // namespace

bool RtpDependencyDescriptorReader::Parse(
    rtc::ArrayView<const uint8_t> raw_data,
    const FrameDependencyStructure* latest_structure,
    DependencyDescriptor* descriptor) {
  RTC_DCHECK(descriptor);
  rtc::BitBuffer bit_reader(raw_data.data(), raw_data.size());

  bool ok = true;
  ok = ok && ReadMandatoryFields(&bit_reader, descriptor);
  if (frame_dependency_template_id_ == kExtendedFieldsIndicator) {
    ok = ok && ReadExtendedFields(&bit_reader, descriptor);
  } else {
    descriptor->attached_structure = nullptr;
    custom_dtis_flag_ = false;
    custom_fdiffs_flag_ = false;
    custom_chains_flag_ = false;
  }
  structure_ = descriptor->attached_structure
                   ? descriptor->attached_structure.get()
                   : latest_structure;
  ok = ok && structure_ != nullptr;
  ok = ok && ReadFrameDependencyDefinition(&bit_reader, descriptor);
  return ok;
}

bool RtpDependencyDescriptorReader::ReadMandatoryFields(
    rtc::BitBuffer* buffer,
    DependencyDescriptor* descriptor) {
  bool ok = true;
  uint8_t flags_and_template_id = 0;
  ok = ok && buffer->ReadUInt8(&flags_and_template_id);
  descriptor->first_packet_in_frame = (flags_and_template_id & 0b10000000) != 0;
  descriptor->last_packet_in_frame = (flags_and_template_id & 0b01000000) != 0;
  frame_dependency_template_id_ = flags_and_template_id & 0b00111111;
  uint16_t frame_number = 0;
  ok = ok && buffer->ReadUInt16(&frame_number);
  descriptor->frame_number = frame_number;
  return ok;
}

bool RtpDependencyDescriptorReader::ReadExtendedFields(
    rtc::BitBuffer* buffer,
    DependencyDescriptor* descriptor) {
  bool ok = true;
  uint32_t value = 0;
  // 6 + 1 + 1 + 1 + 1 bits.
  ok = ok && buffer->ReadBits(&value, 10);
  // frame_dependency_template_id >> 4;
  ok = ok && frame_dependency_template_id_ != kExtendedFieldsIndicator;
  bool template_dependency_structure_present_flag = (value & 0b1000) != 0;
  custom_dtis_flag_ = (value & 0b0100) != 0;
  custom_fdiffs_flag_ = (value & 0b0010) != 0;
  custom_chains_flag_ = (value & 0b0001) != 0;
  if (template_dependency_structure_present_flag) {
    descriptor->attached_structure = ReadTemplateDependencyStructure(buffer);
    ok = ok && descriptor->attached_structure != nullptr;
  } else {
    descriptor->attached_structure = nullptr;
  }
  return ok;
}

bool RtpDependencyDescriptorReader::ReadFrameDependencyDefinition(
    rtc::BitBuffer* buffer,
    DependencyDescriptor* descriptor) {
  bool ok = true;
  size_t template_index = (frame_dependency_template_id_ +
                           (kMaxTemplateId + 1) - structure_->structure_id) %
                          (kMaxTemplateId + 1);

  if (template_index >= structure_->templates.size()) {
    return false;
  }

  // Copy all the fields from the matching template
  descriptor->frame_dependencies = structure_->templates[template_index];

  if (custom_dtis_flag_) {
    ok = ok && ReadFrameDtis(buffer, structure_->num_decode_targets,
                             &descriptor->frame_dependencies);
  }
  if (custom_fdiffs_flag_) {
    ok = ok && ReadFrameFdiffs(buffer, &descriptor->frame_dependencies);
  }
  if (custom_chains_flag_) {
    for (int i = 0; i < structure_->num_chains; ++i) {
      uint32_t chain_diff = 0;
      ok = ok && buffer->ReadBits(&chain_diff, 8);
      descriptor->frame_dependencies.chain_diffs[i] = chain_diff;
    }
  }
  if (structure_->resolutions.empty()) {
    descriptor->resolution = absl::nullopt;
  } else {
    // Format guarantees that if there were resolutions in the last structure,
    // then each spatial layer got one.
    RTC_DCHECK_LE(descriptor->frame_dependencies.spatial_id,
                  structure_->resolutions.size());
    descriptor->resolution =
        structure_->resolutions[descriptor->frame_dependencies.spatial_id];
  }

  return ok;
}
}  // namespace webrtc
