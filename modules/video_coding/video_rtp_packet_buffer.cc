/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/video_rtp_packet_buffer.h"

#include <string.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <utility>

#include "absl/types/variant.h"
#include "api/video/encoded_frame.h"
#include "common_video/h264/h264_common.h"
#include "modules/rtp_rtcp/source/rtp_generic_frame_descriptor.h"
#include "modules/rtp_rtcp/source/rtp_generic_frame_descriptor_extension.h"
#include "modules/rtp_rtcp/source/rtp_header_extensions.h"
#include "modules/rtp_rtcp/source/rtp_video_header.h"
#include "modules/video_coding/codecs/h264/include/h264_globals.h"
#include "modules/video_coding/frame_object.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/mod_ops.h"
#include "system_wrappers/include/clock.h"
#include "system_wrappers/include/field_trial.h"

namespace webrtc {

VideoRtpPacketBuffer::VideoRtpPacketBuffer(size_t max_buffer_size,
                                           size_t start_buffer_size)
    : max_size_(max_buffer_size), buffer_(start_buffer_size) {
  RTC_DCHECK_LE(start_buffer_size, max_buffer_size);
}

VideoRtpPacketBuffer::~VideoRtpPacketBuffer() = default;

void VideoRtpPacketBuffer::ExpandBuffer() {
  std::vector<absl::optional<PacketSlot>> new_buffer(2 * buffer_.size());
  for (absl::optional<PacketSlot>& entry : buffer_) {
    if (entry == absl::nullopt)
      continue;
    int new_index = entry->SequenceNumber() % new_buffer.size();
    RTC_CHECK(new_buffer[new_index] == absl::nullopt);
    new_buffer[new_index].emplace(*std::move(entry));
  }
  buffer_ = std::move(new_buffer);
}

VideoRtpPacketBuffer::PacketSlot* VideoRtpPacketBuffer::FindUsedSlot(
    int64_t sequence_number) {
  absl::optional<PacketSlot>& entry = buffer_[sequence_number % buffer_.size()];
  if (entry.has_value() && entry->SequenceNumber() == sequence_number) {
    return &*entry;
  }
  return nullptr;
}

VideoRtpPacketBuffer::InsertResult VideoRtpPacketBuffer::InsertPacket(
    const RtpPacketReceived& packet,
    const RtpVideoDepacketizer::FrameBoundaries& boundaries,
    int times_nacked,
    int64_t packet_receive_time_ms,
    int64_t frame_ntp_time_ms) {
  // RTC_DCHECK_EQ(GetFrameBoundaries(packet), boundaries);
  InsertResult result;
  int64_t sequence_number = unwrapper_.Unwrap(packet.SequenceNumber());
  if (sequence_number < cleared_up_to_) {
    // Too old packet.
    return {};
  }
  // Find free slot for the new packet.
  PacketSlot* slot;
  while (true) {
    absl::optional<PacketSlot>& entry =
        buffer_[sequence_number % buffer_.size()];
    if (!entry.has_value()) {
      entry.emplace(sequence_number);
      slot = &*entry;
      break;
    }
    if (entry->SequenceNumber() == sequence_number) {
      // Duplicate packet.
      return result;
    }
    // slot is used by another sequence number.
    // Check if buffer size can be increased.
    if (buffer_.size() >= max_size_) {
      Clear();
      result.buffer_cleared = true;
      return result;
    }
    // Expand the buffer and try again.
    ExpandBuffer();
  }

  if (boundaries.begins_frame.value_or(false)) {
    slot->SetBof();
  }

  if (boundaries.ends_frame.value_or(packet.Marker())) {
    slot->SetEof();
  }

  slot->SetPacket(packet, times_nacked, frame_ntp_time_ms,
                  packet_receive_time_ms);

  result.frames = FindFrames(slot);
  return result;
}

VideoRtpPacketBuffer::InsertResult VideoRtpPacketBuffer::InsertPadding(
    uint16_t seq_num) {
  InsertResult result;
  int64_t sequence_number = unwrapper_.Unwrap(seq_num);
  if (sequence_number < cleared_up_to_) {
    // Too old packet.
    return {};
  }
  // Find free slot for the new packet.
  PacketSlot* slot;
  while (true) {
    absl::optional<PacketSlot>& entry =
        buffer_[sequence_number % buffer_.size()];
    if (!entry.has_value()) {
      entry.emplace(sequence_number);
      slot = &*entry;
      break;
    }
    if (entry->SequenceNumber() == sequence_number) {
      // Duplicate packet.
      return result;
    }
    // slot is used by another sequence number.
    // Check if buffer size can be increased.
    if (buffer_.size() >= max_size_) {
      Clear();
      result.buffer_cleared = true;
      return result;
    }
    // Expand the buffer and try again.
    ExpandBuffer();
  }

  slot->SetIsPadding();
  if (auto* next_slot = FindUsedSlot(sequence_number + 1)) {
    result.frames = FindFrames(next_slot);
  }
  return result;
}

absl::optional<RtpVideoDepacketizer::FrameBoundaries>
VideoRtpPacketBuffer::GetFrameBoundaries(const RtpPacketReceived& rtp_packet) {
  RtpVideoDepacketizer::FrameBoundaries boundaries;

  RtpGenericFrameDescriptor gfd;
  if (rtp_packet.GetExtension<RtpGenericFrameDescriptorExtension01>(&gfd) ||
      rtp_packet.GetExtension<RtpGenericFrameDescriptorExtension00>(&gfd)) {
    boundaries.begins_frame = gfd.FirstPacketInSubFrame();
    boundaries.ends_frame = gfd.LastPacketInSubFrame();
    boundaries.begins_keyframe =
        boundaries.begins_frame && gfd.FrameDependenciesDiffs().empty();
    return boundaries;
  }

  if (rtp_packet.payload_size() == 0) {
    boundaries.begins_frame = true;
    boundaries.ends_frame = true;
    boundaries.begins_keyframe = false;
    return boundaries;
  }

  auto depacketize_it = depacketizers_.find(rtp_packet.PayloadType());
  if (depacketize_it == depacketizers_.end()) {
    // Treat unknown payload same as empty payload (padding).
    boundaries.begins_frame = true;
    boundaries.ends_frame = true;
    boundaries.begins_keyframe = false;
    return boundaries;
  }

  return depacketize_it->second->GetFrameBoundaries(rtp_packet.payload());
}

void VideoRtpPacketBuffer::UpdateVideoHeader(
    const RtpPacketReceived& first_packet,
    const RtpPacketReceived& last_packet,
    RTPVideoHeader* video_header) {
  RtpGenericFrameDescriptor gfd;
  if (first_packet.GetExtension<RtpGenericFrameDescriptorExtension01>(&gfd) ||
      first_packet.GetExtension<RtpGenericFrameDescriptorExtension00>(&gfd)) {
    video_header->generic.emplace();
    video_header->generic->frame_id = gfd.FrameId();  // may be unwrap.
    video_header->width = gfd.Width();
    // ...
  }

  last_packet.GetExtension<VideoContentTypeExtension>(
      &video_header->content_type);
  last_packet.GetExtension<VideoTimingExtension>(&video_header->video_timing);
  //... all other rtp header extensions common for all payload types.
}

bool VideoRtpPacketBuffer::PotentialNewFrame(PacketSlot* packet) const {
  return true;
}

std::vector<std::unique_ptr<video_coding::RtpFrameObject>>
VideoRtpPacketBuffer::FindFrames(PacketSlot* new_packet) {
  std::vector<std::unique_ptr<video_coding::RtpFrameObject>> frames;
  for (PacketSlot* packet = new_packet; PotentialNewFrame(packet);
       packet = FindUsedSlot(packet->SequenceNumber() + 1)) {
    //    buffer_[index].continuous = true;// copy bof for previous packet
    //    instead

    if (packet->IsPadding()) {
      continue;
    }
    if (packet->frame_end()) {
      const int64_t last_sequence_number = packet->SequenceNumber();
      const int64_t first_sequence_number = *packet->continuous();

      if (auto frame =
              AssembleFrame(first_sequence_number, last_sequence_number)) {
        frames.push_back(std::move(frame));
      }

      ClearInterval(first_sequence_number, last_sequence_number);
    }
  }
  return frames;
}

std::unique_ptr<video_coding::RtpFrameObject>
VideoRtpPacketBuffer::AssembleFrame(int64_t first_sequence_number,
                                    int64_t last_sequence_number) {
  int64_t num_packets = last_sequence_number - first_sequence_number + 1;
  RTC_CHECK_GT(num_packets, 0);
  auto* first_packet_slot = FindUsedSlot(first_sequence_number);
  const RtpPacketReceived& first_packet = first_packet_slot->RtpPacket();
  const RtpPacketReceived& last_packet =
      FindUsedSlot(last_sequence_number)->RtpPacket();

  int payload_id = first_packet.PayloadType();
  auto depacketize_it = depacketizers_.find(payload_id);
  if (depacketize_it == depacketizers_.end()) {
    return nullptr;
  }
  auto get_packet =
      [this, first_sequence_number](size_t index) -> const RtpPacketReceived& {
    return FindUsedSlot(first_sequence_number + index)->RtpPacket();
  };
  auto frame = depacketize_it->second->AssembleFrame(num_packets, get_packet);
  if (!frame) {
    return nullptr;
  }
  UpdateVideoHeader(first_packet, last_packet, &frame->video_header);

  int max_times_nacked = -1;
  int64_t min_recv_time = std::numeric_limits<int64_t>::max();
  int64_t max_recv_time = std::numeric_limits<int64_t>::min();
  RtpPacketInfos::vector_type packet_infos;
  for (int64_t seq_num = first_sequence_number; seq_num <= last_sequence_number;
       ++seq_num) {
    PacketSlot* slot = FindUsedSlot(seq_num);
    max_times_nacked = std::max(max_times_nacked, slot->times_nacked());
    min_recv_time = std::min(min_recv_time, slot->recv_time_ms());
    max_recv_time = std::max(max_recv_time, slot->recv_time_ms());
    const RtpPacketReceived& rtp_packet = slot->RtpPacket();
    packet_infos.emplace_back(
        rtp_packet.Ssrc(), rtp_packet.Csrcs(), rtp_packet.Timestamp(),
        /*audio_level=*/absl::nullopt,
        rtp_packet.GetExtension<AbsoluteCaptureTimeExtension>(),
        slot->recv_time_ms());
  }

  absl::optional<RtpGenericFrameDescriptor> generic_descriptor;
  if ((generic_descriptor =
           first_packet.GetExtension<RtpGenericFrameDescriptorExtension00>())) {
    generic_descriptor->SetByteRepresentation(
        first_packet.GetRawExtension<RtpGenericFrameDescriptorExtension00>());
  }

  return std::make_unique<video_coding::RtpFrameObject>(
      first_sequence_number,                    //
      last_sequence_number,                     //
      last_packet.Marker(),                     //
      max_times_nacked,                         //
      min_recv_time,                            //
      max_recv_time,                            //
      first_packet.Timestamp(),                 //
      first_packet_slot->ntp_time_ms(),         //
      frame->video_header.video_timing,         //
      first_packet.PayloadType(),               //
      frame->video_header.codec,                //
      frame->video_header.rotation,             //
      frame->video_header.content_type,         //
      frame->video_header,                      //
      frame->video_header.color_space,          //
      generic_descriptor,                       //
      RtpPacketInfos(std::move(packet_infos)),  //
      std::move(frame->bitstream));
}

}  // namespace webrtc
