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
    std::unique_ptr<RtpPacketReceived> packet,
    const RtpVideoDepacketizer::FrameBoundaries& boundaries,
    int times_nacked) {
  RTC_DCHECK(packet);
  InsertResult result;
  int64_t sequence_number = unwrapper_.Unwrap(packet->SequenceNumber());
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

  if (boundaries.ends_frame.value_or(packet->Marker())) {
    slot->SetEof();
  }

  slot->SetPacket(std::move(packet), times_nacked);

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

bool VideoRtpPacketBuffer::PotentialNewFrame(PacketSlot* packet) const {
  return true;
}

std::vector<VideoRtpPacketBuffer::RtpFrame> VideoRtpPacketBuffer::FindFrames(
    PacketSlot* new_packet) {
  std::vector<VideoRtpPacketBuffer::RtpFrame> frames;
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

      VideoRtpPacketBuffer::RtpFrame frame;
      frame.max_times_nacked = -1;
      for (int64_t seq_num = first_sequence_number;
           seq_num <= last_sequence_number; ++seq_num) {
        PacketSlot* slot = FindUsedSlot(seq_num);
        frame.max_times_nacked =
            std::max(frame.max_times_nacked, slot->times_nacked());
        frame.rtp_packets.push_back(slot->ExtractRtpPacket());
      }
      frames.push_back(std::move(frame));
      ClearInterval(first_sequence_number, last_sequence_number);
    }
  }
  return frames;
}

}  // namespace webrtc
