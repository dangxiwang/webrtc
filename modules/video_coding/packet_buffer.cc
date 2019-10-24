/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/packet_buffer.h"

#include <string.h>

#include <algorithm>
#include <cstdint>
#include <utility>

#include "absl/types/variant.h"
#include "api/video/encoded_frame.h"
#include "common_video/h264/h264_common.h"
#include "modules/rtp_rtcp/source/rtp_video_header.h"
#include "modules/video_coding/codecs/h264/include/h264_globals.h"
#include "modules/video_coding/frame_object.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/mod_ops.h"
#include "system_wrappers/include/clock.h"
#include "system_wrappers/include/field_trial.h"

namespace webrtc {
namespace video_coding {

PacketBuffer::PacketBuffer(Clock* clock,
                           size_t start_buffer_size,
                           size_t max_buffer_size)
    : clock_(clock),
      max_size_(max_buffer_size),
      first_seq_num_(0),
      first_packet_received_(false),
      is_cleared_to_first_seq_num_(false),
      buffer_(start_buffer_size),
      unique_frames_seen_(0),
      sps_pps_idr_is_h264_keyframe_(
          field_trial::IsEnabled("WebRTC-SpsPpsIdrIsH264Keyframe")) {
  RTC_DCHECK_LE(start_buffer_size, max_buffer_size);
  // Buffer size must always be a power of 2.
  RTC_DCHECK((start_buffer_size & (start_buffer_size - 1)) == 0);
  RTC_DCHECK((max_buffer_size & (max_buffer_size - 1)) == 0);
}

PacketBuffer::~PacketBuffer() {
  Clear();
}

PacketBuffer::InsertResult PacketBuffer::InsertPacket(VCMPacket* packet) {
  PacketBuffer::InsertResult result;
  rtc::CritScope lock(&crit_);
  OnTimestampReceived(packet->timestamp);

  uint16_t seq_num = packet->seqNum;
  size_t index = seq_num % buffer_.size();

  if (!first_packet_received_) {
    first_seq_num_ = seq_num;
    first_packet_received_ = true;
  } else if (AheadOf(first_seq_num_, seq_num)) {
    // If we have explicitly cleared past this packet then it's old,
    // don't insert it, just silently ignore it.
    if (is_cleared_to_first_seq_num_) {
      delete[] packet->dataPtr;
      packet->dataPtr = nullptr;
      return result;
    }

    first_seq_num_ = seq_num;
  }

  if (buffer_[index].used) {
    // Duplicate packet, just delete the payload.
    if (buffer_[index].seq_num() == packet->seqNum) {
      delete[] packet->dataPtr;
      packet->dataPtr = nullptr;
      return result;
    }

    // The packet buffer is full, try to expand the buffer.
    while (ExpandBufferSize() && buffer_[seq_num % buffer_.size()].used) {
    }
    index = seq_num % buffer_.size();

    // Packet buffer is still full since we were unable to expand the buffer.
    if (buffer_[index].used) {
      // Clear the buffer, delete payload, and return false to signal that a
      // new keyframe is needed.
      RTC_LOG(LS_WARNING) << "Clear PacketBuffer and request key frame.";
      Clear();
      delete[] packet->dataPtr;
      packet->dataPtr = nullptr;
      result.buffer_cleared = true;
      return result;
    }
  }

  StoredPacket& new_entry = buffer_[index];
  new_entry.continuous = false;
  new_entry.used = true;
  new_entry.data = *packet;
  packet->dataPtr = nullptr;

  UpdateMissingPackets(packet->seqNum);

  int64_t now_ms = clock_->TimeInMilliseconds();
  last_received_packet_ms_ = now_ms;
  if (packet->video_header.frame_type == VideoFrameType::kVideoFrameKey)
    last_received_keyframe_packet_ms_ = now_ms;

  result.frames = FindFrames(seq_num);
  return result;
}

void PacketBuffer::ClearTo(uint16_t seq_num) {
  rtc::CritScope lock(&crit_);
  // We have already cleared past this sequence number, no need to do anything.
  if (is_cleared_to_first_seq_num_ &&
      AheadOf<uint16_t>(first_seq_num_, seq_num)) {
    return;
  }

  // If the packet buffer was cleared between a frame was created and returned.
  if (!first_packet_received_)
    return;

  // Avoid iterating over the buffer more than once by capping the number of
  // iterations to the |size_| of the buffer.
  ++seq_num;
  size_t diff = ForwardDiff<uint16_t>(first_seq_num_, seq_num);
  size_t iterations = std::min(diff, buffer_.size());
  for (size_t i = 0; i < iterations; ++i) {
    size_t index = first_seq_num_ % buffer_.size();
    if (AheadOf<uint16_t>(seq_num, buffer_[index].seq_num())) {
      delete[] buffer_[index].data.dataPtr;
      buffer_[index].data.dataPtr = nullptr;
      buffer_[index].used = false;
    }
    ++first_seq_num_;
  }

  // If |diff| is larger than |iterations| it means that we don't increment
  // |first_seq_num_| until we reach |seq_num|, so we set it here.
  first_seq_num_ = seq_num;

  is_cleared_to_first_seq_num_ = true;
  auto clear_to_it = missing_packets_.upper_bound(seq_num);
  if (clear_to_it != missing_packets_.begin()) {
    --clear_to_it;
    missing_packets_.erase(missing_packets_.begin(), clear_to_it);
  }
}

void PacketBuffer::ClearInterval(uint16_t start_seq_num,
                                 uint16_t stop_seq_num) {
  size_t iterations = ForwardDiff<uint16_t>(start_seq_num, stop_seq_num + 1);
  RTC_DCHECK_LE(iterations, buffer_.size());
  uint16_t seq_num = start_seq_num;
  for (size_t i = 0; i < iterations; ++i) {
    size_t index = seq_num % buffer_.size();
    RTC_DCHECK_EQ(buffer_[index].seq_num(), seq_num);
    delete[] buffer_[index].data.dataPtr;
    buffer_[index].data.dataPtr = nullptr;
    buffer_[index].used = false;

    ++seq_num;
  }
}

void PacketBuffer::Clear() {
  rtc::CritScope lock(&crit_);
  for (StoredPacket& entry : buffer_) {
    delete[] entry.data.dataPtr;
    entry.data.dataPtr = nullptr;
    entry.used = false;
  }

  first_packet_received_ = false;
  is_cleared_to_first_seq_num_ = false;
  last_received_packet_ms_.reset();
  last_received_keyframe_packet_ms_.reset();
  newest_inserted_seq_num_.reset();
  missing_packets_.clear();
}

PacketBuffer::InsertResult PacketBuffer::InsertPadding(uint16_t seq_num) {
  PacketBuffer::InsertResult result;
  rtc::CritScope lock(&crit_);
  UpdateMissingPackets(seq_num);
  result.frames = FindFrames(static_cast<uint16_t>(seq_num + 1));
  return result;
}

absl::optional<int64_t> PacketBuffer::LastReceivedPacketMs() const {
  rtc::CritScope lock(&crit_);
  return last_received_packet_ms_;
}

absl::optional<int64_t> PacketBuffer::LastReceivedKeyframePacketMs() const {
  rtc::CritScope lock(&crit_);
  return last_received_keyframe_packet_ms_;
}

int PacketBuffer::GetUniqueFramesSeen() const {
  rtc::CritScope lock(&crit_);
  return unique_frames_seen_;
}

bool PacketBuffer::ExpandBufferSize() {
  if (buffer_.size() == max_size_) {
    RTC_LOG(LS_WARNING) << "PacketBuffer is already at max size (" << max_size_
                        << "), failed to increase size.";
    return false;
  }

  size_t new_size = std::min(max_size_, 2 * buffer_.size());
  std::vector<StoredPacket> new_buffer(new_size);
  for (StoredPacket& entry : buffer_) {
    if (entry.used) {
      new_buffer[entry.seq_num() % new_size] = entry;
    }
  }
  buffer_ = std::move(new_buffer);
  RTC_LOG(LS_INFO) << "PacketBuffer size expanded to " << new_size;
  return true;
}

bool PacketBuffer::PotentialNewFrame(uint16_t seq_num) const {
  size_t index = seq_num % buffer_.size();
  int prev_index = index > 0 ? index - 1 : buffer_.size() - 1;
  const StoredPacket& entry = buffer_[index];
  const StoredPacket& prev_entry = buffer_[prev_index];

  if (!entry.used)
    return false;
  if (entry.seq_num() != seq_num)
    return false;
  if (entry.frame_begin())
    return true;
  if (!prev_entry.used)
    return false;
  if (prev_entry.seq_num() != static_cast<uint16_t>(entry.seq_num() - 1))
    return false;
  if (prev_entry.data.timestamp != entry.data.timestamp)
    return false;
  if (prev_entry.continuous)
    return true;

  return false;
}

std::vector<std::unique_ptr<RtpFrameObject>> PacketBuffer::FindFrames(
    uint16_t seq_num) {
  std::vector<std::unique_ptr<RtpFrameObject>> found_frames;
  for (size_t i = 0; i < buffer_.size() && PotentialNewFrame(seq_num); ++i) {
    size_t index = seq_num % buffer_.size();
    buffer_[index].continuous = true;

    // If all packets of the frame is continuous, find the first packet of the
    // frame and create an RtpFrameObject.
    if (buffer_[index].frame_end()) {
      uint16_t start_seq_num = seq_num;

      // Find the start index by searching backward until the packet with
      // the |frame_begin| flag is set.
      int start_index = index;
      size_t tested_packets = 0;
      int64_t frame_timestamp = buffer_[start_index].data.timestamp;

      // Identify H.264 keyframes by means of SPS, PPS, and IDR.
      bool is_h264 = buffer_[start_index].data.codec() == kVideoCodecH264;
      bool has_h264_sps = false;
      bool has_h264_pps = false;
      bool has_h264_idr = false;
      bool is_h264_keyframe = false;
      while (true) {
        ++tested_packets;

        if (!is_h264 && buffer_[start_index].frame_begin())
          break;

        if (is_h264) {
          const auto* h264_header = absl::get_if<RTPVideoHeaderH264>(
              &buffer_[start_index].data.video_header.video_type_header);
          if (!h264_header || h264_header->nalus_length >= kMaxNalusPerPacket)
            return found_frames;

          for (size_t j = 0; j < h264_header->nalus_length; ++j) {
            if (h264_header->nalus[j].type == H264::NaluType::kSps) {
              has_h264_sps = true;
            } else if (h264_header->nalus[j].type == H264::NaluType::kPps) {
              has_h264_pps = true;
            } else if (h264_header->nalus[j].type == H264::NaluType::kIdr) {
              has_h264_idr = true;
            }
          }
          is_h264_keyframe =
              sps_pps_idr_is_h264_keyframe_
                  ? (has_h264_idr && has_h264_sps && has_h264_pps)
                  : has_h264_idr;
        }

        if (tested_packets == buffer_.size())
          break;

        start_index = start_index > 0 ? start_index - 1 : buffer_.size() - 1;

        // In the case of H264 we don't have a frame_begin bit (yes,
        // |frame_begin| might be set to true but that is a lie). So instead
        // we traverese backwards as long as we have a previous packet and
        // the timestamp of that packet is the same as this one. This may cause
        // the PacketBuffer to hand out incomplete frames.
        // See: https://bugs.chromium.org/p/webrtc/issues/detail?id=7106
        if (is_h264 &&
            (!buffer_[start_index].used ||
             buffer_[start_index].data.timestamp != frame_timestamp)) {
          break;
        }

        --start_seq_num;
      }

      if (is_h264) {
        // With IPPP, if this is not a keyframe, make sure there are no gaps
        // in the packet sequence numbers up until this point.
        const uint8_t h264tid =
            buffer_[start_index].data.video_header.frame_marking.temporal_id;
        if (h264tid == kNoTemporalIdx && !is_h264_keyframe &&
            missing_packets_.upper_bound(start_seq_num) !=
                missing_packets_.begin()) {
          return found_frames;
        }
      }

      found_frames.push_back(AssembleFrame(start_seq_num, seq_num));

      missing_packets_.erase(missing_packets_.begin(),
                             missing_packets_.upper_bound(seq_num));
      ClearInterval(start_seq_num, seq_num);
    }
    ++seq_num;
  }
  return found_frames;
}

std::unique_ptr<RtpFrameObject> PacketBuffer::AssembleFrame(
    uint16_t first_seq_num,
    uint16_t last_seq_num) {
  const uint16_t end_seq_num = last_seq_num + 1;
  // use uint16_t for number of packets for defined uint16_t underflow.
  const uint16_t num_packets = end_seq_num - first_seq_num;
  RTC_DCHECK_LE(num_packets, buffer_.size());
  RTC_DCHECK_GT(num_packets, 0);
  VCMPacket& first_packet = buffer_[first_seq_num % buffer_.size()].data;
  const VCMPacket& last_packet = buffer_[last_seq_num % buffer_.size()].data;
  RTPVideoHeader& video_header = first_packet.video_header;
  bool is_h264 = first_packet.codec() == kVideoCodecH264;
  bool has_h264_sps = false;
  bool has_h264_pps = false;
  bool has_h264_idr = false;

  // Calculate size of the buffer and collect various stats.
  size_t frame_size = 0;
  RtpPacketInfos::vector_type packet_infos;
  packet_infos.reserve(num_packets);
  int max_nack_count = -1;
  int64_t min_recv_time = first_packet.packet_info.receive_time_ms();
  int64_t max_recv_time = first_packet.packet_info.receive_time_ms();
  for (uint16_t seq_num = first_seq_num; seq_num != end_seq_num; ++seq_num) {
    int index = seq_num % buffer_.size();
    RTC_DCHECK(buffer_[index].used);
    RTC_DCHECK_EQ(buffer_[index].seq_num(), seq_num);
    const VCMPacket& packet = buffer_[index].data;

    frame_size += packet.sizeBytes;
    packet_infos.push_back(packet.packet_info);
    max_nack_count = std::max(max_nack_count, packet.timesNacked);
    min_recv_time =
        std::min(min_recv_time, packet.packet_info.receive_time_ms());
    max_recv_time =
        std::max(max_recv_time, packet.packet_info.receive_time_ms());

    if (is_h264) {
      const auto& h264_header =
          absl::get<RTPVideoHeaderH264>(packet.video_header.video_type_header);
      RTC_DCHECK_LE(h264_header.nalus_length, kMaxNalusPerPacket);

      for (size_t i = 0; i < h264_header.nalus_length; ++i) {
        if (h264_header.nalus[i].type == H264::NaluType::kSps) {
          has_h264_sps = true;
        } else if (h264_header.nalus[i].type == H264::NaluType::kPps) {
          has_h264_pps = true;
        } else if (h264_header.nalus[i].type == H264::NaluType::kIdr) {
          has_h264_idr = true;
        }
      }
      // Store the resolution of the packet with smallest index and valid
      // resolution; typically its IDR or SPS packet; there may be packet
      // preceeding this packet, IDR's resolution will be applied to them.
      if ((video_header.width == 0 || video_header.height == 0) &&
          packet.width() > 0 && packet.height() > 0) {
        video_header.width = packet.width();
        video_header.height = packet.height();
      }
    }
  }

  if (is_h264) {
    // Identify H.264 keyframes by means of SPS, PPS, and IDR.
    bool is_h264_keyframe = sps_pps_idr_is_h264_keyframe_
                                ? (has_h264_idr && has_h264_sps && has_h264_pps)
                                : has_h264_idr;
    video_header.frame_type = is_h264_keyframe
                                  ? VideoFrameType::kVideoFrameKey
                                  : VideoFrameType::kVideoFrameDelta;

    // Warn if this is an unsafe frame.
    if (has_h264_idr && (!has_h264_sps || !has_h264_pps)) {
      RTC_LOG(LS_WARNING) << "Received H.264-IDR frame "
                          << "(SPS: " << has_h264_sps
                          << ", PPS: " << has_h264_pps << "). Treating as "
                          << (is_h264_keyframe ? "delta" : "key")
                          << " frame since WebRTC-SpsPpsIdrIsH264Keyframe is "
                          << (sps_pps_idr_is_h264_keyframe_ ? "enabled."
                                                            : "disabled");
    }
  }

  // Assemble the buffer.
  auto buffer = EncodedImageBuffer::Create(frame_size);
  size_t offset = 0;
  for (uint16_t seq_num = first_seq_num; seq_num != end_seq_num; ++seq_num) {
    const VCMPacket& packet = buffer_[seq_num % buffer_.size()].data;

    size_t length = packet.sizeBytes;
    RTC_CHECK_LE(offset + length, buffer->size());
    memcpy(buffer->data() + offset, packet.dataPtr, length);
    offset += length;
  }

  return std::make_unique<RtpFrameObject>(
      first_seq_num, last_seq_num, last_packet.markerBit, max_nack_count,
      min_recv_time, max_recv_time, first_packet.timestamp,
      first_packet.ntp_time_ms_, last_packet.video_header.video_timing,
      first_packet.payloadType, first_packet.codec(),
      last_packet.video_header.rotation, last_packet.video_header.content_type,
      video_header, last_packet.video_header.color_space,
      first_packet.generic_descriptor, RtpPacketInfos(std::move(packet_infos)),
      std::move(buffer));
}

void PacketBuffer::UpdateMissingPackets(uint16_t seq_num) {
  if (!newest_inserted_seq_num_)
    newest_inserted_seq_num_ = seq_num;

  const int kMaxPaddingAge = 1000;
  if (AheadOf(seq_num, *newest_inserted_seq_num_)) {
    uint16_t old_seq_num = seq_num - kMaxPaddingAge;
    auto erase_to = missing_packets_.lower_bound(old_seq_num);
    missing_packets_.erase(missing_packets_.begin(), erase_to);

    // Guard against inserting a large amount of missing packets if there is a
    // jump in the sequence number.
    if (AheadOf(old_seq_num, *newest_inserted_seq_num_))
      *newest_inserted_seq_num_ = old_seq_num;

    ++*newest_inserted_seq_num_;
    while (AheadOf(seq_num, *newest_inserted_seq_num_)) {
      missing_packets_.insert(*newest_inserted_seq_num_);
      ++*newest_inserted_seq_num_;
    }
  } else {
    missing_packets_.erase(seq_num);
  }
}

void PacketBuffer::OnTimestampReceived(uint32_t rtp_timestamp) {
  const size_t kMaxTimestampsHistory = 1000;
  if (rtp_timestamps_history_set_.insert(rtp_timestamp).second) {
    rtp_timestamps_history_queue_.push(rtp_timestamp);
    ++unique_frames_seen_;
    if (rtp_timestamps_history_set_.size() > kMaxTimestampsHistory) {
      uint32_t discarded_timestamp = rtp_timestamps_history_queue_.front();
      rtp_timestamps_history_set_.erase(discarded_timestamp);
      rtp_timestamps_history_queue_.pop();
    }
  }
}

}  // namespace video_coding
}  // namespace webrtc
