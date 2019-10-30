/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_VIDEO_CODING_VIDEO_RTP_PACKET_BUFFER_H_
#define MODULES_VIDEO_CODING_VIDEO_RTP_PACKET_BUFFER_H_

#include <map>
#include <memory>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "api/video/encoded_image.h"
#include "modules/video_coding/frame_object.h"
#include "modules/video_coding/packet.h"
#include "modules/video_coding/rtp_video_depacketizer.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/numerics/sequence_number_util.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

class VideoRtpPacketBuffer {
 public:
  struct InsertResult {
    std::vector<std::unique_ptr<video_coding::RtpFrameObject>> frames;
    bool buffer_cleared;
  };
  VideoRtpPacketBuffer(size_t max_buffer_size, size_t start_buffer_size);
  ~VideoRtpPacketBuffer();

  void AddReceivePayloadType(
      int payload_id,
      std::unique_ptr<RtpVideoDepacketizer> depacketizer) {
    auto inserted = depacketizers_.emplace(payload_id, std::move(depacketizer));
    RTC_DCHECK(inserted.second);
  }

  absl::optional<RtpVideoDepacketizer::FrameBoundaries> GetFrameBoundaries(
      const RtpPacketReceived& rtp_packet);

  InsertResult InsertPacket(
      const RtpPacketReceived& packet,
      const RtpVideoDepacketizer::FrameBoundaries& boundaries,
      int times_nacked,
      int64_t packet_receive_time_ms,
      int64_t frame_ntp_time_ms);
  InsertResult InsertPadding(uint16_t seq_num);
  void ClearTo(uint16_t seq_num) {}
  void Clear() {}

  absl::optional<int64_t> LastReceivedPacketMs() const {
    return last_received_packet_ms_;
  }
  absl::optional<int64_t> LastReceivedKeyframePacketMs() const {
    return last_received_keyframe_packet_ms_;
  }

 private:
  class PacketSlot {
   public:
    explicit PacketSlot(int64_t sequence_number)
        : sequence_number_(sequence_number) {}
    PacketSlot(const PacketSlot&) = delete;
    PacketSlot(PacketSlot&&) = default;
    PacketSlot& operator=(const PacketSlot&) = delete;
    PacketSlot& operator=(PacketSlot&&) = delete;

    void SetIsPadding() {
      SetBof();
      SetEof();
      packet_ = absl::nullopt;
    }
    void SetContinious(int64_t bof_sequence_number) {
      bof_seq_num_ = bof_sequence_number;
    }

    void SetBof() { bof_seq_num_ = sequence_number_; }
    void SetEof() { frame_end_ = true; }
    void SetPacket(const RtpPacketReceived& packet,
                   int times_nacked,
                   int64_t ntp_time_ms,
                   int64_t receive_time_ms) {
      packet_ = packet;
      times_nacked_ = times_nacked;
      ntp_time_ms_ = ntp_time_ms;
      receive_time_ms_ = receive_time_ms;
    }

    int64_t SequenceNumber() const { return sequence_number_; }
    bool IsPadding() const { return !packet_.has_value(); }
    const RtpPacketReceived& RtpPacket() const { return *packet_; }

    bool frame_begin() const { return bof_seq_num_ == sequence_number_; }
    bool frame_end() const { return frame_end_; }
    absl::optional<int64_t> continuous() const { return bof_seq_num_; }

    int times_nacked() const { return times_nacked_; }
    int64_t ntp_time_ms() const { return ntp_time_ms_; }
    int64_t recv_time_ms() const { return receive_time_ms_; }

   private:
    // Sequence number if slot is used (or about to be used).
    // nullopt for unused slot.
    const int64_t sequence_number_ = 0;
    // Sequence number of the first packet of a frame this packet belongs too
    // or nullopt if unknown (!continious)
    // bof_seq_num == sequence_number imply this is beginning of a frame.
    absl::optional<int64_t> bof_seq_num_;

    // If this is the last packet of the frame.
    bool frame_end_ = false;

    // the original rtp packet or nullopt for padding-only packet.
    absl::optional<RtpPacketReceived> packet_;
    // A bit of extra packet buffer specific meta-data for the packet.
    int times_nacked_;
    int64_t ntp_time_ms_;
    int64_t receive_time_ms_;
  };

  PacketSlot* FindUsedSlot(int64_t sequence_number);

  RtpVideoDepacketizer::FrameBoundaries FindPacketBoundaries(
      const RtpPacketReceived& rtp_packet) const;

  void UpdateVideoHeader(const RtpPacketReceived& first_packet,
                         const RtpPacketReceived& last_packet,
                         RTPVideoHeader* header);

  // Expands the buffer.
  void ExpandBuffer();

  // Test if all previous packets has arrived for the given sequence number.
  bool PotentialNewFrame(PacketSlot* packet) const;

  // Test if all packets of a frame has arrived, and if so, creates a frame.
  // Returns a vector of received frames.
  std::vector<std::unique_ptr<video_coding::RtpFrameObject>> FindFrames(
      PacketSlot* new_packet);
  std::unique_ptr<video_coding::RtpFrameObject> AssembleFrame(
      int64_t first_sequence_number,
      int64_t last_sequence_number);

  void ClearInterval(int64_t first_seq_num, int64_t last_seq_num) {}

  // buffer_.size() and max_size_ must always be a power of two.
  const size_t max_size_;
  std::map<int, std::unique_ptr<RtpVideoDepacketizer>> depacketizers_;

  SeqNumUnwrapper<uint16_t> unwrapper_;
  // Buffer that holds the the inserted packets and information needed to
  // determine continuity between them.
  std::vector<absl::optional<PacketSlot>> buffer_;

  int64_t cleared_up_to_ = -1;

  absl::optional<int64_t> last_received_packet_ms_;
  absl::optional<int64_t> last_received_keyframe_packet_ms_;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_VIDEO_RTP_PACKET_BUFFER_H_
