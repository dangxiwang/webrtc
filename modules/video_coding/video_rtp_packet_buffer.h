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
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/rtp_rtcp/source/rtp_video_depacketizer.h"
#include "modules/video_coding/frame_object.h"
#include "modules/video_coding/packet.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/numerics/sequence_number_util.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

class VideoRtpPacketBuffer {
 public:
  struct RtpFrame {
    std::vector<std::unique_ptr<RtpPacketReceived>> rtp_packets;
    int max_times_nacked;
  };
  struct InsertResult {
    std::vector<RtpFrame> frames;
    bool buffer_cleared;
  };
  VideoRtpPacketBuffer(size_t max_buffer_size, size_t start_buffer_size);
  ~VideoRtpPacketBuffer();

  InsertResult InsertPacket(
      std::unique_ptr<RtpPacketReceived> packet,
      const RtpVideoDepacketizer::FrameBoundaries& boundaries,
      int times_nacked);
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
      packet_ = nullptr;
    }
    void SetContinious(int64_t bof_sequence_number) {
      bof_seq_num_ = bof_sequence_number;
    }

    void SetBof() { bof_seq_num_ = sequence_number_; }
    void SetEof() { frame_end_ = true; }
    void SetPacket(std::unique_ptr<RtpPacketReceived> packet,
                   int times_nacked) {
      packet_ = std::move(packet);
      times_nacked_ = times_nacked;
    }

    int64_t SequenceNumber() const { return sequence_number_; }
    bool IsPadding() const { return packet_ == nullptr; }
    std::unique_ptr<RtpPacketReceived> ExtractRtpPacket() {
      return std::move(packet_);
    }
    const RtpPacketReceived& RtpPacket() const { return *packet_; }

    bool frame_begin() const { return bof_seq_num_ == sequence_number_; }
    bool frame_end() const { return frame_end_; }
    absl::optional<int64_t> continuous() const { return bof_seq_num_; }

    int times_nacked() const { return times_nacked_; }

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

    // the original rtp packet or nullptr for padding-only packet.
    std::unique_ptr<RtpPacketReceived> packet_;
    // A bit of extra packet buffer specific meta-data for the packet.
    int times_nacked_;
  };

  PacketSlot* FindUsedSlot(int64_t sequence_number);

  // Expands the buffer.
  void ExpandBuffer();

  // Test if all previous packets has arrived for the given sequence number.
  bool PotentialNewFrame(PacketSlot* packet) const;

  // Test if all packets of a frame has arrived, and if so, creates a frame.
  // Returns a vector of received frames.
  std::vector<RtpFrame> FindFrames(PacketSlot* new_packet);
  RtpFrame AssembleFrame(int64_t first_sequence_number,
                         int64_t last_sequence_number);

  void ClearInterval(int64_t first_seq_num, int64_t last_seq_num) {}

  const size_t max_size_;

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
