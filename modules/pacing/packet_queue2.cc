/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/pacing/packet_queue2.h"

#include <algorithm>

#include "rtc_base/checks.h"
#include "system_wrappers/include/clock.h"

namespace webrtc {

PacketQueue2::StreamInfo::StreamInfo() : bytes(0) {}
PacketQueue2::StreamInfo::~StreamInfo() {}

PacketQueue2::Packet::Packet(RtpPacketSender::Priority priority,
                             uint32_t ssrc,
                             uint16_t seq_number,
                             int64_t capture_time_ms,
                             int64_t enqueue_time_ms,
                             size_t length_in_bytes,
                             bool retransmission,
                             uint64_t enqueue_order)
    : priority(priority),
      ssrc(ssrc),
      sequence_number(seq_number),
      capture_time_ms(capture_time_ms),
      enqueue_time_ms(enqueue_time_ms),
      bytes(length_in_bytes),
      retransmission(retransmission),
      enqueue_order(enqueue_order) {}

PacketQueue2::Packet::Packet(const Packet& other) = default;

PacketQueue2::Packet::~Packet() {}

PacketQueue2::PacketQueue2(const Clock* clock)
    : clock_(clock), time_last_updated_(clock_->TimeInMilliseconds()) {}

PacketQueue2::~PacketQueue2() {}

void PacketQueue2::Push(const Packet& packet) {
  Packet pkt(packet);

  auto stream_info_it = stream_infos_.find(pkt.ssrc);
  if (stream_info_it == stream_infos_.end()) {
    stream_info_it = stream_infos_.emplace(pkt.ssrc, StreamInfo()).first;
    stream_info_it->second.priority_it = stream_priorities_.end();
    stream_info_it->second.ssrc = pkt.ssrc;
  }

  StreamInfo* stream_info = &stream_info_it->second;

  if (stream_info->priority_it == stream_priorities_.end()) {
    // If the SSRC is not currently scheduled, add it to |stream_priorities_|.
    stream_info->priority_it = stream_priorities_.emplace(
        StreamPrioKey(pkt.priority, stream_info->bytes), pkt.ssrc);
  } else if (pkt.priority < stream_info->priority_it->first.priority) {
    // If the priority of this SSRC increased, remove the outdated StreamPrioKey
    // and insert a new one with the new priority.
    stream_priorities_.erase(stream_info->priority_it);
    stream_info->priority_it = stream_priorities_.emplace(
        StreamPrioKey(pkt.priority, stream_info->bytes), pkt.ssrc);
  }
  RTC_CHECK(stream_info->priority_it != stream_priorities_.end());

  pkt.enqueue_time_it = enqueue_times_.insert(pkt.enqueue_time_ms);

  // In order to figure out how much time a packet has spent in the queue while
  // not in a paused state, we add the total amount of time the queue has been
  // paused so far, and when the packet is poped we subtract the total amount of
  // time the queue has been paused at that moment. This way we subtract the
  // total amount of time the packet has spent in the queue while in a paused
  // state.
  UpdateQueueTime(pkt.enqueue_time_ms);
  pkt.enqueue_time_ms += pause_time_sum_ms_;
  stream_info->packet_queue.push(pkt);

  num_packets_ += 1;
  num_bytes_ += pkt.bytes;
}

const PacketQueue2::Packet& PacketQueue2::Top() {
  return GetHighestPriorityStream()->packet_queue.top();
}

void PacketQueue2::Pop() {
  RTC_CHECK(!paused_);
  if (!Empty()) {
    StreamInfo* stream_info = GetHighestPriorityStream();
    stream_priorities_.erase(stream_info->priority_it);
    const Packet& packet = stream_info->packet_queue.top();

    // Calculate the total amount of time spent by this packet in the queue
    // while in a non-paused state. Note that the |pause_time_sum_ms_| was
    // added to |packet.enqueue_time_ms| when the packet was pushed, and by
    // subtracting it now we effectively remove the time spent in in the queue
    // while in a paused state.
    int64_t time_in_non_paused_state_ms = clock_->TimeInMilliseconds() -
                                          packet.enqueue_time_ms -
                                          pause_time_sum_ms_;
    queue_time_sum_ms_ -= time_in_non_paused_state_ms;

    RTC_CHECK(packet.enqueue_time_it != enqueue_times_.end());
    enqueue_times_.erase(packet.enqueue_time_it);

    // Update |bytes| of this stream. The general idea is that the stream that
    // has sent the least amount of bytes should should have the highest
    // priority. The problem with that is if streams send with different rates,
    // in which case a "budget" will be built up for the stream sending at a
    // lower rate. To avoid building a too large budget we limit |bytes| to be
    // withing kMaxLeading bytes of the stream that has sent the most amount of
    // bytes.
    stream_info->bytes = std::max(stream_info->bytes + packet.bytes,
                                  max_bytes_ - kMaxLeadingBytes);
    max_bytes_ = std::max(max_bytes_, stream_info->bytes);

    num_bytes_ -= packet.bytes;
    num_packets_ -= 1;
    stream_info->packet_queue.pop();

    // If there are packets left to be sent, schedule the stream again.
    if (!stream_info->packet_queue.empty()) {
      RtpPacketSender::Priority priority =
          stream_info->packet_queue.top().priority;
      stream_info->priority_it = stream_priorities_.emplace(
          StreamPrioKey(priority, stream_info->bytes), stream_info->ssrc);
    } else {
      stream_info->priority_it = stream_priorities_.end();
    }
  }
}

bool PacketQueue2::Empty() const {
  RTC_CHECK((!stream_priorities_.empty() && num_packets_ > 0) ||
            (stream_priorities_.empty() && num_packets_ == 0));
  return stream_priorities_.empty();
}

size_t PacketQueue2::SizeInPackets() const {
  return num_packets_;
}

uint64_t PacketQueue2::SizeInBytes() const {
  return num_bytes_;
}

int64_t PacketQueue2::OldestEnqueueTimeMs() const {
  if (Empty())
    return 0;
  RTC_CHECK(!enqueue_times_.empty());
  return *enqueue_times_.begin();
}

void PacketQueue2::UpdateQueueTime(int64_t timestamp_ms) {
  RTC_CHECK_GE(timestamp_ms, time_last_updated_);
  if (timestamp_ms == time_last_updated_)
    return;

  int64_t delta_ms = timestamp_ms - time_last_updated_;

  if (paused_)
    pause_time_sum_ms_ += delta_ms;
  else
    queue_time_sum_ms_ += delta_ms * num_packets_;

  time_last_updated_ = timestamp_ms;
}

void PacketQueue2::SetPauseState(bool paused, int64_t timestamp_ms) {
  if (paused_ == paused)
    return;
  UpdateQueueTime(timestamp_ms);
  paused_ = paused;
}

int64_t PacketQueue2::AverageQueueTimeMs() const {
  if (Empty())
    return 0;
  return queue_time_sum_ms_ / num_packets_;
}

PacketQueue2::StreamInfo* PacketQueue2::GetHighestPriorityStream() {
  RTC_CHECK(!stream_priorities_.empty());
  uint32_t ssrc = stream_priorities_.begin()->second;

  auto stream_info_it = stream_infos_.find(ssrc);
  RTC_CHECK(stream_info_it != stream_infos_.end());
  RTC_CHECK(stream_info_it->second.priority_it == stream_priorities_.begin());
  RTC_CHECK(!stream_info_it->second.packet_queue.empty());
  return &stream_info_it->second;
}

}  // namespace webrtc
