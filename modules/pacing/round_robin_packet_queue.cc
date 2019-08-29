/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/pacing/round_robin_packet_queue.h"

#include <algorithm>
#include <cstdint>
#include <utility>

#include "rtc_base/checks.h"

namespace webrtc {
namespace {
static constexpr DataSize kMaxLeadingSize = DataSize::Bytes<1400>();
}

RoundRobinPacketQueue::QueuedPacket::QueuedPacket(const QueuedPacket& rhs) =
    default;
RoundRobinPacketQueue::QueuedPacket::~QueuedPacket() = default;

RoundRobinPacketQueue::QueuedPacket::QueuedPacket(
    int priority,
    Timestamp enqueue_time,
    DataSize size,
    bool retransmission,
    uint64_t enqueue_order,
    std::multiset<Timestamp>::iterator enqueue_time_it,
    std::list<std::unique_ptr<RtpPacketToSend>>::iterator packet_it)
    : priority_(priority),
      enqueue_time_(enqueue_time),
      size_(size),
      retransmission_(retransmission),
      enqueue_order_(enqueue_order),
      enqueue_time_it_(enqueue_time_it),
      packet_it_(packet_it) {}

std::unique_ptr<RtpPacketToSend>
RoundRobinPacketQueue::QueuedPacket::ReleasePacket() {
  if (packet_it_->get() != nullptr) {
    return std::move(*packet_it_);
  }
  return nullptr;
}

void RoundRobinPacketQueue::QueuedPacket::SubtractPauseTime(
    TimeDelta pause_time_sum) {
  enqueue_time_ -= pause_time_sum;
}

bool RoundRobinPacketQueue::QueuedPacket::operator<(
    const RoundRobinPacketQueue::QueuedPacket& other) const {
  if (priority_ != other.priority_)
    return priority_ > other.priority_;
  if (retransmission_ != other.retransmission_)
    return other.retransmission_;

  return enqueue_order_ > other.enqueue_order_;
}

RoundRobinPacketQueue::Stream::Stream() : size(DataSize::Zero()), ssrc(0) {}
RoundRobinPacketQueue::Stream::Stream(const Stream& stream) = default;
RoundRobinPacketQueue::Stream::~Stream() {}

RoundRobinPacketQueue::RoundRobinPacketQueue(
    Timestamp start_time,
    const WebRtcKeyValueConfig* field_trials)
    : time_last_updated_(start_time),
      paused_(false),
      size_packets_(0),
      size_(DataSize::Zero()),
      max_size_(kMaxLeadingSize),
      queue_time_sum_(TimeDelta::Zero()),
      pause_time_sum_(TimeDelta::Zero()) {}

RoundRobinPacketQueue::~RoundRobinPacketQueue() {}

void RoundRobinPacketQueue::Push(int priority,
                                 Timestamp enqueue_time,
                                 uint64_t enqueue_order,
                                 DataSize size,
                                 std::unique_ptr<RtpPacketToSend> packet) {
  const uint32_t ssrc = packet->Ssrc();
  const bool retransmission =
      packet->packet_type() == RtpPacketToSend::Type::kRetransmission;
  rtp_packets_.push_front(std::move(packet));
  Push(QueuedPacket(priority, enqueue_time, size, retransmission, enqueue_order,
                    enqueue_times_.insert(enqueue_time), rtp_packets_.begin()),
       ssrc);
}

const RtpPacketToSend& RoundRobinPacketQueue::Top() const {
  RTC_DCHECK(!Empty());

  const Stream& stream = GetHighestPriorityStream();
  auto packet = stream.packet_queue.top();
  return packet.get_packet();
}

std::unique_ptr<RtpPacketToSend> RoundRobinPacketQueue::Pop() {
  RTC_DCHECK(!Empty());

  Stream* stream = GetHighestPriorityStream();
  auto packet = stream->packet_queue.top();
  stream->packet_queue.pop();

  std::unique_ptr<RtpPacketToSend> rtp_packet = packet.ReleasePacket();

  stream_priorities_.erase(stream->priority_it);

  // Calculate the total amount of time spent by this packet in the queue
  // while in a non-paused state. Note that the |pause_time_sum_ms_| was
  // subtracted from |packet.enqueue_time_ms| when the packet was pushed, and
  // by subtracting it now we effectively remove the time spent in in the
  // queue while in a paused state.
  TimeDelta time_in_non_paused_state =
      time_last_updated_ - packet.enqueue_time() - pause_time_sum_;
  queue_time_sum_ -= time_in_non_paused_state;

  RTC_CHECK(packet.EnqueueTimeIterator() != enqueue_times_.end());
  enqueue_times_.erase(packet.EnqueueTimeIterator());

  rtp_packets_.erase(packet.PacketIterator());

  // Update |bytes| of this stream. The general idea is that the stream that
  // has sent the least amount of bytes should have the highest priority.
  // The problem with that is if streams send with different rates, in which
  // case a "budget" will be built up for the stream sending at the lower
  // rate. To avoid building a too large budget we limit |bytes| to be within
  // kMaxLeading bytes of the stream that has sent the most amount of bytes.
  stream->size =
      std::max(stream->size + packet.size(), max_size_ - kMaxLeadingSize);
  max_size_ = std::max(max_size_, stream->size);

  size_ -= packet.size();
  size_packets_ -= 1;
  RTC_CHECK(size_packets_ > 0 || queue_time_sum_ == TimeDelta::Zero());

  // If there are packets left to be sent, schedule the stream again.
  RTC_CHECK(!IsSsrcScheduled(stream->ssrc));
  if (stream->packet_queue.empty()) {
    stream->priority_it = stream_priorities_.end();
  } else {
    int priority = stream->packet_queue.top().priority();
    stream->priority_it = stream_priorities_.emplace(
        StreamPrioKey(priority, stream->size), stream->ssrc);
  }

  return rtp_packet;
}

bool RoundRobinPacketQueue::Empty() const {
  RTC_CHECK((!stream_priorities_.empty() && size_packets_ > 0) ||
            (stream_priorities_.empty() && size_packets_ == 0));
  return stream_priorities_.empty();
}

size_t RoundRobinPacketQueue::SizeInPackets() const {
  return size_packets_;
}

DataSize RoundRobinPacketQueue::Size() const {
  return size_;
}

Timestamp RoundRobinPacketQueue::OldestEnqueueTime() const {
  if (Empty())
    return Timestamp::MinusInfinity();
  RTC_CHECK(!enqueue_times_.empty());
  return *enqueue_times_.begin();
}

void RoundRobinPacketQueue::UpdateQueueTime(Timestamp now) {
  RTC_CHECK_GE(now, time_last_updated_);
  if (now == time_last_updated_)
    return;

  TimeDelta delta = now - time_last_updated_;

  if (paused_) {
    pause_time_sum_ += delta;
  } else {
    queue_time_sum_ += TimeDelta::us(delta.us() * size_packets_);
  }

  time_last_updated_ = now;
}

void RoundRobinPacketQueue::SetPauseState(bool paused, Timestamp now) {
  if (paused_ == paused)
    return;
  UpdateQueueTime(now);
  paused_ = paused;
}

TimeDelta RoundRobinPacketQueue::AverageQueueTime() const {
  if (Empty())
    return TimeDelta::Zero();
  return queue_time_sum_ / size_packets_;
}

void RoundRobinPacketQueue::Push(QueuedPacket packet, uint32_t ssrc) {
  auto stream_info_it = streams_.find(ssrc);
  if (stream_info_it == streams_.end()) {
    stream_info_it = streams_.emplace(ssrc, Stream()).first;
    stream_info_it->second.priority_it = stream_priorities_.end();
    stream_info_it->second.ssrc = ssrc;
  }

  Stream* stream = &stream_info_it->second;

  if (stream->priority_it == stream_priorities_.end()) {
    // If the SSRC is not currently scheduled, add it to |stream_priorities_|.
    RTC_CHECK(!IsSsrcScheduled(stream->ssrc));
    stream->priority_it = stream_priorities_.emplace(
        StreamPrioKey(packet.priority(), stream->size), ssrc);
  } else if (packet.priority() < stream->priority_it->first.priority) {
    // If the priority of this SSRC increased, remove the outdated StreamPrioKey
    // and insert a new one with the new priority. Note that |priority_| uses
    // lower ordinal for higher priority.
    stream_priorities_.erase(stream->priority_it);
    stream->priority_it = stream_priorities_.emplace(
        StreamPrioKey(packet.priority(), stream->size), ssrc);
  }
  RTC_CHECK(stream->priority_it != stream_priorities_.end());

  // In order to figure out how much time a packet has spent in the queue while
  // not in a paused state, we subtract the total amount of time the queue has
  // been paused so far, and when the packet is popped we subtract the total
  // amount of time the queue has been paused at that moment. This way we
  // subtract the total amount of time the packet has spent in the queue while
  // in a paused state.
  UpdateQueueTime(packet.enqueue_time());
  packet.SubtractPauseTime(pause_time_sum_);

  size_packets_ += 1;
  size_ += packet.size();

  stream->packet_queue.push(packet);
}

RoundRobinPacketQueue::Stream*
RoundRobinPacketQueue::GetHighestPriorityStream() {
  RTC_CHECK(!stream_priorities_.empty());
  uint32_t ssrc = stream_priorities_.begin()->second;

  auto stream_info_it = streams_.find(ssrc);
  RTC_CHECK(stream_info_it != streams_.end());
  RTC_CHECK(stream_info_it->second.priority_it == stream_priorities_.begin());
  RTC_CHECK(!stream_info_it->second.packet_queue.empty());
  return &stream_info_it->second;
}

const RoundRobinPacketQueue::Stream&
RoundRobinPacketQueue::GetHighestPriorityStream() const {
  RTC_CHECK(!stream_priorities_.empty());
  uint32_t ssrc = stream_priorities_.begin()->second;

  auto stream_info_it = streams_.find(ssrc);
  RTC_CHECK(stream_info_it != streams_.end());
  RTC_CHECK(stream_info_it->second.priority_it == stream_priorities_.begin());
  RTC_CHECK(!stream_info_it->second.packet_queue.empty());
  return stream_info_it->second;
}

bool RoundRobinPacketQueue::IsSsrcScheduled(uint32_t ssrc) const {
  for (const auto& scheduled_stream : stream_priorities_) {
    if (scheduled_stream.second == ssrc)
      return true;
  }
  return false;
}

}  // namespace webrtc
