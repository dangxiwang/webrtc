/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_CODING_NETEQ_TOOLS_NETEQ_INPUT_H_
#define MODULES_AUDIO_CODING_NETEQ_TOOLS_NETEQ_INPUT_H_

#include <algorithm>
#include <memory>
#include <string>

#include "api/optional.h"
#include "common_types.h"  // NOLINT(build/include)
#include "modules/audio_coding/neteq/tools/packet.h"
#include "modules/audio_coding/neteq/tools/packet_source.h"
#include "rtc_base/buffer.h"

namespace webrtc {
namespace test {

// Interface class for input to the NetEqTest class.
class NetEqInput {
 public:
  struct PacketData {
    std::string ToString() const;

    RTPHeader header;
    rtc::Buffer payload;
    int64_t time_ms;
  };

  virtual ~NetEqInput() = default;

  // Returns at what time (in ms) NetEq::InsertPacket should be called next, or
  // empty if the source is out of packets.
  virtual rtc::Optional<int64_t> NextPacketTime() const = 0;

  // Returns at what time (in ms) NetEq::GetAudio should be called next, or
  // empty if no more output events are available.
  virtual rtc::Optional<int64_t> NextOutputEventTime() const = 0;

  // Returns the time (in ms) for the next event from either NextPacketTime()
  // or NextOutputEventTime(), or empty if both are out of events.
  rtc::Optional<int64_t> NextEventTime() const {
    const auto a = NextPacketTime();
    const auto b = NextOutputEventTime();
    // Return the minimum of non-empty |a| and |b|, or empty if both are empty.
    if (a) {
      return b ? std::min(*a, *b) : a;
    }
    return b ? b : rtc::nullopt;
  }

  // Returns the next packet to be inserted into NetEq. The packet following the
  // returned one is pre-fetched in the NetEqInput object, such that future
  // calls to NextPacketTime() or NextHeader() will return information from that
  // packet.
  virtual std::unique_ptr<PacketData> PopPacket() = 0;

  // Move to the next output event. This will make NextOutputEventTime() return
  // a new value (potentially the same if several output events share the same
  // time).
  virtual void AdvanceOutputEvent() = 0;

  // Returns true if the source has come to an end. An implementation must
  // eventually return true from this method, or the test will end up in an
  // infinite loop.
  virtual bool ended() const = 0;

  // Returns the RTP header for the next packet, i.e., the packet that will be
  // delivered next by PopPacket().
  virtual rtc::Optional<RTPHeader> NextHeader() const = 0;
};

// Wrapper class to create impose a time limit on a NetEqInput object, typically
// another time limit that what the object itself provides. For example, an
// input taken from a file can be cut shorter by wrapping it in this class.
class InputTimeLimit : public NetEqInput {
 public:
  InputTimeLimit(std::unique_ptr<NetEqInput> input, int64_t duration_ms)
      : input_(std::move(input)),
        start_time_ms_(input_->NextEventTime()),
        duration_ms_(duration_ms) {}

  rtc::Optional<int64_t> NextPacketTime() const {
    return input_->NextPacketTime();
  }
  rtc::Optional<int64_t> NextOutputEventTime() const {
    return input_->NextOutputEventTime();
  }
  std::unique_ptr<PacketData> PopPacket() { return input_->PopPacket(); }
  void AdvanceOutputEvent() { return input_->AdvanceOutputEvent(); }
  bool ended() const {
    if (input_->ended()) {
      return true;
    }
    if (NextEventTime() && start_time_ms_ &&
        *NextEventTime() - *start_time_ms_ > duration_ms_) {
      return true;
    }
    return false;
  }
  rtc::Optional<RTPHeader> NextHeader() const { return input_->NextHeader(); }

 private:
  std::unique_ptr<NetEqInput> input_;
  const rtc::Optional<int64_t> start_time_ms_;
  const int64_t duration_ms_;
};

}  // namespace test
}  // namespace webrtc
#endif  // MODULES_AUDIO_CODING_NETEQ_TOOLS_NETEQ_INPUT_H_
