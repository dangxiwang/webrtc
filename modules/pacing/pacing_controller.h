/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_PACING_PACING_CONTROLLER_H_
#define MODULES_PACING_PACING_CONTROLLER_H_

#include <stddef.h>
#include <stdint.h>

#include <atomic>
#include <memory>
#include <vector>

#include "absl/types/optional.h"
#include "api/function_view.h"
#include "api/rtc_event_log/rtc_event_log.h"
#include "api/transport/field_trial_based_config.h"
#include "api/transport/network_types.h"
#include "api/transport/webrtc_key_value_config.h"
#include "modules/pacing/bitrate_prober.h"
#include "modules/pacing/interval_budget.h"
#include "modules/pacing/round_robin_packet_queue.h"
#include "modules/pacing/rtp_packet_pacer.h"
#include "modules/rtp_rtcp/include/rtp_packet_sender.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

// This class implements a leaky-buck packet pacing algorithm. It handles the
// logic of determining which packets to send when, but the actual timing of
// the processing is done externally (e.g. PacedSender). Furthermore, the
// forwarding of packets when they are ready to be sent is also handled
// externally, via the PacedSendingController::PacketSender interface.
//
class PacingController {
 public:
  class PacketSender {
   public:
    virtual ~PacketSender() = default;
    virtual void SendRtpPacket(std::unique_ptr<RtpPacketToSend> packet,
                               const PacedPacketInfo& cluster_info) = 0;
    virtual std::vector<std::unique_ptr<RtpPacketToSend>> GeneratePadding(
        DataSize size) = 0;
  };

  // Expected max pacer delay. If ExpectedQueueTime() is higher than
  // this value, the packet producers should wait (eg drop frames rather than
  // encoding them). Bitrate sent may temporarily exceed target set by
  // UpdateBitrate() so that this limit will be upheld.
  static const TimeDelta kMaxExpectedQueueLength;
  // Pacing-rate relative to our target send rate.
  // Multiplicative factor that is applied to the target bitrate to calculate
  // the number of bytes that can be transmitted per interval.
  // Increasing this factor will result in lower delays in cases of bitrate
  // overshoots from the encoder.
  static const float kDefaultPaceMultiplier;
  // If no media or paused, wake up at least every |kPausedProcessIntervalMs| in
  // order to send a keep-alive packet so we don't get stuck in a bad state due
  // to lack of feedback.
  static const TimeDelta kPausedProcessInterval;

  PacingController(Clock* clock,
                   PacketSender* packet_sender,
                   RtcEventLog* event_log,
                   const WebRtcKeyValueConfig* field_trials);

  ~PacingController();

  // Adds the packet to the queue and calls PacketRouter::SendPacket() when
  // it's time to send.
  void EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet);

  void CreateProbeCluster(DataRate bitrate, int cluster_id);

  void Pause();   // Temporarily pause all sending.
  void Resume();  // Resume sending packets.
  bool IsPaused() const;

  void SetCongestionWindow(DataSize congestion_window_size);
  void UpdateOutstandingData(DataSize outstanding_data);

  // Sets the pacing rates. Must be called once before packets can be sent.
  void SetPacingRates(DataRate pacing_rate, DataRate padding_rate);

  // Currently audio traffic is not accounted by pacer and passed through.
  // With the introduction of audio BWE audio traffic will be accounted for
  // the pacer budget calculation. The audio traffic still will be injected
  // at high priority.
  void SetAccountForAudioPackets(bool account_for_audio);

  // Returns the time since the oldest queued packet was enqueued.
  TimeDelta OldestPacketWaitTime() const;

  size_t QueueSizePackets() const;
  DataSize QueueSizeData() const;

  // Returns the time when the first packet was sent;
  absl::optional<Timestamp> FirstSentPacketTime() const;

  // Returns the number of milliseconds it will take to send the current
  // packets in the queue, given the current size and bitrate, ignoring prio.
  TimeDelta ExpectedQueueTime() const;

  void SetQueueTimeLimit(TimeDelta limit);

  // Enable bitrate probing. Enabled by default, mostly here to simplify
  // testing. Must be called before any packets are being sent to have an
  // effect.
  void SetProbingEnabled(bool enabled);

  // Time until next probe should be sent. If this value is set, it should be
  // respected - i.e. don't call ProcessPackets() before this specified time as
  // that can have unintended side effects.
  absl::optional<TimeDelta> TimeUntilNextProbe();

  // Time since ProcessPackets() was last executed.
  TimeDelta TimeElapsedSinceLastProcess() const;

  TimeDelta TimeUntilAvailableBudget() const;

  // Check queue of pending packets and send them or padding packets, if budget
  // is available.
  void ProcessPackets();

  bool Congested() const;

 private:
  TimeDelta UpdateTimeAndGetElapsed(Timestamp now);
  bool ShouldSendKeepalive(Timestamp now) const;

  // Updates the number of bytes that can be sent for the next time interval.
  void UpdateBudgetWithElapsedTime(TimeDelta delta);
  void UpdateBudgetWithSentData(DataSize size);

  DataSize PaddingToAdd(absl::optional<DataSize> recommended_probe_size,
                        DataSize data_sent);

  RoundRobinPacketQueue::QueuedPacket* GetPendingPacket(
      const PacedPacketInfo& pacing_info);
  void OnPacketSent(RoundRobinPacketQueue::QueuedPacket* packet);
  void OnPaddingSent(DataSize padding_sent);

  Timestamp CurrentTime() const;

  Clock* const clock_;
  PacketSender* const packet_sender_;
  const std::unique_ptr<FieldTrialBasedConfig> fallback_field_trials_;
  const WebRtcKeyValueConfig* field_trials_;

  const bool drain_large_queues_;
  const bool send_padding_if_silent_;
  const bool pace_audio_;
  TimeDelta min_packet_limit_;

  // TODO(webrtc:9716): Remove this when we are certain clocks are monotonic.
  // The last millisecond timestamp returned by |clock_|.
  mutable Timestamp last_timestamp_;
  bool paused_;
  // This is the media budget, keeping track of how many bits of media
  // we can pace out during the current interval.
  IntervalBudget media_budget_;
  // This is the padding budget, keeping track of how many bits of padding we're
  // allowed to send out during the current interval. This budget will be
  // utilized when there's no media to send.
  IntervalBudget padding_budget_;

  BitrateProber prober_;
  bool probing_send_failure_;
  bool padding_failure_state_;

  DataRate pacing_bitrate_;

  Timestamp time_last_process_;
  Timestamp last_send_time_;
  absl::optional<Timestamp> first_sent_packet_time_;

  RoundRobinPacketQueue packet_queue_;
  uint64_t packet_counter_;

  DataSize congestion_window_size_;
  DataSize outstanding_data_;

  TimeDelta queue_time_limit;
  bool account_for_audio_;
};
}  // namespace webrtc

#endif  // MODULES_PACING_PACING_CONTROLLER_H_
