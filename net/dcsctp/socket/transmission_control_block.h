/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef NET_DCSCTP_SOCKET_TRANSMISSION_CONTROL_BLOCK_H_
#define NET_DCSCTP_SOCKET_TRANSMISSION_CONTROL_BLOCK_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "net/dcsctp/common/sequence_numbers.h"
#include "net/dcsctp/packet/sctp_packet.h"
#include "net/dcsctp/public/dcsctp_options.h"
#include "net/dcsctp/public/dcsctp_socket.h"
#include "net/dcsctp/rx/data_tracker.h"
#include "net/dcsctp/rx/reassembly_queue.h"
#include "net/dcsctp/socket/capabilities.h"
#include "net/dcsctp/socket/context.h"
#include "net/dcsctp/socket/heartbeat_handler.h"
#include "net/dcsctp/socket/stream_reset_handler.h"
#include "net/dcsctp/timer/timer.h"
#include "net/dcsctp/tx/retransmission_error_counter.h"
#include "net/dcsctp/tx/retransmission_queue.h"
#include "net/dcsctp/tx/retransmission_timeout.h"
#include "net/dcsctp/tx/send_queue.h"

namespace dcsctp {

// The TransmissionControlBlock (TCB) represents an open connection to a peer,
// and holds all the resources for that. If the connection is e.g. shutdown,
// closed or restarted, this object will be deleted and/or replaced.
class TransmissionControlBlock : public Context {
 public:
  TransmissionControlBlock(TimerManager* timer_manager,
                           absl::string_view log_prefix,
                           const DcSctpOptions& options,
                           const Capabilities& capabilities,
                           DcSctpSocketCallbacks* callbacks,
                           SendQueue* send_queue,
                           VerificationTag my_verification_tag,
                           TSN my_initial_tsn,
                           VerificationTag peer_verification_tag,
                           TSN peer_initial_tsn,
                           size_t a_rwnd,
                           TieTag tie_tag,
                           std::function<bool()> is_connection_established,
                           std::function<void(SctpPacket::Builder&)> send_fn)
      : log_prefix_(log_prefix),
        options_(options),
        timer_manager_(*timer_manager),
        capabilities_(capabilities),
        callbacks_(*callbacks),
        t3_rtx_(timer_manager_.CreateTimer(
            "t3-rtx",
            [this]() { return OnRtxTimerExpiry(); },
            TimerOptions(options.rto_initial))),
        delayed_ack_timer_(timer_manager_.CreateTimer(
            "delayed-ack",
            [this]() { return OnDelayedAckTimerExpiry(); },
            TimerOptions(DurationMs(*options.rto_initial / 2),
                         TimerBackoffAlgorithm::kExponential,
                         /*max_restarts=*/0))),
        rto_(options),
        tx_error_counter_(log_prefix, options),
        my_verification_tag_(my_verification_tag),
        my_initial_tsn_(my_initial_tsn),
        peer_verification_tag_(peer_verification_tag),
        peer_initial_tsn_(peer_initial_tsn),
        tie_tag_(tie_tag),
        is_connection_established_(std::move(is_connection_established)),
        send_fn_(std::move(send_fn)),
        data_tracker_(log_prefix, delayed_ack_timer_.get(), peer_initial_tsn),
        reassembly_queue_(log_prefix,
                          peer_initial_tsn,
                          options.max_receiver_window_buffer_size),
        retransmission_queue_(
            log_prefix,
            my_initial_tsn,
            a_rwnd,
            send_queue,
            [this](DurationMs rtt) { return ObserveRTT(rtt); },
            [this]() { callbacks_.NotifyOutgoingMessageBufferEmpty(); },
            [this]() { tx_error_counter_.Clear(); },
            t3_rtx_.get(),
            options,
            capabilities.partial_reliability,
            capabilities.message_interleaving),
        stream_reset_handler_(log_prefix,
                              this,
                              timer_manager,
                              &data_tracker_,
                              &reassembly_queue_,
                              &retransmission_queue_),
        heartbeat_handler_(log_prefix, options, this, &timer_manager_) {}

  // Implementation of Context
  bool IsConnectionEstablished() const override {
    return is_connection_established_();
  }
  TSN my_initial_tsn() const override { return my_initial_tsn_; }
  TSN peer_initial_tsn() const override { return peer_initial_tsn_; }
  DcSctpSocketCallbacks& callbacks() const override { return callbacks_; }
  void ObserveRTT(DurationMs rtt) override;
  DurationMs current_rto() const override { return rto_.rto(); }
  bool IncrementTxErrorCounter(absl::string_view reason) override {
    return tx_error_counter_.Increment(reason);
  }
  void ClearTxErrorCounter() override { tx_error_counter_.Clear(); }
  SctpPacket::Builder PacketBuilder() const override {
    return SctpPacket::Builder(peer_verification_tag_, options_);
  }
  bool HasTooManyTxErrors() const override {
    return tx_error_counter_.IsExhausted();
  }
  void Send(SctpPacket::Builder& builder) override { send_fn_(builder); }

  // Other accessors
  DataTracker& data_tracker() { return data_tracker_; }
  ReassemblyQueue& reassembly_queue() { return reassembly_queue_; }
  RetransmissionQueue& retransmission_queue() { return retransmission_queue_; }
  StreamResetHandler& stream_reset_handler() { return stream_reset_handler_; }
  HeartbeatHandler& heartbeat_handler() { return heartbeat_handler_; }
  VerificationTag my_verification_tag() const { return my_verification_tag_; }
  VerificationTag peer_verification_tag() const {
    return peer_verification_tag_;
  }
  const Capabilities& capabilities() const { return capabilities_; }
  TieTag tie_tag() const { return tie_tag_; }

  void MaybeSendSack();
  void SendBufferedPackets(SctpPacket::Builder& builder,
                           bool only_one_packet = false);
  void SendBufferedPackets() {
    SctpPacket::Builder builder(peer_verification_tag_, options_);
    SendBufferedPackets(builder, /*only_one_packet=*/false);
  }

  std::string ToString() const;

 private:
  absl::optional<DurationMs> OnRtxTimerExpiry();
  absl::optional<DurationMs> OnDelayedAckTimerExpiry();

  const std::string log_prefix_;
  const DcSctpOptions options_;
  TimerManager& timer_manager_;
  const Capabilities capabilities_;
  DcSctpSocketCallbacks& callbacks_;
  std::unique_ptr<Timer> t3_rtx_;
  std::unique_ptr<Timer> delayed_ack_timer_;
  RetransmissionTimeout rto_;
  RetransmissionErrorCounter tx_error_counter_;
  const VerificationTag my_verification_tag_;
  const TSN my_initial_tsn_;
  const VerificationTag peer_verification_tag_;
  const TSN peer_initial_tsn_;
  const TieTag tie_tag_;
  std::function<bool()> is_connection_established_;
  std::function<void(SctpPacket::Builder&)> send_fn_;
  DataTracker data_tracker_;
  ReassemblyQueue reassembly_queue_;
  RetransmissionQueue retransmission_queue_;
  StreamResetHandler stream_reset_handler_;
  HeartbeatHandler heartbeat_handler_;
};
}  // namespace dcsctp

#endif  // NET_DCSCTP_SOCKET_TRANSMISSION_CONTROL_BLOCK_H_
