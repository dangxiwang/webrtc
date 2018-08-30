/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef TEST_SCENARIO_NETWORK_NODE_H_
#define TEST_SCENARIO_NETWORK_NODE_H_

#include <deque>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "api/call/transport.h"
#include "call/simulated_network.h"
#include "rtc_base/copyonwritebuffer.h"
#include "test/scenario/call_client.h"
#include "test/scenario/column_printer.h"
#include "test/scenario/scenario_config.h"

namespace webrtc {
namespace test {

class NetworkReceiverInterface {
 public:
  virtual bool TryDeliverPacket(rtc::CopyOnWriteBuffer packet,
                                uint64_t receiver) = 0;
  virtual ~NetworkReceiverInterface() = default;
};
class NullReceiver : public NetworkReceiverInterface {
 public:
  bool TryDeliverPacket(rtc::CopyOnWriteBuffer packet,
                        uint64_t receiver) override;
};
class ActionReceiver : public NetworkReceiverInterface {
 public:
  explicit ActionReceiver(std::function<void()> action);
  virtual ~ActionReceiver() = default;
  bool TryDeliverPacket(rtc::CopyOnWriteBuffer packet,
                        uint64_t receiver) override;

 private:
  std::function<void()> action_;
};

// NetworkNode represents one link in a simulated network. It is created by a
// scenario and can be used when setting upp audio and video stream sessions.
class NetworkNode : public NetworkReceiverInterface {
 public:
  ~NetworkNode() override;
  NetworkNode(const NetworkNode&) = delete;

  bool TryDeliverPacket(rtc::CopyOnWriteBuffer packet,
                        uint64_t receiver) override;

 protected:
  friend class Scenario;
  friend class AudioStreamPair;
  friend class VideoStreamPair;

  NetworkNode(Clock* clock,
              NetworkNodeConfig config,
              std::unique_ptr<NetworkSimulationInterface> simulation);
  static void Route(int64_t receiver_id,
                    NetworkReceiverInterface* receiver,
                    std::vector<NetworkNode*> nodes);
  static void ClearRoute(int64_t receiver_id, std::vector<NetworkNode*> nodes);
  void Process();

  Clock* const clock_;

 private:
  struct StoredPacket {
    rtc::CopyOnWriteBuffer packet_data;
    uint64_t receiver;
    uint64_t id;
    bool removed;
  };
  void SetRoute(uint64_t receiver, NetworkReceiverInterface* node);
  void ClearRoute(uint64_t receiver_id);
  size_t packet_overhead_;
  const std::unique_ptr<NetworkSimulationInterface> simulation_;
  std::map<uint64_t, NetworkReceiverInterface*> routing_;
  rtc::CriticalSection packets_lock_;
  std::deque<StoredPacket> packets_ RTC_GUARDED_BY(packets_lock_);

  uint64_t next_packet_id_ = 1;
};
// SimulationNode is a NetworkNode that expose an interface for changing run
// time behavior of the underlying simulation.
class SimulationNode : public NetworkNode {
 public:
  void UpdateConfig(std::function<void(NetworkNodeConfig*)> modifier);
  void TriggerDelay(TimeDelta duration);

 private:
  friend class Scenario;
  using NetworkNode::NetworkNode;
  static std::unique_ptr<SimulationNode> Create(Clock* clock,
                                                NetworkNodeConfig config);
  SimulatedNetwork* simulated_network_;
  NetworkNodeConfig config_;
};

class NetworkNodeTransport : public Transport {
 public:
  NetworkNodeTransport(CallClient* sender,
                       NetworkNode* send_net,
                       uint64_t receiver,
                       DataSize packet_overhead);
  ~NetworkNodeTransport() override;

  bool SendRtp(const uint8_t* packet,
               size_t length,
               const PacketOptions& options) override;
  bool SendRtcp(const uint8_t* packet, size_t length) override;
  uint64_t ReceiverId() const;

 private:
  CallClient* const sender_;
  NetworkNode* const send_net_;
  const uint64_t receiver_id_;
  const DataSize packet_overhead_;
};

// CrossTrafficSource is created by a Scenario and generates cross traffic. It
// provides methods to access and print internal state.
class CrossTrafficSource {
 public:
  DataRate TrafficRate() const;
  ColumnPrinter StatsPrinter();

 private:
  friend class Scenario;
  CrossTrafficSource(NetworkReceiverInterface* target,
                     uint64_t receiver_id,
                     CrossTrafficConfig config);
  void Process(TimeDelta delta);

  NetworkReceiverInterface* const target_;
  const uint64_t receiver_id_;
  CrossTrafficConfig config_;
  webrtc::Random random_;

  TimeDelta time_since_update_ = TimeDelta::Zero();
  double intensity_ = 0;
  DataSize pending_size_ = DataSize::Zero();
};
}  // namespace test
}  // namespace webrtc
#endif  // TEST_SCENARIO_NETWORK_NODE_H_
