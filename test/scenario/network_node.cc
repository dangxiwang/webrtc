/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "test/scenario/network_node.h"

#include <algorithm>
#include <vector>

#include "absl/memory/memory.h"
#include "rtc_base/numerics/safe_minmax.h"

namespace webrtc {
namespace test {
namespace {
constexpr char kDummyTransportName[] = "dummy";
SimulatedNetwork::Config CreateSimulationConfig(
    NetworkSimulationConfig config) {
  SimulatedNetwork::Config sim_config;
  sim_config.link_capacity_kbps = config.bandwidth.kbps_or(0);
  sim_config.loss_percent = config.loss_rate * 100;
  sim_config.queue_delay_ms = config.delay.ms();
  sim_config.delay_standard_deviation_ms = config.delay_std_dev.ms();
  sim_config.packet_overhead = config.packet_overhead.bytes<int>();
  sim_config.codel_active_queue_management =
      config.codel_active_queue_management;
  return sim_config;
}
}  // namespace

void NullReceiver::OnPacketReceived(EmulatedIpPacket packet) {}

ActionReceiver::ActionReceiver(std::function<void()> action)
    : action_(action) {}

void ActionReceiver::OnPacketReceived(EmulatedIpPacket packet) {
  action_();
}

std::unique_ptr<SimulationNode> SimulationNode::Create(
    Clock* clock,
    rtc::TaskQueue* task_queue,
    NetworkSimulationConfig config) {
  SimulatedNetwork::Config sim_config = CreateSimulationConfig(config);
  auto network = absl::make_unique<SimulatedNetwork>(sim_config);
  SimulatedNetwork* simulation_ptr = network.get();
  return std::unique_ptr<SimulationNode>(new SimulationNode(
      clock, task_queue, config, std::move(network), simulation_ptr));
}

void SimulationNode::UpdateConfig(
    std::function<void(NetworkSimulationConfig*)> modifier) {
  modifier(&config_);
  SimulatedNetwork::Config sim_config = CreateSimulationConfig(config_);
  simulated_network_->SetConfig(sim_config);
}

void SimulationNode::PauseTransmissionUntil(Timestamp until) {
  simulated_network_->PauseTransmissionUntil(until.us());
}

ColumnPrinter SimulationNode::ConfigPrinter() const {
  return ColumnPrinter::Lambda(
      "propagation_delay capacity loss_rate",
      [this](rtc::SimpleStringBuilder& sb) {
        sb.AppendFormat("%.3lf %.0lf %.2lf", config_.delay.seconds<double>(),
                        config_.bandwidth.bps() / 8.0, config_.loss_rate);
      });
}

SimulationNode::SimulationNode(
    Clock* clock,
    rtc::TaskQueue* task_queue,
    NetworkSimulationConfig config,
    std::unique_ptr<NetworkBehaviorInterface> behavior,
    SimulatedNetwork* simulation)
    : EmulatedNetworkNode(clock, task_queue, std::move(behavior)),
      simulated_network_(simulation),
      config_(config) {}

NetworkNodeTransport::NetworkNodeTransport(Clock* sender_clock,
                                           Call* sender_call)
    : sender_clock_(sender_clock), sender_call_(sender_call) {}

NetworkNodeTransport::~NetworkNodeTransport() = default;

bool NetworkNodeTransport::SendRtp(const uint8_t* packet,
                                   size_t length,
                                   const PacketOptions& options) {
  int64_t send_time_ms = sender_clock_->TimeInMilliseconds();
  rtc::SentPacket sent_packet;
  sent_packet.packet_id = options.packet_id;
  sent_packet.info.included_in_feedback = options.included_in_feedback;
  sent_packet.info.included_in_allocation = options.included_in_allocation;
  sent_packet.send_time_ms = send_time_ms;
  sent_packet.info.packet_size_bytes = length;
  sent_packet.info.packet_type = rtc::PacketType::kData;
  sender_call_->OnSentPacket(sent_packet);

  Timestamp send_time = Timestamp::ms(send_time_ms);
  rtc::CritScope crit(&crit_sect_);
  if (!send_net_)
    return false;
  rtc::CopyOnWriteBuffer buffer(packet, length,
                                length + packet_overhead_.bytes());
  buffer.SetSize(length + packet_overhead_.bytes());
  send_net_->OnPacketReceived(
      EmulatedIpPacket(local_address_, receiver_address_, buffer, send_time));
  return true;
}

bool NetworkNodeTransport::SendRtcp(const uint8_t* packet, size_t length) {
  rtc::CopyOnWriteBuffer buffer(packet, length);
  Timestamp send_time = Timestamp::ms(sender_clock_->TimeInMilliseconds());
  rtc::CritScope crit(&crit_sect_);
  buffer.SetSize(length + packet_overhead_.bytes());
  if (!send_net_)
    return false;
  send_net_->OnPacketReceived(
      EmulatedIpPacket(local_address_, receiver_address_, buffer, send_time));
  return true;
}

void NetworkNodeTransport::Connect(EmulatedNetworkNode* send_node,
                                   rtc::IPAddress receiver_ip,
                                   DataSize packet_overhead) {
  rtc::NetworkRoute route;
  route.connected = true;
  route.local_network_id =
      static_cast<uint16_t>(receiver_ip.v4AddressAsHostOrderInteger());
  route.remote_network_id =
      static_cast<uint16_t>(receiver_ip.v4AddressAsHostOrderInteger());
  {
    // Only IPv4 address is supported. We don't use full range of IPs in
    // scenario framework and also we need a simple way to convert IP into
    // network_id to signal network route.
    RTC_CHECK_EQ(receiver_ip.family(), AF_INET);
    RTC_CHECK_LE(receiver_ip.v4AddressAsHostOrderInteger(),
                 std::numeric_limits<uint16_t>::max());
    rtc::CritScope crit(&crit_sect_);
    send_net_ = send_node;
    receiver_address_ = rtc::SocketAddress(receiver_ip, 0);
    packet_overhead_ = packet_overhead;
    current_network_route_ = route;
  }

  sender_call_->GetTransportControllerSend()->OnNetworkRouteChanged(
      kDummyTransportName, route);
}

void NetworkNodeTransport::Disconnect() {
  rtc::CritScope crit(&crit_sect_);
  current_network_route_.connected = false;
  sender_call_->GetTransportControllerSend()->OnNetworkRouteChanged(
      kDummyTransportName, current_network_route_);
  current_network_route_ = {};
  send_net_ = nullptr;
}

CrossTrafficSource::CrossTrafficSource(EmulatedNetworkReceiverInterface* target,
                                       rtc::IPAddress receiver_ip,
                                       CrossTrafficConfig config)
    : target_(target),
      receiver_address_(receiver_ip, 0),
      config_(config),
      random_(config.random_seed) {}

CrossTrafficSource::~CrossTrafficSource() = default;

DataRate CrossTrafficSource::TrafficRate() const {
  return config_.peak_rate * intensity_;
}

void CrossTrafficSource::Process(Timestamp at_time, TimeDelta delta) {
  time_since_update_ += delta;
  if (config_.mode == CrossTrafficConfig::Mode::kRandomWalk) {
    if (time_since_update_ >= config_.random_walk.update_interval) {
      intensity_ += random_.Gaussian(config_.random_walk.bias,
                                     config_.random_walk.variance) *
                    time_since_update_.seconds<double>();
      intensity_ = rtc::SafeClamp(intensity_, 0.0, 1.0);
      time_since_update_ = TimeDelta::Zero();
    }
  } else if (config_.mode == CrossTrafficConfig::Mode::kPulsedPeaks) {
    if (intensity_ == 0 && time_since_update_ >= config_.pulsed.hold_duration) {
      intensity_ = 1;
      time_since_update_ = TimeDelta::Zero();
    } else if (intensity_ == 1 &&
               time_since_update_ >= config_.pulsed.send_duration) {
      intensity_ = 0;
      time_since_update_ = TimeDelta::Zero();
    }
  }
  pending_size_ += TrafficRate() * delta;
  if (pending_size_ > config_.min_packet_size) {
    target_->OnPacketReceived(EmulatedIpPacket(
        /*from=*/rtc::SocketAddress(), receiver_address_,
        rtc::CopyOnWriteBuffer(pending_size_.bytes()), at_time));
    pending_size_ = DataSize::Zero();
  }
}

ColumnPrinter CrossTrafficSource::StatsPrinter() {
  return ColumnPrinter::Lambda("cross_traffic_rate",
                               [this](rtc::SimpleStringBuilder& sb) {
                                 sb.AppendFormat("%.0lf",
                                                 TrafficRate().bps() / 8.0);
                               },
                               32);
}

}  // namespace test
}  // namespace webrtc
