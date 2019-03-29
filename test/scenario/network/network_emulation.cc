/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "test/scenario/network/network_emulation.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "rtc_base/bind.h"
#include "rtc_base/logging.h"

namespace webrtc {

EmulatedIpPacket::EmulatedIpPacket(const rtc::SocketAddress& from,
                                   const rtc::SocketAddress& to,
                                   uint64_t dest_endpoint_id,
                                   rtc::CopyOnWriteBuffer data,
                                   Timestamp arrival_time)
    : from(from),
      to(to),
      dest_endpoint_id(dest_endpoint_id),
      data(data),
      arrival_time(arrival_time) {}
EmulatedIpPacket::~EmulatedIpPacket() = default;
EmulatedIpPacket::EmulatedIpPacket(EmulatedIpPacket&&) = default;
EmulatedIpPacket& EmulatedIpPacket::operator=(EmulatedIpPacket&&) = default;

void EmulatedNetworkNode::CreateRoute(
    uint64_t receiver_id,
    std::vector<EmulatedNetworkNode*> nodes,
    EmulatedNetworkReceiverInterface* receiver) {
  RTC_CHECK(!nodes.empty());
  for (size_t i = 0; i + 1 < nodes.size(); ++i)
    nodes[i]->SetReceiver(receiver_id, nodes[i + 1]);
  nodes.back()->SetReceiver(receiver_id, receiver);
}

void EmulatedNetworkNode::ClearRoute(uint64_t receiver_id,
                                     std::vector<EmulatedNetworkNode*> nodes) {
  for (EmulatedNetworkNode* node : nodes)
    node->RemoveReceiver(receiver_id);
}

EmulatedNetworkNode::EmulatedNetworkNode(
    Clock* clock,
    rtc::TaskQueue* task_queue,
    std::unique_ptr<NetworkBehaviorInterface> network_behavior)
    : clock_(clock),
      task_queue_(task_queue),
      network_behavior_(std::move(network_behavior)) {}

EmulatedNetworkNode::~EmulatedNetworkNode() = default;

void EmulatedNetworkNode::OnPacketReceived(EmulatedIpPacket packet) {
  struct Closure {
    void operator()() {
      RTC_DCHECK_RUN_ON(node->task_queue_);
      node->HandlePacketReceived(std::move(packet));
    }
    EmulatedNetworkNode* node;
    EmulatedIpPacket packet;
  };
  task_queue_->PostTask(Closure{this, std::move(packet)});
}

void EmulatedNetworkNode::HandlePacketReceived(EmulatedIpPacket packet) {
  if (routing_.find(packet.dest_endpoint_id) == routing_.end()) {
    return;
  }
  uint64_t packet_id = next_packet_id_++;
  bool sent = network_behavior_->EnqueuePacket(
      PacketInFlightInfo(packet.size(), packet.arrival_time.us(), packet_id));
  if (sent) {
    packets_.emplace_back(StoredPacket{packet_id, std::move(packet), false});
  }
  if (!process_task_.Running()) {
    process_task_ = RepeatingTaskHandle::Start(task_queue_->Get(), [this]() {
      RTC_DCHECK_RUN_ON(task_queue_);
      return Process();
    });
  }
}

TimeDelta EmulatedNetworkNode::Process() {
  absl::optional<int64_t> next_time_us =
      network_behavior_->NextDeliveryTimeUs();
  int64_t current_time_us = clock_->TimeInMicroseconds();
  if (next_time_us && *next_time_us <= current_time_us) {
    Process(Timestamp::us(current_time_us));
    next_time_us = network_behavior_->NextDeliveryTimeUs();
    if (next_time_us) {
      return Timestamp::us(*next_time_us) - Timestamp::us(current_time_us);
    }
  }
  process_task_.Stop();
  return TimeDelta::Zero();  // This is ignored.
}

void EmulatedNetworkNode::Process(Timestamp at_time) {
  std::vector<PacketDeliveryInfo> delivery_infos =
      network_behavior_->DequeueDeliverablePackets(at_time.us());
  for (PacketDeliveryInfo& delivery_info : delivery_infos) {
    StoredPacket* packet = nullptr;
    for (auto& stored_packet : packets_) {
      if (stored_packet.id == delivery_info.packet_id) {
        packet = &stored_packet;
        break;
      }
    }
    RTC_CHECK(packet);
    RTC_DCHECK(!packet->removed);
    auto receiver_it = routing_.find(packet->packet.dest_endpoint_id);
    RTC_CHECK(receiver_it != routing_.end());
    packet->removed = true;

    // We don't want to keep the lock here. Otherwise we would get a deadlock if
    // the receiver tries to push a new packet.
    if (delivery_info.receive_time_us != PacketDeliveryInfo::kNotReceived) {
      packet->packet.arrival_time =
          Timestamp::us(delivery_info.receive_time_us);
      receiver_it->second->OnPacketReceived(std::move(packet->packet));
    }
    while (!packets_.empty() && packets_.front().removed) {
      packets_.pop_front();
    }
  }
}

void EmulatedNetworkNode::SetReceiver(
    uint64_t dest_endpoint_id,
    EmulatedNetworkReceiverInterface* receiver) {
  task_queue_->PostTask([=] {
    RTC_DCHECK_RUN_ON(task_queue_);
    auto route = routing_.insert({dest_endpoint_id, receiver});
    RTC_CHECK(route.second)
        << "Routing for endpoint " << dest_endpoint_id << " already exists";
  });
}

void EmulatedNetworkNode::RemoveReceiver(uint64_t dest_endpoint_id) {
  RTC_DCHECK_RUN_ON(task_queue_);
  routing_.erase(dest_endpoint_id);
}

EmulatedEndpoint::EmulatedEndpoint(uint64_t id,
                                   rtc::IPAddress ip,
                                   bool is_enabled,
                                   Clock* clock)
    : id_(id),
      peer_local_addr_(ip),
      is_enabled_(is_enabled),
      send_node_(nullptr),
      clock_(clock),
      next_port_(kFirstEphemeralPort),
      connected_endpoint_id_(absl::nullopt) {
  constexpr int kIPv4NetworkPrefixLength = 24;
  constexpr int kIPv6NetworkPrefixLength = 64;

  int prefix_length = 0;
  if (ip.family() == AF_INET) {
    prefix_length = kIPv4NetworkPrefixLength;
  } else if (ip.family() == AF_INET6) {
    prefix_length = kIPv6NetworkPrefixLength;
  }
  rtc::IPAddress prefix = TruncateIP(ip, prefix_length);
  network_ = absl::make_unique<rtc::Network>(
      ip.ToString(), "Endpoint id=" + std::to_string(id_), prefix,
      prefix_length, rtc::AdapterType::ADAPTER_TYPE_UNKNOWN);
  network_->AddIP(ip);

  enabled_state_checker_.DetachFromThread();
}
EmulatedEndpoint::~EmulatedEndpoint() = default;

uint64_t EmulatedEndpoint::GetId() const {
  return id_;
}

void EmulatedEndpoint::SetSendNode(EmulatedNetworkNode* send_node) {
  send_node_ = send_node;
}

void EmulatedEndpoint::SendPacket(const rtc::SocketAddress& from,
                                  const rtc::SocketAddress& to,
                                  rtc::CopyOnWriteBuffer packet) {
  RTC_CHECK(from.ipaddr() == peer_local_addr_);
  RTC_CHECK(connected_endpoint_id_);
  RTC_CHECK(send_node_);
  send_node_->OnPacketReceived(EmulatedIpPacket(
      from, to, connected_endpoint_id_.value(), std::move(packet),
      Timestamp::us(clock_->TimeInMicroseconds())));
}

absl::optional<uint16_t> EmulatedEndpoint::BindReceiver(
    uint16_t desired_port,
    EmulatedNetworkReceiverInterface* receiver) {
  rtc::CritScope crit(&receiver_lock_);
  uint16_t port = desired_port;
  if (port == 0) {
    // Because client can specify its own port, next_port_ can be already in
    // use, so we need to find next available port.
    int ports_pool_size =
        std::numeric_limits<uint16_t>::max() - kFirstEphemeralPort + 1;
    for (int i = 0; i < ports_pool_size; ++i) {
      uint16_t next_port = NextPort();
      if (port_to_receiver_.find(next_port) == port_to_receiver_.end()) {
        port = next_port;
        break;
      }
    }
  }
  RTC_CHECK(port != 0) << "Can't find free port for receiver in endpoint "
                       << id_;
  bool result = port_to_receiver_.insert({port, receiver}).second;
  if (!result) {
    RTC_LOG(INFO) << "Can't bind receiver to used port " << desired_port
                  << " in endpoint " << id_;
    return absl::nullopt;
  }
  RTC_LOG(INFO) << "New receiver is binded to endpoint " << id_ << " on port "
                << port;
  return port;
}

uint16_t EmulatedEndpoint::NextPort() {
  uint16_t out = next_port_;
  if (next_port_ == std::numeric_limits<uint16_t>::max()) {
    next_port_ = kFirstEphemeralPort;
  } else {
    next_port_++;
  }
  return out;
}

void EmulatedEndpoint::UnbindReceiver(uint16_t port) {
  rtc::CritScope crit(&receiver_lock_);
  port_to_receiver_.erase(port);
}

rtc::IPAddress EmulatedEndpoint::GetPeerLocalAddress() const {
  return peer_local_addr_;
}

void EmulatedEndpoint::OnPacketReceived(EmulatedIpPacket packet) {
  RTC_CHECK(packet.dest_endpoint_id == id_)
      << "Routing error: wrong destination endpoint. Destination id: "
      << packet.dest_endpoint_id << "; Receiver id: " << id_;
  rtc::CritScope crit(&receiver_lock_);
  auto it = port_to_receiver_.find(packet.to.port());
  if (it == port_to_receiver_.end()) {
    // It can happen, that remote peer closed connection, but there still some
    // packets, that are going to it. It can happen during peer connection close
    // process: one peer closed connection, second still sending data.
    RTC_LOG(INFO) << "No receiver registered in " << id_ << " on port "
                  << packet.to.port();
    return;
  }
  // Endpoint assumes frequent calls to bind and unbind methods, so it holds
  // lock during packet processing to ensure that receiver won't be deleted
  // before call to OnPacketReceived.
  it->second->OnPacketReceived(std::move(packet));
}

void EmulatedEndpoint::Enable() {
  RTC_DCHECK_RUN_ON(&enabled_state_checker_);
  RTC_CHECK(!is_enabled_);
  is_enabled_ = true;
}

void EmulatedEndpoint::Disable() {
  RTC_DCHECK_RUN_ON(&enabled_state_checker_);
  RTC_CHECK(is_enabled_);
  is_enabled_ = false;
}

bool EmulatedEndpoint::Enabled() const {
  RTC_DCHECK_RUN_ON(&enabled_state_checker_);
  return is_enabled_;
}

EmulatedNetworkNode* EmulatedEndpoint::GetSendNode() const {
  return send_node_;
}

void EmulatedEndpoint::SetConnectedEndpointId(uint64_t endpoint_id) {
  connected_endpoint_id_ = endpoint_id;
}

EndpointsContainer::EndpointsContainer(
    const std::vector<EmulatedEndpoint*>& endpoints)
    : endpoints_(endpoints) {}

EmulatedEndpoint* EndpointsContainer::LookupByLocalAddress(
    const rtc::IPAddress& local_ip) const {
  for (auto* endpoint : endpoints_) {
    rtc::IPAddress peerLocalAddress = endpoint->GetPeerLocalAddress();
    if (peerLocalAddress == local_ip) {
      return endpoint;
    }
  }
  RTC_CHECK(false) << "No network found for address" << local_ip.ToString();
}

bool EndpointsContainer::HasEndpoint(EmulatedEndpoint* endpoint) const {
  for (auto* e : endpoints_) {
    if (e->GetId() == endpoint->GetId()) {
      return true;
    }
  }
  return false;
}

std::vector<std::unique_ptr<rtc::Network>>
EndpointsContainer::GetEnabledNetworks() const {
  std::vector<std::unique_ptr<rtc::Network>> networks;
  for (auto* endpoint : endpoints_) {
    if (endpoint->Enabled()) {
      networks.emplace_back(
          absl::make_unique<rtc::Network>(endpoint->network()));
    }
  }
  return networks;
}

}  // namespace webrtc
