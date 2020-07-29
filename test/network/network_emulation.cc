/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "test/network/network_emulation.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "api/units/data_size.h"
#include "rtc_base/bind.h"
#include "rtc_base/logging.h"

namespace webrtc {
namespace {

void AppendEmulatedNetworkOutgoingStats(
    const EmulatedNetworkOutgoingStats& source,
    EmulatedNetworkOutgoingStats* dest) {
  dest->packets_sent += source.packets_sent;
  dest->bytes_sent += source.bytes_sent;
  if (dest->first_packet_sent_time > source.first_packet_sent_time) {
    dest->first_packet_sent_time = source.first_packet_sent_time;
    dest->first_sent_packet_size = source.first_sent_packet_size;
  }
  if (dest->last_packet_sent_time < source.last_packet_sent_time) {
    dest->last_packet_sent_time = source.last_packet_sent_time;
  }
}

void AppendEmulatedNetworkIncomingStats(
    const EmulatedNetworkIncomingStats& source,
    EmulatedNetworkIncomingStats* dest) {
  dest->packets_received += source.packets_received;
  dest->bytes_received += source.bytes_received;
  dest->packets_dropped += source.packets_dropped;
  dest->bytes_dropped += source.bytes_dropped;
  if (dest->first_packet_received_time > source.first_packet_received_time) {
    dest->first_packet_received_time = source.first_packet_received_time;
    dest->first_received_packet_size = source.first_received_packet_size;
  }
  if (dest->last_packet_received_time < source.last_packet_received_time) {
    dest->last_packet_received_time = source.last_packet_received_time;
  }
}

}  // namespace

EmulatedNetworkOutgoingStats EmulatedNetworkStatsImpl::GetOverallOutgoingStats()
    const {
  EmulatedNetworkOutgoingStats stats;
  for (const auto& entry : outgoing_stats_per_destination_) {
    AppendEmulatedNetworkOutgoingStats(entry.second, &stats);
  }
  return stats;
}

EmulatedNetworkIncomingStats EmulatedNetworkStatsImpl::GetOverallIncomingStats()
    const {
  EmulatedNetworkIncomingStats stats;
  for (const auto& entry : incoming_stats_per_source_) {
    AppendEmulatedNetworkIncomingStats(entry.second, &stats);
  }
  return stats;
}

EmulatedNetworkStatsBuilder::EmulatedNetworkStatsBuilder() {
  sequence_checker_.Detach();
}
EmulatedNetworkStatsBuilder::EmulatedNetworkStatsBuilder(
    rtc::IPAddress local_ip) {
  local_addresses_.push_back(local_ip);
  sequence_checker_.Detach();
}

void EmulatedNetworkStatsBuilder::OnPacketSent(Timestamp sent_time,
                                               rtc::IPAddress dest_ip,
                                               DataSize packet_size) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  RTC_CHECK_GE(packet_size, DataSize::Zero());
  EmulatedNetworkOutgoingStats& dest_stats =
      outgoing_stats_per_destination_[dest_ip];
  if (dest_stats.first_packet_sent_time.IsInfinite()) {
    dest_stats.first_packet_sent_time = sent_time;
    dest_stats.first_sent_packet_size = packet_size;
  }
  dest_stats.last_packet_sent_time = sent_time;
  dest_stats.packets_sent++;
  dest_stats.bytes_sent += packet_size;
}

void EmulatedNetworkStatsBuilder::OnPacketDropped(rtc::IPAddress source_ip,
                                                  DataSize packet_size) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  RTC_CHECK_GE(packet_size, DataSize::Zero());
  EmulatedNetworkIncomingStats& source_stats =
      incoming_stats_per_source_[source_ip];
  source_stats.packets_dropped++;
  source_stats.bytes_dropped += packet_size;
}

void EmulatedNetworkStatsBuilder::OnPacketReceived(Timestamp received_time,
                                                   rtc::IPAddress source_ip,
                                                   DataSize packet_size) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  RTC_CHECK_GE(packet_size, DataSize::Zero());
  EmulatedNetworkIncomingStats& source_stats =
      incoming_stats_per_source_[source_ip];
  if (source_stats.first_packet_received_time.IsInfinite()) {
    source_stats.first_packet_received_time = received_time;
    source_stats.first_received_packet_size = packet_size;
  }
  source_stats.last_packet_received_time = received_time;
  source_stats.packets_received++;
  source_stats.bytes_received += packet_size;
}

void EmulatedNetworkStatsBuilder::AppendEmulatedNetworkStats(
    std::unique_ptr<EmulatedNetworkStats> stats) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  RTC_CHECK(stats);

  for (const rtc::IPAddress& addr : stats->LocalAddresses()) {
    local_addresses_.push_back(addr);
  }

  const std::map<rtc::IPAddress, EmulatedNetworkOutgoingStats>
      outgoing_stats_per_destination = stats->OutgoingStatsPerDestination();
  for (const auto& entry : outgoing_stats_per_destination) {
    AppendEmulatedNetworkOutgoingStats(
        entry.second, &outgoing_stats_per_destination_[entry.first]);
  }

  const std::map<rtc::IPAddress, EmulatedNetworkIncomingStats>
      incoming_stats_per_source = stats->IncomingStatsPerSource();
  for (const auto& entry : incoming_stats_per_source) {
    AppendEmulatedNetworkIncomingStats(
        entry.second, &incoming_stats_per_source_[entry.first]);
  }
}

std::unique_ptr<EmulatedNetworkStats> EmulatedNetworkStatsBuilder::Build()
    const {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  return std::make_unique<EmulatedNetworkStatsImpl>(
      local_addresses_, outgoing_stats_per_destination_,
      incoming_stats_per_source_);
}

void LinkEmulation::OnPacketReceived(EmulatedIpPacket packet) {
  task_queue_->PostTask([this, packet = std::move(packet)]() mutable {
    RTC_DCHECK_RUN_ON(task_queue_);

    uint64_t packet_id = next_packet_id_++;
    bool sent = network_behavior_->EnqueuePacket(PacketInFlightInfo(
        packet.ip_packet_size(), packet.arrival_time.us(), packet_id));
    if (sent) {
      packets_.emplace_back(StoredPacket{packet_id, std::move(packet), false});
    }
    if (process_task_.Running())
      return;
    absl::optional<int64_t> next_time_us =
        network_behavior_->NextDeliveryTimeUs();
    if (!next_time_us)
      return;
    Timestamp current_time = clock_->CurrentTime();
    process_task_ = RepeatingTaskHandle::DelayedStart(
        task_queue_->Get(),
        std::max(TimeDelta::Zero(),
                 Timestamp::Micros(*next_time_us) - current_time),
        [this]() {
          RTC_DCHECK_RUN_ON(task_queue_);
          Timestamp current_time = clock_->CurrentTime();
          Process(current_time);
          absl::optional<int64_t> next_time_us =
              network_behavior_->NextDeliveryTimeUs();
          if (!next_time_us) {
            process_task_.Stop();
            return TimeDelta::Zero();  // This is ignored.
          }
          RTC_DCHECK_GE(*next_time_us, current_time.us());
          return Timestamp::Micros(*next_time_us) - current_time;
        });
  });
}

void LinkEmulation::Process(Timestamp at_time) {
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
    packet->removed = true;

    if (delivery_info.receive_time_us != PacketDeliveryInfo::kNotReceived) {
      packet->packet.arrival_time =
          Timestamp::Micros(delivery_info.receive_time_us);
      receiver_->OnPacketReceived(std::move(packet->packet));
    }
    while (!packets_.empty() && packets_.front().removed) {
      packets_.pop_front();
    }
  }
}

NetworkRouterNode::NetworkRouterNode(rtc::TaskQueue* task_queue)
    : task_queue_(task_queue) {}

void NetworkRouterNode::OnPacketReceived(EmulatedIpPacket packet) {
  RTC_DCHECK_RUN_ON(task_queue_);
  if (watcher_) {
    watcher_(packet);
  }
  if (filter_) {
    if (!filter_(packet))
      return;
  }
  auto receiver_it = routing_.find(packet.to.ipaddr());
  if (receiver_it == routing_.end()) {
    return;
  }
  RTC_CHECK(receiver_it != routing_.end());

  receiver_it->second->OnPacketReceived(std::move(packet));
}

void NetworkRouterNode::SetReceiver(
    const rtc::IPAddress& dest_ip,
    EmulatedNetworkReceiverInterface* receiver) {
  task_queue_->PostTask([=] {
    RTC_DCHECK_RUN_ON(task_queue_);
    EmulatedNetworkReceiverInterface* cur_receiver = routing_[dest_ip];
    RTC_CHECK(cur_receiver == nullptr || cur_receiver == receiver)
        << "Routing for dest_ip=" << dest_ip.ToString() << " already exists";
    routing_[dest_ip] = receiver;
  });
}

void NetworkRouterNode::RemoveReceiver(const rtc::IPAddress& dest_ip) {
  RTC_DCHECK_RUN_ON(task_queue_);
  routing_.erase(dest_ip);
}

void NetworkRouterNode::SetWatcher(
    std::function<void(const EmulatedIpPacket&)> watcher) {
  task_queue_->PostTask([=] {
    RTC_DCHECK_RUN_ON(task_queue_);
    watcher_ = watcher;
  });
}

void NetworkRouterNode::SetFilter(
    std::function<bool(const EmulatedIpPacket&)> filter) {
  task_queue_->PostTask([=] {
    RTC_DCHECK_RUN_ON(task_queue_);
    filter_ = filter;
  });
}

EmulatedNetworkNode::EmulatedNetworkNode(
    Clock* clock,
    rtc::TaskQueue* task_queue,
    std::unique_ptr<NetworkBehaviorInterface> network_behavior)
    : router_(task_queue),
      link_(clock, task_queue, std::move(network_behavior), &router_) {}

void EmulatedNetworkNode::OnPacketReceived(EmulatedIpPacket packet) {
  link_.OnPacketReceived(std::move(packet));
}

void EmulatedNetworkNode::CreateRoute(
    const rtc::IPAddress& receiver_ip,
    std::vector<EmulatedNetworkNode*> nodes,
    EmulatedNetworkReceiverInterface* receiver) {
  RTC_CHECK(!nodes.empty());
  for (size_t i = 0; i + 1 < nodes.size(); ++i)
    nodes[i]->router()->SetReceiver(receiver_ip, nodes[i + 1]);
  nodes.back()->router()->SetReceiver(receiver_ip, receiver);
}

void EmulatedNetworkNode::ClearRoute(const rtc::IPAddress& receiver_ip,
                                     std::vector<EmulatedNetworkNode*> nodes) {
  for (EmulatedNetworkNode* node : nodes)
    node->router()->RemoveReceiver(receiver_ip);
}

EmulatedNetworkNode::~EmulatedNetworkNode() = default;

EmulatedEndpointImpl::EmulatedEndpointImpl(uint64_t id,
                                           const rtc::IPAddress& ip,
                                           bool is_enabled,
                                           rtc::AdapterType type,
                                           rtc::TaskQueue* task_queue,
                                           Clock* clock)
    : id_(id),
      peer_local_addr_(ip),
      is_enabled_(is_enabled),
      type_(type),
      clock_(clock),
      task_queue_(task_queue),
      router_(task_queue_),
      next_port_(kFirstEphemeralPort),
      stats_builder_(peer_local_addr_) {
  constexpr int kIPv4NetworkPrefixLength = 24;
  constexpr int kIPv6NetworkPrefixLength = 64;

  int prefix_length = 0;
  if (ip.family() == AF_INET) {
    prefix_length = kIPv4NetworkPrefixLength;
  } else if (ip.family() == AF_INET6) {
    prefix_length = kIPv6NetworkPrefixLength;
  }
  rtc::IPAddress prefix = TruncateIP(ip, prefix_length);
  network_ = std::make_unique<rtc::Network>(
      ip.ToString(), "Endpoint id=" + std::to_string(id_), prefix,
      prefix_length, type_);
  network_->AddIP(ip);

  enabled_state_checker_.Detach();
}
EmulatedEndpointImpl::~EmulatedEndpointImpl() = default;

uint64_t EmulatedEndpointImpl::GetId() const {
  return id_;
}

void EmulatedEndpointImpl::SendPacket(const rtc::SocketAddress& from,
                                      const rtc::SocketAddress& to,
                                      rtc::CopyOnWriteBuffer packet_data,
                                      uint16_t application_overhead) {
  RTC_CHECK(from.ipaddr() == peer_local_addr_);
  EmulatedIpPacket packet(from, to, std::move(packet_data),
                          clock_->CurrentTime(), application_overhead);
  task_queue_->PostTask([this, packet = std::move(packet)]() mutable {
    RTC_DCHECK_RUN_ON(task_queue_);
    stats_builder_.OnPacketSent(clock_->CurrentTime(), packet.to.ipaddr(),
                                DataSize::Bytes(packet.ip_packet_size()));

    router_.OnPacketReceived(std::move(packet));
  });
}

absl::optional<uint16_t> EmulatedEndpointImpl::BindReceiver(
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

uint16_t EmulatedEndpointImpl::NextPort() {
  uint16_t out = next_port_;
  if (next_port_ == std::numeric_limits<uint16_t>::max()) {
    next_port_ = kFirstEphemeralPort;
  } else {
    next_port_++;
  }
  return out;
}

void EmulatedEndpointImpl::UnbindReceiver(uint16_t port) {
  rtc::CritScope crit(&receiver_lock_);
  port_to_receiver_.erase(port);
}

rtc::IPAddress EmulatedEndpointImpl::GetPeerLocalAddress() const {
  return peer_local_addr_;
}

void EmulatedEndpointImpl::OnPacketReceived(EmulatedIpPacket packet) {
  RTC_DCHECK_RUN_ON(task_queue_);
  RTC_CHECK(packet.to.ipaddr() == peer_local_addr_)
      << "Routing error: wrong destination endpoint. Packet.to.ipaddr()=: "
      << packet.to.ipaddr().ToString()
      << "; Receiver peer_local_addr_=" << peer_local_addr_.ToString();
  rtc::CritScope crit(&receiver_lock_);
  stats_builder_.OnPacketReceived(clock_->CurrentTime(), packet.from.ipaddr(),
                                  DataSize::Bytes(packet.ip_packet_size()));
  auto it = port_to_receiver_.find(packet.to.port());
  if (it == port_to_receiver_.end()) {
    // It can happen, that remote peer closed connection, but there still some
    // packets, that are going to it. It can happen during peer connection close
    // process: one peer closed connection, second still sending data.
    RTC_LOG(INFO) << "Drop packet: no receiver registered in " << id_
                  << " on port " << packet.to.port();
    stats_builder_.OnPacketDropped(packet.from.ipaddr(),
                                   DataSize::Bytes(packet.ip_packet_size()));
    return;
  }
  // Endpoint assumes frequent calls to bind and unbind methods, so it holds
  // lock during packet processing to ensure that receiver won't be deleted
  // before call to OnPacketReceived.
  it->second->OnPacketReceived(std::move(packet));
}

void EmulatedEndpointImpl::Enable() {
  RTC_DCHECK_RUN_ON(&enabled_state_checker_);
  RTC_CHECK(!is_enabled_);
  is_enabled_ = true;
}

void EmulatedEndpointImpl::Disable() {
  RTC_DCHECK_RUN_ON(&enabled_state_checker_);
  RTC_CHECK(is_enabled_);
  is_enabled_ = false;
}

bool EmulatedEndpointImpl::Enabled() const {
  RTC_DCHECK_RUN_ON(&enabled_state_checker_);
  return is_enabled_;
}

std::unique_ptr<EmulatedNetworkStats> EmulatedEndpointImpl::stats() const {
  RTC_DCHECK_RUN_ON(task_queue_);
  return stats_builder_.Build();
}

EndpointsContainer::EndpointsContainer(
    const std::vector<EmulatedEndpointImpl*>& endpoints)
    : endpoints_(endpoints) {}

EmulatedEndpointImpl* EndpointsContainer::LookupByLocalAddress(
    const rtc::IPAddress& local_ip) const {
  for (auto* endpoint : endpoints_) {
    rtc::IPAddress peer_local_address = endpoint->GetPeerLocalAddress();
    if (peer_local_address == local_ip) {
      return endpoint;
    }
  }
  RTC_CHECK(false) << "No network found for address" << local_ip.ToString();
}

bool EndpointsContainer::HasEndpoint(EmulatedEndpointImpl* endpoint) const {
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
          std::make_unique<rtc::Network>(endpoint->network()));
    }
  }
  return networks;
}

std::vector<EmulatedEndpoint*> EndpointsContainer::endpoints() const {
  return std::vector<EmulatedEndpoint*>(endpoints_.begin(), endpoints_.end());
}

std::unique_ptr<EmulatedNetworkStats> EndpointsContainer::GetStats() const {
  EmulatedNetworkStatsBuilder stats_builder;
  for (auto* endpoint : endpoints_) {
    stats_builder.AppendEmulatedNetworkStats(endpoint->stats());
  }
  return stats_builder.Build();
}

}  // namespace webrtc
