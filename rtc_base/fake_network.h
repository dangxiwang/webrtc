/*
 *  Copyright 2009 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_FAKE_NETWORK_H_
#define RTC_BASE_FAKE_NETWORK_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "rtc_base/checks.h"
#include "rtc_base/mdns_responder_interface.h"
#include "rtc_base/message_handler.h"
#include "rtc_base/network.h"
#include "rtc_base/socket_address.h"
#include "rtc_base/string_encode.h"
#include "rtc_base/thread.h"

namespace rtc {

const int kFakeIPv4NetworkPrefixLength = 24;
const int kFakeIPv6NetworkPrefixLength = 64;

// Fake network manager that allows us to manually specify the IPs to use.
class FakeNetworkManager : public NetworkManagerBase,
                           public MessageHandlerAutoCleanup {
 public:
  FakeNetworkManager() {}

  struct Iface {
    SocketAddress socket_address;
    AdapterType adapter_type;
    absl::optional<AdapterType> underlying_vpn_adapter_type;
  };
  typedef std::vector<Iface> IfaceList;

  void AddInterface(const SocketAddress& iface) {
    // Ensure a unique name for the interface if its name is not given.
    AddInterface(iface, "test" + rtc::ToString(next_index_++));
  }

  void AddInterface(const SocketAddress& iface, const std::string& if_name) {
    AddInterface(iface, if_name, ADAPTER_TYPE_UNKNOWN);
  }

  void AddInterface(
      const SocketAddress& iface,
      const std::string& if_name,
      AdapterType type,
      absl::optional<AdapterType> underlying_vpn_adapter_type = absl::nullopt) {
    SocketAddress address(if_name, 0);
    address.SetResolvedIP(iface.ipaddr());
    ifaces_.push_back({address, type, underlying_vpn_adapter_type});
    DoUpdateNetworks();
  }

  void RemoveInterface(const SocketAddress& iface) {
    for (IfaceList::iterator it = ifaces_.begin(); it != ifaces_.end(); ++it) {
      if (it->socket_address.EqualIPs(iface)) {
        ifaces_.erase(it);
        break;
      }
    }
    DoUpdateNetworks();
  }

  void StartUpdating() override {
    ++start_count_;
    if (start_count_ == 1) {
      sent_first_update_ = false;
      rtc::Thread::Current()->Post(RTC_FROM_HERE, this, kUpdateNetworksMessage);
    } else {
      if (sent_first_update_) {
        rtc::Thread::Current()->Post(RTC_FROM_HERE, this,
                                     kSignalNetworksMessage);
      }
    }
  }

  void StopUpdating() override { --start_count_; }

  // MessageHandler interface.
  void OnMessage(Message* msg) override {
    if (msg->message_id == kUpdateNetworksMessage) {
      DoUpdateNetworks();
    } else if (msg->message_id == kSignalNetworksMessage) {
      SignalNetworksChanged();
    } else {
      RTC_CHECK(false);
    }
  }

#if 0
  using NetworkManagerBase::set_default_local_addresses;
#endif
  using NetworkManagerBase::set_enumeration_permission;
  // rtc::NetworkManager override.
  webrtc::MdnsResponderInterface* GetMdnsResponder() const override {
    return mdns_responder_.get();
  }

  void set_mdns_responder(
      std::unique_ptr<webrtc::MdnsResponderInterface> mdns_responder) {
    mdns_responder_ = std::move(mdns_responder);
  }

 private:
  void DoUpdateNetworks() {
    if (start_count_ == 0)
      return;
    std::vector<Network*> networks;
    for (IfaceList::iterator it = ifaces_.begin(); it != ifaces_.end(); ++it) {
      int prefix_length = 0;
      if (it->socket_address.ipaddr().family() == AF_INET) {
        prefix_length = kFakeIPv4NetworkPrefixLength;
      } else if (it->socket_address.ipaddr().family() == AF_INET6) {
        prefix_length = kFakeIPv6NetworkPrefixLength;
      }
      IPAddress prefix = TruncateIP(it->socket_address.ipaddr(), prefix_length);
      std::unique_ptr<Network> net(new Network(
          it->socket_address.hostname(), it->socket_address.hostname(), prefix,
          prefix_length, it->adapter_type));
      if (it->underlying_vpn_adapter_type.has_value()) {
        net->set_underlying_type_for_vpn(*it->underlying_vpn_adapter_type);
      }
#if 0
      net->set_default_local_address_provider(this);
#endif
      net->AddIP(it->socket_address.ipaddr());
      networks.push_back(net.release());
    }
    bool changed;
    MergeNetworkList(networks, &changed);
    if (changed || !sent_first_update_) {
      SignalNetworksChanged();
      sent_first_update_ = true;
    }
  }

  IfaceList ifaces_;
  int next_index_ = 0;
  int start_count_ = 0;
  bool sent_first_update_ = false;

  static constexpr uint32_t kUpdateNetworksMessage = 1;
  static constexpr uint32_t kSignalNetworksMessage = 2;

  std::unique_ptr<webrtc::MdnsResponderInterface> mdns_responder_;
};

}  // namespace rtc

#endif  // RTC_BASE_FAKE_NETWORK_H_
