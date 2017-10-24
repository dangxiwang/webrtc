/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "video/transport_adapter.h"

#include "rtc_base/checks.h"

namespace webrtc {
namespace internal {

TransportAdapter::TransportAdapter(Transport* transport)
    : transport_(transport), enabled_(false) {
  RTC_DCHECK(nullptr != transport);
}

bool TransportAdapter::SendRtp(const uint8_t* packet,
                               size_t length,
                               const PacketOptions& options) {
  if (!enabled_.load())
    return false;

  return transport_->SendRtp(packet, length, options);
}

bool TransportAdapter::SendRtcp(const uint8_t* packet, size_t length) {
  if (!enabled_.load())
    return false;

  return transport_->SendRtcp(packet, length);
}

void TransportAdapter::Enable() {
  // If this exchange fails it means enabled_ was already true, no need to
  // check result and iterate.
  bool expectedValue = false;
  enabled_.compare_exchange_strong(expectedValue, true);
}

void TransportAdapter::Disable() {
  bool expectedValue = true;
  enabled_.compare_exchange_strong(expectedValue, false);
}

}  // namespace internal
}  // namespace webrtc
