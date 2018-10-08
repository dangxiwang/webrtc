/*
 *  Copyright 2017 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "p2p/base/icecredentialsfactory.h"
#include "absl/memory/memory.h"
#include "p2p/base/p2pconstants.h"
#include "rtc_base/helpers.h"

namespace cricket {

IceCredentialsFactory::IceCredentialsFactory() {}

IceCredentialsFactory::~IceCredentialsFactory() {}

IceParameters IceCredentialsFactory::AllocateIceCredentials() {
  generated_ice_credentials_.emplace_front(CreateRandomIceCredentials());
  return generated_ice_credentials_.front();
}

void IceCredentialsFactory::ConsumeIceCredentials(
    const IceParameters& credentials) {
  generated_ice_credentials_.erase(
      std::remove(generated_ice_credentials_.begin(),
                  generated_ice_credentials_.end(), credentials),
      generated_ice_credentials_.end());
}

IceParameters IceCredentialsFactory::CreateRandomIceCredentials() {
  return IceParameters(rtc::CreateRandomString(ICE_UFRAG_LENGTH),
                       rtc::CreateRandomString(ICE_PWD_LENGTH), false);
}

IceCredentialsIterator::IceCredentialsIterator(
    const std::deque<IceParameters>& credentials)
    : ice_credentials_(credentials) {}

IceCredentialsIterator::~IceCredentialsIterator() {}

IceParameters IceCredentialsIterator::GetIceCredentials() {
  if (ice_credentials_.empty()) {
    return IceCredentialsFactory::CreateRandomIceCredentials();
  }
  IceParameters credentials = ice_credentials_.front();
  ice_credentials_.pop_front();
  return credentials;
}

std::unique_ptr<IceCredentialsIterator>
IceCredentialsFactory::GetIceCredentialsIterator() const {
  return absl::make_unique<IceCredentialsIterator>(generated_ice_credentials_);
}

}  // namespace cricket
