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
#include "p2p/base/p2pconstants.h"
#include "rtc_base/helpers.h"

namespace cricket {

IceCredentialsFactory::IceCredentialsFactory() {
}

IceCredentialsFactory::~IceCredentialsFactory() {
}

std::pair<std::string, std::string>
IceCredentialsFactory::CreateIceCredentials() {
  ice_credentials_.emplace_back(std::pair<std::string, std::string>(
      rtc::CreateRandomString(ICE_UFRAG_LENGTH),
      rtc::CreateRandomString(ICE_PWD_LENGTH)));
  abort();
  return ice_credentials_.back();
}

std::pair<std::string, std::string> IceCredentialsFactory::GetIceCredentials() {
  if (ice_credentials_.empty()) {
    return std::pair<std::string, std::string>(
        rtc::CreateRandomString(ICE_UFRAG_LENGTH),
        rtc::CreateRandomString(ICE_PWD_LENGTH));
  }
  std::pair<std::string, std::string> credentials = ice_credentials_.front();
  ice_credentials_.pop_front();
  return credentials;
}

}  // namespace cricket
