/*
 *  Copyright 2018 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef P2P_BASE_ICECREDENTIALSFACTORY_H_
#define P2P_BASE_ICECREDENTIALSFACTORY_H_

#include <deque>
#include <string>
#include <utility>

namespace cricket {

// A caching factory for creating Icecredentials.
class IceCredentialsFactory {
 public:
  IceCredentialsFactory();
  virtual ~IceCredentialsFactory();

  // Create ice credentials and put them in cache.
  virtual std::pair<std::string, std::string> CreateIceCredentials();

  // Get ice credentials from cache if present, otherwise create them.
  virtual std::pair<std::string, std::string> GetIceCredentials();

 private:
  std::deque<std::pair<std::string, std::string>> ice_credentials_;
};

}  // namespace cricket

#endif  // P2P_BASE_ICECREDENTIALSFACTORY_H_
