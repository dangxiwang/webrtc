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
#include <memory>

#include "p2p/base/transportdescription.h"

namespace cricket {

class IceCredentialsIterator {
 public:
  explicit IceCredentialsIterator(const std::deque<IceParameters>&);
  virtual ~IceCredentialsIterator();

  // Get and consume cached ice credentials.
  // Returns random after cache is empty.
  virtual IceParameters GetIceCredentials();

  // For test.
  int GetCount() { return ice_credentials_.size(); }

 private:
  std::deque<IceParameters> ice_credentials_;
};

// The IceCredentialsFactory creates randomly-generated ICE credentials
// (ufrag/pwd) with additional support for pre-generating credentials that
// can be used later.
//
class IceCredentialsFactory {
 public:
  IceCredentialsFactory();
  virtual ~IceCredentialsFactory();

  // Create ice credentials and put them in cache.
  virtual IceParameters AllocateIceCredentials();

  // Get iterator of ice credentials.
  virtual std::unique_ptr<IceCredentialsIterator> GetIceCredentialsIterator()
      const;

  // Remove ice credentials from cache if present.
  virtual void ConsumeIceCredentials(const IceParameters&);

  // static method to create random ice credentials.
  static IceParameters CreateRandomIceCredentials();

 private:
  std::deque<IceParameters> generated_ice_credentials_;
};

}  // namespace cricket

#endif  // P2P_BASE_ICECREDENTIALSFACTORY_H_
