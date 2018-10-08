/*
 *  Copyright 2018 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <memory>
#include <string>
#include <vector>

#include "p2p/base/icecredentialsfactory.h"
#include "rtc_base/gunit.h"

using cricket::IceParameters;
using cricket::IceCredentialsFactory;

class IceCredentialsFactoryTest : public testing::Test {
 public:
  IceCredentialsFactoryTest() {}
  IceCredentialsFactory ice_credentials_factory_;
};

TEST_F(IceCredentialsFactoryTest, TestCreateAndIterate) {
  IceParameters credentials = ice_credentials_factory_.AllocateIceCredentials();
  EXPECT_EQ(1,
            ice_credentials_factory_.GetIceCredentialsIterator()->GetCount());
  EXPECT_EQ(credentials, ice_credentials_factory_.GetIceCredentialsIterator()
                             ->GetIceCredentials());
}

TEST_F(IceCredentialsFactoryTest, TestCreateAndIterate2) {
  IceParameters credentials1 =
      ice_credentials_factory_.AllocateIceCredentials();
  IceParameters credentials2 =
      ice_credentials_factory_.AllocateIceCredentials();

  auto iterator = ice_credentials_factory_.GetIceCredentialsIterator();
  EXPECT_EQ(2, iterator->GetCount());
  EXPECT_EQ(credentials2, iterator->GetIceCredentials());
  EXPECT_EQ(credentials1, iterator->GetIceCredentials());
  EXPECT_EQ(0, iterator->GetCount());
  // one can still get new (random) credentials.
  iterator->GetIceCredentials();
}

TEST_F(IceCredentialsFactoryTest, TestCreateAndConsume) {
  IceParameters credentials1 =
      ice_credentials_factory_.AllocateIceCredentials();
  IceParameters credentials2 =
      ice_credentials_factory_.AllocateIceCredentials();

  auto iterator = ice_credentials_factory_.GetIceCredentialsIterator();
  ice_credentials_factory_.ConsumeIceCredentials(credentials1);

  // The iterator has take a copy of the array.
  EXPECT_EQ(2, iterator->GetCount());
  EXPECT_EQ(credentials2, iterator->GetIceCredentials());
  EXPECT_EQ(credentials1, iterator->GetIceCredentials());

  auto iterator2 = ice_credentials_factory_.GetIceCredentialsIterator();
  EXPECT_EQ(1, iterator2->GetCount());
  EXPECT_EQ(credentials2, iterator2->GetIceCredentials());

  ice_credentials_factory_.ConsumeIceCredentials(credentials2);
  EXPECT_EQ(0,
            ice_credentials_factory_.GetIceCredentialsIterator()->GetCount());

  // one can still get new (random) credentials.
  ice_credentials_factory_.GetIceCredentialsIterator()->GetIceCredentials();
}
