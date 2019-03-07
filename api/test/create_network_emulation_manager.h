
/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_TEST_CREATE_NETWORK_EMULATION_MANAGER_H_
#define API_TEST_CREATE_NETWORK_EMULATION_MANAGER_H_

#include <memory>

#include "api/test/network_emulation_manager_interface.h"

namespace webrtc {
namespace test {

std::unique_ptr<NetworkEmulationManagerInterface>
CreateNetworkEmulationManager();

}  // namespace test
}  // namespace webrtc

#endif  // API_TEST_CREATE_NETWORK_EMULATION_MANAGER_H_
