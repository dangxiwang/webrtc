/*
 *  Copyright 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "network_monitor_factory.h"

#include "sdk/objc/native/src/objc_network_monitor.h"

#include "rtc_base/logging.h"

namespace webrtc {

std::unique_ptr<rtc::NetworkMonitorFactory> CreateNetworkMonitorFactory() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  return std::make_unique<ObjCNetworkMonitorFactory>();
}

}
