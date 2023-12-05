/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef CALL_CALL_FACTORY_H_
#define CALL_CALL_FACTORY_H_

#include <memory>

#include "api/sequence_checker.h"
#include "call/call.h"
#include "call/call_config.h"
#include "rtc_base/system/no_unique_address.h"

namespace webrtc {

// TODO(bugs.webrtc.org/15574): Delete CallFactory in favor of a free function
// to create `Call`.
class CallFactory {
 public:
  CallFactory();
  ~CallFactory() = default;

  std::unique_ptr<Call> CreateCall(const CallConfig& config);

 private:
  RTC_NO_UNIQUE_ADDRESS SequenceChecker call_thread_;
};

}  // namespace webrtc

#endif  // CALL_CALL_FACTORY_H_
