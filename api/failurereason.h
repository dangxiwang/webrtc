/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_FAILUREREASON_H_
#define API_FAILUREREASON_H_

#include <string>
#include <utility>

#include "api/optional.h"

namespace webrtc {

struct FailureReason {
  FailureReason() : FailureReason(rtc::Optional<std::string>()) {}
  explicit FailureReason(std::string message)
      : FailureReason(rtc::Optional<std::string>(std::move(message))) {}
  explicit FailureReason(rtc::Optional<std::string> message)
      : message(message) {}

  rtc::Optional<std::string> message;
  // TODO(hbos): Add exception type enum. https://crbug.com/webrtc/8473
};

}  // namespace webrtc

#endif  // API_FAILUREREASON_H_
