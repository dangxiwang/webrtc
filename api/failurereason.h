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

#include "api/optional.h"

namespace webrtc {

struct FailureReason {
  explicit FailureReason(rtc::Optional<std::string> message)
      : message(message) {}

  rtc::Optional<std::string> message;
  // TODO(hbos): Add exception type enum.
};

}  // namespace webrtc

#endif  // API_FAILUREREASON_H_
