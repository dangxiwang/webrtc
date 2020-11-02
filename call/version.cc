/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "call/version.h"

namespace webrtc {

const char* const kSourceTimestamp =
    "WebRTC source timestamp 2020-09-02T09:49:00";

void LoadWebRTCVersionInRegister() {
#ifdef __GNUC__
  // This function loads the address of kSourceTimestamp into a register, and
  // then does nothing with it, but in such a way that the compiler can't tell
  // it's nothing. This prevents the compiler from eliminating the value even if
  // it's otherwise not used in the program.
  asm(""::"r"(kSourceTimestamp));
#endif
}

}  // namespace webrtc
