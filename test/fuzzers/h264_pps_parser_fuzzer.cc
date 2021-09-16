/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include <stddef.h>
#include <stdint.h>

#include "common_video/h264/pps_parser.h"

namespace webrtc {
void FuzzOneInput(const uint8_t* data, size_t size) {
  PpsParser::ParsePps(data, size);
}
}  // namespace webrtc
