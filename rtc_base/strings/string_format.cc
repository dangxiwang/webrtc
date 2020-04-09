/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/strings/string_format.h"

#include "rtc_base/checks.h"

namespace rtc {

namespace {

constexpr int kMaxSize = 512;

}  // namespace

std::string StringFormat(const char* fmt, ...) {
  char buffer[kMaxSize];
  va_list args;
  va_start(args, fmt);
  int result = vsnprintf(buffer, kMaxSize, fmt, args);
  va_end(args);
  RTC_DCHECK(result >= 0 && result < kMaxSize);
  std::string str(buffer);
  return str;
}

}  // namespace rtc
