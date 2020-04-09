/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_STRINGS_STRING_FORMAT_H_
#define RTC_BASE_STRINGS_STRING_FORMAT_H_

#include <cstdarg>
#include <string>

namespace rtc {
#if defined(__GNUC__)
__attribute__((__format__(__printf__, 1, 2)))
#endif
std::string
StringFormat(const char* fmt, ...);
}  // namespace rtc

#endif  // RTC_BASE_STRINGS_STRING_FORMAT_H_
