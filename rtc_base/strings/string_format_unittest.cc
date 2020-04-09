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
#include "test/gtest.h"

namespace rtc {

TEST(StringFormat, Basic) {
  EXPECT_EQ(StringFormat("%d = %s", 1 + 2, "three"), "3 = three");
}

}  // namespace rtc
