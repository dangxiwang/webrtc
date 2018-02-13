/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/limiter.h"

#include "rtc_base/gunit.h"

namespace webrtc {
namespace {}  // namespace

TEST(FixedDigitalGainController2Limiter, ConstructDestruct) {
  Limiter l;
}

TEST(FixedDigitalGainController2Limiter, GainCurveShouldBeMonotone) {
  Limiter l;
  float last_output_level = 0.f;
  bool has_last_output_level = false;
  for (float level = -90; level <= l.max_input_level_db(); level += 0.5f) {
    const float current_output_level = l.GetOutputLevelDbfs(level);
    if (!has_last_output_level) {
      last_output_level = current_output_level;
      has_last_output_level = true;
    }
    EXPECT_LE(last_output_level, current_output_level);
    last_output_level = current_output_level;
  }
}

TEST(FixedDigitalGainController2Limiter, GainCurveShouldBeContinuous) {
  Limiter l;
  float last_output_level = 0.f;
  bool has_last_output_level = false;
  constexpr float kMaxDelta = 0.5f;
  for (float level = -90; level <= l.max_input_level_db(); level += 0.5f) {
    const float current_output_level = l.GetOutputLevelDbfs(level);
    if (!has_last_output_level) {
      last_output_level = current_output_level;
      has_last_output_level = true;
    }
    EXPECT_LE(current_output_level, last_output_level + kMaxDelta);
    last_output_level = current_output_level;
  }
}

TEST(FixedDigitalGainController2Limiter, OutputGainShouldBeLessThanFullScale) {
  Limiter l;
  for (float level = -90; level <= l.max_input_level_db(); level += 0.5f) {
    const float current_output_level = l.GetOutputLevelDbfs(level);
    EXPECT_LE(current_output_level, 0.f);
  }
}

}  // namespace webrtc
