/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/adaptive_mode/saturation_protector.h"

namespace webrtc {

float SaturationProtector::CurrentMargin(
    VadWithLevel::LevelAndProbability vad_data,
    float last_speech_level_estimate) {
  last_margin_ = 20;
  return last_margin_;
}

float SaturationProtector::LastMargin() const {
  return last_margin_;
}

}  // namespace webrtc
