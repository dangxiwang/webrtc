/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_ADAPTIVE_MODE_LEVEL_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_ADAPTIVE_MODE_LEVEL_ESTIMATOR_H_

#include "modules/audio_processing/agc2/adaptive_mode/vad_with_level.h"

namespace webrtc {

class AdaptiveModeLevelEstimator {
 public:
  virtual float EstimateLevel(VadWithLevel::LevelAndProbability vad_data) = 0;
  virtual ~AdaptiveModeLevelEstimator() = default;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_ADAPTIVE_MODE_LEVEL_ESTIMATOR_H_
