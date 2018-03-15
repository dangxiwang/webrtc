/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_VAD_VAD_WITH_LEVEL_H_
#define MODULES_AUDIO_PROCESSING_VAD_VAD_WITH_LEVEL_H_

#include "api/array_view.h"
#include "modules/audio_processing/include/audio_frame_view.h"

namespace webrtc {
class VadWithLevel {
 public:
  struct LevelAndProbability {
    constexpr LevelAndProbability(float prob, float rms, float peak)
        : speech_probability(prob),
          speech_rms_linear(rms),
          speech_peak_linear(peak) {}
    LevelAndProbability() = default;
    float speech_probability = 0;
    float speech_rms_linear = 0;  // Root mean square in decibel full-scale.
    float speech_peak_linear = 0;
  };

  // TODO(aleloi): Make decisions about the interface.
  virtual rtc::ArrayView<const LevelAndProbability> AnalyzeFrame(
      AudioFrameView<const float> frame) = 0;
  virtual ~VadWithLevel() = default;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_VAD_VAD_WITH_LEVEL_H_
