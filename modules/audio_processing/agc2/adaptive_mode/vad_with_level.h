/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_VAD_WITH_LEVEL_H_
#define MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_VAD_WITH_LEVEL_H_

#include <vector>

#include "modules/audio_processing/include/float_audio_frame.h"

namespace webrtc {
class VadWithLevel {
 public:
  struct LevelAndProbability {
    LevelAndProbability(float prob, float rms, float peak)
        : speech_probability(prob),
          speech_rms_dbfs(rms),
          speech_peak_dbfs(peak) {}
    float speech_probability = 0;
    float speech_rms_dbfs = 0;  // Root mean square in decibel full-scale.
    float speech_peak_dbfs = 0;
  };

  // TODO(aleloi): We can't return std::vector<something> on every
  // frame. The vector thing is to make the interface work with
  // audio_processing/voice_activity_detector.h. If we're going for
  // that VAD, it should be accessible from adaptive_agc instead.
  virtual std::vector<LevelAndProbability> AnalyzeFrame(
      FloatAudioFrame frame) = 0;
  virtual ~VadWithLevel() = default;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_VAD_WITH_LEVEL_H_
