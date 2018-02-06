/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_APM_VAD_H_
#define MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_APM_VAD_H_

#include <vector>

#include "modules/audio_processing/agc2/adaptive_mode/vad_with_level.h"
#include "modules/audio_processing/vad/voice_activity_detector.h"

namespace webrtc {

// Note: if we're going to use the ApmVadWrapper, we should add the
// FloatAudioFrame functionality to voice_activity_detector.h instead!
// No need for an extra wrapper!
class ApmVad : public VadWithLevel {
 public:
  ApmVad() = default;
  ~ApmVad() override = default;
  std::vector<LevelAndProbability> AnalyzeFrame(FloatAudioFrame frame) override;

 private:
  VoiceActivityDetector vad_;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_APM_VAD_H_
