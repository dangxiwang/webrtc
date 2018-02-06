/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_ADAPTIVE_AGC_H_
#define MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_ADAPTIVE_AGC_H_

#include <memory>

#include "modules/audio_processing/agc2/adaptive_mode/adaptive_mode_level_estimator.h"
#include "modules/audio_processing/agc2/adaptive_mode/vad_with_level.h"

#include "modules/audio_processing/include/float_audio_frame.h"

namespace webrtc {
class ApmDataDumper;

class AdaptiveAgc {
 public:
  explicit AdaptiveAgc(ApmDataDumper* apm_data_dumper);
  void Process(MutableFloatAudioFrame float_frame);
  ~AdaptiveAgc();

 private:
  // float gain_linear_ = ;
  std::unique_ptr<AdaptiveModeLevelEstimator> level_estimator_;
  std::unique_ptr<VadWithLevel> vad_;
  ApmDataDumper* const apm_data_dumper_;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_ADAPTIVE_AGC_H_
