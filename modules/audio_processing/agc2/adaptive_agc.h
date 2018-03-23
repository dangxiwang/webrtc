/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_AGC_H_
#define MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_AGC_H_

#include <memory>

#include "modules/audio_processing/agc2/adaptive_digital_gain_applier.h"
#include "modules/audio_processing/agc2/adaptive_mode_level_estimator.h"
#include "modules/audio_processing/agc2/noise_level_estimator.h"
#include "modules/audio_processing/include/audio_frame_view.h"
#include "modules/audio_processing/vad/vad_with_level.h"

namespace webrtc {
class ApmDataDumper;

class AdaptiveAgc {
 public:
  explicit AdaptiveAgc(ApmDataDumper* apm_data_dumper);
  void Process(AudioFrameView<float> float_frame);
  ~AdaptiveAgc();

 private:
  AdaptiveModeLevelEstimator speech_level_estimator_;
  VadWithLevel vad_;
  AdaptiveDigitalGainApplier gain_applier_;
  ApmDataDumper* const apm_data_dumper_;
  NoiseLevelEstimator noise_level_estimator_;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_AGC_H_
