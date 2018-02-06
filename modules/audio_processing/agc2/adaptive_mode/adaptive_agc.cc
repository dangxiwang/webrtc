/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/adaptive_mode/adaptive_agc.h"

#include "modules/audio_processing/agc2/adaptive_mode/apm_vad.h"
#include "modules/audio_processing/agc2/adaptive_mode/rms_level_estimator.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"

namespace webrtc {
AdaptiveAgc::AdaptiveAgc(ApmDataDumper* apm_data_dumper)
    : level_estimator_(new RmsLevelEstimator(apm_data_dumper)),
      vad_(new ApmVad()),
      apm_data_dumper_(apm_data_dumper) {
  RTC_DCHECK(apm_data_dumper);
}

AdaptiveAgc::~AdaptiveAgc() = default;

void AdaptiveAgc::Process(MutableFloatAudioFrame float_frame) {
  float level = 100;  // dummy value. TODO(aleloi): how do we do this
                      // better?
  // The VAD produces 3 estimates every 3rd frame. We want to feed all
  // to the level estimator, but only care about the last level it produces.
  for (const auto& vad_result : vad_->AnalyzeFrame(float_frame)) {
    apm_data_dumper_->DumpRaw("apm_vad_probability",
                              vad_result.speech_probability);
    apm_data_dumper_->DumpRaw("apm_vad_rms", vad_result.speech_rms_dbfs);
    level = level_estimator_->EstimateLevel(vad_result);
  }

  // Now do something with the level! Apply the gain!
}

}  // namespace webrtc
