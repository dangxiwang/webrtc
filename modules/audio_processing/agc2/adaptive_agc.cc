/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/adaptive_agc.h"

#include "common_audio/include/audio_util.h"
#include "modules/audio_processing/agc2/rms_level_estimator.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "modules/audio_processing/vad/voice_activity_detector.h"

namespace webrtc {
AdaptiveAgc::AdaptiveAgc(ApmDataDumper* apm_data_dumper)
    : level_estimator_(new RmsLevelEstimator(apm_data_dumper)),
      vad_(new VoiceActivityDetector()),
      gain_applier_(apm_data_dumper),
      apm_data_dumper_(apm_data_dumper) {
  RTC_DCHECK(apm_data_dumper);
}

AdaptiveAgc::~AdaptiveAgc() = default;

void AdaptiveAgc::Process(AudioFrameView<float> float_frame) {
  // The VAD produces 3 estimates every 3rd frame. We want to feed all
  // to the level estimator, but only care about the last level it produces.
  rtc::ArrayView<const VadWithLevel::LevelAndProbability> vad_results =
      vad_->AnalyzeFrame(float_frame);
  for (const auto& vad_result : vad_results) {
    apm_data_dumper_->DumpRaw("agc2_vad_probability",
                              vad_result.speech_probability);
    apm_data_dumper_->DumpRaw("agc2_vad_rms_dbfs",
                              FloatS16ToDbfs(vad_result.speech_rms_linear));
    apm_data_dumper_->DumpRaw("agc2_vad_peak_dbfs",
                              FloatS16ToDbfs(vad_result.speech_peak_linear));
    level_estimator_->EstimateLevel(vad_result);
  }

  const float level = level_estimator_->LatestLevelEstimate();

  // Now do something with the level! Apply the gain!
  // Question: how does the gain applier know about the VAD results?
  gain_applier_.Process(level, vad_results, float_frame);
}

}  // namespace webrtc
