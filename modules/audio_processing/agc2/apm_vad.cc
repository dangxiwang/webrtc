/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/apm_vad.h"

#include <algorithm>
#include <array>
#include <limits>

#include "common_audio/include/audio_util.h"

namespace webrtc {

std::vector<ApmVad::LevelAndProbability> ApmVad::AnalyzeFrame(
    AudioFrameView<const float> frame) {
  // First attempt: we only feed the first channel to the VAD. The VAD
  // takes int16 values. We convert the first channel to int16.
  std::array<int16_t, 480> first_channel_as_int;
  RTC_DCHECK_LE(frame.samples_per_channel(), 480);
  std::transform(frame.channel(0).begin(),
                 frame.channel(0).begin() + frame.samples_per_channel(),
                 first_channel_as_int.begin(), [](float f) {
                   RTC_DCHECK_LE(std::abs(f),
                                 std::numeric_limits<int16_t>::max());
                   return static_cast<int16_t>(
                       f);  // * std::numeric_limits<int16_t>::max();
                 });

  vad_.ProcessChunk(first_channel_as_int.begin(), frame.samples_per_channel(),
                    frame.samples_per_channel() * 100);

  const std::vector<double>& probabilities =
      vad_.chunkwise_voice_probabilities();

  const std::vector<double>& rms_values = vad_.chunkwise_rms();

  std::vector<LevelAndProbability> result;
  for (size_t i = 0; i < probabilities.size(); ++i) {
    result.emplace_back(probabilities[i], FloatS16ToDbfs(rms_values[i]), -90.f);
  }

  return result;
}

}  // namespace webrtc
