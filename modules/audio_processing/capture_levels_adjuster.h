/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef MODULES_AUDIO_PROCESSING_CAPTURE_LEVELS_ADJUSTER_H_
#define MODULES_AUDIO_PROCESSING_CAPTURE_LEVELS_ADJUSTER_H_

#include <stddef.h>

#include "modules/audio_processing/audio_buffer.h"
#include "modules/audio_processing/audio_samples_scaler.h"

namespace webrtc {

// Adjusts the level of the capture signal before and after all capture-side
// processing is done. The pre-adjustment is achieved by combining the gain
// value pre_gain and the level pre_gain_level to form a combined gain
// pre_gain*pre_gain_level/255 which is multiplied to each sample. The intention
// of the pre_gain_level is to be controlled by the analog AGC functionality.
// The post level adjustment is achieved by multiplying each sample with the
// value of post_gain. Any changes in the gains take are done smoothly over one
// frame and the scaled samples are clamped to fit into the allowed S16 sample
// range.

class CaptureLevelsAdjuster {
 public:
  CaptureLevelsAdjuster(int pre_gain_level, float pre_gain, float post_gain);
  CaptureLevelsAdjuster(const CaptureLevelsAdjuster&) = delete;
  CaptureLevelsAdjuster& operator=(const CaptureLevelsAdjuster&) = delete;

  // Adjusts the level of the signal before any of the other processing is
  // performed.
  void PreLevelAdjustment(AudioBuffer& audio_buffer);

  // Adjusts the level of the signal after all of the other processing have been
  // performed.
  void PostLevelAdjustment(AudioBuffer& audio_buffer);

  // Sets the gain to apply to each sample before any of the other processing is
  // performed.
  void SetPreGain(float pre_gain);

  // Sets the gain to apply to each sample after all of the other processing
  // have been performed.
  void SetPostGain(float post_gain);

  // Sets the analog gain level to use for the emulated analog gain.
  // `pre_gain_level` must be in the range [0...255].
  void SetPreGainLevel(int pre_gain_level);

 private:
  int pre_gain_level_;
  float pre_gain_;
  AudioSamplesScaler pre_scaler_;
  AudioSamplesScaler post_scaler_;
};
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_CAPTURE_LEVELS_ADJUSTER_H_
