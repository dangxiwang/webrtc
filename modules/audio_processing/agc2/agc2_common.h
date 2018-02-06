/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_AGC2_COMMON_H_
#define MODULES_AUDIO_PROCESSING_AGC2_AGC2_COMMON_H_

#include <cmath>
#include <vector>

#include "rtc_base/basictypes.h"

namespace webrtc {

constexpr float kMinSampleValue = -32768.f;
constexpr float kMaxSampleValue = 32767.f;

constexpr double kInputLevelScaling = 32768.0;
const double kMinDbfs = -20.0 * std::log10(32768.0);

constexpr size_t kFrameDurationMs = 10;

// Limiter params.
constexpr double kLimiterMaxInputLevel = 1.0;
constexpr double kLimiterKneeSmoothness = 1.0;
constexpr double kLimiterCompressionRatio = 5.0;

// Number of interpolation points for each region of the limiter.
// These values have been tuned to limit the interpolated gain curve error given
// the limiter parameters and allowing a maximum error of +/- 32768^-1.
constexpr size_t kInterpolatedGainCurveKneePoints = 22;
constexpr size_t kInterpolatedGainCurveBeyondKneePoints = 10;

constexpr size_t kSubFramesInFrame = 20;

constexpr float kAttackFilterConstant = 0.f;
constexpr float kAttackMs = 0.f;

constexpr size_t kMaximalNumberOfSamplesPerChannel = 480;

// This is computed from kDecayMs by
// 10 ** (-1/20 * subframe_duration / kDecayMs).
// |subframe_duration| is |kFrameDurationMs / kSubFramesInFrame|.
constexpr float kDecayFilterConstant = 0.9998848773724686f;
constexpr float kDecayMs = 500.f;

double DbfsToLinear(const double level);

double LinearToDbfs(const double level);

std::vector<double> LinSpace(const double l, const double r, size_t num_points);

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_AGC2_COMMON_H_
