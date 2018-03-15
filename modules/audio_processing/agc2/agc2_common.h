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

#include "rtc_base/basictypes.h"

namespace webrtc {

constexpr float kMinFloatS16Value = -32768.f;
constexpr float kMaxFloatS16Value = 32767.f;
constexpr double kMaxAbsFloatS16Value = 32768.0;

constexpr size_t kFrameDurationMs = 10;
constexpr size_t kSubFramesInFrame = 20;
constexpr size_t kMaximalNumberOfSamplesPerChannel = 480;

constexpr float kAttackFilterConstant = 0.f;

// Adaptive Mode below.
constexpr float kMaxGainChangePerSecondDb = 3.f;
constexpr float kMaxGainChangePerFrameDb =
    kMaxGainChangePerSecondDb * kFrameDurationMs / 1000;
constexpr float kHeadroomDbfs = 1.f;
constexpr float kMaxGainDb = 40.f;  // Maybe make a wee bit smaller?
                                    // 40.f is a lot! It's a factor of
                                    // 100!

// Threshold 0.9 gave the most stable results with all history
// window sizes when using APM-VAD. With RNN-VAD, we should use 0.4
// instead.
constexpr float kVadConfidenceThreshold = 0.9;

// When using APM at confidence 0.9, this gives a target history of
// ~4 seconds (meaning it takes ~4 seconds of real input audio to
// fully adjust the level). If we use RNN(confidence = 0.4), this
// should be ~2500 instead for the same target history window size.
constexpr float kFullBufferSizeMs = 1000;

constexpr float kFullBufferLeakFactor = 1.f - 1.f / kFullBufferSizeMs;

constexpr float kInitialSpeechLevelEstimateDbfs = -30.f;

constexpr float kInitialSaturationMarginDb = 17.f;

constexpr float kPeakEnveloperSuperFrameLengthMs = 500.f;

// The peak enveloper delay is 'kFullBufferSizeMs * DelayFactor'.
constexpr float kPeakEnveloperDelayFactor = 1.1f;  // TODO(aleloi): tune!

constexpr int kPeakEnveloperBufferSize =
    static_cast<int>(kPeakEnveloperDelayFactor * kFullBufferSizeMs /
                         kPeakEnveloperSuperFrameLengthMs +
                     1);

// 500, 2500 - changes faster, 1000, 5000 - more conservative.
// kSatProcAttackMs = 1000
// kSatProcDecayMs = 5000
// 10 ** (-1/20 * frame_size_ms / satproc_attack_ms)
constexpr float kSaturationProtectorAttackConstant =
    0.9977000638225533;  // 0.9988493699365052;

// 10 ** (-1/20 * frame_size_ms / satproc_decay_ms)
constexpr float kSaturationProtectorDecayConstant =
    0.9995395890030878;  // 0.9997697679981565; //

// This is computed from kDecayMs by
// 10 ** (-1/20 * subframe_duration / kDecayMs).
// |subframe_duration| is |kFrameDurationMs / kSubFramesInFrame|.
// kDecayMs is defined in agc2_testing_common.h
constexpr float kDecayFilterConstant = 0.9998848773724686f;

// Number of interpolation points for each region of the limiter.
// These values have been tuned to limit the interpolated gain curve error given
// the limiter parameters and allowing a maximum error of +/- 32768^-1.
constexpr size_t kInterpolatedGainCurveKneePoints = 22;
constexpr size_t kInterpolatedGainCurveBeyondKneePoints = 10;
constexpr size_t kInterpolatedGainCurveTotalPoints =
    kInterpolatedGainCurveKneePoints + kInterpolatedGainCurveBeyondKneePoints;

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_AGC2_COMMON_H_
