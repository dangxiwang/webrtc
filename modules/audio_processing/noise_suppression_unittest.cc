/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include <vector>

#include "api/array_view.h"
#include "modules/audio_processing/audio_buffer.h"
#include "modules/audio_processing/noise_suppression.h"
#include "modules/audio_processing/test/audio_buffer_tools.h"
#include "modules/audio_processing/test/bitexactness_tools.h"
#include "test/gtest.h"

namespace webrtc {
namespace {

const int kNumFramesToProcess = 1000;

// Process one frame of data and produce the output.
void ProcessOneFrame(int sample_rate_hz,
                     AudioBuffer* capture_buffer,
                     NoiseSuppression* noise_suppressor) {
  if (sample_rate_hz > AudioProcessing::kSampleRate16kHz) {
    capture_buffer->SplitIntoFrequencyBands();
  }

  noise_suppressor->AnalyzeCaptureAudio(capture_buffer);
  noise_suppressor->ProcessCaptureAudio(capture_buffer);

  if (sample_rate_hz > AudioProcessing::kSampleRate16kHz) {
    capture_buffer->MergeFrequencyBands();
  }
}

// Processes a specified amount of frames, verifies the results and reports
// any errors.
void RunBitexactnessTest(int sample_rate_hz,
                         size_t num_channels,
                         NoiseSuppression::Level level,
                         float speech_probability_reference,
                         rtc::ArrayView<const float> noise_estimate_reference,
                         rtc::ArrayView<const float> output_reference) {
  NoiseSuppression noise_suppressor(num_channels, sample_rate_hz, level);

  int samples_per_channel = rtc::CheckedDivExact(sample_rate_hz, 100);
  const StreamConfig capture_config(sample_rate_hz, num_channels, false);
  AudioBuffer capture_buffer(
      capture_config.sample_rate_hz(), capture_config.num_channels(),
      capture_config.sample_rate_hz(), capture_config.num_channels(),
      capture_config.sample_rate_hz(), capture_config.num_channels());
  test::InputAudioFile capture_file(
      test::GetApmCaptureTestVectorFileName(sample_rate_hz));
  std::vector<float> capture_input(samples_per_channel * num_channels);
  for (size_t frame_no = 0; frame_no < kNumFramesToProcess; ++frame_no) {
    ReadFloatSamplesFromStereoFile(samples_per_channel, num_channels,
                                   &capture_file, capture_input);

    test::CopyVectorToAudioBuffer(capture_config, capture_input,
                                  &capture_buffer);

    ProcessOneFrame(sample_rate_hz, &capture_buffer, &noise_suppressor);
  }

  // Extract test results.
  std::vector<float> capture_output;
  test::ExtractVectorFromAudioBuffer(capture_config, &capture_buffer,
                                     &capture_output);
  float speech_probability = noise_suppressor.speech_probability();
  std::vector<float> noise_estimate = noise_suppressor.NoiseEstimate();

  const float kVectorElementErrorBound = 1.0f / 32768.0f;
  EXPECT_FLOAT_EQ(speech_probability_reference, speech_probability);
  EXPECT_TRUE(test::VerifyArray(noise_estimate_reference, noise_estimate,
                                kVectorElementErrorBound));

  // Compare the output with the reference. Only the first values of the output
  // from last frame processed are compared in order not having to specify all
  // preceeding frames as testvectors. As the algorithm being tested has a
  // memory, testing only the last frame implicitly also tests the preceeding
  // frames.
  EXPECT_TRUE(test::VerifyDeinterleavedArray(
      capture_config.num_frames(), capture_config.num_channels(),
      output_reference, capture_output, kVectorElementErrorBound));
}

}  // namespace

TEST(NoiseSuppresionBitExactnessTest, Mono8kHzLow) {
#if defined(WEBRTC_ARCH_ARM64)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {1432.341431f, 3321.919922f,
                                           7677.521973f};
  const float kOutputReference[] = {0.003510f, 0.004517f, 0.004669f};
#elif defined(WEBRTC_ARCH_ARM)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {1432.341431f, 3321.919922f,
                                           7677.521973f};
  const float kOutputReference[] = {0.003510f, 0.004517f, 0.004669f};
#else
  const float kSpeechProbabilityReference = 0.73650402f;
  const float kNoiseEstimateReference[] = {1176.856812f, 3287.490967f,
                                           7525.964844f};
  const float kOutputReference[] = {0.003306f, 0.004442f, 0.004574f};
#endif

  RunBitexactnessTest(8000, 1, NoiseSuppression::Level::kLow,
                      kSpeechProbabilityReference, kNoiseEstimateReference,
                      kOutputReference);
}

TEST(NoiseSuppresionBitExactnessTest, Mono16kHzLow) {
#if defined(WEBRTC_ARCH_ARM64)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2534.461914f, 6277.638672f,
                                           14367.499023f};
  const float kOutputReference[] = {0.003449f, 0.004334f, 0.004303f};
#elif defined(WEBRTC_ARCH_ARM)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2534.461914f, 6277.638672f,
                                           14367.499023f};
  const float kOutputReference[] = {0.003449f, 0.004334f, 0.004303f};
#else
  const float kSpeechProbabilityReference = 0.71743423f;
  const float kNoiseEstimateReference[] = {2179.853027f, 6507.995117f,
                                           15652.758789f};
  const float kOutputReference[] = {0.003574f, 0.004494f, 0.004499f};
#endif

  RunBitexactnessTest(16000, 1, NoiseSuppression::Level::kLow,
                      kSpeechProbabilityReference, kNoiseEstimateReference,
                      kOutputReference);
}

TEST(NoiseSuppresionBitExactnessTest, Mono32kHzLow) {
#if defined(WEBRTC_ARCH_ARM64)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2540.059082f, 6317.822754f,
                                           14440.845703f};
  const float kOutputReference[] = {0.001679f, 0.002411f, 0.002594f};
#elif defined(WEBRTC_ARCH_ARM)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2540.059082f, 6317.822754f,
                                           14440.845703f};
  const float kOutputReference[] = {0.001679f, 0.002411f, 0.002594f};
#else
  const float kSpeechProbabilityReference = 0.67999554f;
  const float kNoiseEstimateReference[] = {2149.780518f, 7076.936035f,
                                           14939.945312f};
  const float kOutputReference[] = {0.001221f, 0.001984f, 0.002228f};
#endif

  RunBitexactnessTest(32000, 1, NoiseSuppression::Level::kLow,
                      kSpeechProbabilityReference, kNoiseEstimateReference,
                      kOutputReference);
}

TEST(NoiseSuppresionBitExactnessTest, Mono48kHzLow) {
#if defined(WEBRTC_ARCH_ARM64)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2135.292480f, 6692.695801f,
                                           14647.632812f};
  const float kOutputReference[] = {-0.012738f, -0.012312f, -0.011576f};
#elif defined(WEBRTC_ARCH_ARM)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2135.292480f, 6692.695801f,
                                           14647.632812f};
  const float kOutputReference[] = {-0.012738f, -0.012312f, -0.011576f};
#else
  const float kSpeechProbabilityReference = 0.70737761f;
  const float kNoiseEstimateReference[] = {2187.394043f, 6913.306641f,
                                           13182.945312f};
  const float kOutputReference[] = {-0.013062f, -0.012657f, -0.011934f};
#endif

  RunBitexactnessTest(48000, 1, NoiseSuppression::Level::kLow,
                      kSpeechProbabilityReference, kNoiseEstimateReference,
                      kOutputReference);
}

TEST(NoiseSuppresionBitExactnessTest, Stereo16kHzLow) {
#if defined(WEBRTC_ARCH_ARM64)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {9992.127930f, 12689.569336f,
                                           11589.296875f};
  const float kOutputReference[] = {-0.011108f, -0.007904f, -0.012390f,
                                    -0.002441f, 0.000855f,  -0.003204f};
#elif defined(WEBRTC_ARCH_ARM)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {10321.353516f, 12133.852539f,
                                           10923.060547f};
  const float kOutputReference[] = {-0.011108f, -0.007904f, -0.012390f,
                                    -0.002472f, 0.000916f,  -0.003235f};
#else
  const float kSpeechProbabilityReference = 0.67285913f;
  const float kNoiseEstimateReference[] = {9753.257812f, 11515.603516f,
                                           10503.309570f};
  const float kOutputReference[] = {-0.011459f, -0.008110f, -0.012728f,
                                    -0.002399f, 0.001018f,  -0.003189f};
#endif

  RunBitexactnessTest(16000, 2, NoiseSuppression::Level::kLow,
                      kSpeechProbabilityReference, kNoiseEstimateReference,
                      kOutputReference);
}

TEST(NoiseSuppresionBitExactnessTest, Mono16kHzModerate) {
#if defined(WEBRTC_ARCH_ARM64)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2057.085938f, 7601.055176f,
                                           19666.187500f};
  const float kOutputReference[] = {0.004669f, 0.005524f, 0.005432f};
#elif defined(WEBRTC_ARCH_ARM)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2244.497803f, 6864.164062f,
                                           16726.523438f};
  const float kOutputReference[] = {0.004669f, 0.005615f, 0.005585f};
#else
  const float kSpeechProbabilityReference = 0.70916927f;
  const float kNoiseEstimateReference[] = {2172.830566f, 6552.661133f,
                                           15624.025391f};
  const float kOutputReference[] = {0.004513f, 0.005590f, 0.005614f};
#endif

  RunBitexactnessTest(16000, 1, NoiseSuppression::Level::kModerate,
                      kSpeechProbabilityReference, kNoiseEstimateReference,
                      kOutputReference);
}

TEST(NoiseSuppresionBitExactnessTest, Mono16kHzHigh) {
#if defined(WEBRTC_ARCH_ARM64)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2095.148193f, 7698.553711f,
                                           19689.533203f};
  const float kOutputReference[] = {0.004639f, 0.005402f, 0.005310f};
#elif defined(WEBRTC_ARCH_ARM)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2282.515625f, 6984.408203f,
                                           16920.960938f};
  const float kOutputReference[] = {0.004547f, 0.005432f, 0.005402f};
#else
  const float kSpeechProbabilityReference = 0.70104003f;
  const float kNoiseEstimateReference[] = {2225.081055f, 6711.529785f,
                                           15785.949219};
  const float kOutputReference[] = {0.004394f, 0.005406f, 0.005416f};
#endif

  RunBitexactnessTest(16000, 1, NoiseSuppression::Level::kHigh,
                      kSpeechProbabilityReference, kNoiseEstimateReference,
                      kOutputReference);
}

TEST(NoiseSuppresionBitExactnessTest, Mono16kHzVeryHigh) {
#if defined(WEBRTC_ARCH_ARM64)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2677.733398f, 6186.987305f,
                                           14365.744141f};
  const float kOutputReference[] = {0.004273f, 0.005127f, 0.005188f};
#elif defined(WEBRTC_ARCH_ARM)
  const float kSpeechProbabilityReference = -4.0f;
  const float kNoiseEstimateReference[] = {2677.733398f, 6186.987305f,
                                           14365.744141f};
  const float kOutputReference[] = {0.004273f, 0.005127f, 0.005188f};
#else
  const float kSpeechProbabilityReference = 0.70290041f;
  const float kNoiseEstimateReference[] = {2254.921875f, 6723.172852f,
                                           15770.559570f};
  const float kOutputReference[] = {0.004321f, 0.005247f, 0.005263f};
#endif

  RunBitexactnessTest(16000, 1, NoiseSuppression::Level::kVeryHigh,
                      kSpeechProbabilityReference, kNoiseEstimateReference,
                      kOutputReference);
}
}  // namespace webrtc
