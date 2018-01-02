/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "test/fuzzers/audio_processing_fuzzer.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "modules/audio_processing/include/audio_processing.h"
#include "modules/include/module_common_types.h"
#include "rtc_base/checks.h"

namespace webrtc {
namespace {
void GenerateFloatFrame(test::FuzzDataHelper* fuzz_data,
                        size_t input_rate,
                        std::array<float, 480>* float_frame) {
  const size_t samples_per_input_channel =
      rtc::CheckedDivExact(input_rate, 100ul);
  RTC_DCHECK_LE(samples_per_input_channel, 480);
  for (size_t i = 0; i < samples_per_input_channel; ++i) {
    (*float_frame)[i] = fuzz_data->ReadOrDefaultValue<int16_t>(0);
  }
}

void GenerateFixedFrame(test::FuzzDataHelper* fuzz_data,
                        size_t input_rate,
                        bool use_stereo, AudioFrame* fixed_frame) {
  const size_t samples_per_input_channel =
      rtc::CheckedDivExact(input_rate, 100ul);
  fixed_frame->samples_per_channel_ = samples_per_input_channel;
  fixed_frame->sample_rate_hz_ = input_rate;
  fixed_frame->num_channels_ = use_stereo ? 1 : 2;

  for (size_t i = 0; i < samples_per_input_channel * fixed_frame->num_channels_;
       ++i) {
    fixed_frame->mutable_data()[i] =
        fuzz_data->ReadOrDefaultValue(fixed_frame->mutable_data()[i]);
  }
}
}  // namespace

void FuzzAudioProcessing(test::FuzzDataHelper* fuzz_data,
                         std::unique_ptr<AudioProcessing> apm) {
  AudioFrame fixed_frame;
  std::array<float, 480> float_frame{};
  float* const first_channel = &float_frame[0];

  using Rate = AudioProcessing::NativeRate;
  const Rate rate_kinds[] = {Rate::kSampleRate8kHz, Rate::kSampleRate16kHz,
                             Rate::kSampleRate32kHz, Rate::kSampleRate48kHz};

  while (fuzz_data->CanReadBytes(1)) {
    const bool is_float = fuzz_data->ReadOrDefaultValue(true);
    // Decide input/output rate for this iteration.
    const auto input_rate =
        static_cast<size_t>(fuzz_data->SelectOneOf(rate_kinds));
    const auto output_rate =
        static_cast<size_t>(fuzz_data->SelectOneOf(rate_kinds));

    const bool use_stereo = fuzz_data->ReadOrDefaultValue(true);
    const uint8_t stream_delay = fuzz_data->ReadOrDefaultValue(0);

    // API call needed for AEC-2 and AEC-m to run.
    apm->set_stream_delay_ms(stream_delay);

    // Make the APM call depending on capture/render mode and float /
    // fix interface.
    const bool is_capture = fuzz_data->ReadOrDefaultValue(true);

    // Fill the arrays with audio samples from the data.
    int apm_return_code = AudioProcessing::Error::kNoError;
    if (is_float) {
      GenerateFloatFrame(fuzz_data, input_rate, &float_frame);
      if (is_capture) {
        apm_return_code =
            apm->ProcessStream(&first_channel, StreamConfig(input_rate, 1),
                               StreamConfig(output_rate, 1), &first_channel);
      } else {
        apm_return_code = apm->ProcessReverseStream(
            &first_channel, StreamConfig(input_rate, 1),
            StreamConfig(output_rate, 1), &first_channel);
      }
    } else {
      GenerateFixedFrame(fuzz_data, input_rate, use_stereo,
                         &fixed_frame);

      if (is_capture) {
        apm_return_code = apm->ProcessStream(&fixed_frame);
      } else {
        apm_return_code = apm->ProcessReverseStream(&fixed_frame);
      }
    }

    RTC_DCHECK_NE(apm_return_code, AudioProcessing::kBadDataLengthError);
  }
}
}  // namespace webrtc
