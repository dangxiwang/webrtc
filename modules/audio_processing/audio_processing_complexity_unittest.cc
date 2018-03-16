/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <math.h>

#include <memory>
#include <vector>

#include "modules/audio_processing/audio_buffer.h"
#include "modules/audio_processing/beamformer/nonlinear_beamformer.h"
#include "modules/audio_processing/echo_cancellation_impl.h"
#include "modules/audio_processing/echo_control_mobile_impl.h"
#include "modules/audio_processing/gain_control_impl.h"
#include "modules/audio_processing/gain_controller2.h"
#include "modules/audio_processing/noise_suppression_impl.h"
#include "modules/audio_processing/test/audio_buffer_tools.h"
#include "modules/audio_processing/vad/voice_activity_detector.h"
#include "modules/audio_processing/voice_detection_impl.h"
#include "rtc_base/numerics/safe_conversions.h"
#include "rtc_base/random.h"
#include "system_wrappers/include/clock.h"
#include "test/gtest.h"
#include "test/testsupport/perf_test.h"

namespace webrtc {

namespace {

const size_t kNumFramesToProcess = 20000;

struct SimulatorBuffers {
  SimulatorBuffers(int render_input_sample_rate_hz,
                   int capture_input_sample_rate_hz,
                   int render_output_sample_rate_hz,
                   int capture_output_sample_rate_hz,
                   size_t num_capture_channels) {
    CreateTwoDimensionalArrayBuffers(render_input_sample_rate_hz, 1,
                                     &render_input_config, &render_input,
                                     &render_input_samples);

    CreateTwoDimensionalArrayBuffers(render_output_sample_rate_hz, 1,
                                     &render_output_config, &render_output,
                                     &render_output_samples);

    CreateTwoDimensionalArrayBuffers(
        capture_input_sample_rate_hz, num_capture_channels,
        &capture_input_config, &capture_input, &capture_input_samples);

    CreateTwoDimensionalArrayBuffers(
        capture_output_sample_rate_hz, num_capture_channels,
        &capture_output_config, &capture_output, &capture_output_samples);

    render_buffer.reset(new AudioBuffer(render_input_config.num_frames(), 1,
                                        render_output_config.num_frames(), 1,
                                        render_output_config.num_frames()));

    capture_buffer.reset(
        new AudioBuffer(capture_input_config.num_frames(), num_capture_channels,
                        capture_input_config.num_frames(), num_capture_channels,
                        capture_input_config.num_frames()));
  }

  void CreateTwoDimensionalArrayBuffers(
      int sample_rate_hz,
      size_t num_channels,
      StreamConfig* config,
      std::vector<float*>* buffer_data,
      std::vector<float>* buffer_data_samples) {
    int samples_per_channel = rtc::CheckedDivExact(sample_rate_hz, 100);
    *config = StreamConfig(sample_rate_hz, num_channels, false);

    buffer_data_samples->resize(samples_per_channel * num_channels);
    buffer_data->resize(num_channels);
    for (size_t ch = 0; ch < num_channels; ++ch) {
      (*buffer_data)[ch] = &(*buffer_data_samples)[ch * samples_per_channel];
    }
  }

  std::unique_ptr<AudioBuffer> render_buffer;
  std::unique_ptr<AudioBuffer> capture_buffer;
  StreamConfig render_input_config;
  StreamConfig capture_input_config;
  StreamConfig render_output_config;
  StreamConfig capture_output_config;
  std::vector<float*> render_input;
  std::vector<float> render_input_samples;
  std::vector<float*> capture_input;
  std::vector<float> capture_input_samples;
  std::vector<float*> render_output;
  std::vector<float> render_output_samples;
  std::vector<float*> capture_output;
  std::vector<float> capture_output_samples;
};

void RandomizeBuffer(StreamConfig config,
                     Random* rand_gen,
                     std::vector<float>* buffer_samples,
                     AudioBuffer* buffer) {
  for (auto& v : *buffer_samples) {
    v = rand_gen->Rand<float>();
  }
  test::CopyVectorToAudioBuffer(config, *buffer_samples, buffer);
}

void UpdateInputBuffers(Random* rand_gen, SimulatorBuffers* buffers) {
  RandomizeBuffer(buffers->render_input_config, rand_gen,
                  &buffers->render_input_samples, buffers->render_buffer.get());
  RandomizeBuffer(buffers->capture_input_config, rand_gen,
                  &buffers->capture_input_samples,
                  buffers->capture_buffer.get());
}

class TestTimer {
 public:
  explicit TestTimer(size_t num_values_to_store)
      : clock_(webrtc::Clock::GetRealTimeClock()) {
    timestamps_.resize(num_values_to_store);
  }

  void ResetTimer() { start_timestamp_ = clock_->TimeInMicroseconds(); }
  void AddTimeStamp() {
    RTC_CHECK_LE(num_timestamps_stored_, timestamps_.size());
    timestamps_[num_timestamps_stored_] =
        clock_->TimeInMicroseconds() - start_timestamp_;
    ++num_timestamps_stored_;
  }

  double GetDurationAverage() const {
    RTC_DCHECK_EQ(num_timestamps_stored_, timestamps_.size());
    int64_t durations_sum = 0;
    for (size_t k = kNumTimestampsToExclude; k < timestamps_.size(); k++) {
      durations_sum += timestamps_[k];
    }

    RTC_DCHECK_LT(kNumTimestampsToExclude, timestamps_.size());
    return static_cast<double>(durations_sum) /
           (timestamps_.size() - kNumTimestampsToExclude);
  }

  double GetDurationStandardDeviationGetVarianceTime() const {
    int32_t average_duration = GetDurationAverage();
    int64_t variance = 0;
    for (size_t k = kNumTimestampsToExclude; k < timestamps_.size(); k++) {
      variance += timestamps_[k] - average_duration;
    }

    RTC_DCHECK_LT(kNumTimestampsToExclude, timestamps_.size());
    return sqrt(static_cast<double>(variance) /
                (timestamps_.size() - kNumTimestampsToExclude));
  }

 private:
  const size_t kNumTimestampsToExclude = 10u;
  webrtc::Clock* clock_;
  int64_t start_timestamp_ = 0;
  size_t num_timestamps_stored_ = 0;
  std::vector<int64_t> timestamps_;
};

void RunBandSplit(int sample_rate_hz) {
  Random rand_gen(42);
  SimulatorBuffers buffers(sample_rate_hz, sample_rate_hz, sample_rate_hz,
                           sample_rate_hz, 1);
  TestTimer timer(kNumFramesToProcess);

  for (size_t frame_no = 0; frame_no < kNumFramesToProcess; ++frame_no) {
    UpdateInputBuffers(&rand_gen, &buffers);
    timer.ResetTimer();
    if (sample_rate_hz > AudioProcessing::kSampleRate16kHz) {
      buffers.render_buffer->SplitIntoFrequencyBands();
      buffers.capture_buffer->SplitIntoFrequencyBands();
    }
    timer.AddTimeStamp();
  }

  webrtc::test::PrintResultMeanAndError(
      "apm_submodule_call_durations",
      "_" + std::to_string(sample_rate_hz) + "Hz", "Band-split",
      timer.GetDurationAverage(),  // MEAN
      timer.GetDurationStandardDeviationGetVarianceTime(), "us", false);
}

void RunGainControl(int sample_rate_hz, GainControl::Mode mode) {
  Random rand_gen(42);
  SimulatorBuffers buffers(sample_rate_hz, sample_rate_hz, sample_rate_hz,
                           sample_rate_hz, 1);
  TestTimer timer(kNumFramesToProcess);

  rtc::CriticalSection crit_render;
  rtc::CriticalSection crit_capture;
  GainControlImpl gain_controller(&crit_render, &crit_capture);

  gain_controller.Initialize(1, sample_rate_hz);
  static_cast<GainControl*>(&gain_controller)->Enable(true);
  static_cast<GainControl*>(&gain_controller)->set_mode(mode);
  static_cast<GainControl*>(&gain_controller)->set_stream_analog_level(60);
  static_cast<GainControl*>(&gain_controller)->set_target_level_dbfs(10);
  static_cast<GainControl*>(&gain_controller)->set_compression_gain_db(30);
  static_cast<GainControl*>(&gain_controller)->enable_limiter(true);
  static_cast<GainControl*>(&gain_controller)->set_analog_level_limits(0, 100);

  for (size_t frame_no = 0; frame_no < kNumFramesToProcess; ++frame_no) {
    UpdateInputBuffers(&rand_gen, &buffers);

    if (sample_rate_hz > AudioProcessing::kSampleRate16kHz) {
      buffers.render_buffer->SplitIntoFrequencyBands();
      buffers.capture_buffer->SplitIntoFrequencyBands();
    }

    std::vector<int16_t> render_audio;
    GainControlImpl::PackRenderAudioBuffer(buffers.render_buffer.get(),
                                           &render_audio);

    timer.ResetTimer();
    gain_controller.ProcessRenderAudio(render_audio);
    if (mode == GainControl::Mode::kAdaptiveAnalog) {
      static_cast<GainControl*>(&gain_controller)->set_stream_analog_level(60);
    }
    RTC_CHECK_EQ(AudioProcessing::kNoError, gain_controller.AnalyzeCaptureAudio(
                                                buffers.capture_buffer.get()));
    RTC_CHECK_EQ(AudioProcessing::kNoError,
                 gain_controller.ProcessCaptureAudio(
                     buffers.capture_buffer.get(), false));
    timer.AddTimeStamp();

    if (sample_rate_hz > AudioProcessing::kSampleRate16kHz) {
      buffers.capture_buffer->MergeFrequencyBands();
    }
  }
  std::string description;
  switch (mode) {
    case GainControl::Mode::kAdaptiveAnalog:
      description = "GainControl_kAdaptiveAnalog";
      break;
    case GainControl::Mode::kAdaptiveDigital:
      description = "GainControl_kAdaptiveDigital";
      break;
    case GainControl::Mode::kFixedDigital:
      description = "GainControl_kFixedDigital";
      break;
    default:
      RTC_CHECK(false);
  }

  webrtc::test::PrintResultMeanAndError(
      "apm_submodule_call_durations",
      "_" + std::to_string(sample_rate_hz) + "Hz", description,
      timer.GetDurationAverage(),  // MEAN
      timer.GetDurationStandardDeviationGetVarianceTime(), "us", false);
}

void RunAGC2GainControl(int sample_rate_hz) {
  Random rand_gen(42);
  SimulatorBuffers buffers(sample_rate_hz, sample_rate_hz, sample_rate_hz,
                           sample_rate_hz, 1);
  TestTimer timer(kNumFramesToProcess);

  GainController2 gain_controller;
  gain_controller.Initialize(sample_rate_hz);
  AudioProcessing::Config::GainController2 config;
  config.enabled = true;
  config.enable_adaptive_digital = true;
  config.fixed_gain_db = 0;
  config.enable_limiter = true;

  gain_controller.ApplyConfig(config);

  for (size_t frame_no = 0; frame_no < kNumFramesToProcess; ++frame_no) {
    UpdateInputBuffers(&rand_gen, &buffers);

    timer.ResetTimer();
    gain_controller.Process(buffers.render_buffer.get());
    timer.AddTimeStamp();
  }
  std::string description = "AGC2";

  webrtc::test::PrintResultMeanAndError(
      "apm_submodule_call_durations",
      "_" + std::to_string(sample_rate_hz) + "Hz", description,
      timer.GetDurationAverage(),  // MEAN
      timer.GetDurationStandardDeviationGetVarianceTime(), "us", false);
}

void RunApm_Vad(int sample_rate_hz, bool use_float_interface) {
  Random rand_gen(42);
  SimulatorBuffers buffers(sample_rate_hz, sample_rate_hz, sample_rate_hz,
                           sample_rate_hz, 1);
  TestTimer timer(kNumFramesToProcess);

  VoiceActivityDetector vad;

  for (size_t frame_no = 0; frame_no < kNumFramesToProcess; ++frame_no) {
    UpdateInputBuffers(&rand_gen, &buffers);

    timer.ResetTimer();

    auto render = buffers.render_buffer.get();
    if (use_float_interface) {
      AudioFrameView<const float> float_frame(
          render->channels_f(), render->num_channels(), render->num_frames());
      vad.AnalyzeFrame(float_frame);
    } else {
      vad.ProcessChunk(render->channels()[0], render->num_frames(),
                       sample_rate_hz);
    }

    timer.AddTimeStamp();
  }
  std::string description = "APM-VAD";
  if (use_float_interface) {
    description += "-float";
  } else {
    description += "-int16";
  }

  webrtc::test::PrintResultMeanAndError(
      "apm_submodule_call_durations",
      "_" + std::to_string(sample_rate_hz) + "Hz", description,
      timer.GetDurationAverage(),  // MEAN
      timer.GetDurationStandardDeviationGetVarianceTime(), "us", false);
}

const int kNativeSampleRatesHz[] = {8000, 16000, 32000, 48000};
}  // anonymous namespace

TEST(ApmComplexityTest, Mono_Separate_Submodules) {
  for (auto rate : kNativeSampleRatesHz) {
    RunGainControl(rate, GainControl::Mode::kAdaptiveAnalog);
    RunGainControl(rate, GainControl::Mode::kAdaptiveDigital);
    RunGainControl(rate, GainControl::Mode::kFixedDigital);
    RunBandSplit(rate);
    RunAGC2GainControl(rate);
    RunApm_Vad(rate, false);
    RunApm_Vad(rate, true);
  }
}
}  // namespace webrtc
