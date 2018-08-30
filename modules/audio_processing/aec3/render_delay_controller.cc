/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "modules/audio_processing/aec3/render_delay_controller.h"

#include <algorithm>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "api/audio/echo_canceller3_config.h"
#include "modules/audio_processing/aec3/aec3_common.h"
#include "modules/audio_processing/aec3/echo_path_delay_estimator.h"
#include "modules/audio_processing/aec3/render_delay_controller_metrics.h"
#include "rtc_base/atomicops.h"
#include "rtc_base/constructormagic.h"
#include "rtc_base/logging.h"
#include "system_wrappers/include/field_trial.h"

namespace webrtc {

namespace {

class RenderDelayControllerImpl final : public RenderDelayController {
 public:
  RenderDelayControllerImpl(const EchoCanceller3Config& config,
                            int non_causal_offset,
                            int sample_rate_hz);
  ~RenderDelayControllerImpl() override;
  void Reset() override;
  void LogRenderCall() override;
  absl::optional<DelayEstimate> GetDelay(
      const DownsampledRenderBuffer& render_buffer,
      size_t render_delay_buffer_delay,
      const absl::optional<int>& echo_remover_delay,
      rtc::ArrayView<const float> capture) override;

 private:
  static int instance_count_;
  std::unique_ptr<ApmDataDumper> data_dumper_;
  const int delay_headroom_blocks_;
  const int hysteresis_limit_1_blocks_;
  const int hysteresis_limit_2_blocks_;
  absl::optional<DelayEstimate> delay_;
  EchoPathDelayEstimator delay_estimator_;
  RenderDelayControllerMetrics metrics_;
  absl::optional<DelayEstimate> delay_samples_;
  absl::optional<int> skew_;
  size_t capture_call_counter_ = 0;
  int delay_change_counter_ = 0;
  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(RenderDelayControllerImpl);
};

DelayEstimate ComputeBufferDelay(
    const absl::optional<DelayEstimate>& current_delay,
    int delay_headroom_blocks,
    int hysteresis_limit_1_blocks,
    int hysteresis_limit_2_blocks,
    int offset_blocks,
    DelayEstimate estimated_delay) {
  // The below division is not exact and the truncation is intended.
  const int echo_path_delay_blocks = estimated_delay.delay >> kBlockSizeLog2;

  // Compute the buffer delay increase required to achieve the desired latency.
  size_t new_delay_blocks = std::max(
      echo_path_delay_blocks + offset_blocks - delay_headroom_blocks, 0);

  // Add hysteresis.
  if (current_delay) {
    size_t current_delay_blocks = current_delay->delay;
    if (new_delay_blocks > current_delay_blocks) {
      if (new_delay_blocks <=
          current_delay_blocks + hysteresis_limit_1_blocks) {
        new_delay_blocks = current_delay_blocks;
      }
    } else if (new_delay_blocks < current_delay_blocks) {
      size_t hysteresis_limit = std::max(
          static_cast<int>(current_delay_blocks) - hysteresis_limit_2_blocks,
          0);
      if (new_delay_blocks >= hysteresis_limit) {
        new_delay_blocks = current_delay_blocks;
      }
    }
  }

  DelayEstimate new_delay = estimated_delay;
  new_delay.delay = new_delay_blocks;
  return new_delay;
}

int RenderDelayControllerImpl::instance_count_ = 0;

RenderDelayControllerImpl::RenderDelayControllerImpl(
    const EchoCanceller3Config& config,
    int non_causal_offset,
    int sample_rate_hz)
    : data_dumper_(
          new ApmDataDumper(rtc::AtomicOps::Increment(&instance_count_))),
      delay_headroom_blocks_(
          static_cast<int>(config.delay.delay_headroom_blocks)),
      hysteresis_limit_1_blocks_(
          static_cast<int>(config.delay.hysteresis_limit_1_blocks)),
      hysteresis_limit_2_blocks_(
          static_cast<int>(config.delay.hysteresis_limit_2_blocks)),
      delay_estimator_(data_dumper_.get(), config) {
  RTC_DCHECK(ValidFullBandRate(sample_rate_hz));
  delay_estimator_.LogDelayEstimationProperties(sample_rate_hz);
}

RenderDelayControllerImpl::~RenderDelayControllerImpl() = default;

void RenderDelayControllerImpl::Reset() {
  delay_ = absl::nullopt;
  delay_samples_ = absl::nullopt;
  delay_estimator_.Reset(false);
  delay_change_counter_ = 0;
}

void RenderDelayControllerImpl::LogRenderCall() {}

absl::optional<DelayEstimate> RenderDelayControllerImpl::GetDelay(
    const DownsampledRenderBuffer& render_buffer,
    size_t render_delay_buffer_delay,
    const absl::optional<int>& echo_remover_delay,
    rtc::ArrayView<const float> capture) {
  RTC_DCHECK_EQ(kBlockSize, capture.size());
  ++capture_call_counter_;

  auto delay_samples = delay_estimator_.EstimateDelay(render_buffer, capture);

  // Overrule the delay estimator delay if the echo remover reports a delay.
  if (echo_remover_delay) {
    int total_echo_remover_delay_samples =
        (render_delay_buffer_delay + *echo_remover_delay) * kBlockSize;
    delay_samples = DelayEstimate(DelayEstimate::Quality::kRefined,
                                  total_echo_remover_delay_samples);
  }

  if (delay_samples) {
    // TODO(peah): Refactor the rest of the code to assume a kRefined estimate
    // quality.
    RTC_DCHECK(DelayEstimate::Quality::kRefined == delay_samples->quality);
    if (!delay_samples_ || delay_samples->delay != delay_samples_->delay) {
      delay_change_counter_ = 0;
    }
    if (delay_samples_) {
      delay_samples_->blocks_since_last_change =
          delay_samples_->delay == delay_samples->delay
              ? delay_samples_->blocks_since_last_change + 1
              : 0;
      delay_samples_->blocks_since_last_update = 0;
      delay_samples_->delay = delay_samples->delay;
      delay_samples_->quality = delay_samples->quality;
    } else {
      delay_samples_ = delay_samples;
    }
  } else {
    if (delay_samples_) {
      ++delay_samples_->blocks_since_last_change;
      ++delay_samples_->blocks_since_last_update;
    }
  }

  if (delay_change_counter_ < 2 * kNumBlocksPerSecond) {
    ++delay_change_counter_;
  }

  if (delay_samples_) {
    // Compute the render delay buffer delay.
    delay_ = ComputeBufferDelay(
        delay_, delay_headroom_blocks_, hysteresis_limit_1_blocks_,
        hysteresis_limit_2_blocks_, 0 /* TODO(gustaf): Cleaning */,
        *delay_samples_);
  }

  metrics_.Update(delay_samples_ ? absl::optional<size_t>(delay_samples_->delay)
                                 : absl::nullopt,
                  delay_ ? delay_->delay : 0);

  data_dumper_->DumpRaw("aec3_render_delay_controller_delay",
                        delay_samples ? delay_samples->delay : 0);
  data_dumper_->DumpRaw("aec3_render_delay_controller_buffer_delay",
                        delay_ ? delay_->delay : 0);

  data_dumper_->DumpRaw("aec3_render_delay_controller_old_skew",
                        skew_ ? *skew_ : 0);

  return delay_;
}

}  // namespace

RenderDelayController* RenderDelayController::Create(
    const EchoCanceller3Config& config,
    int non_causal_offset,
    int sample_rate_hz) {
  return new RenderDelayControllerImpl(config, non_causal_offset,
                                       sample_rate_hz);
}

}  // namespace webrtc
