/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/goog_cc/alr_detector.h"

#include <cstdint>
#include <cstdio>

#include "absl/memory/memory.h"
#include "logging/rtc_event_log/events/rtc_event.h"
#include "logging/rtc_event_log/events/rtc_event_alr_state.h"
#include "logging/rtc_event_log/rtc_event_log.h"
#include "rtc_base/checks.h"
#include "rtc_base/numerics/safe_conversions.h"
#include "rtc_base/time_utils.h"

namespace webrtc {

namespace {
absl::optional<AlrExperimentSettings> GetExperimentSettings(
    const WebRtcKeyValueConfig* key_value_config) {
  RTC_CHECK(AlrExperimentSettings::MaxOneFieldTrialEnabled(*key_value_config));
  absl::optional<AlrExperimentSettings> experiment_settings =
      AlrExperimentSettings::CreateFromFieldTrial(
          *key_value_config,
          AlrExperimentSettings::kScreenshareProbingBweExperimentName);
  if (!experiment_settings) {
    experiment_settings = AlrExperimentSettings::CreateFromFieldTrial(
        *key_value_config,
        AlrExperimentSettings::kStrictPacingAndProbingExperimentName);
  }
  return experiment_settings;
}
}  //  namespace

AlrDetector::AlrDetector(const WebRtcKeyValueConfig* key_value_config)
    : AlrDetector(key_value_config,
                  nullptr,
                  GetExperimentSettings(key_value_config)) {}

AlrDetector::AlrDetector(const WebRtcKeyValueConfig* key_value_config,
                         RtcEventLog* event_log)
    : AlrDetector(key_value_config,
                  nullptr,
                  GetExperimentSettings(key_value_config)) {}

AlrDetector::AlrDetector(
    const WebRtcKeyValueConfig* key_value_config,
    RtcEventLog* event_log,
    absl::optional<AlrExperimentSettings> experiment_settings)
    : bandwidth_usage_percent_(
          "bw_usage_percent",
          experiment_settings ? experiment_settings->alr_bandwidth_usage_percent
                              : kDefaultAlrBandwidthUsagePercent),
      alr_start_budget_level_percent_(
          "start_budget_level_percent",
          experiment_settings
              ? experiment_settings->alr_start_budget_level_percent
              : kDefaultAlrStartBudgetLevelPercent),
      alr_stop_budget_level_percent_(
          "stop_budget_level_percent",
          experiment_settings
              ? experiment_settings->alr_stop_budget_level_percent
              : kDefaultAlrStopBudgetLevelPercent),
      alr_budget_(0, true),
      event_log_(event_log) {
  ParseFieldTrial({&bandwidth_usage_percent_, &alr_start_budget_level_percent_,
                   &alr_stop_budget_level_percent_},
                  key_value_config->Lookup("WebRTC-AlrDetectorParameters"));
}

AlrDetector::~AlrDetector() {}

void AlrDetector::OnBytesSent(size_t bytes_sent, int64_t send_time_ms) {
  if (!last_send_time_ms_.has_value()) {
    last_send_time_ms_ = send_time_ms;
    // Since the duration for sending the bytes is unknwon, return without
    // updating alr state.
    return;
  }
  int64_t delta_time_ms = send_time_ms - *last_send_time_ms_;
  last_send_time_ms_ = send_time_ms;

  alr_budget_.UseBudget(bytes_sent);
  alr_budget_.IncreaseBudget(delta_time_ms);
  bool state_changed = false;
  if (alr_budget_.budget_level_percent() > alr_start_budget_level_percent_ &&
      !alr_started_time_ms_) {
    alr_started_time_ms_.emplace(rtc::TimeMillis());
    state_changed = true;
  } else if (alr_budget_.budget_level_percent() <
                 alr_stop_budget_level_percent_ &&
             alr_started_time_ms_) {
    state_changed = true;
    alr_started_time_ms_.reset();
  }
  if (event_log_ && state_changed) {
    event_log_->Log(
        absl::make_unique<RtcEventAlrState>(alr_started_time_ms_.has_value()));
  }
}

void AlrDetector::SetEstimatedBitrate(int bitrate_bps) {
  RTC_DCHECK(bitrate_bps);
  const auto target_rate_kbps = static_cast<int64_t>(bitrate_bps) *
                                bandwidth_usage_percent_ / (1000 * 100);
  alr_budget_.set_target_rate_kbps(rtc::dchecked_cast<int>(target_rate_kbps));
}

absl::optional<int64_t> AlrDetector::GetApplicationLimitedRegionStartTime()
    const {
  return alr_started_time_ms_;
}

int AlrDetector::bandwidth_usage_percent() const {
  return bandwidth_usage_percent_.Get();
}
int AlrDetector::start_budget_level_percent() const {
  return alr_start_budget_level_percent_.Get();
}
int AlrDetector::stop_budget_level_percent() const {
  return alr_stop_budget_level_percent_.Get();
}

}  // namespace webrtc
