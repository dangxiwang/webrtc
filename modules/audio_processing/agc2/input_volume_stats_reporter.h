/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_INPUT_VOLUME_STATS_REPORTER_H_
#define MODULES_AUDIO_PROCESSING_AGC2_INPUT_VOLUME_STATS_REPORTER_H_

#include "absl/types/optional.h"
#include "rtc_base/gtest_prod_util.h"
#include "system_wrappers/include/metrics.h"

namespace webrtc {

// Input volume statistics calculator. Computes aggregate stats based on the
// framewise input volume observed by `UpdateStatistics()`. Periodically logs
// the statistics into a histogram.
class InputVolumeStatsReporter {
 public:
  // Ctor. Set `is_applied_input_volume` to true if the stats reporter is used
  // to analyze the applied input volue, false for the recommended input volume.
  explicit InputVolumeStatsReporter(bool is_applied_input_volume);
  InputVolumeStatsReporter(const InputVolumeStatsReporter&) = delete;
  InputVolumeStatsReporter operator=(const InputVolumeStatsReporter&) = delete;
  ~InputVolumeStatsReporter();

  // Updates the stats based on `input_volume`. Periodically logs the stats into
  // a histogram.
  void UpdateStatistics(int input_volume);

 private:
  FRIEND_TEST_ALL_PREFIXES(InputVolumeStatsReporterTest,
                           CheckLevelUpdateStatsForEmptyStats);
  FRIEND_TEST_ALL_PREFIXES(InputVolumeStatsReporterTest,
                           CheckLevelUpdateStatsAfterNoGainChange);
  FRIEND_TEST_ALL_PREFIXES(InputVolumeStatsReporterTest,
                           CheckLevelUpdateStatsAfterGainIncrease);
  FRIEND_TEST_ALL_PREFIXES(InputVolumeStatsReporterTest,
                           CheckLevelUpdateStatsAfterGainDecrease);
  FRIEND_TEST_ALL_PREFIXES(InputVolumeStatsReporterTest,
                           CheckLevelUpdateStatsAfterReset);

  // Stores input volume update stats to enable calculation of update rate and
  // average update separately for volume increases and decreases.
  struct LevelUpdateStats {
    int num_decreases = 0;
    int num_increases = 0;
    int sum_decreases = 0;
    int sum_increases = 0;
  } level_update_stats_;

  // Returns a copy of the stored statistics. Use only for testing.
  LevelUpdateStats level_update_stats() const { return level_update_stats_; }

  // Computes aggregate stat and logs them into a histogram.
  void LogLevelUpdateStats() const;

  // Histograms.
  metrics::Histogram* decrease_rate_histogram_;
  metrics::Histogram* decrease_average_histogram_;
  metrics::Histogram* increase_rate_histogram_;
  metrics::Histogram* increase_average_histogram_;
  metrics::Histogram* update_rate_histogram_;
  metrics::Histogram* update_average_histogram_;

  int log_level_update_stats_counter_ = 0;
  absl::optional<int> previous_input_volume_ = absl::nullopt;
};
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_INPUT_VOLUME_STATS_REPORTER_H_
