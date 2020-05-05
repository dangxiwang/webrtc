/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "video/rtp_streams_synchronizer2.h"

#include "absl/types/optional.h"
#include "call/syncable.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"
#include "rtc_base/trace_event.h"
#include "system_wrappers/include/rtp_to_ntp_estimator.h"

namespace webrtc {
namespace internal {
namespace {
// Time interval for logging stats.
constexpr int64_t kStatsLogIntervalMs = 10000;
constexpr uint32_t kSyncIntervalMs = 1000;

bool UpdateMeasurements(StreamSynchronization::Measurements* stream,
                        const Syncable::Info& info) {
  stream->latest_timestamp = info.latest_received_capture_timestamp;
  stream->latest_receive_time_ms = info.latest_receive_time_ms;
  bool new_rtcp_sr = false;
  return stream->rtp_to_ntp.UpdateMeasurements(
      info.capture_time_ntp_secs, info.capture_time_ntp_frac,
      info.capture_time_source_clock, &new_rtcp_sr);
}
}  // namespace

RtpStreamsSynchronizer::RtpStreamsSynchronizer(TaskQueueBase* main_queue,
                                               Syncable* syncable_video)
    : task_queue_(main_queue),
      syncable_video_(syncable_video),
      last_sync_time_(rtc::TimeNanos()),
      last_stats_log_ms_(rtc::TimeMillis()) {
  RTC_DCHECK(syncable_video);
}

RtpStreamsSynchronizer::~RtpStreamsSynchronizer() {
  RTC_DCHECK_RUN_ON(&main_checker_);
  task_safety_flag_->SetNotAlive();
}

void RtpStreamsSynchronizer::ConfigureSync(Syncable* syncable_audio) {
  RTC_DCHECK_RUN_ON(&main_checker_);

  // Prevent expensive no-ops.
  if (syncable_audio == syncable_audio_)
    return;

  syncable_audio_ = syncable_audio;
  sync_.reset(nullptr);
  if (!syncable_audio_)
    return;

  sync_.reset(
      new StreamSynchronization(syncable_video_->id(), syncable_audio_->id()));
  QueueTimer();
}

void RtpStreamsSynchronizer::QueueTimer() {
  RTC_DCHECK_RUN_ON(&main_checker_);
  if (timer_running_)
    return;

  timer_running_ = true;
  uint32_t delay = kSyncIntervalMs - (rtc::TimeNanos() - last_sync_time_) /
                                         rtc::kNumNanosecsPerMillisec;
  RTC_DCHECK_LE(delay, kSyncIntervalMs);
  task_queue_->PostDelayedTask(ToQueuedTask([this, safety = task_safety_flag_] {
                                 if (!safety->alive())
                                   return;
                                 RTC_DCHECK_RUN_ON(&main_checker_);
                                 timer_running_ = false;
                                 UpdateDelay();
                               }),
                               delay);
}

void RtpStreamsSynchronizer::UpdateDelay() {
  RTC_DCHECK_RUN_ON(&main_checker_);
  last_sync_time_ = rtc::TimeNanos();

  if (!syncable_audio_)
    return;

  RTC_DCHECK(sync_.get());

  QueueTimer();

  bool log_stats = false;
  const int64_t now_ms = rtc::TimeMillis();
  if (now_ms - last_stats_log_ms_ > kStatsLogIntervalMs) {
    last_stats_log_ms_ = now_ms;
    log_stats = true;
  }

  absl::optional<Syncable::Info> audio_info = syncable_audio_->GetInfo();
  if (!audio_info || !UpdateMeasurements(&audio_measurement_, *audio_info)) {
    return;
  }

  int64_t last_video_receive_ms = video_measurement_.latest_receive_time_ms;
  absl::optional<Syncable::Info> video_info = syncable_video_->GetInfo();
  if (!video_info || !UpdateMeasurements(&video_measurement_, *video_info)) {
    return;
  }

  if (last_video_receive_ms == video_measurement_.latest_receive_time_ms) {
    // No new video packet has been received since last update.
    return;
  }

  int relative_delay_ms;
  // Calculate how much later or earlier the audio stream is compared to video.
  if (!sync_->ComputeRelativeDelay(audio_measurement_, video_measurement_,
                                   &relative_delay_ms)) {
    return;
  }

  if (log_stats) {
    RTC_LOG(LS_INFO) << "Sync info stats: " << now_ms
                     << ", {ssrc: " << sync_->audio_stream_id() << ", "
                     << "cur_delay_ms: " << audio_info->current_delay_ms
                     << "} {ssrc: " << sync_->video_stream_id() << ", "
                     << "cur_delay_ms: " << video_info->current_delay_ms
                     << "} {relative_delay_ms: " << relative_delay_ms << "} ";
  }

  TRACE_COUNTER1("webrtc", "SyncCurrentVideoDelay",
                 video_info->current_delay_ms);
  TRACE_COUNTER1("webrtc", "SyncCurrentAudioDelay",
                 audio_info->current_delay_ms);
  TRACE_COUNTER1("webrtc", "SyncRelativeDelay", relative_delay_ms);

  int target_audio_delay_ms = 0;
  int target_video_delay_ms = video_info->current_delay_ms;
  // Calculate the necessary extra audio delay and desired total video
  // delay to get the streams in sync.
  if (!sync_->ComputeDelays(relative_delay_ms, audio_info->current_delay_ms,
                            &target_audio_delay_ms, &target_video_delay_ms)) {
    return;
  }

  if (log_stats) {
    RTC_LOG(LS_INFO) << "Sync delay stats: " << now_ms
                     << ", {ssrc: " << sync_->audio_stream_id() << ", "
                     << "target_delay_ms: " << target_audio_delay_ms
                     << "} {ssrc: " << sync_->video_stream_id() << ", "
                     << "target_delay_ms: " << target_video_delay_ms << "} ";
  }

  syncable_audio_->SetMinimumPlayoutDelay(target_audio_delay_ms);
  syncable_video_->SetMinimumPlayoutDelay(target_video_delay_ms);
}

// TODO(https://bugs.webrtc.org/7065): Move RtpToNtpEstimator out of
// RtpStreamsSynchronizer and into respective receive stream to always populate
// the estimated playout timestamp.
bool RtpStreamsSynchronizer::GetStreamSyncOffsetInMs(
    uint32_t rtp_timestamp,
    int64_t render_time_ms,
    int64_t* video_playout_ntp_ms,
    int64_t* stream_offset_ms,
    double* estimated_freq_khz) const {
  RTC_DCHECK_RUN_ON(&main_checker_);

  if (!syncable_audio_)
    return false;

  uint32_t audio_rtp_timestamp;
  int64_t time_ms;
  if (!syncable_audio_->GetPlayoutRtpTimestamp(&audio_rtp_timestamp,
                                               &time_ms)) {
    return false;
  }

  int64_t latest_audio_ntp;
  if (!audio_measurement_.rtp_to_ntp.Estimate(audio_rtp_timestamp,
                                              &latest_audio_ntp)) {
    return false;
  }

  syncable_audio_->SetEstimatedPlayoutNtpTimestampMs(latest_audio_ntp, time_ms);

  int64_t latest_video_ntp;
  if (!video_measurement_.rtp_to_ntp.Estimate(rtp_timestamp,
                                              &latest_video_ntp)) {
    return false;
  }

  // Current audio ntp.
  int64_t now_ms = rtc::TimeMillis();
  latest_audio_ntp += (now_ms - time_ms);

  // Remove video playout delay.
  int64_t time_to_render_ms = render_time_ms - now_ms;
  if (time_to_render_ms > 0)
    latest_video_ntp -= time_to_render_ms;

  *video_playout_ntp_ms = latest_video_ntp;
  *stream_offset_ms = latest_audio_ntp - latest_video_ntp;
  *estimated_freq_khz = video_measurement_.rtp_to_ntp.params()->frequency_khz;
  return true;
}

}  // namespace internal
}  // namespace webrtc
