/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/frame_scheduler.h"

#include "rtc_base/numerics/sequence_number_util.h"

namespace webrtc {

namespace {

constexpr TimeDelta kMaxAllowedFrameDelay = TimeDelta::Millis(5);

}

FrameScheduler::FrameScheduler(Clock* clock,
                               rtc::TaskQueue* task_queue,
                               VCMTiming const* timing,
                               FrameBuffer* frame_buffer,
                               DecodeStreamTimeouts timeouts,
                               FrameReadyCallback frame_ready_callback,
                               TimeoutCallback timeout_callback)
    : clock_(clock),
      frame_buffer_(frame_buffer),
      timing_(timing),
      timeouts_(timeouts),
      frame_ready_callback_(std::move(frame_ready_callback)),
      timeout_callback_(std::move(timeout_callback)),
      task_queue_(task_queue) {
  RTC_DCHECK(clock_);
  RTC_DCHECK(frame_buffer_);
  RTC_DCHECK(timing_);
  RTC_DCHECK(task_queue_);
}

void FrameScheduler::Start() {
  // TODO: This and stop must be called from the same sequence.
  // TODO: Should we be scoped? Not use Stop/Start but rather c'tor and d'tor.

  // No tasks should be started!
  RTC_DCHECK(!timeout_task_.Running());

  TimeDelta timeout_delay = TimeoutForNextFrame();
  timeout_ = clock_->CurrentTime() + timeout_delay;
  timeout_task_ =
      RepeatingTaskHandle::DelayedStart(task_queue_->Get(), timeout_delay,
                                        [this] { return HandleTimeoutTask(); });
}

TimeDelta FrameScheduler::HandleTimeoutTask() {
  Timestamp now = clock_->CurrentTime();
  // `timeout_` is hit and we have timed out. Schedule the next timeout at
  // the timeout delay.
  if (now >= timeout_) {
    TimeDelta timeout_delay = TimeoutForNextFrame();
    timeout_ = now + timeout_delay;
    timeout_callback_();
    return timeout_delay;
  }
  // Otherwise, `timeout_` changed since we scheduled a timeout. Reschedule
  // a timeout check.
  return timeout_ - now;
}

void FrameScheduler::Stop() {
  timeout_task_.Stop();
  // Cancels all delayed tasks.
  next_frame_to_decode_timestamp_.reset();
}

void FrameScheduler::ForceKeyFrame() {
  force_keyframe_ = true;
  // TODO: Reschedule frame lookup.
}

// Returns the id of the last continuous frame if there is one.
void FrameScheduler::OnNewFrameInserted() {
  MaybeScheduleNextFrame();
}

void FrameScheduler::MaybeScheduleNextFrame() {
  if (force_keyframe_) {
    TryForceKeyframe();
    return;
  }
  if (next_frame_to_decode_timestamp_ ==
      frame_buffer_->NextDecodableTemporalUnitRtpTimestamp())
    return;

  // Find candidate frame
  auto next_frame = FindNextDecodableFrame();
  if (!next_frame)
    return;
  TimeDelta max_wait = TimeDelta::Zero();
  std::tie(next_frame_to_decode_timestamp_, max_wait) = *next_frame;

  // Schedule the decoding of this frame.
  // TODO: Task safety for shutdown as we lose reference.
  if (max_wait > TimeDelta::Zero()) {
    task_queue_->PostDelayedTask(
        [this, frame_rtp = next_frame_to_decode_timestamp_.value()] {
          // The active frame to be decoded is no longer valid.
          if (frame_rtp != next_frame_to_decode_timestamp_)
            return;
          RTC_DCHECK_EQ(
              frame_rtp,
              frame_buffer_->NextDecodableTemporalUnitRtpTimestamp().value_or(
                  -1));
          auto frames = frame_buffer_->ExtractNextDecodableTemporalUnit();
          RTC_DCHECK_EQ(frame_rtp, frames.front()->Timestamp());
          YieldReadyFrames(std::move(frames));
        },
        max_wait.ms());
  } else {
    auto frames = frame_buffer_->ExtractNextDecodableTemporalUnit();
    RTC_DCHECK_EQ(next_frame_to_decode_timestamp_.value(),
                  frames.front()->Timestamp());
    YieldReadyFrames(std::move(frames));
  }
}

void FrameScheduler::TryForceKeyframe() {
  // Iterate through the frame buffer until there is a complete keyframe and
  // release this right away.
  // TODO: This is broken. We need to drop frames in the buffer until it is a
  // keyframe.
  while (frame_buffer_->NextDecodableTemporalUnitRtpTimestamp().has_value()) {
    auto next_complete_temporal_unit =
        frame_buffer_->ExtractNextDecodableTemporalUnit();
    if (next_complete_temporal_unit.empty()) {
      // DCHECK fail?
      continue;
    }
    // Found keyframe - decode right away.
    if (next_complete_temporal_unit.front()->is_keyframe()) {
      // Why post task?
      task_queue_->PostTask(
          [this, frames = std::move(next_complete_temporal_unit)]() mutable {
            YieldReadyFrames(std::move(frames));
          });
      return;
    }
  }
}

void FrameScheduler::YieldReadyFrames(
    absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4> frames) {
  Timestamp now = clock_->CurrentTime();
  const EncodedFrame& first_frame = *frames.front();
  const int64_t render_time =
      timing_->RenderTimeMs(first_frame.Timestamp(), now.ms());

  for (auto& frame : frames) {
    frame->SetRenderTime(render_time);
  }
  if (first_frame.is_keyframe()) {
    force_keyframe_ = false;
  }
  next_frame_to_decode_timestamp_.reset();
  timeout_ = clock_->CurrentTime() + TimeoutForNextFrame();
  frame_ready_callback_(std::move(frames));
}

bool FrameScheduler::CheckFrameNotDecodable(uint32_t rtp) const {
  return last_decoded_frame_timestamp_ &&
         AheadOf(*last_decoded_frame_timestamp_, rtp);
}

// TODO: Handle zero-playout delay.
// TODO: Handle invalid old timestamps.
absl::optional<std::pair<uint32_t, TimeDelta>>
FrameScheduler::FindNextDecodableFrame() {
  auto next_rtp = frame_buffer_->NextDecodableTemporalUnitRtpTimestamp();
  const Timestamp now = clock_->CurrentTime();
  // Drop temporal units until we don't skip a frame.
  while (next_rtp.has_value()) {
    // TODO: Move to FB3.
    if (CheckFrameNotDecodable(*next_rtp)) {
      frame_buffer_->DropNextDecodableTemporalUnit();
      next_rtp = frame_buffer_->NextDecodableTemporalUnitRtpTimestamp();
      continue;
    }
    // Current frame with given rtp might be decodable.
    int64_t render_time = timing_->RenderTimeMs(*next_rtp, now.ms());
    TimeDelta max_wait = TimeDelta::Millis(
        timing_->MaxWaitingTime(render_time, now.ms(), false));
    // TODO: Implement unary operator-.
    if (max_wait > -1 * kMaxAllowedFrameDelay ||
        next_rtp == frame_buffer_->LastDecodableTemporalUnitRtpTimestamp()) {
      return std::make_pair(*next_rtp, max_wait);
    }
    frame_buffer_->DropNextDecodableTemporalUnit();
    next_rtp = frame_buffer_->NextDecodableTemporalUnitRtpTimestamp();
  }

  return absl::nullopt;
}

TimeDelta FrameScheduler::TimeoutForNextFrame() const {
  return timeouts_.MaxWait(force_keyframe_);
}

}  // namespace webrtc