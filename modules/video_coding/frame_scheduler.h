/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef VIDEO_CODING_FRAME_SCHEDULER_H_
#define VIDEO_CODING_FRAME_SCHEDULER_H_

#include "api/sequence_checker.h"
#include "modules/video_coding/frame_buffer2_interface.h"
#include "modules/video_coding/frame_buffer3.h"
#include "modules/video_coding/inter_frame_delay.h"
#include "modules/video_coding/jitter_estimator.h"
#include "modules/video_coding/timing.h"
#include "rtc_base/system/no_unique_address.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/task_utils/repeating_task.h"

namespace webrtc {

class FrameScheduler {
 public:
  using FrameReadyCallback = std::function<void(
      absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4>)>;
  using TimeoutCallback = std::function<void()>;

  FrameScheduler(Clock* clock,
                 rtc::TaskQueue* task_queue,
                 VCMTiming const* timing,
                 FrameBuffer* frame_buffer,
                 DecodeStreamTimeouts timeouts,
                 FrameReadyCallback frame_ready_callback,
                 TimeoutCallback timeout_callback);
  FrameScheduler(const FrameScheduler&) = delete;
  FrameScheduler& operator=(const FrameScheduler&) = delete;

  void Start();
  void Stop();
  void ForceKeyFrame();

  void OnNewFrameInserted();

 private:
  TimeDelta TimeoutForNextFrame() const;
  TimeDelta HandleTimeoutTask();

  void MaybeScheduleNextFrame();
  void TryForceKeyframe();

  absl::optional<std::pair<uint32_t, TimeDelta>> FindNextDecodableFrame();
  bool CheckFrameNotDecodable(uint32_t rtp) const;
  TimeDelta ReleaseNextFrame();
  void YieldReadyFrames(
      absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4> frames);

  Clock* const clock_;
  FrameBuffer* const frame_buffer_;
  VCMTiming const* const timing_;
  const DecodeStreamTimeouts timeouts_;
  const FrameReadyCallback frame_ready_callback_;
  const TimeoutCallback timeout_callback_;
  rtc::TaskQueue* const task_queue_;

  // TODO: Maybe use a sequence checkers? Or force task_queue_?
  bool force_keyframe_;

  absl::optional<uint32_t> next_frame_to_decode_timestamp_;
  absl::optional<uint32_t> last_decoded_frame_timestamp_;

  RepeatingTaskHandle timeout_task_;
  // TODO: Maybe change this to last frame time? timeout = last_frame + delay
  Timestamp timeout_ = Timestamp::MinusInfinity();
};

}  // namespace webrtc

#endif  // VIDEO_CODING_FRAME_SCHEDULER_H_