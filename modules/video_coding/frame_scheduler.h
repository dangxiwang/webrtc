/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_VIDEO_CODING_FRAME_SCHEDULER_H_
#define MODULES_VIDEO_CODING_FRAME_SCHEDULER_H_

#include <memory>
#include <utility>

#include "absl/container/inlined_vector.h"
#include "absl/types/optional.h"
#include "api/task_queue/task_queue_base.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/video_coding/frame_buffer3.h"
#include "modules/video_coding/timing.h"
#include "rtc_base/task_utils/repeating_task.h"

namespace webrtc {

class FrameScheduler {
 public:
  class Callbacks {
   public:
    virtual ~Callbacks() = default;
    virtual void OnFrameReady(
        absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4>) = 0;

    virtual void OnTimeout() = 0;
  };

  struct Timeouts {
    TimeDelta max_wait_for_keyframe;
    TimeDelta max_wait_for_frame;

    TimeDelta MaxWait(bool keyframe) const {
      return keyframe ? max_wait_for_keyframe : max_wait_for_frame;
    }
  };

  FrameScheduler(Clock* clock,
                 TaskQueueBase* task_queue,
                 VCMTiming const* timing,
                 FrameBuffer* frame_buffer,
                 Timeouts timeouts,
                 Callbacks* callbacks);
  virtual ~FrameScheduler();
  FrameScheduler(const FrameScheduler&) = delete;
  FrameScheduler& operator=(const FrameScheduler&) = delete;

  void Stop();
  void ForceKeyFrame();

  void OnFrameBufferChanged();
  void OnReadyForNextFrame();

 private:
  class ExtractNextFrameTask;
  void OnTimeToReleaseNextFrame(uint32_t next_frame_rtp);

  TimeDelta TimeoutForNextFrame() const;
  void StartTimeoutTask() RTC_RUN_ON(bookkeeping_queue_);
  TimeDelta HandleTimeoutTask();

  void MaybeScheduleNextFrame();
  void TryForceKeyframe();

  absl::optional<std::pair<uint32_t, TimeDelta>> SelectNextDecodableFrame();
  bool CheckFrameNotDecodable(uint32_t rtp) const;
  TimeDelta ReleaseNextFrame();
  void YieldReadyFrames(
      absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4> frames);

  Clock* const clock_;
  FrameBuffer* const frame_buffer_;
  VCMTiming const* const timing_;
  const Timeouts timeouts_;
  Callbacks* const callbacks_;
  TaskQueueBase* const bookkeeping_queue_;

  // Initial frame will always be forced as a keyframe.
  bool force_keyframe_ = true;

  std::unique_ptr<ExtractNextFrameTask> next_frame_task_
      RTC_GUARDED_BY(bookkeeping_queue_);
  absl::optional<uint32_t> last_released_frame_rtp_;
  bool receiver_ready_for_next_frame_ = false;

  RepeatingTaskHandle timeout_task_ RTC_GUARDED_BY(bookkeeping_queue_);
  Timestamp timeout_ = Timestamp::MinusInfinity();

  // Maximum number of frames in the decode queue to allow pacing. If the
  // queue grows beyond the max limit, pacing will be disabled and frames will
  // be pushed to the decoder as soon as possible. This only has an effect
  // when the low-latency rendering path is active, which is indicated by
  // the frame's render time == 0.
  FieldTrialParameter<unsigned> zero_playout_delay_max_decode_queue_size_;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_FRAME_SCHEDULER_H_
