/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/frame_buffer2.h"

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <queue>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "api/video/encoded_image.h"
#include "api/video/video_timing.h"
#include "modules/video_coding/include/video_coding_defines.h"
#include "modules/video_coding/jitter_estimator.h"
#include "modules/video_coding/timing.h"
#include "rtc_base/checks.h"
#include "rtc_base/experiments/rtt_mult_experiment.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/sequence_number_util.h"
#include "rtc_base/trace_event.h"
#include "system_wrappers/include/clock.h"
#include "system_wrappers/include/field_trial.h"

namespace webrtc {
namespace video_coding {

namespace {
// Max number of frames the buffer will hold.
constexpr size_t kMaxFramesBuffered = 800;

// Max number of decoded frame info that will be saved.
constexpr int kMaxFramesHistory = 1 << 13;

// The time it's allowed for a frame to be late to its rendering prediction and
// still be rendered.
constexpr int kMaxAllowedFrameDelayMs = 5;

constexpr int64_t kLogNonDecodedIntervalMs = 5000;
}  // namespace

FrameBuffer::FrameBuffer(Clock* clock,
                         VCMTiming* timing,
                         VCMReceiveStatisticsCallback* stats_callback)
    : decoded_frames_history_(kMaxFramesHistory),
      clock_(clock),
      callback_queue_(nullptr),
      jitter_estimator_(clock),
      timing_(timing),
      inter_frame_delay_(clock_->TimeInMilliseconds()),
      stopped_(false),
      protection_mode_(kProtectionNack),
      stats_callback_(stats_callback),
      last_log_non_decoded_ms_(-kLogNonDecodedIntervalMs),
      add_rtt_to_playout_delay_(
          webrtc::field_trial::IsEnabled("WebRTC-AddRttToPlayoutDelay")),
      rtt_mult_settings_(RttMultExperiment::GetRttMultValue()) {}

FrameBuffer::~FrameBuffer() {}

void FrameBuffer::NextFrame(
    int64_t max_wait_time_ms,
    bool keyframe_required,
    rtc::TaskQueue* callback_queue,
    std::function<void(std::unique_ptr<EncodedFrame>, ReturnReason)> handler) {
  RTC_DCHECK_RUN_ON(callback_queue);
  TRACE_EVENT0("webrtc", "FrameBuffer::NextFrame");
  int64_t latest_return_time_ms =
      clock_->TimeInMilliseconds() + max_wait_time_ms;
  rtc::CritScope lock(&crit_);
  if (stopped_) {
    return;
  }
  latest_return_time_ms_ = latest_return_time_ms;
  keyframe_required_ = keyframe_required;
  frame_handler_ = handler;
  callback_queue_ = callback_queue;
  StartWaitForNextFrameOnQueue();
}

void FrameBuffer::StartWaitForNextFrameOnQueue() {
  RTC_DCHECK(callback_queue_);
  RTC_DCHECK(!callback_task_.Running());
  int64_t wait_ms = FindNextFrame(clock_->TimeInMilliseconds());
  callback_task_ = RepeatingTaskHandle::DelayedStart(
      callback_queue_->Get(), TimeDelta::ms(wait_ms), [this] {
        // If this task has not been cancelled, we did not get any new frames
        // while waiting. Continue with frame delivery.
        rtc::CritScope lock(&crit_);
        if (!frames_to_decode_.empty()) {
          // We have frames, deliver!
          frame_handler_(absl::WrapUnique(GetNextFrame()), kFrameFound);
          CancelCallback();
          return TimeDelta::Zero();  // Ignored.
        } else if (clock_->TimeInMilliseconds() >= latest_return_time_ms_) {
          // We have timed out, signal this and stop repeating.
          frame_handler_(nullptr, kTimeout);
          CancelCallback();
          return TimeDelta::Zero();  // Ignored.
        } else {
          // If there's no frames to decode and there is still time left, it
          // means that the frame buffer was cleared between creation and
          // execution of this task. Continue waiting for the remaining time.
          int64_t wait_ms = FindNextFrame(clock_->TimeInMilliseconds());
          return TimeDelta::ms(wait_ms);
        }
      });
}

FrameBuffer::ReturnReason FrameBuffer::NextFrame(
    int64_t max_wait_time_ms,
    std::unique_ptr<EncodedFrame>* frame_out,
    bool keyframe_required) {
  TRACE_EVENT0("webrtc", "FrameBuffer::NextFrame");
  int64_t latest_return_time_ms =
      clock_->TimeInMilliseconds() + max_wait_time_ms;
  int64_t wait_ms = max_wait_time_ms;
  int64_t now_ms = 0;

  do {
    now_ms = clock_->TimeInMilliseconds();
    {
      rtc::CritScope lock(&crit_);
      new_continuous_frame_event_.Reset();
      if (stopped_)
        return kStopped;

      keyframe_required_ = keyframe_required;
      latest_return_time_ms_ = latest_return_time_ms;
      wait_ms = FindNextFrame(now_ms);
    }
  } while (new_continuous_frame_event_.Wait(wait_ms));

  {
    rtc::CritScope lock(&crit_);

    if (!frames_to_decode_.empty()) {
      frame_out->reset(GetNextFrame());
      return kFrameFound;
    }
  }

  if (latest_return_time_ms - clock_->TimeInMilliseconds() > 0) {
    // If |next_frame_it_ == frames_.end()| and there is still time left, it
    // means that the frame buffer was cleared as the thread in this function
    // was waiting to acquire |crit_| in order to return. Wait for the
    // remaining time and then return.
    return NextFrame(latest_return_time_ms - now_ms, frame_out,
                     keyframe_required);
  }
  return kTimeout;
}

int64_t FrameBuffer::FindNextFrame(int64_t now_ms) {
  int64_t wait_ms = latest_return_time_ms_ - now_ms;
  frames_to_decode_.clear();

  // |last_continuous_frame_| may be empty below, but nullopt is smaller
  // than everything else and loop will immediately terminate as expected.
  for (auto frame_it = frames_.begin();
       frame_it != frames_.end() && frame_it->first <= last_continuous_frame_;
       ++frame_it) {
    if (!frame_it->second.continuous ||
        frame_it->second.num_missing_decodable > 0) {
      continue;
    }

    EncodedFrame* frame = frame_it->second.frame.get();

    if (keyframe_required_ && !frame->is_keyframe())
      continue;

    auto last_decoded_frame_timestamp =
        decoded_frames_history_.GetLastDecodedFrameTimestamp();

    // TODO(https://bugs.webrtc.org/9974): consider removing this check
    // as it may make a stream undecodable after a very long delay between
    // frames.
    if (last_decoded_frame_timestamp &&
        AheadOf(*last_decoded_frame_timestamp, frame->Timestamp())) {
      continue;
    }

    // Only ever return all parts of a superframe. Therefore skip this
    // frame if it's not a beginning of a superframe.
    if (frame->inter_layer_predicted) {
      continue;
    }

    // Gather all remaining frames for the same superframe.
    std::vector<FrameMap::iterator> current_superframe;
    current_superframe.push_back(frame_it);
    bool last_layer_completed = frame_it->second.frame->is_last_spatial_layer;
    FrameMap::iterator next_frame_it = frame_it;
    while (true) {
      ++next_frame_it;
      if (next_frame_it == frames_.end() ||
          next_frame_it->first.picture_id != frame->id.picture_id ||
          !next_frame_it->second.continuous) {
        break;
      }
      // Check if the next frame has some undecoded references other than
      // the previous frame in the same superframe.
      size_t num_allowed_undecoded_refs =
          (next_frame_it->second.frame->inter_layer_predicted) ? 1 : 0;
      if (next_frame_it->second.num_missing_decodable >
          num_allowed_undecoded_refs) {
        break;
      }
      // All frames in the superframe should have the same timestamp.
      if (frame->Timestamp() != next_frame_it->second.frame->Timestamp()) {
        RTC_LOG(LS_WARNING) << "Frames in a single superframe have different"
                               " timestamps. Skipping undecodable superframe.";
        break;
      }
      current_superframe.push_back(next_frame_it);
      last_layer_completed = next_frame_it->second.frame->is_last_spatial_layer;
    }
    // Check if the current superframe is complete.
    // TODO(bugs.webrtc.org/10064): consider returning all available to
    // decode frames even if the superframe is not complete yet.
    if (!last_layer_completed) {
      continue;
    }

    frames_to_decode_ = std::move(current_superframe);

    if (frame->RenderTime() == -1) {
      frame->SetRenderTime(timing_->RenderTimeMs(frame->Timestamp(), now_ms));
    }
    wait_ms = timing_->MaxWaitingTime(frame->RenderTime(), now_ms);

    // This will cause the frame buffer to prefer high framerate rather
    // than high resolution in the case of the decoder not decoding fast
    // enough and the stream has multiple spatial and temporal layers.
    // For multiple temporal layers it may cause non-base layer frames to be
    // skipped if they are late.
    if (wait_ms < -kMaxAllowedFrameDelayMs)
      continue;

    break;
  }
  wait_ms = std::min<int64_t>(wait_ms, latest_return_time_ms_ - now_ms);
  wait_ms = std::max<int64_t>(wait_ms, 0);
  return wait_ms;
}

EncodedFrame* FrameBuffer::GetNextFrame() {
  int64_t now_ms = clock_->TimeInMilliseconds();
  // TODO(ilnik): remove |frames_out| use frames_to_decode_ directly.
  std::vector<EncodedFrame*> frames_out;

  RTC_DCHECK(!frames_to_decode_.empty());
  bool superframe_delayed_by_retransmission = false;
  size_t superframe_size = 0;
  EncodedFrame* first_frame = frames_to_decode_[0]->second.frame.get();
  int64_t render_time_ms = first_frame->RenderTime();
  int64_t receive_time_ms = first_frame->ReceivedTime();
  // Gracefully handle bad RTP timestamps and render time issues.
  if (HasBadRenderTiming(*first_frame, now_ms)) {
    jitter_estimator_.Reset();
    timing_->Reset();
    render_time_ms = timing_->RenderTimeMs(first_frame->Timestamp(), now_ms);
  }

  for (FrameMap::iterator& frame_it : frames_to_decode_) {
    RTC_DCHECK(frame_it != frames_.end());
    EncodedFrame* frame = frame_it->second.frame.release();

    frame->SetRenderTime(render_time_ms);

    superframe_delayed_by_retransmission |= frame->delayed_by_retransmission();
    receive_time_ms = std::max(receive_time_ms, frame->ReceivedTime());
    superframe_size += frame->size();

    PropagateDecodability(frame_it->second);
    decoded_frames_history_.InsertDecoded(frame_it->first, frame->Timestamp());

    // Remove decoded frame and all undecoded frames before it.
    if (stats_callback_) {
      stats_callback_->OnDroppedFrames(std::count_if(
          frames_.begin(), frame_it,
          [](const std::pair<const VideoLayerFrameId, FrameInfo>& frame) {
            return frame.second.frame != nullptr;
          }));
    }

    frames_.erase(frames_.begin(), ++frame_it);

    frames_out.push_back(frame);
  }

  if (!superframe_delayed_by_retransmission) {
    int64_t frame_delay;

    if (inter_frame_delay_.CalculateDelay(first_frame->Timestamp(),
                                          &frame_delay, receive_time_ms)) {
      jitter_estimator_.UpdateEstimate(frame_delay, superframe_size);
    }

    float rtt_mult = protection_mode_ == kProtectionNackFEC ? 0.0 : 1.0;
    absl::optional<float> rtt_mult_add_cap_ms = absl::nullopt;
    if (rtt_mult_settings_.has_value()) {
      rtt_mult = rtt_mult_settings_->rtt_mult_setting;
      rtt_mult_add_cap_ms = rtt_mult_settings_->rtt_mult_add_cap_ms;
    }
    timing_->SetJitterDelay(
        jitter_estimator_.GetJitterEstimate(rtt_mult, rtt_mult_add_cap_ms));
    timing_->UpdateCurrentDelay(render_time_ms, now_ms);
  } else {
    if (RttMultExperiment::RttMultEnabled() || add_rtt_to_playout_delay_)
      jitter_estimator_.FrameNacked();
  }

  UpdateJitterDelay();
  UpdateTimingFrameInfo();

  if (frames_out.size() == 1) {
    return frames_out[0];
  } else {
    return CombineAndDeleteFrames(frames_out);
  }
}

bool FrameBuffer::HasBadRenderTiming(const EncodedFrame& frame,
                                     int64_t now_ms) {
  // Assume that render timing errors are due to changes in the video stream.
  int64_t render_time_ms = frame.RenderTimeMs();
  // Zero render time means render immediately.
  if (render_time_ms == 0) {
    return false;
  }
  if (render_time_ms < 0) {
    return true;
  }
  const int64_t kMaxVideoDelayMs = 10000;
  if (std::abs(render_time_ms - now_ms) > kMaxVideoDelayMs) {
    int frame_delay = static_cast<int>(std::abs(render_time_ms - now_ms));
    RTC_LOG(LS_WARNING)
        << "A frame about to be decoded is out of the configured "
        << "delay bounds (" << frame_delay << " > " << kMaxVideoDelayMs
        << "). Resetting the video jitter buffer.";
    return true;
  }
  if (static_cast<int>(timing_->TargetVideoDelay()) > kMaxVideoDelayMs) {
    RTC_LOG(LS_WARNING) << "The video target delay has grown larger than "
                        << kMaxVideoDelayMs << " ms.";
    return true;
  }
  return false;
}

void FrameBuffer::SetProtectionMode(VCMVideoProtection mode) {
  TRACE_EVENT0("webrtc", "FrameBuffer::SetProtectionMode");
  rtc::CritScope lock(&crit_);
  protection_mode_ = mode;
}

void FrameBuffer::Start() {
  TRACE_EVENT0("webrtc", "FrameBuffer::Start");
  rtc::CritScope lock(&crit_);
  stopped_ = false;
}

void FrameBuffer::Stop() {
  TRACE_EVENT0("webrtc", "FrameBuffer::Stop");
  rtc::CritScope lock(&crit_);
  stopped_ = true;
  new_continuous_frame_event_.Set();
  CancelCallback();
}

void FrameBuffer::Clear() {
  rtc::CritScope lock(&crit_);
  ClearFramesAndHistory();
}

void FrameBuffer::UpdateRtt(int64_t rtt_ms) {
  rtc::CritScope lock(&crit_);
  jitter_estimator_.UpdateRtt(rtt_ms);
}

bool FrameBuffer::ValidReferences(const EncodedFrame& frame) const {
  for (size_t i = 0; i < frame.num_references; ++i) {
    if (frame.references[i] >= frame.id.picture_id)
      return false;

    for (size_t j = i + 1; j < frame.num_references; ++j) {
      if (frame.references[i] == frame.references[j])
        return false;
    }
  }

  if (frame.inter_layer_predicted && frame.id.spatial_layer == 0)
    return false;

  return true;
}

void FrameBuffer::CancelCallback() {
  frame_handler_ = {};
  callback_task_.Stop();
  callback_queue_ = nullptr;
}

bool FrameBuffer::IsCompleteSuperFrame(const EncodedFrame& frame) {
  if (frame.inter_layer_predicted) {
    // Check that all previous spatial layers are already inserted.
    VideoLayerFrameId id = frame.id;
    RTC_DCHECK_GT(id.spatial_layer, 0);
    --id.spatial_layer;
    FrameMap::iterator prev_frame = frames_.find(id);
    if (prev_frame == frames_.end() || !prev_frame->second.frame)
      return false;
    while (prev_frame->second.frame->inter_layer_predicted) {
      if (prev_frame == frames_.begin())
        return false;
      --prev_frame;
      --id.spatial_layer;
      if (!prev_frame->second.frame ||
          prev_frame->first.picture_id != id.picture_id ||
          prev_frame->first.spatial_layer != id.spatial_layer) {
        return false;
      }
    }
  }

  if (!frame.is_last_spatial_layer) {
    // Check that all following spatial layers are already inserted.
    VideoLayerFrameId id = frame.id;
    ++id.spatial_layer;
    FrameMap::iterator next_frame = frames_.find(id);
    if (next_frame == frames_.end() || !next_frame->second.frame)
      return false;
    while (!next_frame->second.frame->is_last_spatial_layer) {
      ++next_frame;
      ++id.spatial_layer;
      if (next_frame == frames_.end() || !next_frame->second.frame ||
          next_frame->first.picture_id != id.picture_id ||
          next_frame->first.spatial_layer != id.spatial_layer) {
        return false;
      }
    }
  }

  return true;
}

int64_t FrameBuffer::InsertFrame(std::unique_ptr<EncodedFrame> frame) {
  TRACE_EVENT0("webrtc", "FrameBuffer::InsertFrame");
  RTC_DCHECK(frame);

  rtc::CritScope lock(&crit_);

  const VideoLayerFrameId& id = frame->id;
  int64_t last_continuous_picture_id =
      !last_continuous_frame_ ? -1 : last_continuous_frame_->picture_id;

  if (!ValidReferences(*frame)) {
    RTC_LOG(LS_WARNING) << "Frame with (picture_id:spatial_id) ("
                        << id.picture_id << ":"
                        << static_cast<int>(id.spatial_layer)
                        << ") has invalid frame references, dropping frame.";
    return last_continuous_picture_id;
  }

  if (frames_.size() >= kMaxFramesBuffered) {
    if (frame->is_keyframe()) {
      RTC_LOG(LS_WARNING) << "Inserting keyframe (picture_id:spatial_id) ("
                          << id.picture_id << ":"
                          << static_cast<int>(id.spatial_layer)
                          << ") but buffer is full, clearing"
                          << " buffer and inserting the frame.";
      ClearFramesAndHistory();
    } else {
      RTC_LOG(LS_WARNING) << "Frame with (picture_id:spatial_id) ("
                          << id.picture_id << ":"
                          << static_cast<int>(id.spatial_layer)
                          << ") could not be inserted due to the frame "
                          << "buffer being full, dropping frame.";
      return last_continuous_picture_id;
    }
  }

  auto last_decoded_frame = decoded_frames_history_.GetLastDecodedFrameId();
  auto last_decoded_frame_timestamp =
      decoded_frames_history_.GetLastDecodedFrameTimestamp();
  if (last_decoded_frame && id <= *last_decoded_frame) {
    if (AheadOf(frame->Timestamp(), *last_decoded_frame_timestamp) &&
        frame->is_keyframe()) {
      // If this frame has a newer timestamp but an earlier picture id then we
      // assume there has been a jump in the picture id due to some encoder
      // reconfiguration or some other reason. Even though this is not according
      // to spec we can still continue to decode from this frame if it is a
      // keyframe.
      RTC_LOG(LS_WARNING)
          << "A jump in picture id was detected, clearing buffer.";
      ClearFramesAndHistory();
      last_continuous_picture_id = -1;
    } else {
      RTC_LOG(LS_WARNING) << "Frame with (picture_id:spatial_id) ("
                          << id.picture_id << ":"
                          << static_cast<int>(id.spatial_layer)
                          << ") inserted after frame ("
                          << last_decoded_frame->picture_id << ":"
                          << static_cast<int>(last_decoded_frame->spatial_layer)
                          << ") was handed off for decoding, dropping frame.";
      return last_continuous_picture_id;
    }
  }

  // Test if inserting this frame would cause the order of the frames to become
  // ambiguous (covering more than half the interval of 2^16). This can happen
  // when the picture id make large jumps mid stream.
  if (!frames_.empty() && id < frames_.begin()->first &&
      frames_.rbegin()->first < id) {
    RTC_LOG(LS_WARNING)
        << "A jump in picture id was detected, clearing buffer.";
    ClearFramesAndHistory();
    last_continuous_picture_id = -1;
  }

  auto info = frames_.emplace(id, FrameInfo()).first;

  if (info->second.frame) {
    RTC_LOG(LS_WARNING) << "Frame with (picture_id:spatial_id) ("
                        << id.picture_id << ":"
                        << static_cast<int>(id.spatial_layer)
                        << ") already inserted, dropping frame.";
    return last_continuous_picture_id;
  }

  if (!UpdateFrameInfoWithIncomingFrame(*frame, info))
    return last_continuous_picture_id;

  if (!frame->delayed_by_retransmission())
    timing_->IncomingTimestamp(frame->Timestamp(), frame->ReceivedTime());

  if (stats_callback_ && IsCompleteSuperFrame(*frame)) {
    stats_callback_->OnCompleteFrame(frame->is_keyframe(), frame->size(),
                                     frame->contentType());
  }

  info->second.frame = std::move(frame);

  if (info->second.num_missing_continuous == 0) {
    info->second.continuous = true;
    PropagateContinuity(info);
    last_continuous_picture_id = last_continuous_frame_->picture_id;

    // Since we now have new continuous frames there might be a better frame
    // to return from NextFrame.
    new_continuous_frame_event_.Set();

    if (callback_queue_) {
      callback_queue_->PostTask([this] {
        rtc::CritScope lock(&crit_);
        if (!callback_task_.Running())
          return;
        RTC_CHECK(frame_handler_);
        callback_task_.Stop();
        StartWaitForNextFrameOnQueue();
      });
    }
  }

  return last_continuous_picture_id;
}

void FrameBuffer::PropagateContinuity(FrameMap::iterator start) {
  TRACE_EVENT0("webrtc", "FrameBuffer::PropagateContinuity");
  RTC_DCHECK(start->second.continuous);

  std::queue<FrameMap::iterator> continuous_frames;
  continuous_frames.push(start);

  // A simple BFS to traverse continuous frames.
  while (!continuous_frames.empty()) {
    auto frame = continuous_frames.front();
    continuous_frames.pop();

    if (!last_continuous_frame_ || *last_continuous_frame_ < frame->first) {
      last_continuous_frame_ = frame->first;
    }

    // Loop through all dependent frames, and if that frame no longer has
    // any unfulfilled dependencies then that frame is continuous as well.
    for (size_t d = 0; d < frame->second.dependent_frames.size(); ++d) {
      auto frame_ref = frames_.find(frame->second.dependent_frames[d]);
      RTC_DCHECK(frame_ref != frames_.end());

      // TODO(philipel): Look into why we've seen this happen.
      if (frame_ref != frames_.end()) {
        --frame_ref->second.num_missing_continuous;
        if (frame_ref->second.num_missing_continuous == 0) {
          frame_ref->second.continuous = true;
          continuous_frames.push(frame_ref);
        }
      }
    }
  }
}

void FrameBuffer::PropagateDecodability(const FrameInfo& info) {
  TRACE_EVENT0("webrtc", "FrameBuffer::PropagateDecodability");
  for (size_t d = 0; d < info.dependent_frames.size(); ++d) {
    auto ref_info = frames_.find(info.dependent_frames[d]);
    RTC_DCHECK(ref_info != frames_.end());
    // TODO(philipel): Look into why we've seen this happen.
    if (ref_info != frames_.end()) {
      RTC_DCHECK_GT(ref_info->second.num_missing_decodable, 0U);
      --ref_info->second.num_missing_decodable;
    }
  }
}

bool FrameBuffer::UpdateFrameInfoWithIncomingFrame(const EncodedFrame& frame,
                                                   FrameMap::iterator info) {
  TRACE_EVENT0("webrtc", "FrameBuffer::UpdateFrameInfoWithIncomingFrame");
  const VideoLayerFrameId& id = frame.id;

  auto last_decoded_frame = decoded_frames_history_.GetLastDecodedFrameId();
  RTC_DCHECK(!last_decoded_frame || *last_decoded_frame < info->first);

  // In this function we determine how many missing dependencies this |frame|
  // has to become continuous/decodable. If a frame that this |frame| depend
  // on has already been decoded then we can ignore that dependency since it has
  // already been fulfilled.
  //
  // For all other frames we will register a backwards reference to this |frame|
  // so that |num_missing_continuous| and |num_missing_decodable| can be
  // decremented as frames become continuous/are decoded.
  struct Dependency {
    VideoLayerFrameId id;
    bool continuous;
  };
  std::vector<Dependency> not_yet_fulfilled_dependencies;

  // Find all dependencies that have not yet been fulfilled.
  for (size_t i = 0; i < frame.num_references; ++i) {
    VideoLayerFrameId ref_key(frame.references[i], frame.id.spatial_layer);
    // Does |frame| depend on a frame earlier than the last decoded one?
    if (last_decoded_frame && ref_key <= *last_decoded_frame) {
      // Was that frame decoded? If not, this |frame| will never become
      // decodable.
      if (!decoded_frames_history_.WasDecoded(ref_key)) {
        int64_t now_ms = clock_->TimeInMilliseconds();
        if (last_log_non_decoded_ms_ + kLogNonDecodedIntervalMs < now_ms) {
          RTC_LOG(LS_WARNING)
              << "Frame with (picture_id:spatial_id) (" << id.picture_id << ":"
              << static_cast<int>(id.spatial_layer)
              << ") depends on a non-decoded frame more previous than"
              << " the last decoded frame, dropping frame.";
          last_log_non_decoded_ms_ = now_ms;
        }
        return false;
      }
    } else {
      auto ref_info = frames_.find(ref_key);
      bool ref_continuous =
          ref_info != frames_.end() && ref_info->second.continuous;
      not_yet_fulfilled_dependencies.push_back({ref_key, ref_continuous});
    }
  }

  // Does |frame| depend on the lower spatial layer?
  if (frame.inter_layer_predicted) {
    VideoLayerFrameId ref_key(frame.id.picture_id, frame.id.spatial_layer - 1);
    auto ref_info = frames_.find(ref_key);

    bool lower_layer_decoded =
        last_decoded_frame && *last_decoded_frame == ref_key;
    bool lower_layer_continuous =
        lower_layer_decoded ||
        (ref_info != frames_.end() && ref_info->second.continuous);

    if (!lower_layer_continuous || !lower_layer_decoded) {
      not_yet_fulfilled_dependencies.push_back(
          {ref_key, lower_layer_continuous});
    }
  }

  info->second.num_missing_continuous = not_yet_fulfilled_dependencies.size();
  info->second.num_missing_decodable = not_yet_fulfilled_dependencies.size();

  for (const Dependency& dep : not_yet_fulfilled_dependencies) {
    if (dep.continuous)
      --info->second.num_missing_continuous;

    frames_[dep.id].dependent_frames.push_back(id);
  }

  return true;
}

void FrameBuffer::UpdateJitterDelay() {
  TRACE_EVENT0("webrtc", "FrameBuffer::UpdateJitterDelay");
  if (!stats_callback_)
    return;

  int max_decode_ms;
  int current_delay_ms;
  int target_delay_ms;
  int jitter_buffer_ms;
  int min_playout_delay_ms;
  int render_delay_ms;
  if (timing_->GetTimings(&max_decode_ms, &current_delay_ms, &target_delay_ms,
                          &jitter_buffer_ms, &min_playout_delay_ms,
                          &render_delay_ms)) {
    stats_callback_->OnFrameBufferTimingsUpdated(
        max_decode_ms, current_delay_ms, target_delay_ms, jitter_buffer_ms,
        min_playout_delay_ms, render_delay_ms);
  }
}

void FrameBuffer::UpdateTimingFrameInfo() {
  TRACE_EVENT0("webrtc", "FrameBuffer::UpdateTimingFrameInfo");
  absl::optional<TimingFrameInfo> info = timing_->GetTimingFrameInfo();
  if (info && stats_callback_)
    stats_callback_->OnTimingFrameInfoUpdated(*info);
}

void FrameBuffer::ClearFramesAndHistory() {
  TRACE_EVENT0("webrtc", "FrameBuffer::ClearFramesAndHistory");
  if (stats_callback_) {
    stats_callback_->OnDroppedFrames(std::count_if(
        frames_.begin(), frames_.end(),
        [](const std::pair<const VideoLayerFrameId, FrameInfo>& frame) {
          return frame.second.frame != nullptr;
        }));
  }
  frames_.clear();
  last_continuous_frame_.reset();
  frames_to_decode_.clear();
  decoded_frames_history_.Clear();
}

EncodedFrame* FrameBuffer::CombineAndDeleteFrames(
    const std::vector<EncodedFrame*>& frames) const {
  RTC_DCHECK(!frames.empty());
  EncodedFrame* first_frame = frames[0];
  EncodedFrame* last_frame = frames.back();
  size_t total_length = 0;
  for (size_t i = 0; i < frames.size(); ++i) {
    total_length += frames[i]->size();
  }
  first_frame->VerifyAndAllocate(total_length);

  first_frame->SetSpatialLayerFrameSize(first_frame->id.spatial_layer,
                                        first_frame->size());

  // Spatial index of combined frame is set equal to spatial index of its top
  // spatial layer.
  first_frame->SetSpatialIndex(last_frame->id.spatial_layer);
  first_frame->id.spatial_layer = last_frame->id.spatial_layer;

  first_frame->video_timing_mutable()->network2_timestamp_ms =
      last_frame->video_timing().network2_timestamp_ms;
  first_frame->video_timing_mutable()->receive_finish_ms =
      last_frame->video_timing().receive_finish_ms;

  // Append all remaining frames to the first one.
  uint8_t* buffer = first_frame->data() + first_frame->size();
  for (size_t i = 1; i < frames.size(); ++i) {
    EncodedFrame* next_frame = frames[i];
    first_frame->SetSpatialLayerFrameSize(next_frame->id.spatial_layer,
                                          next_frame->size());
    memcpy(buffer, next_frame->data(), next_frame->size());
    buffer += next_frame->size();
    delete next_frame;
  }
  first_frame->set_size(total_length);
  return first_frame;
}

FrameBuffer::FrameInfo::FrameInfo() = default;
FrameBuffer::FrameInfo::FrameInfo(FrameInfo&&) = default;
FrameBuffer::FrameInfo::~FrameInfo() = default;

}  // namespace video_coding
}  // namespace webrtc
