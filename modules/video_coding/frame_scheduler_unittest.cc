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

#include <ostream>

#include "absl/functional/bind_front.h"
#include "test/gmock.h"
#include "test/gtest.h"
#include "test/time_controller/simulated_time_controller.h"

using ::testing::AllOf;
using ::testing::Contains;
using ::testing::Each;
using ::testing::ElementsAre;
using ::testing::Eq;
using ::testing::IsEmpty;
using ::testing::Matches;
using ::testing::Optional;
using ::testing::Pointee;
using ::testing::SizeIs;

namespace webrtc {

// void PrintTo(const EncodedFrame& frame, ::std::ostream* os) {
//   *os << "Frame with id(" << frame.Id()
//       << ") and render-time=" << frame.RenderTime() << "ms";
// }

// void PrintTo(const FakeEncodedFrame& frame, ::std::ostream* os) {
//   PrintTo(static_cast<const EncodedFrame&>(frame), os);
// }

namespace {

constexpr uint32_t kFps30Rtp = 90000 / 30;
constexpr TimeDelta kFps30Delay = 1 / Frequency::Hertz(30);

class FakeEncodedFrame : public EncodedFrame {
 public:
  int64_t ReceivedTime() const override { return 0; }
  int64_t RenderTime() const override { return _renderTimeMs; }
};

MATCHER_P(FrameWithId, id, "") {
  return Matches(Eq(id))(arg->Id());
}

class Builder {
 public:
  Builder& Time(uint32_t rtp_timestamp) {
    rtp_timestamp_ = rtp_timestamp;
    return *this;
  }
  Builder& Id(int64_t frame_id) {
    frame_id_ = frame_id;
    return *this;
  }
  Builder& AsLast() {
    last_spatial_layer_ = true;
    return *this;
  }
  Builder& Refs(const std::vector<int64_t>& references) {
    references_ = references;
    return *this;
  }

  std::unique_ptr<FakeEncodedFrame> Build() {
    RTC_CHECK_LE(references_.size(), EncodedFrame::kMaxFrameReferences);
    RTC_CHECK(rtp_timestamp_);
    RTC_CHECK(frame_id_);

    auto frame = std::make_unique<FakeEncodedFrame>();
    frame->SetTimestamp(*rtp_timestamp_);
    frame->SetId(*frame_id_);
    frame->is_last_spatial_layer = last_spatial_layer_;

    for (int64_t ref : references_) {
      frame->references[frame->num_references] = ref;
      frame->num_references++;
    }

    return frame;
  }

 private:
  absl::optional<uint32_t> rtp_timestamp_;
  absl::optional<int64_t> frame_id_;
  bool last_spatial_layer_ = false;
  std::vector<int64_t> references_;
};

constexpr auto kMaxWaitForKeyframe = TimeDelta::Millis(1500);
constexpr auto kMaxWaitForFrame = TimeDelta::Millis(500);
const DecodeStreamTimeouts config = {kMaxWaitForKeyframe, kMaxWaitForFrame};

class FrameSchedulerTest : public ::testing::Test {
 public:
  FrameSchedulerTest()
      : time_controller_(Timestamp::Zero()),
        clock_(time_controller_.GetClock()),
        task_queue_(time_controller_.GetTaskQueueFactory()->CreateTaskQueue(
            "scheduler",
            TaskQueueFactory::Priority::NORMAL)),
        timing_(time_controller_.GetClock()),
        frame_buffer_(200, 200),
        scheduler_(time_controller_.GetClock(),
                   &task_queue_,
                   &timing_,
                   &frame_buffer_,
                   config,
                   absl::bind_front(&FrameSchedulerTest::OnFrameReady, this),
                   absl::bind_front(&FrameSchedulerTest::OnTimeout, this)) {}

  ~FrameSchedulerTest() override = default;

  int timeouts() const { return timeouts_; }

  std::vector<absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4>>& frames() {
    return frames_;
  }

 protected:
  GlobalSimulatedTimeController time_controller_;
  Clock* const clock_;
  rtc::TaskQueue task_queue_;
  VCMTiming timing_;
  FrameBuffer frame_buffer_;
  FrameScheduler scheduler_;

 private:
  void OnTimeout() { timeouts_++; }

  void OnFrameReady(
      absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4> frames) {
    frames_.push_back(std::move(frames));
  }

  int timeouts_ = 0;
  std::vector<absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4>> frames_;
};

TEST_F(FrameSchedulerTest, TimeoutAfterTimeoutPeriod) {
  task_queue_.PostTask([&] { scheduler_.Start(); });
  time_controller_.AdvanceTime(TimeDelta::Zero());

  time_controller_.AdvanceTime(kMaxWaitForKeyframe);
  EXPECT_EQ(timeouts(), 1);

  task_queue_.PostTask([&] { scheduler_.Stop(); });
  time_controller_.AdvanceTime(TimeDelta::Zero());
}

TEST_F(FrameSchedulerTest, KeyFramesAreScheduled) {
  task_queue_.PostTask([&] { scheduler_.Start(); });
  time_controller_.AdvanceTime(TimeDelta::Zero());

  // TODO: No integration test.
  auto frame = Builder().Id(0).Time(0).AsLast().Build();

  frame_buffer_.InsertFrame(std::move(frame));

  scheduler_.OnNewFrameInserted();
  EXPECT_THAT(frames(), IsEmpty());
  time_controller_.AdvanceTime(TimeDelta::Zero());
  ASSERT_THAT(frames(), ElementsAre(SizeIs(1)));
  const EncodedFrame& keyframe = *frames().front().front();
  EXPECT_EQ(keyframe.Id(), 0);
  EXPECT_EQ(keyframe.RenderTime(), 0);

  task_queue_.PostTask([&] { scheduler_.Stop(); });
  time_controller_.AdvanceTime(TimeDelta::Zero());
  EXPECT_EQ(timeouts(), 0);
}

TEST_F(FrameSchedulerTest, DependantFramesAreScheduled) {
  task_queue_.PostTask([&] { scheduler_.Start(); });
  time_controller_.AdvanceTime(TimeDelta::Zero());

  frame_buffer_.InsertFrame(Builder().Id(0).Time(0).AsLast().Build());
  scheduler_.OnNewFrameInserted();
  // Wait half fps - insert new frame.
  auto wait = kFps30Delay;
  time_controller_.AdvanceTime(wait);
  frame_buffer_.InsertFrame(
      Builder().Id(1).Time(kFps30Rtp).AsLast().Refs({0}).Build());
  scheduler_.OnNewFrameInserted();

  time_controller_.AdvanceTime(kFps30Delay - wait);

  // 2 Frames - no super frames.
  ASSERT_THAT(frames(), ElementsAre(ElementsAre(FrameWithId(0)),
                                    ElementsAre(FrameWithId(1))));

  task_queue_.PostTask([&] { scheduler_.Stop(); });
  time_controller_.AdvanceTime(TimeDelta::Zero());
  EXPECT_EQ(timeouts(), 0);
}

TEST_F(FrameSchedulerTest, SpacialLayersAreScheduled) {
  task_queue_.PostTask([&] { scheduler_.Start(); });
  time_controller_.AdvanceTime(TimeDelta::Zero());

  frame_buffer_.InsertFrame(Builder().Id(0).Time(0).Build());
  frame_buffer_.InsertFrame(Builder().Id(1).Time(0).Build());
  frame_buffer_.InsertFrame(Builder().Id(2).Time(0).AsLast().Build());
  frame_buffer_.InsertFrame(Builder().Id(3).Time(kFps30Rtp).Refs({0}).Build());
  frame_buffer_.InsertFrame(
      Builder().Id(4).Time(kFps30Rtp).Refs({0, 1}).Build());
  frame_buffer_.InsertFrame(
      Builder().Id(5).Time(kFps30Rtp).Refs({0, 1, 2}).AsLast().Build());
  time_controller_.AdvanceTime(kFps30Delay);
  scheduler_.OnNewFrameInserted();
  scheduler_.OnNewFrameInserted();
  scheduler_.OnNewFrameInserted();
  scheduler_.OnNewFrameInserted();
  scheduler_.OnNewFrameInserted();
  scheduler_.OnNewFrameInserted();

  ASSERT_THAT(
      frames(),
      ElementsAre(ElementsAre(FrameWithId(0), FrameWithId(1), FrameWithId(2)),
                  ElementsAre(FrameWithId(3), FrameWithId(4), FrameWithId(5))));

  task_queue_.PostTask([&] { scheduler_.Stop(); });
  time_controller_.AdvanceTime(TimeDelta::Zero());
  EXPECT_EQ(timeouts(), 0);
}

}  // namespace
}  // namespace webrtc