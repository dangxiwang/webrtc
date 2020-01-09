/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "video/video_source_sink_controller.h"

#include <limits>

#include "api/video/video_frame.h"
#include "api/video/video_source_interface.h"
#include "call/adaptation/video_source_restrictions.h"
#include "test/gmock.h"
#include "test/gtest.h"

using testing::_;

namespace webrtc {

namespace {

constexpr int kIntUnconstrained = std::numeric_limits<int>::max();

class MockVideoSinkWithVideoFrame : public rtc::VideoSinkInterface<VideoFrame> {
 public:
  ~MockVideoSinkWithVideoFrame() override {}

  MOCK_METHOD1(OnFrame, void(const VideoFrame& frame));
  MOCK_METHOD0(OnDiscardedFrame, void());
};

class MockVideoSourceWithVideoFrame
    : public rtc::VideoSourceInterface<VideoFrame> {
 public:
  ~MockVideoSourceWithVideoFrame() override {}

  MOCK_METHOD2(AddOrUpdateSink,
               void(rtc::VideoSinkInterface<VideoFrame>*,
                    const rtc::VideoSinkWants&));
  MOCK_METHOD1(RemoveSink, void(rtc::VideoSinkInterface<VideoFrame>*));
};

}  // namespace

TEST(VideoSourceSinkControllerTest, UnconstrainedByDefault) {
  MockVideoSinkWithVideoFrame sink;
  MockVideoSourceWithVideoFrame source;
  VideoSourceSinkController controller(&sink, &source);
  EXPECT_EQ(controller.restrictions(), VideoSourceRestrictions());
  EXPECT_FALSE(controller.pixels_per_frame_upper_limit().has_value());
  EXPECT_FALSE(controller.frame_rate_upper_limit().has_value());
  EXPECT_FALSE(controller.rotation_applied());
  EXPECT_EQ(controller.resolution_alignment(), 1);

  EXPECT_CALL(source, AddOrUpdateSink(_, _))
      .WillOnce([](rtc::VideoSinkInterface<VideoFrame>* sink,
                   const rtc::VideoSinkWants& wants) {
        EXPECT_FALSE(wants.rotation_applied);
        EXPECT_EQ(wants.max_pixel_count, kIntUnconstrained);
        EXPECT_EQ(wants.target_pixel_count, absl::nullopt);
        EXPECT_EQ(wants.max_framerate_fps, kIntUnconstrained);
        EXPECT_EQ(wants.resolution_alignment, 1);
      });
  controller.PushSourceSinkSettings();
}

TEST(VideoSourceSinkControllerTest, VideoRestrictionsToSinkWants) {
  MockVideoSinkWithVideoFrame sink;
  MockVideoSourceWithVideoFrame source;
  VideoSourceSinkController controller(&sink, &source);

  // Balanced degradation preference gives us what we ask for.
  EXPECT_CALL(source, AddOrUpdateSink(_, _)).Times(1);
  controller.SetSource(&source, DegradationPreference::BALANCED);

  VideoSourceRestrictions restrictions = controller.restrictions();
  // max_pixels_per_frame() maps to |max_pixel_count|.
  restrictions.set_max_pixels_per_frame(42u);
  // target_pixels_per_frame() maps to |target_pixel_count|.
  restrictions.set_target_pixels_per_frame(200u);
  // max_frame_rate() maps to |max_framerate_fps|.
  restrictions.set_max_frame_rate(30.0);
  controller.SetRestrictions(restrictions);
  EXPECT_CALL(source, AddOrUpdateSink(_, _))
      .WillOnce([](rtc::VideoSinkInterface<VideoFrame>* sink,
                   const rtc::VideoSinkWants& wants) {
        EXPECT_EQ(wants.max_pixel_count, 42);
        EXPECT_EQ(wants.target_pixel_count, 200);
        EXPECT_EQ(wants.max_framerate_fps, 30);
      });
  controller.PushSourceSinkSettings();

  // Disabled degradation preference makes the "wants" unconstrained despite our
  // restrictions.
  EXPECT_CALL(source, AddOrUpdateSink(_, _)).Times(1);
  controller.SetSource(&source, DegradationPreference::DISABLED);
  EXPECT_CALL(source, AddOrUpdateSink(_, _))
      .WillOnce([](rtc::VideoSinkInterface<VideoFrame>* sink,
                   const rtc::VideoSinkWants& wants) {
        EXPECT_EQ(wants.max_pixel_count, kIntUnconstrained);
        EXPECT_FALSE(wants.target_pixel_count.has_value());
        EXPECT_EQ(wants.max_framerate_fps, kIntUnconstrained);
      });
  controller.PushSourceSinkSettings();

  // pixels_per_frame_upper_limit() caps |max_pixel_count| regardless of
  // degradation preferences.
  controller.SetPixelsPerFrameUpperLimit(24);
  // frame_rate_upper_limit() caps |max_framerate_fps| regardless of degradation
  // preferences.
  controller.SetFrameRateUpperLimit(10.0);

  EXPECT_CALL(source, AddOrUpdateSink(_, _))
      .WillOnce([](rtc::VideoSinkInterface<VideoFrame>* sink,
                   const rtc::VideoSinkWants& wants) {
        EXPECT_EQ(wants.max_pixel_count, 24);
        EXPECT_EQ(wants.max_framerate_fps, 10);
      });
  controller.PushSourceSinkSettings();
}

TEST(VideoSourceSinkControllerTest, RotationApplied) {
  MockVideoSinkWithVideoFrame sink;
  MockVideoSourceWithVideoFrame source;
  VideoSourceSinkController controller(&sink, &source);
  controller.SetRotationApplied(true);
  EXPECT_TRUE(controller.rotation_applied());

  EXPECT_CALL(source, AddOrUpdateSink(_, _))
      .WillOnce([](rtc::VideoSinkInterface<VideoFrame>* sink,
                   const rtc::VideoSinkWants& wants) {
        EXPECT_TRUE(wants.rotation_applied);
      });
  controller.PushSourceSinkSettings();
}

TEST(VideoSourceSinkControllerTest, ResolutionAlignment) {
  MockVideoSinkWithVideoFrame sink;
  MockVideoSourceWithVideoFrame source;
  VideoSourceSinkController controller(&sink, &source);
  controller.SetResolutionAlignment(13);
  EXPECT_EQ(controller.resolution_alignment(), 13);

  EXPECT_CALL(source, AddOrUpdateSink(_, _))
      .WillOnce([](rtc::VideoSinkInterface<VideoFrame>* sink,
                   const rtc::VideoSinkWants& wants) {
        EXPECT_EQ(wants.resolution_alignment, 13);
      });
  controller.PushSourceSinkSettings();
}

TEST(VideoSourceSinkControllerTest,
     PushSourceSinkSettingsWithoutSourceDoesNotCrash) {
  MockVideoSinkWithVideoFrame sink;
  VideoSourceSinkController controller(&sink, nullptr);
  controller.PushSourceSinkSettings();
}

}  // namespace webrtc
