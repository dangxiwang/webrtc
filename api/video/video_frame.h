/*
 *  Copyright (c) 2014 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_VIDEO_VIDEO_FRAME_H_
#define API_VIDEO_VIDEO_FRAME_H_

#include <stdint.h>

#include "absl/types/optional.h"
#include "api/scoped_refptr.h"
#include "api/video/color_space.h"
#include "api/video/hdr_metadata.h"
#include "api/video/video_frame_buffer.h"
#include "api/video/video_rotation.h"
#include "rtc_base/system/rtc_export.h"

namespace webrtc {

class RTC_EXPORT VideoFrame {
 public:
  // Describes a partial frame, which contains only a changed region compared
  // to a previous frame. Shouldn't be set on the fully updated picture.
  struct PartialFrameDescription {
    // Coordinates of top-left corner of the changed region in the full picture.
    int offset_x;
    int offset_y;
  };

  // Preferred way of building VideoFrame objects.
  class Builder {
   public:
    Builder();
    ~Builder();

    VideoFrame build();
    Builder& set_video_frame_buffer(
        const rtc::scoped_refptr<VideoFrameBuffer>& buffer);
    Builder& set_timestamp_ms(int64_t timestamp_ms);
    Builder& set_timestamp_us(int64_t timestamp_us);
    Builder& set_timestamp_rtp(uint32_t timestamp_rtp);
    Builder& set_ntp_time_ms(int64_t ntp_time_ms);
    Builder& set_rotation(VideoRotation rotation);
    Builder& set_color_space(const ColorSpace& color_space);
    Builder& set_color_space(const ColorSpace* color_space);
    Builder& set_id(uint16_t id);
    Builder& set_partial_frame_description(
        const absl::optional<PartialFrameDescription>& description);
    Builder& set_cache_buffer_for_partial_updates(
        bool cache_buffer_for_partial_updates);

   private:
    uint16_t id_ = 0;
    rtc::scoped_refptr<webrtc::VideoFrameBuffer> video_frame_buffer_;
    int64_t timestamp_us_ = 0;
    uint32_t timestamp_rtp_ = 0;
    int64_t ntp_time_ms_ = 0;
    VideoRotation rotation_ = kVideoRotation_0;
    absl::optional<ColorSpace> color_space_;
    absl::optional<PartialFrameDescription> partial_frame_description_;
    bool cache_buffer_for_partial_updates_;
  };

  // To be deprecated. Migrate all use to Builder.
  VideoFrame(const rtc::scoped_refptr<VideoFrameBuffer>& buffer,
             webrtc::VideoRotation rotation,
             int64_t timestamp_us);
  VideoFrame(const rtc::scoped_refptr<VideoFrameBuffer>& buffer,
             uint32_t timestamp_rtp,
             int64_t render_time_ms,
             VideoRotation rotation);

  ~VideoFrame();

  // Support move and copy.
  VideoFrame(const VideoFrame&);
  VideoFrame(VideoFrame&&);
  VideoFrame& operator=(const VideoFrame&);
  VideoFrame& operator=(VideoFrame&&);

  // Get frame width.
  int width() const;
  // Get frame height.
  int height() const;
  // Get frame size in pixels.
  uint32_t size() const;

  // Get frame ID. Returns 0 if ID is not set. Not guarantee to be transferred
  // from the sender to the receiver, but preserved on single side. The id
  // should be propagated between all frame modifications during its lifetime
  // from capturing to sending as encoded image. It is intended to be unique
  // over a time window of a few minutes for peer connection, to which
  // corresponding video stream belongs to.
  uint16_t id() const { return id_; }
  void set_id(uint16_t id) { id_ = id; }

  // System monotonic clock, same timebase as rtc::TimeMicros().
  int64_t timestamp_us() const { return timestamp_us_; }
  void set_timestamp_us(int64_t timestamp_us) { timestamp_us_ = timestamp_us; }

  // TODO(nisse): After the cricket::VideoFrame and webrtc::VideoFrame
  // merge, timestamps other than timestamp_us will likely be
  // deprecated.

  // Set frame timestamp (90kHz).
  void set_timestamp(uint32_t timestamp) { timestamp_rtp_ = timestamp; }

  // Get frame timestamp (90kHz).
  uint32_t timestamp() const { return timestamp_rtp_; }

  // For now, transport_frame_id and rtp timestamp are the same.
  // TODO(nisse): Must be handled differently for QUIC.
  uint32_t transport_frame_id() const { return timestamp(); }

  // Set capture ntp time in milliseconds.
  // TODO(nisse): Deprecated. Migrate all users to timestamp_us().
  void set_ntp_time_ms(int64_t ntp_time_ms) { ntp_time_ms_ = ntp_time_ms; }

  // Get capture ntp time in milliseconds.
  // TODO(nisse): Deprecated. Migrate all users to timestamp_us().
  int64_t ntp_time_ms() const { return ntp_time_ms_; }

  // Naming convention for Coordination of Video Orientation. Please see
  // http://www.etsi.org/deliver/etsi_ts/126100_126199/126114/12.07.00_60/ts_126114v120700p.pdf
  //
  // "pending rotation" or "pending" = a frame that has a VideoRotation > 0.
  //
  // "not pending" = a frame that has a VideoRotation == 0.
  //
  // "apply rotation" = modify a frame from being "pending" to being "not
  //                    pending" rotation (a no-op for "unrotated").
  //
  VideoRotation rotation() const { return rotation_; }
  void set_rotation(VideoRotation rotation) { rotation_ = rotation; }

  // Get color space when available.
  const ColorSpace* color_space() const {
    return color_space_ ? &*color_space_ : nullptr;
  }
  void set_color_space(ColorSpace* color_space) {
    color_space_ =
        color_space ? absl::make_optional(*color_space) : absl::nullopt;
  }

  const PartialFrameDescription* partial_frame_description() const {
    return partial_frame_description_ ? &partial_frame_description_.value()
                                      : nullptr;
  }
  void set_partial_frame_description(
      const absl::optional<PartialFrameDescription>& description) {
    partial_frame_description_ = description;
  }

  void set_cache_buffer_for_partial_updates(
      bool cache_buffer_for_partial_updates) {
    cache_buffer_for_partial_updates_ = cache_buffer_for_partial_updates;
  }
  bool cache_buffer_for_partial_updates() const {
    return cache_buffer_for_partial_updates_;
  }

  // Get render time in milliseconds.
  // TODO(nisse): Deprecated. Migrate all users to timestamp_us().
  int64_t render_time_ms() const;

  // Return the underlying buffer. This can only be a nullptr for a partial
  // update VideoFrame with no changed pixels.
  rtc::scoped_refptr<webrtc::VideoFrameBuffer> video_frame_buffer() const;
  void set_video_frame_buffer(rtc::scoped_refptr<VideoFrameBuffer> buffer);

  // TODO(nisse): Deprecated.
  // Return true if the frame is stored in a texture.
  bool is_texture() const {
    return video_frame_buffer()->type() == VideoFrameBuffer::Type::kNative;
  }

 private:
  VideoFrame(
      uint16_t id,
      const rtc::scoped_refptr<VideoFrameBuffer>& buffer,
      int64_t timestamp_us,
      uint32_t timestamp_rtp,
      int64_t ntp_time_ms,
      VideoRotation rotation,
      const absl::optional<ColorSpace>& color_space,
      const absl::optional<PartialFrameDescription> partial_frame_description,
      bool cache_buffer_for_partial_updates_);

  uint16_t id_;
  // An opaque reference counted handle that stores the pixel data.
  rtc::scoped_refptr<webrtc::VideoFrameBuffer> video_frame_buffer_;
  uint32_t timestamp_rtp_;
  int64_t ntp_time_ms_;
  int64_t timestamp_us_;
  VideoRotation rotation_;
  absl::optional<ColorSpace> color_space_;
  absl::optional<PartialFrameDescription> partial_frame_description_;
  // Should be set on all frames, if the source may produce partial updates.
  bool cache_buffer_for_partial_updates_;
};

}  // namespace webrtc

#endif  // API_VIDEO_VIDEO_FRAME_H_
