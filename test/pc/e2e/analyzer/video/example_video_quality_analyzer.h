/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef TEST_PC_E2E_ANALYZER_VIDEO_EXAMPLE_VIDEO_QUALITY_ANALYZER_H_
#define TEST_PC_E2E_ANALYZER_VIDEO_EXAMPLE_VIDEO_QUALITY_ANALYZER_H_

#include <atomic>
#include <set>
#include <string>

#include "api/video/encoded_image.h"
#include "api/video/video_frame.h"
#include "rtc_base/critical_section.h"
#include "test/pc/e2e/api/video_quality_analyzer_interface.h"

namespace webrtc {
namespace test {

// This class is an example implementation of
// webrtc::VideoQualityAnalyzerInterface and calculates simple metrics
// just to demonstration purposes. Assumed to be used in the single process
// test cases, where both peers are in the same process.
class ExampleVideoQualityAnalyzer : public VideoQualityAnalyzerInterface {
 public:
  ExampleVideoQualityAnalyzer();
  ~ExampleVideoQualityAnalyzer() override;

  void Start(int max_threads_count) override;
  uint16_t OnFrameCaptured(const std::string& stream_label,
                           const VideoFrame& frame) override;
  void OnFramePreEncode(const VideoFrame& frame) override;
  void OnFrameEncoded(uint16_t frame_id,
                      const EncodedImage& encoded_image) override;
  void OnFrameDropped(EncodedImageCallback::DropReason reason) override;
  void OnFrameReceived(uint16_t frame_id,
                       const EncodedImage& encoded_image) override;
  void OnFrameDecoded(const VideoFrame& frame,
                      absl::optional<int32_t> decode_time_ms,
                      absl::optional<uint8_t> qp) override;
  void OnFrameRendered(const VideoFrame& frame) override;
  void OnEncoderError(const VideoFrame& frame, int32_t error_code) override;
  void OnDecoderError(uint16_t frame_id, int32_t error_code) override;
  void Stop() override;

  uint64_t frames_captured() const;
  uint64_t frames_sent() const;
  uint64_t frames_received() const;
  uint64_t frames_dropped() const;
  uint64_t frames_rendered() const;

 private:
  // When peer A captured the frame it will come into analyzer's OnFrameCaptured
  // and will be stored in frames_in_flight_. It will be removed from there
  // when it will be received in peer B, so we need to guard it with lock.
  // Also because analyzer will serve for all video streams it can be called
  // from different threads inside one peer.
  rtc::CriticalSection lock_;
  // Stores frame ids, that are currently going from one peer to another. We
  // need to keep them to correctly determine dropped frames and also correctly
  // process frame id overlap.
  std::set<uint16_t> frames_in_flight_ RTC_GUARDED_BY(lock_);
  uint16_t next_frame_id_ RTC_GUARDED_BY(lock_) = 0;
  uint64_t frames_captured_ RTC_GUARDED_BY(lock_) = 0;
  uint64_t frames_sent_ RTC_GUARDED_BY(lock_) = 0;
  uint64_t frames_received_ RTC_GUARDED_BY(lock_) = 0;
  uint64_t frames_dropped_ RTC_GUARDED_BY(lock_) = 0;
  uint64_t frames_rendered_ RTC_GUARDED_BY(lock_) = 0;
};

}  // namespace test
}  // namespace webrtc

#endif  // TEST_PC_E2E_ANALYZER_VIDEO_EXAMPLE_VIDEO_QUALITY_ANALYZER_H_
