/*
 *  Copyright (c) 2022 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef PC_TEST_SVC_VIDEO_QUALITY_ANALYZER_H_
#define PC_TEST_SVC_VIDEO_QUALITY_ANALYZER_H_

#include <map>

#include "rtc_base/containers/flat_map.h"
#include "test/gmock.h"
#include "test/gtest.h"
#include "test/pc/e2e/analyzer/video/default_video_quality_analyzer.h"

namespace webrtc {

// Records how many frames are seen for each spatial and temporal index at the
// encoder and decoder level.
class SvcVideoQualityAnalyzer : public DefaultVideoQualityAnalyzer {
 public:
  struct FrameSize {
    FrameSize(int width, int height) : width(width), height(height) {}

    bool operator<(const FrameSize& other) const {
      return std::tie(width, height) < std::tie(other.width, other.height);
    }

    int width;
    int height;
  };

  using SpatialTemporalLayerCounts =
      webrtc::flat_map<int, webrtc::flat_map<int, int>>;

  explicit SvcVideoQualityAnalyzer(webrtc::Clock* clock);
  ~SvcVideoQualityAnalyzer() override;

  void OnFrameEncoded(absl::string_view peer_name,
                      uint16_t frame_id,
                      const EncodedImage& encoded_image,
                      const EncoderStats& stats) override;

  void OnFramePreDecode(absl::string_view peer_name,
                        uint16_t frame_id,
                        const EncodedImage& input_image) override;

  void OnFrameDecoded(absl::string_view peer_name,
                      const VideoFrame& frame,
                      const DecoderStats& stats) override;

  const SpatialTemporalLayerCounts& encoder_layers_seen() const {
    return encoder_layers_seen_;
  }
  const SpatialTemporalLayerCounts& decoder_layers_seen() const {
    return decoder_layers_seen_;
  }
  const std::map<FrameSize, int>& frames_decoded() const {
    return frames_decoded_;
  }

  static int CountDecodedFramesForLayer(
      const SpatialTemporalLayerCounts& counts,
      int target_spatial_index,
      int target_temporal_index);

 private:
  SpatialTemporalLayerCounts encoder_layers_seen_;
  SpatialTemporalLayerCounts decoder_layers_seen_;
  std::map<FrameSize, int> frames_decoded_;
};

MATCHER_P2(HasSpatialAndTemporalLayers,
           expected_spatial_layers,
           expected_temporal_layers,
           "") {
  if (arg.size() != (size_t)expected_spatial_layers) {
    *result_listener << "spatial layer count mismatch expected "
                     << expected_spatial_layers << " but got " << arg.size();
    return false;
  }
  for (const auto& spatial_layer : arg) {
    if (spatial_layer.first < 0 ||
        spatial_layer.first >= expected_spatial_layers) {
      *result_listener << "spatial layer index is not in range [0,"
                       << expected_spatial_layers << "[.";
      return false;
    }

    if (spatial_layer.second.size() != (size_t)expected_temporal_layers) {
      *result_listener << "temporal layer count mismatch on spatial layer "
                       << spatial_layer.first << ", expected "
                       << expected_temporal_layers << " but got "
                       << spatial_layer.second.size();
      return false;
    }
    for (const auto& temporal_layer : spatial_layer.second) {
      if (temporal_layer.first < 0 ||
          temporal_layer.first >= expected_temporal_layers) {
        *result_listener << "temporal layer index on spatial layer "
                         << spatial_layer.first << " is not in range [0,"
                         << expected_temporal_layers << "[.";
        return false;
      }
    }
  }
  return true;
}

}  // namespace webrtc

#endif  // PC_TEST_SVC_VIDEO_QUALITY_ANALYZER_H_
