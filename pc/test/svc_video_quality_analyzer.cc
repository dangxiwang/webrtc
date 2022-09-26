/*
 *  Copyright (c) 2022 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "pc/test/svc_video_quality_analyzer.h"

#include "api/test/metrics/global_metrics_logger_and_exporter.h"

namespace webrtc {

SvcVideoQualityAnalyzer::SvcVideoQualityAnalyzer(webrtc::Clock* clock)
    : DefaultVideoQualityAnalyzer(clock,
                                  test::GetGlobalMetricsLogger(),
                                  DefaultVideoQualityAnalyzerOptions{
                                      .compute_psnr = false,
                                      .compute_ssim = false,
                                  }) {}

SvcVideoQualityAnalyzer::~SvcVideoQualityAnalyzer() = default;

void SvcVideoQualityAnalyzer::OnFrameEncoded(absl::string_view peer_name,
                                             uint16_t frame_id,
                                             const EncodedImage& encoded_image,
                                             const EncoderStats& stats,
                                             bool discarded) {
  absl::optional<int> spatial_id = encoded_image.SpatialIndex();
  absl::optional<int> temporal_id = encoded_image.TemporalIndex();
  encoder_layers_seen_[spatial_id.value_or(0)][temporal_id.value_or(0)]++;
  DefaultVideoQualityAnalyzer::OnFrameEncoded(peer_name, frame_id,
                                              encoded_image, stats, discarded);
}

void SvcVideoQualityAnalyzer::OnFramePreDecode(
    absl::string_view peer_name,
    uint16_t frame_id,
    const EncodedImage& input_image) {
  absl::optional<int> spatial_id = input_image.SpatialIndex();
  absl::optional<int> temporal_id = input_image.TemporalIndex();
  if (!spatial_id) {
    decoder_layers_seen_[0][temporal_id.value_or(0)]++;
  } else {
    for (int i = 0; i <= *spatial_id; ++i) {
      // If there are no spatial layers (for example VP8), we still want to
      // record the temporal index for pseudo-layer "0" frames.
      if (*spatial_id == 0 ||
          input_image.SpatialLayerFrameSize(i).value_or(0) > 0) {
        decoder_layers_seen_[i][temporal_id.value_or(0)]++;
      }
    }
  }
  DefaultVideoQualityAnalyzer::OnFramePreDecode(peer_name, frame_id,
                                                input_image);
}

void SvcVideoQualityAnalyzer::OnFrameDecoded(absl::string_view peer_name,
                                             const VideoFrame& frame,
                                             const DecoderStats& stats) {
  frames_decoded_[FrameSize{frame.width(), frame.height()}]++;
  DefaultVideoQualityAnalyzer::OnFrameDecoded(peer_name, frame, stats);
}

int SvcVideoQualityAnalyzer::CountDecodedFramesForLayer(
    const SpatialTemporalLayerCounts& counts,
    int target_spatial_index,
    int target_temporal_index) {
  int sum = 0;
  for (const auto& spatial_layer_counts : counts) {
    if (spatial_layer_counts.first != target_spatial_index) {
      continue;
    }
    for (const auto& temporal_layer_counts : spatial_layer_counts.second) {
      if (temporal_layer_counts.first > target_temporal_index) {
        continue;
      }
      sum += temporal_layer_counts.second;
    }
  }
  return sum;
}

}  // namespace webrtc
