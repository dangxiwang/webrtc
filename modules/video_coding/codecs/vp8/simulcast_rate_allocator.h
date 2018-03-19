/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_VIDEO_CODING_CODECS_VP8_SIMULCAST_RATE_ALLOCATOR_H_
#define MODULES_VIDEO_CODING_CODECS_VP8_SIMULCAST_RATE_ALLOCATOR_H_

#include <stdint.h>

#include <map>
#include <memory>
#include <vector>

#include "api/video_codecs/video_encoder.h"
#include "common_video/include/video_bitrate_allocator.h"
#include "modules/video_coding/codecs/vp8/temporal_layers.h"
#include "rtc_base/constructormagic.h"

namespace webrtc {

class SimulcastRateAllocator : public VideoBitrateAllocator,
                               public TemporalLayersListener {
 public:
  explicit SimulcastRateAllocator(
      const VideoCodec& codec,
      std::unique_ptr<TemporalLayersFactory> tl_factory);

  void OnTemporalLayersCreated(int simulcast_id,
                               TemporalLayers* layers) override;

  BitrateAllocation GetAllocation(uint32_t total_bitrate_bps,
                                  uint32_t framerate) override;
  uint32_t GetPreferredBitrateBps(uint32_t framerate) override;
  const VideoCodec& GetCodec() const;

 private:
  void DistributeAllocationToSimulcastLayers(
      uint32_t total_bitrate_bps,
      BitrateAllocation* allocated_bitrates_bps) const;
  void DistributeAllocationToTemporalLayers(
      uint32_t framerate,
      BitrateAllocation* allocated_bitrates_bps) const;
  std::vector<uint32_t> DefaultTemporalLayerAllocation(int bitrate_kbps,
                                                       int max_bitrate_kbps,
                                                       int framerate,
                                                       int simulcast_id) const;
  std::vector<uint32_t> ScreenshareTemporalLayerAllocation(
      int bitrate_kbps,
      int max_bitrate_kbps,
      int framerate,
      int simulcast_id) const;
  int NumTemporalStreams(size_t simulcast_id) const;

  const VideoCodec codec_;
  std::map<uint32_t, TemporalLayers*> temporal_layers_;
  std::unique_ptr<TemporalLayersFactory> tl_factory_;

  RTC_DISALLOW_COPY_AND_ASSIGN(SimulcastRateAllocator);
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_CODECS_VP8_SIMULCAST_RATE_ALLOCATOR_H_
