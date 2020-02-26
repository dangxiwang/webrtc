/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef VIDEO_QUALITY_SCALER_RESOURCE_H_
#define VIDEO_QUALITY_SCALER_RESOURCE_H_

#include <memory>
#include <string>

#include "api/rtp_parameters.h"
#include "api/video_codecs/video_encoder.h"
#include "call/adaptation/resource.h"
#include "modules/video_coding/utility/quality_scaler.h"
#include "rtc_base/experiments/balanced_degradation_settings.h"

namespace webrtc {

// Handles interaction with the QualityScaler.
// TODO(hbos): Add unittests specific to this class, it is currently only tested
// indirectly by usage in the OveruseFrameDetectorResourceAdaptationModule
// (which is only tested because of its usage in VideoStreamEncoder); all tests
// are currently in video_stream_encoder_unittest.cc.
class QualityScalerResource : public Resource,
                              public AdaptationObserverInterface {
 public:
  QualityScalerResource();
  virtual ~QualityScalerResource();

  void Configure(const VideoEncoder::EncoderInfo& encoder_info,
                 DegradationPreference degradation_preference,
                 VideoCodecType codec_type,
                 int pixels);
  void StopCheckForOveruse();

  bool is_started() const;


  bool QpFastFilterLow();
  void OnEncodeCompleted(const EncodedImage& encoded_image,
                         int64_t time_sent_in_us);
  void OnFrameDropped(EncodedImageCallback::DropReason reason);

  // AdaptationObserverInterface implementation.
  // TODO(https://crbug.com/webrtc/11222, 11172): This resource also needs to
  // signal when its stable to support multi-stream aware modules.
  void AdaptUp(AdaptReason reason) override;
  bool AdaptDown(AdaptReason reason) override;

  std::string name() const override { return "QualityScalerResource"; }

 private:
  std::unique_ptr<QualityScaler> quality_scaler_;
  const bool quality_scaling_experiment_enabled_;
  const BalancedDegradationSettings balanced_settings_;
};

}  // namespace webrtc

#endif  // VIDEO_QUALITY_SCALER_RESOURCE_H_
