/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_VIDEO_CODECS_VIDEO_ENCODER_SOFTWARE_FALLBACK_WRAPPER_H_
#define API_VIDEO_CODECS_VIDEO_ENCODER_SOFTWARE_FALLBACK_WRAPPER_H_

#include <memory>

#include "api/video_codecs/video_encoder.h"
#include "rtc_base/system/rtc_export.h"

namespace webrtc {

// Used to wrap external VideoEncoders to provide a fallback option on
// software encoding when a hardware encoder fails to encode a stream due to
// hardware restrictions, such as max resolution.
RTC_EXPORT std::unique_ptr<VideoEncoder>
CreateVideoEncoderSoftwareFallbackWrapper(
    std::unique_ptr<VideoEncoder> sw_fallback_encoder,
    std::unique_ptr<VideoEncoder> hw_encoder);

}  // namespace webrtc

#endif  // API_VIDEO_CODECS_VIDEO_ENCODER_SOFTWARE_FALLBACK_WRAPPER_H_
