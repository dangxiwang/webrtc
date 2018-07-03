/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/video/video_stream_decoder_create.h"

#include "absl/memory/memory.h"
#include "video/video_stream_decoder_impl.h"

namespace webrtc {
std::unique_ptr<VideoStreamDecoder> CreateVideoStreamDecoder(
    VideoStreamDecoder::Callbacks* callbacks,
    VideoDecoderFactory* decoder_factory,
    std::map<int, std::pair<SdpVideoFormat, int>> decoder_settings) {
  return absl::make_unique<VideoStreamDecoderImpl>(callbacks, decoder_factory,
                                                   std::move(decoder_settings));
}
}  // namespace webrtc
