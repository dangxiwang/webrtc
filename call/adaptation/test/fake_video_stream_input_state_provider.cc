/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "call/adaptation/test/fake_video_stream_input_state_provider.h"

namespace webrtc {

FakeVideoStreamInputStateProvider::FakeVideoStreamInputStateProvider()
    : VideoStreamInputStateProvider(nullptr) {}

FakeVideoStreamInputStateProvider::~FakeVideoStreamInputStateProvider() =
    default;

void FakeVideoStreamInputStateProvider::SetInputState(
    int input_pixels,
    int input_fps,
    int min_pixels_per_frame) {
  VideoStreamInputState input_state;
  input_state.set_has_input(true);
  input_state.set_frame_size_pixels(input_pixels);
  input_state.set_frames_per_second(input_fps);
  input_state.set_min_pixels_per_frame(min_pixels_per_frame);
  fake_input_state_ = input_state;
}

VideoStreamInputState FakeVideoStreamInputStateProvider::InputState() {
  return fake_input_state_;
}

}  // namespace webrtc
