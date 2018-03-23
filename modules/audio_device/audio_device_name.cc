/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_device/audio_device_name.h"
#include "modules/audio_device/audio_device_description.h"

namespace webrtc {

AudioDeviceName::AudioDeviceName() = default;

AudioDeviceName::AudioDeviceName(const std::string& device_name,
                                 const std::string& unique_id)
    : device_name(device_name), unique_id(unique_id) {}

}  // namespace webrtc
