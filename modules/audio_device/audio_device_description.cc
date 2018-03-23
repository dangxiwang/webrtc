/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_device/audio_device_description.h"

#include <utility>

namespace webrtc {

const char AudioDeviceDescription::kDefaultDeviceId[] = "default";
const char AudioDeviceDescription::kCommunicationsDeviceId[] = "communications";

// static
bool AudioDeviceDescription::IsDefaultDevice(const std::string& device_id) {
  return device_id.empty() ||
         device_id == AudioDeviceDescription::kDefaultDeviceId;
}

// static
bool AudioDeviceDescription::IsCommunicationsDevice(
    const std::string& device_id) {
  return device_id == AudioDeviceDescription::kCommunicationsDeviceId;
}

AudioDeviceDescription::AudioDeviceDescription(std::string device_name,
                                               std::string unique_id,
                                               std::string group_id)
    : device_name(std::move(device_name)),
      unique_id(std::move(unique_id)),
      group_id(std::move(group_id)) {}

}  // namespace webrtc
