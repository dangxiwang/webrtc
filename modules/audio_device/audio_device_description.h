/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_DEVICE_AUDIO_DEVICE_DESCRIPTION_H_
#define MODULES_AUDIO_DEVICE_AUDIO_DEVICE_DESCRIPTION_H_

#include <string>
#include <vector>

namespace webrtc {

// Provides common information on audio device names and ids.
// TODO(henrika): add comments and mention that the implementation is a
// simplified version of the same class in Chrome.
struct AudioDeviceDescription {
  // Unique Id of the generic "default" device. Associated with the localized
  // name returned from GetDefaultDeviceName().
  static const char kDefaultDeviceId[];

  // Unique Id of the generic default communications device. Associated with
  // the localized name returned from GetCommunicationsDeviceName().
  static const char kCommunicationsDeviceId[];

  // Returns true if |device_id| represents the default device.
  static bool IsDefaultDevice(const std::string& device_id);

  // Returns true if |device_id| represents the communications device.
  static bool IsCommunicationsDevice(const std::string& device_id);

  AudioDeviceDescription() = default;
  AudioDeviceDescription(const AudioDeviceDescription& other) = default;
  AudioDeviceDescription(std::string device_name,
                         std::string unique_id,
                         std::string group_id);

  ~AudioDeviceDescription() = default;

  std::string device_name;  // Friendly name of the device.
  std::string unique_id;    // Unique identifier for the device.
  std::string group_id;     // Group identifier.
};

typedef std::vector<AudioDeviceDescription> AudioDeviceDescriptions;

}  // namespace webrtc

#endif  // MODULES_AUDIO_DEVICE_AUDIO_DEVICE_DESCRIPTION_H_
