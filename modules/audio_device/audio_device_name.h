/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_DEVICE_AUDIO_DEVICE_NAME_H_
#define MODULES_AUDIO_DEVICE_AUDIO_DEVICE_NAME_H_

#include <list>
#include <string>

namespace webrtc {

struct AudioDeviceName {
  AudioDeviceName();
  AudioDeviceName(const std::string& device_name, const std::string& unique_id);

  // TODO(henrika): do we need these?
  // Creates default device representation.
  // static AudioDeviceName CreateDefault();

  // Creates communications device representation.
  // static AudioDeviceName CreateCommunications();

  std::string device_name;  // Friendly name of the device.
  std::string unique_id;    // Unique identifier for the device.
};

typedef std::list<AudioDeviceName> AudioDeviceNames;

}  // namespace webrtc

#endif  // MODULES_AUDIO_DEVICE_AUDIO_DEVICE_NAME_H_
