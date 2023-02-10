/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef SDK_OBJC_NATIVE_API_AUDIO_DEVICE_MODULE_H_
#define SDK_OBJC_NATIVE_API_AUDIO_DEVICE_MODULE_H_

#ifdef WEBRTC_IOS
#include <memory>

#include "modules/audio_device/include/audio_device.h"
#include "sdk/objc/native/src/audio/audio_device_module_ios.h"

namespace webrtc {

// If `bypass_voice_processing` is true, WebRTC will attempt to disable hardware
// audio processing on iOS.
// Warning: Setting `bypass_voice_processing` will have unpredictable
// consequences for the audio path in the device. It is not advisable to use in
// most scenarios.
rtc::scoped_refptr<ios_adm::AudioDeviceModuleIOS> CreateAudioDeviceModule(
    bool bypass_voice_processing = false);

}  // namespace webrtc

#endif  // WEBRTC_IOS

#endif  // SDK_OBJC_NATIVE_API_AUDIO_DEVICE_MODULE_H_
