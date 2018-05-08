/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_INPUT_WIN_H_
#define MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_INPUT_WIN_H_

#include <string>

#include "modules/audio_device/win/audio_device_module_win.h"
#include "modules/audio_device/win/core_audio_base_win.h"
#include "modules/audio_device/win/core_audio_utility_win.h"
#include "rtc_base/thread_checker.h"

namespace webrtc {

class AudioDeviceBuffer;

namespace win_adm {

class CoreAudioInput final : public CoreAudioBase, public AudioInput {
 public:
  CoreAudioInput();
  ~CoreAudioInput() override;

  // AudioInput implementation.
  int Init() override;
  int Terminate() override;
  int NumDevices() const override;
  int SetDevice(int index) override;
  int SetDevice(AudioDeviceModule::WindowsDeviceType device) override;
  int DeviceName(int index,
                 char name[kAdmMaxDeviceNameSize],
                 char guid[kAdmMaxGuidSize]) override;

  void AttachAudioBuffer(AudioDeviceBuffer* audio_buffer) override;

  CoreAudioInput(const CoreAudioInput&) = delete;
  CoreAudioInput& operator=(const CoreAudioInput&) = delete;

 private:
};

}  // namespace win_adm

}  // namespace webrtc

#endif  // MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_INPUT_WIN_H_
