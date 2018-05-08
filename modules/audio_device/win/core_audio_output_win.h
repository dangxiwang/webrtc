/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_OUTPUT_WIN_H_
#define MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_OUTPUT_WIN_H_

#include <memory>
#include <string>

#include "modules/audio_device/win/audio_device_module_win.h"
#include "modules/audio_device/win/core_audio_base_win.h"

namespace webrtc {

class AudioDeviceBuffer;
class FineAudioBuffer;

namespace win_adm {

class CoreAudioOutput final : public CoreAudioBase, public AudioOutput {
 public:
  CoreAudioOutput();
  ~CoreAudioOutput() override;

  // AudioOutput implementation.
  int Init() override;
  int Terminate() override;
  int NumDevices() const override;
  int SetDevice(int index) override;
  int SetDevice(AudioDeviceModule::WindowsDeviceType device) override;
  int DeviceName(int index,
                 char name[kAdmMaxDeviceNameSize],
                 char guid[kAdmMaxGuidSize]) override;
  void AttachAudioBuffer(AudioDeviceBuffer* audio_buffer) override;
  bool PlayoutIsInitialized() const override;
  int InitPlayout() override;
  int StartPlayout() override;
  int StopPlayout() override;
  bool Playing() override;

  CoreAudioOutput(const CoreAudioOutput&) = delete;
  CoreAudioOutput& operator=(const CoreAudioOutput&) = delete;

 private:
  bool playing_ = false;
  std::unique_ptr<FineAudioBuffer> fine_audio_buffer_;
  Microsoft::WRL::ComPtr<IAudioRenderClient> audio_render_client_;
};

}  // namespace win_adm

}  // namespace webrtc

#endif  // MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_OUTPUT_WIN_H_
