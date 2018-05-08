/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_device/win/core_audio_input_win.h"

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {

using win::CoreAudioUtility;

namespace win_adm {

CoreAudioInput::CoreAudioInput()
    : CoreAudioBase(CoreAudioBase::Direction::kInput) {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
}

CoreAudioInput::~CoreAudioInput() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
}

int CoreAudioInput::Init() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  return 0;
}

int CoreAudioInput::Terminate() {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  return 0;
}

int CoreAudioInput::NumDevices() const {
  RTC_DCHECK_RUN_ON(&thread_checker_);
  return CoreAudioUtility::NumberOfActiveDevices(eCapture);
}

int CoreAudioInput::SetDevice(int index) {
  RTC_DLOG(INFO) << __FUNCTION__ << ": " << index;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  if (initialized_) {
    return -1;
  }

  std::string device_id = GetDeviceID(index);
  RTC_DLOG(INFO) << "index=" << index << " => device_id: " << device_id;
  device_id_ = device_id;

  return device_id_.empty() ? -1 : 0;
}

int CoreAudioInput::SetDevice(AudioDeviceModule::WindowsDeviceType device) {
  RTC_DLOG(INFO) << __FUNCTION__ << ": " << device;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  return SetDevice((device == AudioDeviceModule::kDefaultDevice) ? 0 : 1);
}

int CoreAudioInput::DeviceName(int index,
                               char name[kAdmMaxDeviceNameSize],
                               char guid[kAdmMaxGuidSize]) {
  RTC_DLOG(INFO) << __FUNCTION__ << ": " << index;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  RTC_DCHECK(name);
  return CoreAudioBase::DeviceName(index, name, guid);
}

void CoreAudioInput::AttachAudioBuffer(AudioDeviceBuffer* audio_buffer) {
  RTC_DLOG(INFO) << __FUNCTION__;
  RTC_DCHECK_RUN_ON(&thread_checker_);
  audio_device_buffer_ = audio_buffer;
}

}  // namespace win_adm

}  // namespace webrtc
