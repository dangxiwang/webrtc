/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_UTILITY_WIN_H_
#define MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_UTILITY_WIN_H_

#include <mmdeviceapi.h>
#include <wrl/client.h>

#include <string>

#include "modules/audio_device/audio_device_description.h"
#include "rtc_base/thread_checker.h"

namespace webrtc {
namespace win {

// Initializes COM in the constructor (STA or MTA), and uninitializes COM in the
// destructor. Taken from base::win::ScopedCOMInitializer.
//
// WARNING: This should only be used once per thread, ideally scoped to a
// similar lifetime as the thread itself.  You should not be using this in
// random utility functions that make COM calls; instead ensure that these
// functions are running on a COM-supporting thread!
class ScopedCOMInitializer {
 public:
  // Enum value provided to initialize the thread as an MTA instead of STA.
  enum SelectMTA { kMTA };

  // Constructor for STA initialization.
  ScopedCOMInitializer() { Initialize(COINIT_APARTMENTTHREADED); }

  // Constructor for MTA initialization.
  explicit ScopedCOMInitializer(SelectMTA mta) {
    Initialize(COINIT_MULTITHREADED);
  }

  ScopedCOMInitializer(const ScopedCOMInitializer&) = delete;
  ScopedCOMInitializer& operator=(const ScopedCOMInitializer&) = delete;

  ~ScopedCOMInitializer() {
    RTC_DCHECK_RUN_ON(&thread_checker_);
    if (Succeeded()) {
      CoUninitialize();
    }
  }

  bool Succeeded() { return SUCCEEDED(hr_); }

 private:
  void Initialize(COINIT init) {
    RTC_DCHECK_RUN_ON(&thread_checker_);
    // Initializes the COM library for use by the calling thread, sets the
    // thread's concurrency model, and creates a new apartment for the thread
    // if one is required.
    hr_ = CoInitializeEx(NULL, init);
    RTC_CHECK_NE(RPC_E_CHANGED_MODE, hr_) << "Invalid COM thread model change";
  }
  HRESULT hr_;
  rtc::ThreadChecker thread_checker_;
};

// Utility methods for the Core Audio API on Windows.
// Always ensure that Core Audio is supported before using these methods.
// Use webrtc::CoreAudioUtility::IsSupported() for this purpose.
// Also, all methods must be called on a valid COM thread. This can be done
// by using the webrtc::win::ScopedCOMInitializer helper class.
// TODO(henrika): mention that this class is based on media::CoreAudioUtil
// in Chrome.
class CoreAudioUtility {
 public:
  // Returns true if Windows Core Audio is supported.
  // Always verify that this method returns true before using any of the
  // other methods in this class.
  static bool IsSupported();

  // Number of active audio devices in the specified data flow direction.
  // Set |data_flow| to eAll to retrieve the total number of active audio
  // devices.
  static int NumberOfActiveDevices(EDataFlow data_flow);

  // Creates an IMMDeviceEnumerator interface which provides methods for
  // enumerating audio endpoint devices.
  static Microsoft::WRL::ComPtr<IMMDeviceEnumerator> CreateDeviceEnumerator();

  // Creates an endpoint device specified by |device_id| or a default device
  // specified by data-flow direction and role if
  // AudioDeviceDescription::IsDefaultDevice(|device_id|) is true.
  static Microsoft::WRL::ComPtr<IMMDevice>
  CreateDevice(const std::string& device_id, EDataFlow data_flow, ERole role);

  // Query if the audio device is a rendering device or a capture device.
  static EDataFlow GetDataFlow(IMMDevice* device);
};

}  // namespace win
}  // namespace webrtc

#endif  //  MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_UTILITY_WIN_H_
