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
#include <objbase.h>
#include <propidl.h>
#include <wrl/client.h>

#include <string>

#include "modules/audio_device/audio_device_description.h"
#include "modules/audio_device/audio_device_name.h"
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

// A PROPVARIANT that is automatically initialized and cleared upon respective
// construction and destruction of this class.
class ScopedPropVariant {
 public:
  ScopedPropVariant() { PropVariantInit(&pv_); }

  ScopedPropVariant(const ScopedPropVariant&) = delete;
  ScopedPropVariant& operator=(const ScopedPropVariant&) = delete;

  ~ScopedPropVariant() { Reset(); }

  // Returns a pointer to the underlying PROPVARIANT for use as an out param in
  // a function call.
  PROPVARIANT* Receive() {
    RTC_DCHECK_EQ(pv_.vt, VT_EMPTY);
    return &pv_;
  }

  // Clears the instance to prepare it for re-use (e.g., via Receive).
  void Reset() {
    if (pv_.vt != VT_EMPTY) {
      HRESULT result = PropVariantClear(&pv_);
      RTC_DCHECK_EQ(result, S_OK);
    }
  }

  const PROPVARIANT& get() const { return pv_; }
  const PROPVARIANT* ptr() const { return &pv_; }

 private:
  PROPVARIANT pv_;

  // Comparison operators for ScopedPropVariant are not supported at this point.
  bool operator==(const ScopedPropVariant&) const;
  bool operator!=(const ScopedPropVariant&) const;
};

// Simple scoped memory releaser class for COM allocated memory.
// Example:
//   wbrtc::win::ScopedCoMem<ITEMIDLIST> file_item;
//   SHGetSomeInfo(&file_item, ...);
//   ...
//   return;  <-- memory released
template <typename T>
class ScopedCoMem {
 public:
  ScopedCoMem() : mem_ptr_(nullptr) {}
  ~ScopedCoMem() { Reset(nullptr); }

  ScopedCoMem(const ScopedCoMem&) = delete;
  ScopedCoMem& operator=(const ScopedCoMem&) = delete;

  T** operator&() {                   // NOLINT
    RTC_DCHECK(mem_ptr_ == nullptr);  // To catch memory leaks.
    return &mem_ptr_;
  }

  operator T*() { return mem_ptr_; }

  T* operator->() {
    DCHECK(mem_ptr_ != nullptr);
    return mem_ptr_;
  }

  const T* operator->() const {
    DCHECK(mem_ptr_ != nullptr);
    return mem_ptr_;
  }

  void Reset(T* ptr) {
    if (mem_ptr_)
      CoTaskMemFree(mem_ptr_);
    mem_ptr_ = ptr;
  }

  T* get() const { return mem_ptr_; }

 private:
  T* mem_ptr_;
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

  // Returns the unique ID and user-friendly name of a given endpoint device.
  // Example: "{0.0.1.00000000}.{8db6020f-18e3-4f25-b6f5-7726c9122574}", and
  //          "Microphone (Realtek High Definition Audio)".
  static HRESULT GetDeviceName(IMMDevice* device, AudioDeviceName* name);

  // Gets the user-friendly name of the endpoint device which is represented
  // by a unique id in |device_id|.
  static std::string GetFriendlyName(const std::string& device_id,
                                     EDataFlow data_flow,
                                     ERole role);

  // Query if the audio device is a rendering device or a capture device.
  static EDataFlow GetDataFlow(IMMDevice* device);
};

}  // namespace win
}  // namespace webrtc

#endif  //  MODULES_AUDIO_DEVICE_WIN_CORE_AUDIO_UTILITY_WIN_H_
