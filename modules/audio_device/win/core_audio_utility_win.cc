/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_device/win/core_audio_utility_win.h"

#include <comdef.h>
#include <stdio.h>
#include <tchar.h>

#include <iomanip>
#include <string>

#include "rtc_base/arraysize.h"
#include "rtc_base/logging.h"
#include "rtc_base/platform_thread_types.h"
#include "rtc_base/strings/string_builder.h"
#include "rtc_base/stringutils.h"

using Microsoft::WRL::ComPtr;

namespace webrtc {
namespace win {

namespace {

std::string ErrorToString(const _com_error& error) {
  char ss_buf[1024];
  rtc::SimpleStringBuilder ss(ss_buf);
  ss.AppendFormat("%s (0x%08X)", rtc::ToUtf8(error.ErrorMessage()).c_str(),
                  error.Error());
  return ss.str();
}

bool LoadAudiosesDll() {
  static const wchar_t* const kAudiosesDLL =
      L"%WINDIR%\\system32\\audioses.dll";
  wchar_t path[MAX_PATH] = {0};
  ExpandEnvironmentStringsW(kAudiosesDLL, path, arraysize(path));
  // RTC_DLOG(INFO) << rtc::ToUtf8(path);
  return (LoadLibraryExW(path, NULL, LOAD_WITH_ALTERED_SEARCH_PATH) != NULL);
}

ComPtr<IMMDeviceEnumerator> CreateDeviceEnumeratorInternal(
    bool allow_reinitialize) {
  ComPtr<IMMDeviceEnumerator> device_enumerator;
  _com_error error = ::CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL,
                                        CLSCTX_INPROC_SERVER,
                                        IID_PPV_ARGS(&device_enumerator));

  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "CoCreateInstance failed: " << ErrorToString(error);
  }

  if (error.Error() == CO_E_NOTINITIALIZED && allow_reinitialize) {
    RTC_LOG(LS_ERROR) << "CoCreateInstance failed with CO_E_NOTINITIALIZED";
    // We have seen crashes which indicates that this method can in fact
    // fail with CO_E_NOTINITIALIZED in combination with certain 3rd party
    // modules. Calling CoInitializeEx() is an attempt to resolve the reported
    // issues. See http://crbug.com/378465 for details.
    error = CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if (error.Error() != S_OK) {
      error = ::CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL,
                                 CLSCTX_INPROC_SERVER,
                                 IID_PPV_ARGS(&device_enumerator));
      if (error.Error() != S_OK) {
        RTC_LOG(LS_ERROR) << "CoCreateInstance failed: "
                          << ErrorToString(error);
      }
    }
  }
  return device_enumerator;
}

bool IsSupportedInternal() {
  // The Core Audio APIs are implemented in the user-mode system components
  // Audioses.dll and Mmdevapi.dll. Dependency Walker shows that it is
  // enough to verify possibility to load the Audioses DLL since it depends
  // on Mmdevapi.dll. See http://crbug.com/166397 why this extra step is
  // required to guarantee Core Audio support.
  if (!LoadAudiosesDll())
    return false;

  // Being able to load the Audioses.dll does not seem to be sufficient for
  // all devices to guarantee Core Audio support. To be 100%, we also verify
  // that it is possible to a create the IMMDeviceEnumerator interface. If
  // this works as well we should be home free.
  ComPtr<IMMDeviceEnumerator> device_enumerator =
      CreateDeviceEnumeratorInternal(false);
  if (!device_enumerator) {
    RTC_LOG(LS_ERROR)
        << "Failed to create Core Audio device enumerator on thread with ID "
        << rtc::CurrentThreadId();
    return false;
  }

  return true;
}

}  // namespace

bool CoreAudioUtility::IsSupported() {
  static bool g_is_supported = IsSupportedInternal();
  return g_is_supported;
}

}  // namespace win
}  // namespace webrtc
