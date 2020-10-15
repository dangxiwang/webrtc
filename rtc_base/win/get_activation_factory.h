/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_WIN_GET_ACTIVATION_FACTORY_H_
#define RTC_BASE_WIN_GET_ACTIVATION_FACTORY_H_

#include <winerror.h>

#include "rtc_base/system/rtc_export.h"
#include "rtc_base/win/hstring.h"

namespace rtc_base {
namespace win {

// Provides access to Core WinRT functions which may not be available on
// Windows 7. Loads functions dynamically at runtime to prevent library
// dependencies.

// Callers must check the return value of ResolveCoreWinRTDelayLoad() before
// using these functions.

RTC_EXPORT bool ResolveCoreWinRTDelayload();

RTC_EXPORT HRESULT RoGetActivationFactory(HSTRING class_id,
                                          const IID& iid,
                                          void** out_factory);

// Retrieves an activation factory for the type specified.
template <typename InterfaceType, wchar_t const* runtime_class_id>
HRESULT GetActivationFactory(InterfaceType** factory) {
  HSTRING class_id_hstring;
  HRESULT hr = rtc_base::win::CreateHstring(
      runtime_class_id, wcslen(runtime_class_id), &class_id_hstring);
  if (FAILED(hr))
    return hr;

  hr = rtc_base::win::RoGetActivationFactory(class_id_hstring,
                                             IID_PPV_ARGS(factory));
  if (FAILED(hr))
    return hr;

  return rtc_base::win::DeleteHstring(class_id_hstring);
}

}  // namespace win
}  // namespace rtc_base

#endif  // RTC_BASE_WIN_GET_ACTIVATION_FACTORY_H_
