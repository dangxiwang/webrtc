/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_BUILD_RTC_EXPORT_H_
#define RTC_BASE_BUILD_RTC_EXPORT_H_

// RTC_EXPORT is used to mark symbols as exported or imported when WebRTC is
// built or used as a shared library.
// When WebRTC is built as a static library the RTC_EXPORT macro expands to
// nothing.

#ifdef COMPONENT_BUILD

#ifdef WEBRTC_WIN

#ifdef WEBRTC_LIBRARY_IMPL
#define RTC_EXPORT __declspec(dllexport)
#else
#define RTC_EXPORT __declspec(dllimport)
#endif

#else  // WEBRTC_WIN

#if __has_attribute(visibility) && defined(WEBRTC_LIBRARY_IMPL)
#define RTC_EXPORT __attribute__((visibility("default")))
#endif

#endif  // WEBRTC_WIN

#endif  // COMPONENT_BUILD

#ifndef RTC_EXPORT
#define RTC_EXPORT
#endif

#endif  // RTC_BASE_BUILD_RTC_EXPORT_H_
