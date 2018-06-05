/*
 *  Copyright 2004 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// RTC_LOG(...) an ostream target that can be used to send formatted
// output to a variety of logging targets, such as debugger console, stderr,
// or any LogSink.
// The severity level passed as the first argument to the logging
// functions is used as a filter, to limit the verbosity of the logging.
// Static members of LogMessage documented below are used to control the
// verbosity and target of the output.
// There are several variations on the RTC_LOG macro which facilitate logging
// of common error conditions, detailed below.

// RTC_LOG(sev) logs the given stream at severity "sev", which must be a
//     compile-time constant of the LoggingSeverity type, without the namespace
//     prefix.
// RTC_LOG_V(sev) Like RTC_LOG(), but sev is a run-time variable of the
//     LoggingSeverity type (basically, it just doesn't prepend the namespace).
// RTC_LOG_F(sev) Like RTC_LOG(), but includes the name of the current function.
// RTC_LOG_T(sev) Like RTC_LOG(), but includes the this pointer.
// RTC_LOG_T_F(sev) Like RTC_LOG_F(), but includes the this pointer.
// RTC_LOG_GLE(sev [, mod]) attempt to add a string description of the
//     HRESULT returned by GetLastError.
// RTC_LOG_ERRNO(sev) attempts to add a string description of an errno-derived
//     error. errno and associated facilities exist on both Windows and POSIX,
//     but on Windows they only apply to the C/C++ runtime.
// RTC_LOG_ERR(sev) is an alias for the platform's normal error system, i.e.
//     _GLE on Windows and _ERRNO on POSIX.
// (The above three also all have _EX versions that let you specify the error
// code, rather than using the last one.)
// RTC_LOG_E(sev, ctx, err, ...) logs a detailed error interpreted using the
//     specified context.
// RTC_LOG_CHECK_LEVEL(sev) (and RTC_LOG_CHECK_LEVEL_V(sev)) can be used as a
//     test before performing expensive or sensitive operations whose sole
//     purpose is to output logging data at the desired level.

#ifndef RTC_BASE_LOGGING_H_
#define RTC_BASE_LOGGING_H_

#include <errno.h>

#include <string>
#include <utility>

#if defined(WEBRTC_MAC) && !defined(WEBRTC_IOS)
#include <CoreServices/CoreServices.h>
#endif

#include "rtc_base/logging_internals.h"

#if !defined(NDEBUG) || defined(DLOG_ALWAYS_ON)
#define RTC_DLOG_IS_ON 1
#else
#define RTC_DLOG_IS_ON 0
#endif

namespace rtc {
//////////////////////////////////////////////////////////////////////
// Logging Helpers
//////////////////////////////////////////////////////////////////////

// DEPRECATED.
// TODO(bugs.webrtc.org/9278): Remove once there are no more users.
#define RTC_LOG_SEVERITY_PRECONDITION(sev) \
  !(rtc::LogMessage::Loggable(sev))        \
      ? static_cast<void>(0)               \
      : rtc::webrtc_logging_impl::LogMessageVoidify()&

#define RTC_LOG(sev)                                                   \
  for (bool do_log = rtc::LogMessage::Loggable<rtc::sev>(); do_log;    \
       do_log = false)                                                 \
  rtc::webrtc_logging_impl::LogCall() &                                \
      rtc::webrtc_logging_impl::LogStreamer<>()                        \
          << rtc::webrtc_logging_impl::LogMetadata(__FILE__, __LINE__, \
                                                   rtc::sev)

// The _V version is for when a variable is passed in.
#define RTC_LOG_V(sev)                                                       \
  for (bool do_log = rtc::LogMessage::Loggable(sev); do_log; do_log = false) \
  rtc::webrtc_logging_impl::LogCall() &                                      \
      rtc::webrtc_logging_impl::LogStreamer<>()                              \
          << rtc::webrtc_logging_impl::LogMetadata(__FILE__, __LINE__, sev)

// The _F version prefixes the message with the current function name.
#if (defined(__GNUC__) && !defined(NDEBUG)) || defined(WANT_PRETTY_LOG_F)
#define RTC_LOG_F(sev) RTC_LOG(sev) << __PRETTY_FUNCTION__ << ": "
#define RTC_LOG_T_F(sev) RTC_LOG(sev) << this << ": " \
  << __PRETTY_FUNCTION__ << ": "
#else
#define RTC_LOG_F(sev) RTC_LOG(sev) << __FUNCTION__ << ": "
#define RTC_LOG_T_F(sev) RTC_LOG(sev) << this << ": " << __FUNCTION__ << ": "
#endif

#define RTC_LOG_CHECK_LEVEL(sev) \
  rtc::LogCheckLevel(rtc::sev)
#define RTC_LOG_CHECK_LEVEL_V(sev) \
  rtc::LogCheckLevel(sev)

inline bool LogCheckLevel(LoggingSeverity sev) {
  return (LogMessage::GetMinLogSeverity() <= sev);
}

#define RTC_LOG_E(sev, ctx, err)                                    \
  for (bool do_log = rtc::LogMessage::Loggable<rtc::sev>(); do_log; \
       do_log = false)                                              \
    rtc::webrtc_logging_impl::LogCall() &                           \
        rtc::webrtc_logging_impl::LogStreamer<>()                   \
            << rtc::webrtc_logging_impl::LogMetadataErr {           \
      {__FILE__, __LINE__, rtc::sev}, rtc::ERRCTX_##ctx, (err)      \
    }

#define RTC_LOG_T(sev) RTC_LOG(sev) << this << ": "

#define RTC_LOG_ERRNO_EX(sev, err) \
  RTC_LOG_E(sev, ERRNO, err)
#define RTC_LOG_ERRNO(sev) \
  RTC_LOG_ERRNO_EX(sev, errno)

#if defined(WEBRTC_WIN)
#define RTC_LOG_GLE_EX(sev, err) \
  RTC_LOG_E(sev, HRESULT, err)
#define RTC_LOG_GLE(sev) RTC_LOG_GLE_EX(sev, static_cast<int>(GetLastError()))
#define RTC_LOG_ERR_EX(sev, err) \
  RTC_LOG_GLE_EX(sev, err)
#define RTC_LOG_ERR(sev) \
  RTC_LOG_GLE(sev)
#elif defined(__native_client__) && __native_client__
#define RTC_LOG_ERR_EX(sev, err) \
  RTC_LOG(sev)
#define RTC_LOG_ERR(sev) \
  RTC_LOG(sev)
#elif defined(WEBRTC_POSIX)
#define RTC_LOG_ERR_EX(sev, err) \
  RTC_LOG_ERRNO_EX(sev, err)
#define RTC_LOG_ERR(sev) \
  RTC_LOG_ERRNO(sev)
#endif  // WEBRTC_WIN

#ifdef WEBRTC_ANDROID

namespace webrtc_logging_impl {
// TODO(kwiberg): Replace these with absl::string_view.
inline const char* AdaptString(const char* str) { return str; }
inline const char* AdaptString(const std::string& str) { return str.c_str(); }
}  // namespace webrtc_logging_impl

#define RTC_LOG_TAG(sev, tag)                                                \
  for (bool do_log = rtc::LogMessage::Loggable(sev); do_log; do_log = false) \
    rtc::webrtc_logging_impl::LogCall() &                                    \
        rtc::webrtc_logging_impl::LogStreamer<>()                            \
            << rtc::webrtc_logging_impl::LogMetadataTag {                    \
      sev, rtc::webrtc_logging_impl::AdaptString(tag)                        \
    }

#else

// DEPRECATED. This macro is only intended for Android.
#define RTC_LOG_TAG(sev, tag) RTC_LOG_V(sev)

#endif

// The RTC_DLOG macros are equivalent to their RTC_LOG counterparts except that
// they only generate code in debug builds.
#if RTC_DLOG_IS_ON
#define RTC_DLOG(sev) RTC_LOG(sev)
#define RTC_DLOG_V(sev) RTC_LOG_V(sev)
#define RTC_DLOG_F(sev) RTC_LOG_F(sev)
#else
#define RTC_DLOG_EAT_STREAM_PARAMS() \
  while (false)                      \
  rtc::webrtc_logging_impl::LogStreamer<>()
#define RTC_DLOG(sev) RTC_DLOG_EAT_STREAM_PARAMS()
#define RTC_DLOG_V(sev) RTC_DLOG_EAT_STREAM_PARAMS()
#define RTC_DLOG_F(sev) RTC_DLOG_EAT_STREAM_PARAMS()
#endif

}  // namespace rtc

#endif  // RTC_BASE_LOGGING_H_
