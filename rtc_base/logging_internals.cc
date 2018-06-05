/*
 *  Copyright 2018 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#if defined(WEBRTC_WIN)
#include <windows.h>
#define LAST_SYSTEM_ERROR (::GetLastError())
#elif defined(__native_client__) && __native_client__
#define LAST_SYSTEM_ERROR (0)
#elif defined(WEBRTC_POSIX)
#include <errno.h>
#define LAST_SYSTEM_ERROR (errno)
#endif  // WEBRTC_WIN

#if defined(WEBRTC_ANDROID)
#define RTC_LOG_TAG_ANDROID "rtc"
#include <android/log.h>  // NOLINT
#endif

#include "rtc_base/logging_internals.h"

namespace rtc {
namespace webrtc_logging_impl {

void LogImpl(const LogArgType* fmt, va_list args) {
  LogMetadataErr meta;
  const char* tag = nullptr;
  const char* check_message = nullptr;

  switch (*fmt) {
    case LogArgType::kLogMetadata: {
      meta = {va_arg(args, LogMetadata), ERRCTX_NONE, 0};
      break;
    }
    case LogArgType::kLogMetadataErr: {
      meta = va_arg(args, LogMetadataErr);
      break;
    }
    case LogArgType::kCheckMetadata: {
      CheckMetadata cmd = va_arg(args, CheckMetadata);
      meta = {{cmd.file, cmd.line, LS_ERROR}, ERRCTX_NONE, 0};
      check_message = cmd.message;
      break;
    }
#ifdef WEBRTC_ANDROID
    case LogArgType::kLogMetadataTag: {
      const LogMetadataTag tag_meta = va_arg(args, LogMetadataTag);
      meta = {{nullptr, 0, tag_meta.severity}, ERRCTX_NONE, 0};
      tag = tag_meta.tag;
      break;
    }
#endif
    default: {
#ifdef DEBUG
      std::fprintf(stderr, "Open() must be called before Write.\n");
      abort();
#endif
      va_end(args);
      return;
    }
  }
  LogMessage log_message(meta.meta.File(), meta.meta.Line(),
                         meta.meta.Severity(), meta.err_ctx, meta.err);
  if (tag) {
    log_message.AddTag(tag);
  } else if (check_message) {
    log_message.stream() << "\n#\n#";
    const char* message = log_message.print_stream_.str().c_str();
#if defined(WEBRTC_ANDROID)
    __android_log_print(ANDROID_LOG_ERROR, RTC_LOG_TAG_ANDROID, "%s", message);
#else
    fprintf(stderr, "%s", message);
#endif
  }
}

RTC_NORETURN void FatalLog(const LogArgType* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  LogImpl(fmt, args);
  va_end(args);

  fflush(stdout);
  fflush(stderr);
  abort();
}

void Log(const LogArgType* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  LogImpl(fmt, args);
  va_end(args);
}

}  // namespace webrtc_logging_impl
}  // namespace rtc
