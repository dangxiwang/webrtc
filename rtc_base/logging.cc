/*
 *  Copyright 2004 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#if defined(WEBRTC_WIN)
#if !defined(WIN32_LEAN_AND_MEAN)
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#if _MSC_VER < 1900
#define snprintf _snprintf
#endif
#undef ERROR  // wingdi.h
#endif

#if defined(WEBRTC_MAC) && !defined(WEBRTC_IOS)
#include <CoreServices/CoreServices.h>
#elif defined(WEBRTC_ANDROID)
#include <android/log.h>
// Android has a 1024 limit on log inputs. We use 60 chars as an
// approx for the header/tag portion.
// See android/system/core/liblog/logd_write.c
static const int kMaxLogLineSize = 1024 - 60;
#endif  // WEBRTC_MAC && !defined(WEBRTC_IOS) || WEBRTC_ANDROID

#include <limits.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <cstdarg>
#include <iomanip>
#include <mutex>
#include <ostream>
#include <vector>

#include "rtc_base/logging.h"
#include "rtc_base/platform_thread_types.h"

namespace rtc {
namespace {
// By default, release builds don't log, debug builds at info level
#if !defined(NDEBUG)
static LoggingSeverity g_min_sev = LS_INFO;
static LoggingSeverity g_dbg_sev = LS_INFO;
#else
static LoggingSeverity g_min_sev = LS_NONE;
static LoggingSeverity g_dbg_sev = LS_NONE;
#endif

// Return the filename portion of the string (that following the last slash).
const char* FilenameFromPath(const char* file) {
  const char* end1 = ::strrchr(file, '/');
  const char* end2 = ::strrchr(file, '\\');
  if (!end1 && !end2)
    return file;
  else
    return (end1 > end2) ? end1 + 1 : end2 + 1;
}

std::ostream& GetNoopStream() {
  class NoopStreamBuf : public std::streambuf {
   public:
    int overflow(int c) override { return c; }
  };
  static NoopStreamBuf noop_buffer;
  static std::ostream noop_stream(&noop_buffer);
  return noop_stream;
}

// Copied from timeutils.h. We want to log timestamps, but timeutils uses
// checks, which uses the logging backend. This version returns a zero timestamp
// instead of crasing on errors and overflows.
int64_t SystemTimeMillis() {
  static const int64_t kNumNanosecsPerMillisec = 1000000;
  static const int64_t kNumNanosecsPerSec = 1000000000;
  int64_t ticks;
#if defined(WEBRTC_MAC)
  static mach_timebase_info_data_t timebase;
  if (timebase.denom == 0) {
    // Get the timebase if this is the first time we run.
    // Recommended by Apple's QA1398.
    if (mach_timebase_info(&timebase) != KERN_SUCCESS) {
      return 0;
    }
  }
  // Use timebase to convert absolute time tick units into nanoseconds.
  uint64_t a = mach_absolute_time();
  uint32_t b = timebase.numer;
  if (b == 0 || a > std::numeric_limits<int64_t>::max() / b)
    return kError;

  ticks = mach_absolute_time() * timebase.numer / timebase.denom;
#elif defined(WEBRTC_POSIX)
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  ticks = kNumNanosecsPerSec * static_cast<int64_t>(ts.tv_sec) +
          static_cast<int64_t>(ts.tv_nsec);
#elif defined(WEBRTC_WIN)
  static volatile LONG last_timegettime = 0;
  static volatile int64_t num_wrap_timegettime = 0;
  volatile LONG* last_timegettime_ptr = &last_timegettime;
  DWORD now = timeGetTime();
  // Atomically update the last gotten time
  DWORD old = InterlockedExchange(last_timegettime_ptr, now);
  if (now < old) {
    // If now is earlier than old, there may have been a race between threads.
    // 0x0fffffff ~3.1 days, the code will not take that long to execute
    // so it must have been a wrap around.
    if (old > 0xf0000000 && now < 0x0fffffff) {
      num_wrap_timegettime++;
    }
  }
  ticks = now + (num_wrap_timegettime << 32);
  // TODO(deadbeef): Calculate with nanosecond precision. Otherwise, we're
  // just wasting a multiply and divide when doing Time() on Windows.
  ticks = ticks * kNumNanosecsPerMillisec;
#else
#error Unsupported platform.
#endif
  return static_cast<int64_t>(ticks / kNumNanosecsPerMillisec);
}

// Global lock for log subsystem, only needed to serialize access to streams_.
std::mutex g_log_crit;
}  // namespace

void LogSink::OnLogMessage(const std::string& msg,
                           LoggingSeverity severity,
                           const char* tag) {
  OnLogMessage(msg);
}

/////////////////////////////////////////////////////////////////////////////
// LogMessage
/////////////////////////////////////////////////////////////////////////////

bool LogMessage::log_to_stderr_ = true;

// The list of logging streams currently configured.
// Note: we explicitly do not clean this up, because of the uncertain ordering
// of destructors at program exit.  Let the person who sets the stream trigger
// cleanup by setting to null, or let it leak (safe at program exit).
// May only be accessed while g_log_crit is held.
LogMessage::StreamList LogMessage::streams_;

// Boolean options default to false (0)
bool LogMessage::thread_, LogMessage::timestamp_;

LogMessage::LogMessage(const char* file, int line, LoggingSeverity sev)
    : LogMessage(file, line, sev, ERRCTX_NONE, 0) {}

LogMessage::LogMessage(const char* file,
                       int line,
                       LoggingSeverity sev,
                       LogErrorContext err_ctx,
                       int err)
    : severity_(sev),
      error_context_(err_ctx),
      error_number_(err),
      is_noop_(IsNoop(sev)) {
  // If there's no need to do any work, let's not :)
  if (is_noop_)
    return;

  if (timestamp_) {
    // Use SystemTimeMillis so that even if tests use fake clocks, the timestamp
    // in log messages represents the real system time.
    int64_t time = SystemTimeMillis() - LogStartTime();
    print_stream_ << "[" << std::setfill('0') << std::setw(3) << (time / 1000)
                  << ":" << std::setw(3) << (time % 1000) << std::setfill(' ')
                  << "] ";
  }

  if (thread_) {
    PlatformThreadId id = CurrentThreadId();
    print_stream_ << "[" << std::dec << id << "] ";
  }

  if (file != nullptr) {
#if defined(WEBRTC_ANDROID)
    tag_ = FilenameFromPath(file);
    print_stream_ << "(line " << line << "): ";
#else
    print_stream_ << "(" << FilenameFromPath(file)  << ":" << line << "): ";
#endif
  }
}

#if defined(WEBRTC_ANDROID)
LogMessage::LogMessage(const char* file,
                       int line,
                       LoggingSeverity sev,
                       const char* tag)
    : LogMessage(file,
                 line,
                 sev,
                 ERRCTX_NONE,
                 0 /* err */) {
  if (!is_noop_) {
    tag_ = tag;
  }
}
#endif

// DEPRECATED. Currently only used by downstream projects that use
// implementation details of logging.h. Work is ongoing to remove those
// dependencies.
LogMessage::LogMessage(const char* file, int line, LoggingSeverity sev,
                       const std::string& tag)
    : LogMessage(file, line, sev) {
  if (!is_noop_)
    print_stream_ << tag << ": ";
}

LogMessage::~LogMessage() {
  if (is_noop_)
    return;

  FinishPrintStream();

  // TODO(tommi): Unfortunately |ostringstream::str()| always returns a copy
  // of the constructed string. This means that we always end up creating
  // two copies here (one owned by the stream, one by the return value of
  // |str()|). It would be nice to switch to something else.
  const std::string str = print_stream_.str();

  if (severity_ >= g_dbg_sev) {
#if defined(WEBRTC_ANDROID)
    OutputToDebug(str, severity_, tag_);
#else
    OutputToDebug(str, severity_);
#endif
  }

  std::lock_guard<std::mutex> lock(g_log_crit);
  for (auto& kv : streams_) {
    if (severity_ >= kv.second) {
#if defined(WEBRTC_ANDROID)
      kv.first->OnLogMessage(str, severity_, tag_);
#else
      kv.first->OnLogMessage(str);
#endif
    }
  }
}

void LogMessage::AddTag(const char* tag) {
#ifdef WEBRTC_ANDROID
  if (!is_noop_) {
    tag_ = tag;
  }
#endif
}

std::ostream& LogMessage::stream() {
  return is_noop_ ? GetNoopStream() : print_stream_;
}

bool LogMessage::Loggable(LoggingSeverity sev) {
  return sev >= g_min_sev;
}

int LogMessage::GetMinLogSeverity() {
  return g_min_sev;
}

LoggingSeverity LogMessage::GetLogToDebug() {
  return g_dbg_sev;
}
int64_t LogMessage::LogStartTime() {
  static const int64_t g_start = SystemTimeMillis();
  return g_start;
}

void LogMessage::LogThreads(bool on) {
  thread_ = on;
}

void LogMessage::LogTimestamps(bool on) {
  timestamp_ = on;
}

void LogMessage::LogToDebug(LoggingSeverity min_sev) {
  g_dbg_sev = min_sev;
  std::lock_guard<std::mutex> lock(g_log_crit);
  UpdateMinLogSeverity();
}

void LogMessage::SetLogToStderr(bool log_to_stderr) {
  log_to_stderr_ = log_to_stderr;
}

int LogMessage::GetLogToStream(LogSink* stream) {
  std::lock_guard<std::mutex> lock(g_log_crit);
  LoggingSeverity sev = LS_NONE;
  for (auto& kv : streams_) {
    if (!stream || stream == kv.first) {
      sev = std::min(sev, kv.second);
    }
  }
  return sev;
}

void LogMessage::AddLogToStream(LogSink* stream, LoggingSeverity min_sev) {
  std::lock_guard<std::mutex> lock(g_log_crit);
  streams_.push_back(std::make_pair(stream, min_sev));
  UpdateMinLogSeverity();
}

void LogMessage::RemoveLogToStream(LogSink* stream) {
  std::lock_guard<std::mutex> lock(g_log_crit);
  for (StreamList::iterator it = streams_.begin(); it != streams_.end(); ++it) {
    if (stream == it->first) {
      streams_.erase(it);
      break;
    }
  }
  UpdateMinLogSeverity();
}

void LogMessage::ConfigureLogging(const char* params) {
  LoggingSeverity current_level = LS_VERBOSE;
  LoggingSeverity debug_level = GetLogToDebug();

  std::string s(params);
  std::string token;
  size_t start = -1;
  size_t end = 0;

  while (end != std::string::npos) {
    start = end + 1;
    end = s.find_first_of(" ", start);
    token = s.substr(start, end - start);

    if (token.empty())
      continue;

    // Logging features
    if (token == "tstamp") {
      LogTimestamps();
    } else if (token == "thread") {
      LogThreads();

    // Logging levels
    } else if (token == "sensitive") {
      current_level = LS_SENSITIVE;
    } else if (token == "verbose") {
      current_level = LS_VERBOSE;
    } else if (token == "info") {
      current_level = LS_INFO;
    } else if (token == "warning") {
      current_level = LS_WARNING;
    } else if (token == "error") {
      current_level = LS_ERROR;
    } else if (token == "none") {
      current_level = LS_NONE;

    // Logging targets
    } else if (token == "debug") {
      debug_level = current_level;
    }
  }

#if defined(WEBRTC_WIN)
  if ((LS_NONE != debug_level) && !::IsDebuggerPresent()) {
    // First, attempt to attach to our parent's console... so if you invoke
    // from the command line, we'll see the output there.  Otherwise, create
    // our own console window.
    // Note: These methods fail if a console already exists, which is fine.
    if (!AttachConsole(ATTACH_PARENT_PROCESS))
      ::AllocConsole();
  }
#endif  // WEBRTC_WIN

  LogToDebug(debug_level);
}

// The caller must hold g_log_crit.
void LogMessage::UpdateMinLogSeverity() {
  LoggingSeverity min_sev = g_dbg_sev;
  for (auto& kv : streams_) {
    min_sev = std::min(g_dbg_sev, kv.second);
  }
  g_min_sev = min_sev;
}

#if defined(WEBRTC_ANDROID)
void LogMessage::OutputToDebug(const std::string& str,
                               LoggingSeverity severity,
                               const char* tag) {
#else
void LogMessage::OutputToDebug(const std::string& str,
                               LoggingSeverity severity) {
#endif
  bool log_to_stderr = log_to_stderr_;
#if defined(WEBRTC_MAC) && !defined(WEBRTC_IOS) && defined(NDEBUG)
  // On the Mac, all stderr output goes to the Console log and causes clutter.
  // So in opt builds, don't log to stderr unless the user specifically sets
  // a preference to do so.
  CFStringRef key = CFStringCreateWithCString(kCFAllocatorDefault,
                                              "logToStdErr",
                                              kCFStringEncodingUTF8);
  CFStringRef domain = CFBundleGetIdentifier(CFBundleGetMainBundle());
  if (key != nullptr && domain != nullptr) {
    Boolean exists_and_is_valid;
    Boolean should_log =
        CFPreferencesGetAppBooleanValue(key, domain, &exists_and_is_valid);
    // If the key doesn't exist or is invalid or is false, we will not log to
    // stderr.
    log_to_stderr = exists_and_is_valid && should_log;
  }
  if (key != nullptr) {
    CFRelease(key);
  }
#endif  // defined(WEBRTC_MAC) && !defined(WEBRTC_IOS) && defined(NDEBUG)

#if defined(WEBRTC_WIN)
  // Always log to the debugger.
  // Perhaps stderr should be controlled by a preference, as on Mac?
  OutputDebugStringA(str.c_str());
  if (log_to_stderr) {
    // This handles dynamically allocated consoles, too.
    if (HANDLE error_handle = ::GetStdHandle(STD_ERROR_HANDLE)) {
      log_to_stderr = false;
      DWORD written = 0;
      ::WriteFile(error_handle, str.data(), static_cast<DWORD>(str.size()),
                  &written, 0);
    }
  }
#endif  // WEBRTC_WIN

#if defined(WEBRTC_ANDROID)
  // Android's logging facility uses severity to log messages but we
  // need to map libjingle's severity levels to Android ones first.
  // Also write to stderr which maybe available to executable started
  // from the shell.
  int prio;
  switch (severity) {
    case LS_SENSITIVE:
      __android_log_write(ANDROID_LOG_INFO, tag, "SENSITIVE");
      if (log_to_stderr) {
        fprintf(stderr, "SENSITIVE");
        fflush(stderr);
      }
      return;
    case LS_VERBOSE:
      prio = ANDROID_LOG_VERBOSE;
      break;
    case LS_INFO:
      prio = ANDROID_LOG_INFO;
      break;
    case LS_WARNING:
      prio = ANDROID_LOG_WARN;
      break;
    case LS_ERROR:
      prio = ANDROID_LOG_ERROR;
      break;
    default:
      prio = ANDROID_LOG_UNKNOWN;
  }

  int size = str.size();
  int line = 0;
  int idx = 0;
  const int max_lines = size / kMaxLogLineSize + 1;
  if (max_lines == 1) {
    __android_log_print(prio, tag, "%.*s", size, str.c_str());
  } else {
    while (size > 0) {
      const int len = std::min(size, kMaxLogLineSize);
      // Use the size of the string in the format (str may have \0 in the
      // middle).
      __android_log_print(prio, tag, "[%d/%d] %.*s", line + 1, max_lines, len,
                          str.c_str() + idx);
      idx += len;
      size -= len;
      ++line;
    }
  }
#endif  // WEBRTC_ANDROID
  if (log_to_stderr) {
    fprintf(stderr, "%s", str.c_str());
    fflush(stderr);
  }
}

// static
bool LogMessage::IsNoop(LoggingSeverity severity) {
  if (severity >= g_dbg_sev)
    return false;

  // TODO(tommi): We're grabbing this lock for every LogMessage instance that
  // is going to be logged. This introduces unnecessary synchronization for
  // a feature that's mostly used for testing.
  std::lock_guard<std::mutex> lock(g_log_crit);
  return streams_.size() == 0;
}

void LogMessage::FinishPrintStream() {
  if (is_noop_)
    return;

  if (error_context_ != ERRCTX_NONE) {
    print_stream_ << " : error:" << error_number_;
    switch (error_context_) {
      case ERRCTX_ERRNO:
        print_stream_ << " " << strerror(error_number_);
        break;
#ifdef WEBRTC_WIN
      case ERRCTX_HRESULT: {
        char msgbuf[256];
        DWORD flags =
            FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS;
        if (DWORD len = FormatMessageA(
                flags, nullptr, err, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                msgbuf, sizeof(msgbuf) / sizeof(msgbuf[0]), nullptr)) {
          while ((len > 0) &&
                 isspace(static_cast<unsigned char>(msgbuf[len - 1]))) {
            msgbuf[--len] = 0;
          }
          print_stream_ << " " << msgbuf;
        }
        break;
      }
#endif  // WEBRTC_WIN
#if defined(WEBRTC_MAC) && !defined(WEBRTC_IOS)
      case ERRCTX_OSSTATUS: {
        std::string desc(DescriptionFromOSStatus(error_number_));
        print_stream_ << " " << (desc.empty() ? "Unknown error" : desc.c_str());
        break;
      }
#endif  // WEBRTC_MAC && !defined(WEBRTC_IOS)
      default:
        break;
    }
  }
  print_stream_ << std::endl;
}

namespace webrtc_logging_impl {

void LogImpl(const LogArgType* fmt, va_list args) {
  LogMetadataErr meta;
  const char* tag = nullptr;
  switch (*fmt) {
    case LogArgType::kLogMetadata: {
      meta = {va_arg(args, LogMetadata), ERRCTX_NONE, 0};
      break;
    }
    case LogArgType::kLogMetadataErr: {
      meta = va_arg(args, LogMetadataErr);
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
  }

  for (++fmt; *fmt != LogArgType::kEnd; ++fmt) {
    switch (*fmt) {
      case LogArgType::kInt:
        log_message.stream() << va_arg(args, int);
        break;
      case LogArgType::kLong:
        log_message.stream() << va_arg(args, long);
        break;
      case LogArgType::kLongLong:
        log_message.stream() << va_arg(args, long long);
        break;
      case LogArgType::kUInt:
        log_message.stream() << va_arg(args, unsigned);
        break;
      case LogArgType::kULong:
        log_message.stream() << va_arg(args, unsigned long);
        break;
      case LogArgType::kULongLong:
        log_message.stream() << va_arg(args, unsigned long long);
        break;
      case LogArgType::kDouble:
        log_message.stream() << va_arg(args, double);
        break;
      case LogArgType::kLongDouble:
        log_message.stream() << va_arg(args, long double);
        break;
      case LogArgType::kCharP:
        log_message.stream() << va_arg(args, const char*);
        break;
      case LogArgType::kStdString:
        log_message.stream() << *va_arg(args, const std::string*);
        break;
      case LogArgType::kVoidP:
        log_message.stream() << va_arg(args, const void*);
        break;
      default:
#ifdef DEBUG
        std::fprintf(stderr, "Open() must be called before Write.\n");
        abort();
#endif
        va_end(args);
        return;
    }
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
