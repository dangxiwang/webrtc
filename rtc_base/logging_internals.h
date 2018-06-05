/*
 *  Copyright 2018 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef RTC_BASE_LOGGING_INTERNALS_H_
#define RTC_BASE_LOGGING_INTERNALS_H_

#include <list>
#include <sstream>  // no-presubmit-check TODO(webrtc:8982)
#include <string>
#include <utility>

#include "rtc_base/constructormagic.h"
#include "rtc_base/deprecation.h"
#include "rtc_base/no_return.h"
#include "rtc_base/system/inline.h"

namespace rtc {
//////////////////////////////////////////////////////////////////////
// Note that the non-standard LoggingSeverity aliases exist because they are
// still in broad use.  The meanings of the levels are:
//  LS_SENSITIVE: Information which should only be logged with the consent
//   of the user, due to privacy concerns.
//  LS_VERBOSE: This level is for data which we do not want to appear in the
//   normal debug log, but should appear in diagnostic logs.
//  LS_INFO: Chatty level used in debugging for all sorts of things, the default
//   in debug builds.
//  LS_WARNING: Something that may warrant investigation.
//  LS_ERROR: Something that should not have occurred.
//  LS_NONE: Don't log.
enum LoggingSeverity {
  LS_SENSITIVE,
  LS_VERBOSE,
  LS_INFO,
  LS_WARNING,
  LS_ERROR,
  LS_NONE,
  INFO = LS_INFO,
  WARNING = LS_WARNING,
  LERROR = LS_ERROR
};

// LogErrorContext assists in interpreting the meaning of an error value.
enum LogErrorContext {
  ERRCTX_NONE,
  ERRCTX_ERRNO,     // System-local errno
  ERRCTX_HRESULT,   // Windows HRESULT
  ERRCTX_OSSTATUS,  // MacOS OSStatus

  // Abbreviations for LOG_E macro
  ERRCTX_EN = ERRCTX_ERRNO,     // LOG_E(sev, EN, x)
  ERRCTX_HR = ERRCTX_HRESULT,   // LOG_E(sev, HR, x)
  ERRCTX_OS = ERRCTX_OSSTATUS,  // LOG_E(sev, OS, x)
};

#if defined(WEBRTC_MAC) && !defined(WEBRTC_IOS)
// Returns a UTF8 description from an OS X Status error.
std::string DescriptionFromOSStatus(OSStatus err);
#endif

// Virtual sink interface that can receive log messages.
class LogSink {
 public:
  LogSink() {}
  virtual ~LogSink() {}
  virtual void OnLogMessage(const std::string& message) = 0;
};

// Direct use of this class is deprecated; please use the logging macros
// instead.
// TODO(bugs.webrtc.org/9278): Move this class to an unnamed namespace in the
// .cc file.
class LogMessage {
 public:
  LogMessage(const char* file, int line, LoggingSeverity sev);

  // Same as the above, but using a compile-time constant for the logging
  // severity. This saves space at the call site, since passing an empty struct
  // is generally the same as not passing an argument at all.
  template <LoggingSeverity S>
  RTC_NO_INLINE LogMessage(const char* file,
                           int line,
                           std::integral_constant<LoggingSeverity, S>)
      : LogMessage(file, line, S) {}

  LogMessage(const char* file,
             int line,
             LoggingSeverity sev,
             LogErrorContext err_ctx,
             int err);

#if defined(WEBRTC_ANDROID)
  LogMessage(const char* file, int line, LoggingSeverity sev, const char* tag);
#endif

  // DEPRECATED - DO NOT USE - PLEASE USE THE MACROS INSTEAD OF THE CLASS.
  // Android code should use the 'const char*' version since tags are static
  // and we want to avoid allocating a std::string copy per log line.
  RTC_DEPRECATED
  LogMessage(const char* file,
             int line,
             LoggingSeverity sev,
             const std::string& tag);

  ~LogMessage();

  void AddTag(const char* tag);

  static bool Loggable(LoggingSeverity sev);

  // Same as the above, but using a template argument instead of a function
  // argument. (When the logging severity is statically known, passing it as a
  // template argument instead of as a function argument saves space at the
  // call site.)
  template <LoggingSeverity S>
  RTC_NO_INLINE static bool Loggable() {
    return Loggable(S);
  }

  std::ostream& stream();  // no-presubmit-check TODO(webrtc:8982)

  // Returns the time at which this function was called for the first time.
  // The time will be used as the logging start time.
  // If this is not called externally, the LogMessage ctor also calls it, in
  // which case the logging start time will be the time of the first LogMessage
  // instance is created.
  static int64_t LogStartTime();

  //  LogThreads: Display the thread identifier of the current thread
  static void LogThreads(bool on = true);

  //  LogTimestamps: Display the elapsed time of the program
  static void LogTimestamps(bool on = true);

  // These are the available logging channels
  //  Debug: Debug console on Windows, otherwise stderr
  static void LogToDebug(LoggingSeverity min_sev);
  static LoggingSeverity GetLogToDebug();

  // Sets whether logs will be directed to stderr in debug mode.
  static void SetLogToStderr(bool log_to_stderr);

  //  Stream: Any non-blocking stream interface.  LogMessage takes ownership of
  //   the stream. Multiple streams may be specified by using AddLogToStream.
  //   LogToStream is retained for backwards compatibility; when invoked, it
  //   will discard any previously set streams and install the specified stream.
  //   GetLogToStream gets the severity for the specified stream, of if none
  //   is specified, the minimum stream severity.
  //   RemoveLogToStream removes the specified stream, without destroying it.
  static int GetLogToStream(LogSink* stream = nullptr);
  static void AddLogToStream(LogSink* stream, LoggingSeverity min_sev);
  static void RemoveLogToStream(LogSink* stream);

  // Testing against MinLogSeverity allows code to avoid potentially expensive
  // logging operations by pre-checking the logging level.
  static int GetMinLogSeverity();

  // Parses the provided parameter stream to configure the options above.
  // Useful for configuring logging from the command line.
  static void ConfigureLogging(const char* params);

  // The ostream that buffers the formatted message before output
  // Pretend this is private, it's not because LogImpl uses it.
  std::ostringstream print_stream_;  // no-presubmit-check TODO(webrtc:8982)

 private:
  friend class LogMessageForTesting;
  typedef std::pair<LogSink*, LoggingSeverity> StreamAndSeverity;
  typedef std::list<StreamAndSeverity> StreamList;

  // Updates min_sev_ appropriately when debug sinks change.
  static void UpdateMinLogSeverity();

// These write out the actual log messages.
#if defined(WEBRTC_ANDROID)
  static void OutputToDebug(const std::string& msg,
                            LoggingSeverity severity,
                            const char* tag);
#else
  static void OutputToDebug(const std::string& msg, LoggingSeverity severity);
#endif

  // Checks the current global debug severity and if the |streams_| collection
  // is empty. If |severity| is smaller than the global severity and if the
  // |streams_| collection is empty, the LogMessage will be considered a noop
  // LogMessage.
  static bool IsNoop(LoggingSeverity severity);

  // Called from the dtor (or from a test) to append optional extra error
  // information to the log stream and a newline character.
  void FinishPrintStream();

  // The severity level of this message
  LoggingSeverity severity_;

#if defined(WEBRTC_ANDROID)
  // The Android debug output tag.
  const char* tag_ = "libjingle";
#endif

  const LogErrorContext error_context_;
  const int error_number_;

  const bool is_noop_;

  // The output streams and their associated severities
  static StreamList streams_;

  // Flags for formatting options
  static bool thread_, timestamp_;

  // Determines if logs will be directed to stderr in debug mode.
  static bool log_to_stderr_;

  RTC_DISALLOW_COPY_AND_ASSIGN(LogMessage);
};

namespace webrtc_logging_impl {

class LogMetadata {
 public:
  LogMetadata(const char* file, int line, LoggingSeverity severity)
      : file_(file),
        line_and_sev_(static_cast<uint32_t>(line) << 3 | severity) {}
  LogMetadata() = default;

  const char* File() const { return file_; }
  int Line() const { return line_and_sev_ >> 3; }
  LoggingSeverity Severity() const {
    return static_cast<LoggingSeverity>(line_and_sev_ & 0x7);
  }

 private:
  const char* file_;

  // Line number and severity, the former in the most significant 29 bits, the
  // latter in the least significant 3 bits. (This is an optimization; since
  // both numbers are usually compile-time constants, this way we can load them
  // both with a single instruction.)
  uint32_t line_and_sev_;
};
static_assert(std::is_trivial<LogMetadata>::value, "");

struct LogMetadataErr {
  LogMetadata meta;
  LogErrorContext err_ctx;
  int err;
};

struct CheckMetadata {
  const char* file;
  int line;
  const char* message;
};

static_assert(std::is_trivial<CheckMetadata>::value, "");

#ifdef WEBRTC_ANDROID
struct LogMetadataTag {
  LoggingSeverity severity;
  const char* tag;
};
#endif

enum class LogArgType : int8_t {
  kEnd = 0,
  kInt,
  kLong,
  kLongLong,
  kUInt,
  kULong,
  kULongLong,
  kDouble,
  kLongDouble,
  kCharP,
  kStdString,
  // TODO(kwiberg): Add absl::StringView.
  kVoidP,
  kLogMetadata,
  kLogMetadataErr,
  kCheckMetadata,
#ifdef WEBRTC_ANDROID
  kLogMetadataTag,
#endif
};

// Wrapper for log arguments. Only ever make values of this type with the
// MakeVal() functions.
template <LogArgType N, typename T>
struct Val {
  static constexpr LogArgType Type() { return N; }
  T GetVal() const { return val; }
  T val;
};

// TODO(bugs.webrtc.org/9278): Get rid of this specialization when callers
// don't need it anymore. No in-tree caller does, but some external callers
// still do.
template <>
struct Val<LogArgType::kStdString, std::string> {
  static constexpr LogArgType Type() { return LogArgType::kStdString; }
  const std::string* GetVal() const { return &val; }
  std::string val;
};

inline Val<LogArgType::kInt, int> MakeVal(int x) {
  return {x};
}
inline Val<LogArgType::kLong, long> MakeVal(long x) {
  return {x};
}
inline Val<LogArgType::kLongLong, long long> MakeVal(long long x) {
  return {x};
}
inline Val<LogArgType::kUInt, unsigned int> MakeVal(unsigned int x) {
  return {x};
}
inline Val<LogArgType::kULong, unsigned long> MakeVal(unsigned long x) {
  return {x};
}
inline Val<LogArgType::kULongLong, unsigned long long> MakeVal(
    unsigned long long x) {
  return {x};
}

inline Val<LogArgType::kDouble, double> MakeVal(double x) {
  return {x};
}
inline Val<LogArgType::kLongDouble, long double> MakeVal(long double x) {
  return {x};
}

inline Val<LogArgType::kCharP, const char*> MakeVal(const char* x) {
  return {x};
}
inline Val<LogArgType::kStdString, const std::string*> MakeVal(
    const std::string& x) {
  return {&x};
}
// TODO(kwiberg): Add absl::string_view

inline Val<LogArgType::kVoidP, const void*> MakeVal(const void* x) {
  return {x};
}

inline Val<LogArgType::kLogMetadata, LogMetadata> MakeVal(
    const LogMetadata& x) {
  return {x};
}
inline Val<LogArgType::kLogMetadataErr, LogMetadataErr> MakeVal(
    const LogMetadataErr& x) {
  return {x};
}
inline Val<LogArgType::kCheckMetadata, CheckMetadata> MakeVal(
    const CheckMetadata& x) {
  return {x};
}

#ifdef WEBRTC_ANDROID
inline Val<LogArgType::kLogMetadataTag, LogMetadataTag> MakeVal(
    const LogMetadataTag& x) {
  return {x};
}
#endif

// Handle arbitrary types other than the above by falling back to stringstream.
// TODO(bugs.webrtc.org/9278): Get rid of this overload when callers don't need
// it anymore. No in-tree caller does, but some external callers still do.
template <
    typename T,
    typename T1 =
        typename std::remove_cv<typename std::remove_reference<T>::type>::type,
    typename std::enable_if<
        std::is_class<T1>::value && !std::is_same<T1, std::string>::value &&
        !std::is_same<T1, LogMetadata>::value &&
#ifdef WEBRTC_ANDROID
        !std::is_same<T1, LogMetadataTag>::value &&
#endif
        !std::is_same<T1, LogMetadataErr>::value>::type* = nullptr>
Val<LogArgType::kStdString, std::string> MakeVal(const T& x) {
  std::ostringstream os;  // no-presubmit-check TODO(webrtc:8982)
  os << x;
  return {os.str()};
}

void Log(const LogArgType* fmt, ...);
RTC_NORETURN void FatalLog(const LogArgType* fmt, ...);

// Ephemeral type that represents the result of the logging << operator.
template <typename... Ts>
class LogStreamer;

// Base case: Before the first << argument.
template <>
class LogStreamer<> final {
 public:
  template <
      typename U,
      typename std::enable_if<std::is_arithmetic<U>::value>::type* = nullptr>
  RTC_FORCE_INLINE LogStreamer<decltype(MakeVal(std::declval<U>()))> operator<<(
      U arg) const {
    return LogStreamer<decltype(MakeVal(std::declval<U>()))>(MakeVal(arg),
                                                             this);
  }

  template <
      typename U,
      typename std::enable_if<!std::is_arithmetic<U>::value>::type* = nullptr>
  RTC_FORCE_INLINE LogStreamer<decltype(MakeVal(std::declval<U>()))> operator<<(
      const U& arg) const {
    return LogStreamer<decltype(MakeVal(std::declval<U>()))>(MakeVal(arg),
                                                             this);
  }

  template <typename... Us>
  RTC_FORCE_INLINE static void Call(const Us&... args) {
    static constexpr LogArgType t[] = {Us::Type()..., LogArgType::kEnd};
    Log(t, args.GetVal()...);
  }

  template <typename... Us>
  RTC_NORETURN RTC_FORCE_INLINE static void FatalCall(const Us&... args) {
    static constexpr LogArgType t[] = {Us::Type()..., LogArgType::kEnd};
    FatalLog(t, args.GetVal()...);
  }
};

// Inductive case: We've already seen at least one << argument. The most recent
// one had type `T`, and the earlier ones had types `Ts`.
template <typename T, typename... Ts>
class LogStreamer<T, Ts...> final {
 public:
  RTC_FORCE_INLINE LogStreamer(T arg, const LogStreamer<Ts...>* prior)
      : arg_(arg), prior_(prior) {}

  template <
      typename U,
      typename std::enable_if<std::is_arithmetic<U>::value>::type* = nullptr>
  RTC_FORCE_INLINE LogStreamer<decltype(MakeVal(std::declval<U>())), T, Ts...>
  operator<<(U arg) const {
    return LogStreamer<decltype(MakeVal(std::declval<U>())), T, Ts...>(
        MakeVal(arg), this);
  }

  template <
      typename U,
      typename std::enable_if<!std::is_arithmetic<U>::value>::type* = nullptr>
  RTC_FORCE_INLINE LogStreamer<decltype(MakeVal(std::declval<U>())), T, Ts...>
  operator<<(const U& arg) const {
    return LogStreamer<decltype(MakeVal(std::declval<U>())), T, Ts...>(
        MakeVal(arg), this);
  }

  template <typename... Us>
  RTC_FORCE_INLINE void Call(const Us&... args) const {
    prior_->Call(arg_, args...);
  }

  template <typename... Us>
  RTC_NORETURN RTC_FORCE_INLINE void FatalCall(const Us&... args) const {
    prior_->FatalCall(arg_, args...);
  }

 private:
  // The most recent argument.
  T arg_;

  // Earlier arguments.
  const LogStreamer<Ts...>* prior_;
};

class LogCall final {
 public:
  // This can be any binary operator with precedence lower than <<.
  template <typename... Ts>
  RTC_FORCE_INLINE void operator&(const LogStreamer<Ts...>& streamer) {
    streamer.Call();
  }
};

class FatalLogCall final {
 public:
  // This can be any binary operator with precedence lower than <<.
  template <typename... Ts>
  RTC_NORETURN RTC_FORCE_INLINE void operator&(
      const LogStreamer<Ts...>& streamer) {
    streamer.FatalCall();
  }
};

// TODO(bugs.webrtc.org/9278): Remove this once it's no longer used.
struct LogMessageVoidify {
  void operator&(std::ostream&) {}  // no-presubmit-check TODO(webrtc:8982)
};

}  // namespace webrtc_logging_impl
}  // namespace rtc

#endif  // RTC_BASE_LOGGING_INTERNALS_H_
