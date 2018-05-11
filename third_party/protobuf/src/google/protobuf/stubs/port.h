// Protocol Buffers - Google's data interchange format
// Copyright 2008 Google Inc.  All rights reserved.
// https://developers.google.com/protocol-buffers/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef GOOGLE_PROTOBUF_STUBS_PORT_H_
#define GOOGLE_PROTOBUF_STUBS_PORT_H_

#include <assert.h>
#include <stdlib.h>
#include <cstddef>
#include <string>
#include <string.h>
#if defined(__osf__)
// Tru64 lacks stdint.h, but has inttypes.h which defines a superset of
// what stdint.h would define.
#include <inttypes.h>
#elif !defined(_MSC_VER)
#include <stdint.h>
#endif

#include <google/protobuf/stubs/platform_macros.h>

#undef PROTOBUF_LITTLE_ENDIAN
#ifdef _WIN32
  // Assuming windows is always little-endian.
  // TODO(xiaofeng): The PROTOBUF_LITTLE_ENDIAN is not only used for
  // optimization but also for correctness. We should define an
  // different macro to test the big-endian code path in coded_stream.
  #if !defined(PROTOBUF_DISABLE_LITTLE_ENDIAN_OPT_FOR_TEST)
    #define PROTOBUF_LITTLE_ENDIAN 1
  #endif
  #if _MSC_VER >= 1300 && !defined(__INTEL_COMPILER)
    // If MSVC has "/RTCc" set, it will complain about truncating casts at
    // runtime.  This file contains some intentional truncating casts.
    #pragma runtime_checks("c", off)
  #endif
#else
  #include <sys/param.h>   // __BYTE_ORDER
  #if defined(__OpenBSD__)
    #include <endian.h>
  #endif
  #if ((defined(__LITTLE_ENDIAN__) && !defined(__BIG_ENDIAN__)) || \
         (defined(__BYTE_ORDER) && __BYTE_ORDER == __LITTLE_ENDIAN) || \
         (defined(BYTE_ORDER) && BYTE_ORDER == LITTLE_ENDIAN)) && \
      !defined(PROTOBUF_DISABLE_LITTLE_ENDIAN_OPT_FOR_TEST)
    #define PROTOBUF_LITTLE_ENDIAN 1
  #endif
#endif

// The macros defined below are required in order to make protobuf_lite a
// component on all platforms. See http://crbug.com/172800.
#if defined(COMPONENT_BUILD) && defined(PROTOBUF_USE_DLLS)
  #if defined(_MSC_VER)
    #ifdef LIBPROTOBUF_EXPORTS
      #define LIBPROTOBUF_EXPORT __declspec(dllexport)
    #else
      #define LIBPROTOBUF_EXPORT __declspec(dllimport)
    #endif
    #ifdef LIBPROTOC_EXPORTS
      #define LIBPROTOC_EXPORT   __declspec(dllexport)
    #else
      #define LIBPROTOC_EXPORT   __declspec(dllimport)
    #endif
  #else  // defined(_MSC_VER)
    #ifdef LIBPROTOBUF_EXPORTS
      #define LIBPROTOBUF_EXPORT __attribute__((visibility("default")))
    #else
      #define LIBPROTOBUF_EXPORT
    #endif
    #ifdef LIBPROTOC_EXPORTS
      #define LIBPROTOC_EXPORT   __attribute__((visibility("default")))
    #else
      #define LIBPROTOC_EXPORT
    #endif
  #endif
#else  // defined(COMPONENT_BUILD) && defined(PROTOBUF_USE_DLLS)
  #define LIBPROTOBUF_EXPORT
  #define LIBPROTOC_EXPORT
#endif

// These #includes are for the byte swap functions declared later on.
#ifdef _MSC_VER
#include <stdlib.h>  // NOLINT(build/include)
#elif defined(__APPLE__)
#include <libkern/OSByteOrder.h>
#elif defined(__GLIBC__) || defined(__CYGWIN__)
#include <byteswap.h>  // IWYU pragma: export
#endif

// ===================================================================
// from google3/base/port.h

#if (defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L || \
     (defined(_MSC_VER) && _MSC_VER >= 1900))
// Define this to 1 if the code is compiled in C++11 mode; leave it
// undefined otherwise.  Do NOT define it to 0 -- that causes
// '#ifdef LANG_CXX11' to behave differently from '#if LANG_CXX11'.
#define LANG_CXX11 1
#endif

namespace google {
namespace protobuf {

typedef unsigned int uint;

#ifdef _MSC_VER
typedef signed __int8  int8;
typedef __int16 int16;
typedef __int32 int32;
typedef __int64 int64;

typedef unsigned __int8  uint8;
typedef unsigned __int16 uint16;
typedef unsigned __int32 uint32;
typedef unsigned __int64 uint64;
#else
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
#endif

// long long macros to be used because gcc and vc++ use different suffixes,
// and different size specifiers in format strings
#undef GOOGLE_LONGLONG
#undef GOOGLE_ULONGLONG
#undef GOOGLE_LL_FORMAT

#ifdef _MSC_VER
#define GOOGLE_LONGLONG(x) x##I64
#define GOOGLE_ULONGLONG(x) x##UI64
#define GOOGLE_LL_FORMAT "I64"  // As in printf("%I64d", ...)
#else
// By long long, we actually mean int64.
#define GOOGLE_LONGLONG(x) INT64_C(x)
#define GOOGLE_ULONGLONG(x) UINT64_C(x)
// Used to format real long long integers.
#define GOOGLE_LL_FORMAT "ll"  // As in "%lld". Note that "q" is poor form also.
#endif

static const int32 kint32max = 0x7FFFFFFF;
static const int32 kint32min = -kint32max - 1;
static const int64 kint64max = GOOGLE_LONGLONG(0x7FFFFFFFFFFFFFFF);
static const int64 kint64min = -kint64max - 1;
static const uint32 kuint32max = 0xFFFFFFFFu;
static const uint64 kuint64max = GOOGLE_ULONGLONG(0xFFFFFFFFFFFFFFFF);

// -------------------------------------------------------------------
// Annotations:  Some parts of the code have been annotated in ways that might
//   be useful to some compilers or tools, but are not supported universally.
//   You can #define these annotations yourself if the default implementation
//   is not right for you.

#ifndef GOOGLE_ATTRIBUTE_ALWAYS_INLINE
#if defined(__GNUC__) && (__GNUC__ > 3 ||(__GNUC__ == 3 && __GNUC_MINOR__ >= 1))
// For functions we want to force inline.
// Introduced in gcc 3.1.
#define GOOGLE_ATTRIBUTE_ALWAYS_INLINE __attribute__ ((always_inline))
#else
// Other compilers will have to figure it out for themselves.
#define GOOGLE_ATTRIBUTE_ALWAYS_INLINE
#endif
#endif

#ifndef GOOGLE_ATTRIBUTE_NOINLINE
#if defined(__GNUC__) && (__GNUC__ > 3 ||(__GNUC__ == 3 && __GNUC_MINOR__ >= 1))
// For functions we want to force not inline.
// Introduced in gcc 3.1.
#define GOOGLE_ATTRIBUTE_NOINLINE __attribute__ ((noinline))
#elif defined(_MSC_VER) && (_MSC_VER >= 1400)
// Seems to have been around since at least Visual Studio 2005
#define GOOGLE_ATTRIBUTE_NOINLINE __declspec(noinline)
#else
// Other compilers will have to figure it out for themselves.
#define GOOGLE_ATTRIBUTE_NOINLINE
#endif
#endif

#ifndef GOOGLE_ATTRIBUTE_NORETURN
#ifdef __GNUC__
// Tell the compiler that a given function never returns.
#define GOOGLE_ATTRIBUTE_NORETURN __attribute__((noreturn))
#else
#define GOOGLE_ATTRIBUTE_NORETURN
#endif
#endif

#ifndef GOOGLE_ATTRIBUTE_DEPRECATED
#ifdef __GNUC__
// If the method/variable/type is used anywhere, produce a warning.
#define GOOGLE_ATTRIBUTE_DEPRECATED __attribute__((deprecated))
#else
#define GOOGLE_ATTRIBUTE_DEPRECATED
#endif
#endif

#ifndef GOOGLE_PREDICT_TRUE
#ifdef __GNUC__
// Provided at least since GCC 3.0.
#define GOOGLE_PREDICT_TRUE(x) (__builtin_expect(!!(x), 1))
#else
#define GOOGLE_PREDICT_TRUE(x) (x)
#endif
#endif

#ifndef GOOGLE_PREDICT_FALSE
#ifdef __GNUC__
// Provided at least since GCC 3.0.
#define GOOGLE_PREDICT_FALSE(x) (__builtin_expect(x, 0))
#else
#define GOOGLE_PREDICT_FALSE(x) (x)
#endif
#endif

// Delimits a block of code which may write to memory which is simultaneously
// written by other threads, but which has been determined to be thread-safe
// (e.g. because it is an idempotent write).
#ifndef GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN
#define GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN()
#endif
#ifndef GOOGLE_SAFE_CONCURRENT_WRITES_END
#define GOOGLE_SAFE_CONCURRENT_WRITES_END()
#endif

#define GOOGLE_GUARDED_BY(x)
#define GOOGLE_ATTRIBUTE_COLD

#ifdef GOOGLE_PROTOBUF_DONT_USE_UNALIGNED
# define GOOGLE_PROTOBUF_USE_UNALIGNED 0
#else
// x86 and x86-64 can perform unaligned loads/stores directly.
# if defined(_M_X64) || defined(__x86_64__) || defined(_M_IX86) || \
     defined(__i386__)
#  define GOOGLE_PROTOBUF_USE_UNALIGNED 1
# else
#  define GOOGLE_PROTOBUF_USE_UNALIGNED 0
# endif
#endif

#if GOOGLE_PROTOBUF_USE_UNALIGNED

#define GOOGLE_UNALIGNED_LOAD16(_p) (*reinterpret_cast<const uint16 *>(_p))
#define GOOGLE_UNALIGNED_LOAD32(_p) (*reinterpret_cast<const uint32 *>(_p))
#define GOOGLE_UNALIGNED_LOAD64(_p) (*reinterpret_cast<const uint64 *>(_p))

#define GOOGLE_UNALIGNED_STORE16(_p, _val) (*reinterpret_cast<uint16 *>(_p) = (_val))
#define GOOGLE_UNALIGNED_STORE32(_p, _val) (*reinterpret_cast<uint32 *>(_p) = (_val))
#define GOOGLE_UNALIGNED_STORE64(_p, _val) (*reinterpret_cast<uint64 *>(_p) = (_val))

#else
inline uint16 GOOGLE_UNALIGNED_LOAD16(const void *p) {
  uint16 t;
  memcpy(&t, p, sizeof t);
  return t;
}

inline uint32 GOOGLE_UNALIGNED_LOAD32(const void *p) {
  uint32 t;
  memcpy(&t, p, sizeof t);
  return t;
}

inline uint64 GOOGLE_UNALIGNED_LOAD64(const void *p) {
  uint64 t;
  memcpy(&t, p, sizeof t);
  return t;
}

inline void GOOGLE_UNALIGNED_STORE16(void *p, uint16 v) {
  memcpy(p, &v, sizeof v);
}

inline void GOOGLE_UNALIGNED_STORE32(void *p, uint32 v) {
  memcpy(p, &v, sizeof v);
}

inline void GOOGLE_UNALIGNED_STORE64(void *p, uint64 v) {
  memcpy(p, &v, sizeof v);
}
#endif

#if defined(_MSC_VER)
#define GOOGLE_THREAD_LOCAL __declspec(thread)
#else
#define GOOGLE_THREAD_LOCAL __thread
#endif

// The following guarantees declaration of the byte swap functions.
#ifdef _MSC_VER
#define bswap_16(x) _byteswap_ushort(x)
#define bswap_32(x) _byteswap_ulong(x)
#define bswap_64(x) _byteswap_uint64(x)

#elif defined(__APPLE__)
// Mac OS X / Darwin features
#define bswap_16(x) OSSwapInt16(x)
#define bswap_32(x) OSSwapInt32(x)
#define bswap_64(x) OSSwapInt64(x)

#elif !defined(__GLIBC__) && !defined(__CYGWIN__)

static inline uint16 bswap_16(uint16 x) {
  return static_cast<uint16>(((x & 0xFF) << 8) | ((x & 0xFF00) >> 8));
}
#define bswap_16(x) bswap_16(x)
static inline uint32 bswap_32(uint32 x) {
  return (((x & 0xFF) << 24) |
          ((x & 0xFF00) << 8) |
          ((x & 0xFF0000) >> 8) |
          ((x & 0xFF000000) >> 24));
}
#define bswap_32(x) bswap_32(x)
static inline uint64 bswap_64(uint64 x) {
  return (((x & GOOGLE_ULONGLONG(0xFF)) << 56) |
          ((x & GOOGLE_ULONGLONG(0xFF00)) << 40) |
          ((x & GOOGLE_ULONGLONG(0xFF0000)) << 24) |
          ((x & GOOGLE_ULONGLONG(0xFF000000)) << 8) |
          ((x & GOOGLE_ULONGLONG(0xFF00000000)) >> 8) |
          ((x & GOOGLE_ULONGLONG(0xFF0000000000)) >> 24) |
          ((x & GOOGLE_ULONGLONG(0xFF000000000000)) >> 40) |
          ((x & GOOGLE_ULONGLONG(0xFF00000000000000)) >> 56));
}
#define bswap_64(x) bswap_64(x)

#endif

// ===================================================================
// from google3/util/bits/bits.h

class Bits {
 public:
  static uint32 Log2FloorNonZero(uint32 n) {
#if defined(__GNUC__)
  return 31 ^ static_cast<uint32>(__builtin_clz(n));
#elif defined(COMPILER_MSVC) && defined(_M_IX86)
  _asm {
    bsr ebx, n
    mov n, ebx
  }
  return n;
#else
  return Log2FloorNonZero_Portable(n);
#endif
  }

  static uint32 Log2FloorNonZero64(uint64 n) {
    // arm-nacl-clang runs into an instruction-selection failure when it
    // encounters __builtin_clzll:
    // https://bugs.chromium.org/p/nativeclient/issues/detail?id=4395
    // To work around this, when we build for NaCl we use the portable
    // implementation instead.
#if defined(__GNUC__) && !defined(GOOGLE_PROTOBUF_OS_NACL)
  return 63 ^ static_cast<uint32>(__builtin_clzll(n));
#else
  return Log2FloorNonZero64_Portable(n);
#endif
  }
 private:
  static int Log2FloorNonZero_Portable(uint32 n) {
    if (n == 0)
      return -1;
    int log = 0;
    uint32 value = n;
    for (int i = 4; i >= 0; --i) {
      int shift = (1 << i);
      uint32 x = value >> shift;
      if (x != 0) {
        value = x;
        log += shift;
      }
    }
    assert(value == 1);
    return log;
  }

  static int Log2FloorNonZero64_Portable(uint64 n) {
    const uint32 topbits = static_cast<uint32>(n >> 32);
    if (topbits == 0) {
      // Top bits are zero, so scan in bottom bits
      return static_cast<int>(Log2FloorNonZero(static_cast<uint32>(n)));
    } else {
      return 32 + static_cast<int>(Log2FloorNonZero(topbits));
    }
  }
};

// ===================================================================
// from google3/util/endian/endian.h
LIBPROTOBUF_EXPORT uint32 ghtonl(uint32 x);

class BigEndian {
 public:
#ifdef PROTOBUF_LITTLE_ENDIAN

  static uint16 FromHost16(uint16 x) { return bswap_16(x); }
  static uint16 ToHost16(uint16 x) { return bswap_16(x); }

  static uint32 FromHost32(uint32 x) { return bswap_32(x); }
  static uint32 ToHost32(uint32 x) { return bswap_32(x); }

  static uint64 FromHost64(uint64 x) { return bswap_64(x); }
  static uint64 ToHost64(uint64 x) { return bswap_64(x); }

  static bool IsLittleEndian() { return true; }

#else

  static uint16 FromHost16(uint16 x) { return x; }
  static uint16 ToHost16(uint16 x) { return x; }

  static uint32 FromHost32(uint32 x) { return x; }
  static uint32 ToHost32(uint32 x) { return x; }

  static uint64 FromHost64(uint64 x) { return x; }
  static uint64 ToHost64(uint64 x) { return x; }

  static bool IsLittleEndian() { return false; }

#endif /* ENDIAN */

  // Functions to do unaligned loads and stores in big-endian order.
  static uint16 Load16(const void *p) {
    return ToHost16(GOOGLE_UNALIGNED_LOAD16(p));
  }

  static void Store16(void *p, uint16 v) {
    GOOGLE_UNALIGNED_STORE16(p, FromHost16(v));
  }

  static uint32 Load32(const void *p) {
    return ToHost32(GOOGLE_UNALIGNED_LOAD32(p));
  }

  static void Store32(void *p, uint32 v) {
    GOOGLE_UNALIGNED_STORE32(p, FromHost32(v));
  }

  static uint64 Load64(const void *p) {
    return ToHost64(GOOGLE_UNALIGNED_LOAD64(p));
  }

  static void Store64(void *p, uint64 v) {
    GOOGLE_UNALIGNED_STORE64(p, FromHost64(v));
  }
};

}  // namespace protobuf
}  // namespace google

#endif  // GOOGLE_PROTOBUF_STUBS_PORT_H_
