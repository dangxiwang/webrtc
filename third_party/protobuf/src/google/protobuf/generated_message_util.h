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

// Author: kenton@google.com (Kenton Varda)
//  Based on original Protocol Buffers design by
//  Sanjay Ghemawat, Jeff Dean, and others.
//
// This file contains miscellaneous helper code used by generated code --
// including lite types -- but which should not be used directly by users.

#ifndef GOOGLE_PROTOBUF_GENERATED_MESSAGE_UTIL_H__
#define GOOGLE_PROTOBUF_GENERATED_MESSAGE_UTIL_H__

#include <assert.h>
#include <climits>
#include <string>

#include <google/protobuf/stubs/logging.h>
#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/has_bits.h>

#ifndef PROTOBUF_FINAL
#if LANG_CXX11 && !defined(__NVCC__)
#define PROTOBUF_FINAL final
#else
#define PROTOBUF_FINAL
#endif
#endif  // !PROTOBUF_FINAL

namespace google {

namespace protobuf {

class Arena;
namespace io { class CodedInputStream; }

namespace internal {


// Annotation for the compiler to emit a deprecation message if a field marked
// with option 'deprecated=true' is used in the code, or for other things in
// generated code which are deprecated.
//
// For internal use in the pb.cc files, deprecation warnings are suppressed
// there.
#undef DEPRECATED_PROTOBUF_FIELD
#define PROTOBUF_DEPRECATED

#define GOOGLE_PROTOBUF_DEPRECATED_ATTR


// Returns the offset of the given field within the given aggregate type.
// This is equivalent to the ANSI C offsetof() macro.  However, according
// to the C++ standard, offsetof() only works on POD types, and GCC
// enforces this requirement with a warning.  In practice, this rule is
// unnecessarily strict; there is probably no compiler or platform on
// which the offsets of the direct fields of a class are non-constant.
// Fields inherited from superclasses *can* have non-constant offsets,
// but that's not what this macro will be used for.
#if defined(__clang__)
// For Clang we use __builtin_offsetof() and suppress the warning,
// to avoid Control Flow Integrity and UBSan vptr sanitizers from
// crashing while trying to validate the invalid reinterpet_casts.
#define GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TYPE, FIELD)  \
  _Pragma("clang diagnostic push")                                   \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"")         \
  __builtin_offsetof(TYPE, FIELD)                                    \
  _Pragma("clang diagnostic pop")
#else
// Note that we calculate relative to the pointer value 16 here since if we
// just use zero, GCC complains about dereferencing a NULL pointer.  We
// choose 16 rather than some other number just in case the compiler would
// be confused by an unaligned pointer.
#define GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TYPE, FIELD)  \
  static_cast< ::google::protobuf::uint32>(                           \
      reinterpret_cast<const char*>(                                 \
          &reinterpret_cast<const TYPE*>(16)->FIELD) -               \
      reinterpret_cast<const char*>(16))
#endif

#define GOOGLE_PROTOBUF_GENERATED_DEFAULT_ONEOF_FIELD_OFFSET(ONEOF, FIELD)  \
  static_cast< ::google::protobuf::uint32>(                                                    \
      reinterpret_cast<const char*>(&(ONEOF->FIELD))                        \
      - reinterpret_cast<const char*>(ONEOF))
// TODO(acozzette): remove this transitional macro after updating generated code
#define PROTO2_GENERATED_DEFAULT_ONEOF_FIELD_OFFSET(ONEOF, FIELD) \
  GOOGLE_PROTOBUF_GENERATED_DEFAULT_ONEOF_FIELD_OFFSET(ONEOF, FIELD)

// Constants for special floating point values.
LIBPROTOBUF_EXPORT double Infinity();
LIBPROTOBUF_EXPORT double NaN();

// This type is used to define a global variable, without it's constructor
// and destructor run on start and end of the program lifetime. This circumvents
// the initial construction order fiasco, while keeping the address of the
// empty string a compile time constant.
template <typename T>
class ExplicitlyConstructed {
 public:
  void DefaultConstruct() {
    new (&union_) T();
    init_ = true;
  }

  bool IsInitialized() { return init_; }
  void Shutdown() {
    if (init_) {
      init_ = false;
      get_mutable()->~T();
    }
  }

#if LANG_CXX11
  constexpr
#endif
      const T&
      get() const {
    return reinterpret_cast<const T&>(union_);
  }
  T* get_mutable() { return reinterpret_cast<T*>(&union_); }

 private:
  // Prefer c++14 aligned_storage, but for compatibility this will do.
  union AlignedUnion {
    char space[sizeof(T)];
    int64 align_to_int64;
    void* align_to_ptr;
  } union_;
  bool init_;  // false by linker
};

// TODO(jieluo): Change to template. We have tried to use template,
// but it causes net/rpc/python:rpcutil_test fail (the empty string will
// init twice). It may related to swig. Change to template after we
// found the solution.

// Default empty string object. Don't use this directly. Instead, call
// GetEmptyString() to get the reference.
LIBPROTOBUF_EXPORT extern ExplicitlyConstructed< ::std::string> fixed_address_empty_string;
LIBPROTOBUF_EXPORT extern ProtobufOnceType empty_string_once_init_;
LIBPROTOBUF_EXPORT void InitEmptyString();


LIBPROTOBUF_EXPORT inline const ::std::string& GetEmptyStringAlreadyInited() {
  return fixed_address_empty_string.get();
}


LIBPROTOBUF_EXPORT const ::std::string& GetEmptyString();

LIBPROTOBUF_EXPORT size_t StringSpaceUsedExcludingSelfLong(const string& str);


// True if IsInitialized() is true for all elements of t.  Type is expected
// to be a RepeatedPtrField<some message type>.  It's useful to have this
// helper here to keep the protobuf compiler from ever having to emit loops in
// IsInitialized() methods.  We want the C++ compiler to inline this or not
// as it sees fit.
template <class Type> bool AllAreInitialized(const Type& t) {
  for (int i = t.size(); --i >= 0; ) {
    if (!t.Get(i).IsInitialized()) return false;
  }
  return true;
}

LIBPROTOBUF_EXPORT void InitProtobufDefaults();

// We compute sizes as size_t but cache them as int.  This function converts a
// computed size to a cached size.  Since we don't proceed with serialization if
// the total size was > INT_MAX, it is not important what this function returns
// for inputs > INT_MAX.  However this case should not error or GOOGLE_CHECK-fail,
// because the full size_t resolution is still returned from ByteSizeLong() and
// checked against INT_MAX; we can catch the overflow there.
inline int ToCachedSize(size_t size) {
  return static_cast<int>(size);
}

// For cases where a legacy function returns an integer size.  We GOOGLE_DCHECK() that
// the conversion will fit within an integer; if this is false then we are
// losing information.
inline int ToIntSize(size_t size) {
  GOOGLE_DCHECK_LE(size, static_cast<size_t>(INT_MAX));
  return static_cast<int>(size);
}

// We mainly calculate sizes in terms of size_t, but some functions that compute
// sizes return "int".  These int sizes are expected to always be positive.
// This function is more efficient than casting an int to size_t directly on
// 64-bit platforms because it avoids making the compiler emit a sign extending
// instruction, which we don't want and don't want to pay for.
inline size_t FromIntSize(int size) {
  // Convert to unsigned before widening so sign extension is not necessary.
  return static_cast<unsigned int>(size);
}

}  // namespace internal
}  // namespace protobuf

}  // namespace google
#endif  // GOOGLE_PROTOBUF_GENERATED_MESSAGE_UTIL_H__
