/*
 *  Copyright 2022 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_DESKTOP_CAPTURE_LINUX_WAYLAND_SCOPED_GLIB_H_
#define MODULES_DESKTOP_CAPTURE_LINUX_WAYLAND_SCOPED_GLIB_H_

#include <gio/gio.h>

#include "rtc_base/checks.h"

namespace webrtc {

template <class T>
class Scoped {
 public:
  Scoped() {}
  explicit Scoped(T* val) { ptr_ = val; }
  ~Scoped() { RTC_DCHECK_NOTREACHED(); }

  T* operator->() const { return ptr_; }

  explicit operator bool() const { return ptr_ != nullptr; }

  bool operator!() const { return ptr_ == nullptr; }

  T* get() const { return ptr_; }

  T** receive() {
    RTC_CHECK(!ptr_);
    return &ptr_;
  }

  Scoped& operator=(T* val) {
    RTC_DCHECK(val);
    ptr_ = val;
    return *this;
  }

 protected:
  T* ptr_ = nullptr;
};

template <>
inline Scoped<GError>::~Scoped() {
  if (ptr_) {
    g_error_free(ptr_);
  }
}

template <>
inline Scoped<char>::~Scoped() {
  if (ptr_) {
    g_free(ptr_);
  }
}

template <>
inline Scoped<GVariant>::~Scoped() {
  if (ptr_) {
    g_variant_unref(ptr_);
  }
}

template <>
inline Scoped<GVariantIter>::~Scoped() {
  if (ptr_) {
    g_variant_iter_free(ptr_);
  }
}

template <>
inline Scoped<GDBusMessage>::~Scoped() {
  if (ptr_) {
    g_object_unref(ptr_);
  }
}

template <>
inline Scoped<GUnixFDList>::~Scoped() {
  if (ptr_) {
    g_object_unref(ptr_);
  }
}

}  // namespace webrtc

#endif  // MODULES_DESKTOP_CAPTURE_LINUX_WAYLAND_SCOPED_GLIB_H_
