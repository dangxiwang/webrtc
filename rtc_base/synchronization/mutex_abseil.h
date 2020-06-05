/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_SYNCHRONIZATION_MUTEX_ABSEIL_H_
#define RTC_BASE_SYNCHRONIZATION_MUTEX_ABSEIL_H_

#include "rtc_base/thread_annotations.h"

namespace webrtc {

class RTC_LOCKABLE MutexImpl final {
 public:
  MutexImpl() = default;
  MutexImpl(const MutexImpl&) = delete;
  MutexImpl& operator=(const MutexImpl&) = delete;

  void Lock() RTC_EXCLUSIVE_LOCK_FUNCTION() { mutex_.Lock(); }
  RTC_WARN_UNUSED_RESULT bool TryLock() RTC_EXCLUSIVE_TRYLOCK_FUNCTION(true) {
    return mutex_.TryLock();
  }
  void Unlock() RTC_UNLOCK_FUNCTION() { mutex_.Unlock(); }

 private:
  absl::Mutex mutex_;
};

}  // namespace webrtc

#endif  // RTC_BASE_SYNCHRONIZATION_MUTEX_ABSEIL_H_
