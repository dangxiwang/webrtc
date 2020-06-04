/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_SYNCHRONIZATION_MUTEX_PTHREAD_H_
#define RTC_BASE_SYNCHRONIZATION_MUTEX_PTHREAD_H_

#if defined(WEBRTC_POSIX)

#include <pthread.h>

#include "rtc_base/thread_annotations.h"

namespace webrtc {

class RTC_LOCKABLE MutexImpl final {
 public:
  MutexImpl() { pthread_mutex_init(&mutex_, nullptr); }
  MutexImpl(const MutexImpl&) = delete;
  MutexImpl& operator=(const MutexImpl&) = delete;
  ~MutexImpl() { pthread_mutex_destroy(&mutex_); }

  void Lock() RTC_EXCLUSIVE_LOCK_FUNCTION() { pthread_mutex_lock(&mutex_); }
  RTC_WARN_UNUSED_RESULT bool TryLock() RTC_EXCLUSIVE_TRYLOCK_FUNCTION(true) {
    return pthread_mutex_trylock(&mutex_) == 0;
  }
  void Unlock() RTC_UNLOCK_FUNCTION() { pthread_mutex_unlock(&mutex_); }

 private:
  pthread_mutex_t mutex_;
};

}  // namespace webrtc
#endif  // #if defined(WEBRTC_POSIX)
#endif  // RTC_BASE_SYNCHRONIZATION_MUTEX_PTHREAD_H_
