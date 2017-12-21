/*
 *  Copyright 2012 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_SHA1DIGEST_H_
#define RTC_BASE_SHA1DIGEST_H_

#include <openssl/sha.h>

#include "rtc_base/messagedigest.h"

namespace rtc {

// A simple wrapper for the SHA-1 implementation.
class Sha1Digest : public MessageDigest {
 public:
  enum { kSize = SHA_DIGEST_LENGTH };
  Sha1Digest() {
    SHA1_Init(&ctx_);
  }
  size_t Size() const override;
  void Update(const void* buf, size_t len) override;
  size_t Finish(void* buf, size_t len) override;

 private:
  SHA_CTX ctx_;
};

}  // namespace rtc

#endif  // RTC_BASE_SHA1DIGEST_H_
