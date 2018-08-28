
/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_CRYPTO_FRAMECRYPTOPARAMS_H_
#define API_CRYPTO_FRAMECRYPTOPARAMS_H_

#include <string>
#include "rtc_base/buffer.h"

namespace webrtc {

// The media type to use with this.
enum class CryptoMediaType { Video = 0x00, Audio = 0x01 };

#define CIPHER_SUITE_AES_CM_128_HMAC_SHA256_80 "AES_CM_128_HMAC_SHA256_80"
#define CIPHER_SUITE_AES_CM_128_HMAC_SHA256_32 "AES_CM_128_HMAC_SHA256_32"

class FrameCryptoParams final {
 public:
  FrameCryptoParams();
  ~FrameCryptoParams();
  // This object is not copyable or assignable.
  FrameCryptoParams(const FrameCryptoParams&) = delete;
  FrameCryptoParams& operator=(const FrameCryptoParams&) = delete;
  // This object is only moveable.
  FrameCryptoParams(FrameCryptoParams&&);
  FrameCryptoParams& operator=(FrameCryptoParams&&);

  void SetCipherSuite(const std::string& cipher_suite);
  std::string GetCipherSuite() const;
  void SetKey(const rtc::ArrayView<const uint8_t> key);
  rtc::ArrayView<const uint8_t> GetKey() const;

 private:
  std::string cipher_suite_;
  rtc::ZeroOnFreeBuffer<uint8_t> key_;
};

}  // namespace webrtc

#endif  // API_CRYPTO_FRAMECRYPTOPARAMS_H_
