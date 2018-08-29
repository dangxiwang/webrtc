
/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/crypto/framecryptoparams.h"

#include <string>

namespace webrtc {

FrameCryptoParams::FrameCryptoParams() = default;
FrameCryptoParams::~FrameCryptoParams() = default;
FrameCryptoParams::FrameCryptoParams(FrameCryptoParams&&) = default;
FrameCryptoParams& FrameCryptoParams::operator=(FrameCryptoParams&&) = default;

void FrameCryptoParams::SetCipherSuite(const std::string& cipher_suite) {
  cipher_suite_ = cipher_suite;
}

std::string FrameCryptoParams::GetCipherSuite() const {
  return cipher_suite_;
}

void FrameCryptoParams::SetKey(const rtc::ArrayView<const uint8_t> key) {
  key_.SetData(key.data(), key.size());
}

rtc::ArrayView<const uint8_t> FrameCryptoParams::GetKey() const {
  return rtc::ArrayView<const uint8_t>(key_.data(), key_.size());
}

}  // namespace webrtc
