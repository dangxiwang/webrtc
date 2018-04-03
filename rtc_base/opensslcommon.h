/*
 *  Copyright 2018 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_OPENSSLCOMMON_H_
#define RTC_BASE_OPENSSLCOMMON_H_

#include <string>

#if defined(WEBRTC_POSIX)
#include <unistd.h>
#endif

#if defined(WEBRTC_WIN)
// Must be included first before openssl headers.
#include "rtc_base/win32.h"  // NOLINT
#endif                       // WEBRTC_WIN

#include <openssl/crypto.h>
#include <openssl/x509v3.h>
#include "rtc_base/constructormagic.h"
#include "rtc_base/openssl.h"

namespace rtc {
// The OpenSSLCommon class holds static helper methods. All methods related
// to OpenSSL that are commonly used and don't require global state should be
// placed here.
class OpenSSLCommon final {
 public:
  // Verifies that the hostname provided matches that in the peer certificate
  // attached to this SSL state.
  static bool VerifyPeerCertMatchesHost(SSL* ssl, const std::string& host);

 private:
  // If LOG_CERTIFICATES is set enables verbose logging about the certificate.
  static void LogCertificates(SSL* ssl, X509* certificate);
  // Disable being able to construct this as an object.
  OpenSSLCommon() = default;
  // Disable all copy and assign semantics.
  RTC_DISALLOW_COPY_AND_ASSIGN(OpenSSLCommon);
};
}  // namespace rtc

#endif  // RTC_BASE_OPENSSLCOMMON_H_
