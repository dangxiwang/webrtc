/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_CRYPTO_FRAMEENCRYPTORINTERFACE_H_
#define API_CRYPTO_FRAMEENCRYPTORINTERFACE_H_

#include "api/array_view.h"
#include "api/crypto/framecryptoparams.h"
#include "rtc_base/refcount.h"

namespace webrtc {

// FrameEncryptorInterface allows users to provide a custom encryption
// implementation to encrypt all outgoing audio and video frames. The user must
// also provid a FrameDecryptorInterface to be able to decrypt the frames on
// the receiving device. Note this is an additional layer of encryption in
// addition to the standard DTLS-SRTP mechanism and is not intended to be used
// without it. This interface may be stateful and you can assume it will have
// the same lifetime as the RTPSender it is set on.
// Note:
// This interface is not ready for production use.
class FrameEncryptorInterface : public rtc::RefCountInterface {
 public:
  virtual ~FrameEncryptorInterface() {}

  // FrameCryptoParameters defines the keying material and cryptosuite to use
  // when encrypting frames. The decryptor takes ownership of the parameters
  // to prevent keying material existing in multiple locations in memory.
  virtual void SetParameters(FrameCryptoParams params) = 0;

  // Attempts to encrypt the provided frame. You may assume the encrypted_frame
  // will match the size returned by GetOutputSize for a give frame.
  virtual bool Encrypt(CryptoMediaType media_type,
                       uint32_t ssrc,
                       rtc::ArrayView<const uint8_t> frame,
                       rtc::ArrayView<uint8_t> encrypted_frame) = 0;

  // Returns the total required length in bytes for the output of the
  // encryption.
  virtual size_t GetOutputSize(CryptoMediaType media_type,
                               size_t frame_size) = 0;
};

}  // namespace webrtc

#endif  // API_CRYPTO_FRAMEENCRYPTORINTERFACE_H_
