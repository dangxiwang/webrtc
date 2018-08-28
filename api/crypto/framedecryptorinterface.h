/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_CRYPTO_FRAMEDECRYPTORINTERFACE_H_
#define API_CRYPTO_FRAMEDECRYPTORINTERFACE_H_

#include "api/array_view.h"
#include "api/crypto/framecryptoparams.h"
#include "rtc_base/refcount.h"

namespace webrtc {

// FrameDecryptorInterface allows users to provide a custom decryption
// implementation for all incoming audio and video frames. The user must also
// provide a FrameEncryptorInterface to be able to encrypt the frames being
// sent out of the device. Note this is an additional layer of encyrption in
// addition to the standard DTLS-SRTP mechanism and is not intended to be used
// without it. You may assume that this interface can be stateful and will
// have the same lifetime as the RTPReciever it is set on.
// Note:
// This interface is not ready for production use.
class FrameDecryptorInterface : public rtc::RefCountInterface {
 public:
  virtual ~FrameDecryptorInterface() {}

  // FrameCryptoParameters defines the keying material and cryptosuite to use
  // when decrypting frames. The decryptor takes ownership of the parameters
  // to prevent keying material existing in multiple locations in memory.
  virtual void SetParameters(FrameCryptoParams params) = 0;

  // Attempts to decrypt the encrypted frame. You may assume the frame size will
  // be allocated to the size returned from GetOutputSize.
  virtual bool Decrypt(CryptoMediaType media_type,
                       rtc::ArrayView<const uint8_t> encrypted_frame,
                       rtc::ArrayView<uint8_t> frame) = 0;

  // Returns the total required length in bytes for the output of the
  // decryption.
  virtual size_t GetOutputSize(CryptoMediaType media_type,
                               size_t encrypted_frame_size) = 0;
};

}  // namespace webrtc

#endif  // API_CRYPTO_FRAMEDECRYPTORINTERFACE_H_
