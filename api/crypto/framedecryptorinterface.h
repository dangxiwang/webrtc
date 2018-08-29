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
#include "api/mediatypes.h"
#include "rtc_base/buffer.h"
#include "rtc_base/refcount.h"

namespace webrtc {

// FrameDecryptorInterface allows users to provide a custom decryption
// implementation for all incoming audio and video frames. The user must also
// provide a FrameEncryptorInterface to be able to encrypt the frames being
// sent out of the device. Note this is an additional layer of encyrption in
// addition to the standard DTLS-SRTP mechanism and is not intended to be used
// without it. You may assume that this interface will have the same lifetime
// as the RTPReciever it is attached to. It must only be attached to one
// RTPReciever.
// Note:
// This interface is not ready for production use.
class FrameDecryptorInterface : public rtc::RefCountInterface {
 public:
  virtual ~FrameDecryptorInterface() {}

  // Add a key to the frame decryptor that will be used to decrypt frames.
  virtual void RotateKey(rtc::ZeroOnFreeBuffer<uint8_t> key) = 0;

  // Attempts to decrypt the encrypted frame. You may assume the frame size will
  // be allocated to the size returned from GetOutputSize. You may assume that
  // the frames are in order if SRTP is enabled. The stream is not provided here
  // and it is up to the implementor to transport this information to the
  // reciever if they care about it.
  virtual bool Decrypt(cricket::MediaType media_type,
                       rtc::ArrayView<const uint8_t> encrypted_frame,
                       rtc::ArrayView<uint8_t> frame) = 0;

  // Returns the total required length in bytes for the output of the
  // decryption.
  virtual size_t GetOutputSize(cricket::MediaType media_type,
                               size_t encrypted_frame_size) = 0;
};

}  // namespace webrtc

#endif  // API_CRYPTO_FRAMEDECRYPTORINTERFACE_H_
