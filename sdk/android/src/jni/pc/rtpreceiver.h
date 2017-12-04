/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef SDK_ANDROID_SRC_JNI_PC_RTPRECEIVER_H_
#define SDK_ANDROID_SRC_JNI_PC_RTPRECEIVER_H_

#include <jni.h>

#include "api/rtpreceiverinterface.h"
#include "sdk/android/src/jni/jni_helpers.h"

namespace webrtc {
namespace jni {

jobject NativeToJavaRtpReceiver(
    JNIEnv* env,
    rtc::scoped_refptr<RtpReceiverInterface> receiver);

// Takes ownership of the passed |j_receiver| and stores it as a global
// reference. Will call dispose() in the dtor.
class JavaRtpReceiverGlobalOwner {
 public:
  JavaRtpReceiverGlobalOwner(JNIEnv* env, jobject j_receiver);
  JavaRtpReceiverGlobalOwner(JavaRtpReceiverGlobalOwner&& other);
  ~JavaRtpReceiverGlobalOwner();

 private:
  ScopedGlobalRef<jobject> j_receiver_;
};

}  // namespace jni
}  // namespace webrtc

#endif  // SDK_ANDROID_SRC_JNI_PC_RTPRECEIVER_H_
