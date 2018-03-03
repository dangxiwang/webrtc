/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef SDK_ANDROID_SRC_JNI_PC_RTPTRANSCEIVER_H_
#define SDK_ANDROID_SRC_JNI_PC_RTPTRANSCEIVER_H_

#include <jni.h>

#include "api/rtptransceiverinterface.h"
#include "sdk/android/native_api/jni/scoped_java_ref.h"

namespace webrtc {
namespace jni {

RtpTransceiverInit JavaToNativeRtpTransceiverInit(
    JNIEnv* jni,
    const JavaRef<jobject>& j_init);

ScopedJavaLocalRef<jobject> NativeToJavaRtpTransceiver(
    JNIEnv* env,
    rtc::scoped_refptr<RtpTransceiverInterface> transceiver);

}  // namespace jni
}  // namespace webrtc

#endif  // SDK_ANDROID_SRC_JNI_PC_RTPTRANSCEIVER_H_
