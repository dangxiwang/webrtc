/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef SDK_ANDROID_SRC_JNI_ENCODEDIMAGE_H_
#define SDK_ANDROID_SRC_JNI_ENCODEDIMAGE_H_

#include <jni.h>

#include "api/video/video_rotation.h"
#include "common_types.h"  // NOLINT(build/include)

namespace webrtc {
namespace jni {

jobject Java_EncodedImage_createFrameType(JNIEnv* env, FrameType frame_type);

jobject Java_EncodedImage_Create(JNIEnv* env,
                                 jobject buffer,
                                 int encoded_width,
                                 int encoded_height,
                                 int64_t capture_time_ns,
                                 jobject frame_type,
                                 VideoRotation rotation,
                                 bool is_complete_frame,
                                 jobject qp);

}  // namespace jni
}  // namespace webrtc

#endif  // SDK_ANDROID_SRC_JNI_ENCODEDIMAGE_H_
