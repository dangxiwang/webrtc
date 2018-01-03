/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <jni.h>

#include "media/engine/videoencodersoftwarefallbackwrapper.h"
#include "rtc_base/jni/jni_helpers.h"
#include "sdk/android/generated_video_jni/jni/VideoEncoderFallback_jni.h"
#include "sdk/android/src/jni/wrappednativecodec.h"

namespace webrtc {
namespace jni {

static jlong JNI_VideoEncoderFallback_CreateEncoder(
    JNIEnv* jni,
    const JavaParamRef<jclass>&,
    const JavaParamRef<jobject>& j_fallback_encoder,
    const JavaParamRef<jobject>& j_primary_encoder) {
  std::unique_ptr<VideoEncoder> fallback_encoder =
      JavaToNativeVideoEncoder(jni, j_fallback_encoder);
  std::unique_ptr<VideoEncoder> primary_encoder =
      JavaToNativeVideoEncoder(jni, j_primary_encoder);

  VideoEncoderSoftwareFallbackWrapper* nativeWrapper =
      new VideoEncoderSoftwareFallbackWrapper(std::move(fallback_encoder),
                                              std::move(primary_encoder));

  return jlongFromPointer(nativeWrapper);
}

}  // namespace jni
}  // namespace webrtc
