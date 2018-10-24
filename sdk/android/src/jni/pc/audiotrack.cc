/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/mediastreaminterface.h"
#include "sdk/android/generated_peerconnection_jni/jni/AudioTrack_jni.h"
#include "sdk/android/src/jni/audiosink.h"
#include "sdk/android/src/jni/jni_helpers.h"

namespace webrtc {
namespace jni {

static void JNI_AudioTrack_SetVolume(JNIEnv*,
                                     const JavaParamRef<jclass>&,
                                     jlong j_p,
                                     jdouble volume) {
  rtc::scoped_refptr<AudioSourceInterface> source(
      reinterpret_cast<AudioTrackInterface*>(j_p)->GetSource());
  source->SetVolume(volume);
}

static void JNI_AudioTrack_AddSink(JNIEnv*,
                                   const JavaParamRef<jclass>&,
                                   jlong j_native_track,
                                   jlong j_native_sink) {
  rtc::scoped_refptr<AudioSourceInterface> source(
      reinterpret_cast<AudioTrackInterface*>(j_native_track)->GetSource());
  source->AddSink(reinterpret_cast<AudioTrackSinkInterface*>(j_native_sink));
}

static void JNI_AudioTrack_RemoveSink(JNIEnv*,
                                      const JavaParamRef<jclass>&,
                                      jlong j_native_track,
                                      jlong j_native_sink) {
  rtc::scoped_refptr<AudioSourceInterface> source(
      reinterpret_cast<AudioTrackInterface*>(j_native_track)->GetSource());
  source->RemoveSink(reinterpret_cast<AudioTrackSinkInterface*>(j_native_sink));
}

static jlong JNI_AudioTrack_WrapSink(JNIEnv* jni,
                                     const JavaParamRef<jclass>&,
                                     const JavaParamRef<jobject>& sink) {
  return jlongFromPointer(new AudioSinkWrapper(jni, sink));
}

static void JNI_AudioTrack_FreeSink(JNIEnv* jni,
                                    const JavaParamRef<jclass>&,
                                    jlong j_native_sink) {
  delete reinterpret_cast<AudioSinkWrapper*>(j_native_sink);
}

}  // namespace jni
}  // namespace webrtc
