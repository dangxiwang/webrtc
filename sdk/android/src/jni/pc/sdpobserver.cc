/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "sdk/android/src/jni/pc/sdpobserver.h"

#include <utility>

#include "api/mediaconstraintsinterface.h"
#include "sdk/android/generated_peerconnection_jni/jni/SdpObserver_jni.h"
#include "sdk/android/native_api/jni/java_types.h"
#include "sdk/android/src/jni/jni_helpers.h"

namespace webrtc {
namespace jni {

CreateSdpObserverJni::CreateSdpObserverJni(
    JNIEnv* env,
    const JavaRef<jobject>& j_observer,
    std::unique_ptr<MediaConstraintsInterface> constraints)
    : j_observer_global_(env, j_observer),
      constraints_(std::move(constraints)) {}

CreateSdpObserverJni::~CreateSdpObserverJni() = default;

void CreateSdpObserverJni::OnSuccess(SessionDescriptionInterface* desc) {
  JNIEnv* env = AttachCurrentThreadIfNeeded();
  Java_SdpObserver_onCreateSuccess(env, j_observer_global_,
                                   NativeToJavaSessionDescription(env, desc));
  // OnSuccess transfers ownership of the description (there's a TODO to make
  // it use unique_ptr...).
  delete desc;
}

void CreateSdpObserverJni::OnFailure(webrtc::RTCError error) {
  JNIEnv* env = AttachCurrentThreadIfNeeded();
  Java_SdpObserver_onCreateFailure(env, j_observer_global_,
                                   NativeToJavaString(env, error.message()));
}

SetSdpObserverJni::SetSdpObserverJni(
    JNIEnv* env,
    const JavaRef<jobject>& j_observer,
    std::unique_ptr<MediaConstraintsInterface> constraints)
    : j_observer_global_(env, j_observer),
      constraints_(std::move(constraints)) {}

SetSdpObserverJni::~SetSdpObserverJni() = default;

void SetSdpObserverJni::OnSuccess() {
  JNIEnv* env = AttachCurrentThreadIfNeeded();
  Java_SdpObserver_onSetSuccess(env, j_observer_global_);
}

void SetSdpObserverJni::OnFailure(webrtc::RTCError error) {
  JNIEnv* env = AttachCurrentThreadIfNeeded();
  Java_SdpObserver_onSetFailure(env, j_observer_global_,
                                NativeToJavaString(env, error.message()));
}

}  // namespace jni
}  // namespace webrtc
