/*
 *  Copyright 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "RTCSessionDescription.h"

#include "api/jsep.h"

NS_ASSUME_NONNULL_BEGIN

@interface WebRTCSessionDescription ()

/**
 * The native SessionDescriptionInterface representation of this
 * WebRTCSessionDescription object. This is needed to pass to the underlying C++
 * APIs.
 */
@property(nonatomic, readonly, nullable) webrtc::SessionDescriptionInterface *nativeDescription;

/**
 * Initialize an WebRTCSessionDescription from a native
 * SessionDescriptionInterface. No ownership is taken of the native session
 * description.
 */
- (instancetype)initWithNativeDescription:
        (const webrtc::SessionDescriptionInterface *)nativeDescription;

+ (std::string)stdStringForType:(RTCSdpType)type;

+ (RTCSdpType)typeForStdString:(const std::string &)string;

@end

NS_ASSUME_NONNULL_END
