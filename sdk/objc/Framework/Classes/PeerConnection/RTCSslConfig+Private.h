/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "WebRTC/RTCSslConfig.h"

#include "api/peerconnectioninterface.h"

NS_ASSUME_NONNULL_BEGIN

@interface RTCSslConfig ()

- (webrtc::PeerConnectionInterface::SSLConfig)nativeConfig;

/** Initialize an RTCSslConfig from a native SSLConfig. */
- (instancetype)initWithNativeConfig:(const webrtc::PeerConnectionInterface::SSLConfig &)config
    NS_DESIGNATED_INITIALIZER;

@end

NS_ASSUME_NONNULL_END
