/*
 *  Copyright 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "WebRTC/RTCVideoTrack.h"

#include "api/mediastreaminterface.h"

NS_ASSUME_NONNULL_BEGIN

@interface RTCVideoTrack ()

@property(nonatomic, readonly) RTCPeerConnectionFactory *factory;

/** VideoTrackInterface created or passed in at construction. */
@property(nonatomic, readonly)
    rtc::scoped_refptr<webrtc::VideoTrackInterface> nativeVideoTrack;

/** Initialize an RTCVideoTrack with its source and an id. */
- (instancetype)initWithFactory:(RTCPeerConnectionFactory *)factory
                         source:(RTCVideoSource *)source
                        trackId:(NSString *)trackId;

@end

NS_ASSUME_NONNULL_END
