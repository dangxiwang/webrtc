/*
 *  Copyright 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "RTCMediaStreamTrack.h"

#include "api/media_stream_interface.h"

typedef NS_ENUM(NSInteger, RTCMediaStreamTrackType) {
  RTCMediaStreamTrackTypeAudio,
  RTCMediaStreamTrackTypeVideo,
};

NS_ASSUME_NONNULL_BEGIN

@class WebRTCPeerConnectionFactory;

@interface WebRTCMediaStreamTrack ()

@property(nonatomic, readonly) WebRTCPeerConnectionFactory *factory;

/**
 * The native MediaStreamTrackInterface passed in or created during
 * construction.
 */
@property(nonatomic, readonly) rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> nativeTrack;

/**
 * Initialize an WebRTCMediaStreamTrack from a native MediaStreamTrackInterface.
 */
- (instancetype)initWithFactory:(WebRTCPeerConnectionFactory *)factory
                    nativeTrack:(rtc::scoped_refptr<webrtc::MediaStreamTrackInterface>)nativeTrack
                           type:(RTCMediaStreamTrackType)type NS_DESIGNATED_INITIALIZER;

- (instancetype)initWithFactory:(WebRTCPeerConnectionFactory *)factory
                    nativeTrack:(rtc::scoped_refptr<webrtc::MediaStreamTrackInterface>)nativeTrack;

- (BOOL)isEqualToTrack:(WebRTCMediaStreamTrack *)track;

+ (webrtc::MediaStreamTrackInterface::TrackState)nativeTrackStateForState:
        (RTCMediaStreamTrackState)state;

+ (RTCMediaStreamTrackState)trackStateForNativeState:
        (webrtc::MediaStreamTrackInterface::TrackState)nativeState;

+ (NSString *)stringForState:(RTCMediaStreamTrackState)state;

+ (WebRTCMediaStreamTrack *)mediaTrackForNativeTrack:
                             (rtc::scoped_refptr<webrtc::MediaStreamTrackInterface>)nativeTrack
                                          factory:(WebRTCPeerConnectionFactory *)factory;

@end

NS_ASSUME_NONNULL_END
