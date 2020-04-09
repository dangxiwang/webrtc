/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import <Foundation/Foundation.h>

#import "RTCMacros.h"
#import "RTCVideoDecoder.h"

RTC_OBJC_EXPORT
@interface WebRTCVideoDecoderVP8 : NSObject

/* This returns a VP8 decoder that can be returned from a WebRTCVideoDecoderFactory injected into
 * WebRTCPeerConnectionFactory. Even though it implements the WebRTCVideoDecoder protocol, it can not be
 * used independently from the WebRTCPeerConnectionFactory.
 */
+ (id<WebRTCVideoDecoder>)vp8Decoder;

@end
