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

#import <WebRTC/RTCInternalSoftwareVideoCodec.h>
#import <WebRTC/RTCMacros.h>
#import <WebRTC/RTCVideoCodec.h>

RTC_EXPORT
@interface RTCVideoEncoderVP8 : RTCInternalSoftwareVideoEncoder
@end

RTC_EXPORT
@interface RTCVideoDecoderVP8 : RTCInternalSoftwareVideoDecoder
@end
