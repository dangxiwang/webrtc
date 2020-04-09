/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "RTCVideoEncoderFactoryH264.h"

#import "RTCH264ProfileLevelId.h"
#import "RTCVideoEncoderH264.h"

@implementation WebRTCVideoEncoderFactoryH264

- (NSArray<WebRTCVideoCodecInfo *> *)supportedCodecs {
  NSMutableArray<WebRTCVideoCodecInfo *> *codecs = [NSMutableArray array];
  NSString *codecName = kRTCVideoCodecH264Name;

  NSDictionary<NSString *, NSString *> *constrainedHighParams = @{
    @"profile-level-id" : kRTCMaxSupportedH264ProfileLevelConstrainedHigh,
    @"level-asymmetry-allowed" : @"1",
    @"packetization-mode" : @"1",
  };
  WebRTCVideoCodecInfo *constrainedHighInfo =
      [[WebRTCVideoCodecInfo alloc] initWithName:codecName parameters:constrainedHighParams];
  [codecs addObject:constrainedHighInfo];

  NSDictionary<NSString *, NSString *> *constrainedBaselineParams = @{
    @"profile-level-id" : kRTCMaxSupportedH264ProfileLevelConstrainedBaseline,
    @"level-asymmetry-allowed" : @"1",
    @"packetization-mode" : @"1",
  };
  WebRTCVideoCodecInfo *constrainedBaselineInfo =
      [[WebRTCVideoCodecInfo alloc] initWithName:codecName parameters:constrainedBaselineParams];
  [codecs addObject:constrainedBaselineInfo];

  return [codecs copy];
}

- (id<WebRTCVideoEncoder>)createEncoder:(WebRTCVideoCodecInfo *)info {
  return [[WebRTCVideoEncoderH264 alloc] initWithCodecInfo:info];
}

@end
