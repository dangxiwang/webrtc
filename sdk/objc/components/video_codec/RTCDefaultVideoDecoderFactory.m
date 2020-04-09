/*
 *  Copyright 2017 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "RTCDefaultVideoDecoderFactory.h"

#import "RTCH264ProfileLevelId.h"
#import "RTCVideoDecoderH264.h"
#import "api/video_codec/RTCVideoCodecConstants.h"
#import "api/video_codec/RTCVideoDecoderVP8.h"
#import "base/RTCVideoCodecInfo.h"
#if defined(RTC_ENABLE_VP9)
#import "api/video_codec/RTCVideoDecoderVP9.h"
#endif

@implementation WebRTCDefaultVideoDecoderFactory

- (NSArray<WebRTCVideoCodecInfo *> *)supportedCodecs {
  NSDictionary<NSString *, NSString *> *constrainedHighParams = @{
    @"profile-level-id" : kRTCMaxSupportedH264ProfileLevelConstrainedHigh,
    @"level-asymmetry-allowed" : @"1",
    @"packetization-mode" : @"1",
  };
  WebRTCVideoCodecInfo *constrainedHighInfo =
      [[WebRTCVideoCodecInfo alloc] initWithName:kRTCVideoCodecH264Name
                                   parameters:constrainedHighParams];

  NSDictionary<NSString *, NSString *> *constrainedBaselineParams = @{
    @"profile-level-id" : kRTCMaxSupportedH264ProfileLevelConstrainedBaseline,
    @"level-asymmetry-allowed" : @"1",
    @"packetization-mode" : @"1",
  };
  WebRTCVideoCodecInfo *constrainedBaselineInfo =
      [[WebRTCVideoCodecInfo alloc] initWithName:kRTCVideoCodecH264Name
                                   parameters:constrainedBaselineParams];

  WebRTCVideoCodecInfo *vp8Info = [[WebRTCVideoCodecInfo alloc] initWithName:kRTCVideoCodecVp8Name];

#if defined(RTC_ENABLE_VP9)
  WebRTCVideoCodecInfo *vp9Info = [[WebRTCVideoCodecInfo alloc] initWithName:kRTCVideoCodecVp9Name];
#endif

  return @[
    constrainedHighInfo,
    constrainedBaselineInfo,
    vp8Info,
#if defined(RTC_ENABLE_VP9)
    vp9Info,
#endif
  ];
}

- (id<WebRTCVideoDecoder>)createDecoder:(WebRTCVideoCodecInfo *)info {
  if ([info.name isEqualToString:kRTCVideoCodecH264Name]) {
    return [[WebRTCVideoDecoderH264 alloc] init];
  } else if ([info.name isEqualToString:kRTCVideoCodecVp8Name]) {
    return [WebRTCVideoDecoderVP8 vp8Decoder];
#if defined(RTC_ENABLE_VP9)
  } else if ([info.name isEqualToString:kRTCVideoCodecVp9Name]) {
    return [WebRTCVideoDecoderVP9 vp9Decoder];
#endif
  }

  return nil;
}

@end
