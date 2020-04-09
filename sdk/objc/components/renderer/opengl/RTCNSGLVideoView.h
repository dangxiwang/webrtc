/*
 *  Copyright 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import <Foundation/Foundation.h>

#if !TARGET_OS_IPHONE

#import <AppKit/NSOpenGLView.h>

#import "RTCVideoRenderer.h"
#import "RTCVideoViewShading.h"

NS_ASSUME_NONNULL_BEGIN

@class WebRTCNSGLVideoView;

RTC_OBJC_EXPORT
@protocol WebRTCNSGLVideoViewDelegate <WebRTCVideoViewDelegate>
@end

RTC_OBJC_EXPORT
@interface WebRTCNSGLVideoView : NSOpenGLView <WebRTCVideoRenderer>

@property(nonatomic, weak) id<WebRTCVideoViewDelegate> delegate;

- (instancetype)initWithFrame:(NSRect)frameRect
                  pixelFormat:(NSOpenGLPixelFormat *)format
                       shader:(id<WebRTCVideoViewShading>)shader NS_DESIGNATED_INITIALIZER;

@end

NS_ASSUME_NONNULL_END

#endif
