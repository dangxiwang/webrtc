/*
 *  Copyright 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "RTCRtpReceiver+Private.h"

#import "RTCMediaStreamTrack+Private.h"
#import "RTCRtpParameters+Private.h"
#import "RTCRtpReceiver+Native.h"
#import "base/RTCLogging.h"
#import "helpers/NSString+StdString.h"

#include "api/media_stream_interface.h"

namespace webrtc {

RtpReceiverDelegateAdapter::RtpReceiverDelegateAdapter(
    WebRTCRtpReceiver *receiver) {
  RTC_CHECK(receiver);
  receiver_ = receiver;
}

void RtpReceiverDelegateAdapter::OnFirstPacketReceived(
    cricket::MediaType media_type) {
  RTCRtpMediaType packet_media_type =
      [WebRTCRtpReceiver mediaTypeForNativeMediaType:media_type];
  WebRTCRtpReceiver *receiver = receiver_;
  [receiver.delegate rtpReceiver:receiver didReceiveFirstPacketForMediaType:packet_media_type];
}

}  // namespace webrtc

@implementation WebRTCRtpReceiver {
  WebRTCPeerConnectionFactory *_factory;
  rtc::scoped_refptr<webrtc::RtpReceiverInterface> _nativeRtpReceiver;
  std::unique_ptr<webrtc::RtpReceiverDelegateAdapter> _observer;
}

@synthesize delegate = _delegate;

- (NSString *)receiverId {
  return [NSString stringForStdString:_nativeRtpReceiver->id()];
}

- (WebRTCRtpParameters *)parameters {
  return [[WebRTCRtpParameters alloc]
      initWithNativeParameters:_nativeRtpReceiver->GetParameters()];
}

- (nullable WebRTCMediaStreamTrack *)track {
  rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> nativeTrack(
    _nativeRtpReceiver->track());
  if (nativeTrack) {
    return [WebRTCMediaStreamTrack mediaTrackForNativeTrack:nativeTrack factory:_factory];
  }
  return nil;
}

- (NSString *)description {
  return [NSString stringWithFormat:@"WebRTCRtpReceiver {\n  receiverId: %@\n}",
      self.receiverId];
}

- (void)dealloc {
  if (_nativeRtpReceiver) {
    _nativeRtpReceiver->SetObserver(nullptr);
  }
}

- (BOOL)isEqual:(id)object {
  if (self == object) {
    return YES;
  }
  if (object == nil) {
    return NO;
  }
  if (![object isMemberOfClass:[self class]]) {
    return NO;
  }
  WebRTCRtpReceiver *receiver = (WebRTCRtpReceiver *)object;
  return _nativeRtpReceiver == receiver.nativeRtpReceiver;
}

- (NSUInteger)hash {
  return (NSUInteger)_nativeRtpReceiver.get();
}

#pragma mark - Native

- (void)setFrameDecryptor:(rtc::scoped_refptr<webrtc::FrameDecryptorInterface>)frameDecryptor {
  _nativeRtpReceiver->SetFrameDecryptor(frameDecryptor);
}

#pragma mark - Private

- (rtc::scoped_refptr<webrtc::RtpReceiverInterface>)nativeRtpReceiver {
  return _nativeRtpReceiver;
}

- (instancetype)initWithFactory:(WebRTCPeerConnectionFactory *)factory
              nativeRtpReceiver:
                  (rtc::scoped_refptr<webrtc::RtpReceiverInterface>)nativeRtpReceiver {
  if (self = [super init]) {
    _factory = factory;
    _nativeRtpReceiver = nativeRtpReceiver;
    RTCLogInfo(
        @"WebRTCRtpReceiver(%p): created receiver: %@", self, self.description);
    _observer.reset(new webrtc::RtpReceiverDelegateAdapter(self));
    _nativeRtpReceiver->SetObserver(_observer.get());
  }
  return self;
}

+ (RTCRtpMediaType)mediaTypeForNativeMediaType:
    (cricket::MediaType)nativeMediaType {
  switch (nativeMediaType) {
    case cricket::MEDIA_TYPE_AUDIO:
      return RTCRtpMediaTypeAudio;
    case cricket::MEDIA_TYPE_VIDEO:
      return RTCRtpMediaTypeVideo;
    case cricket::MEDIA_TYPE_DATA:
      return RTCRtpMediaTypeData;
  }
}

+ (cricket::MediaType)nativeMediaTypeForMediaType:(RTCRtpMediaType)mediaType {
  switch (mediaType) {
    case RTCRtpMediaTypeAudio:
      return cricket::MEDIA_TYPE_AUDIO;
    case RTCRtpMediaTypeVideo:
      return cricket::MEDIA_TYPE_VIDEO;
    case RTCRtpMediaTypeData:
      return cricket::MEDIA_TYPE_DATA;
  }
}

+ (NSString *)stringForMediaType:(RTCRtpMediaType)mediaType {
  switch (mediaType) {
    case RTCRtpMediaTypeAudio:
      return @"AUDIO";
    case RTCRtpMediaTypeVideo:
      return @"VIDEO";
    case RTCRtpMediaTypeData:
      return @"DATA";
  }
}

@end
