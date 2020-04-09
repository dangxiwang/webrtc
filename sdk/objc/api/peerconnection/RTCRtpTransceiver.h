/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import <Foundation/Foundation.h>

#import "RTCMacros.h"
#import "RTCRtpReceiver.h"
#import "RTCRtpSender.h"

NS_ASSUME_NONNULL_BEGIN

/** https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiverdirection */
typedef NS_ENUM(NSInteger, RTCRtpTransceiverDirection) {
  RTCRtpTransceiverDirectionSendRecv,
  RTCRtpTransceiverDirectionSendOnly,
  RTCRtpTransceiverDirectionRecvOnly,
  RTCRtpTransceiverDirectionInactive,
  RTCRtpTransceiverDirectionStopped
};

/** Structure for initializing an WebRTCRtpTransceiver in a call to
 *  WebRTCPeerConnection.addTransceiver.
 *  https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiverinit
 */
RTC_OBJC_EXPORT
@interface WebRTCRtpTransceiverInit : NSObject

/** Direction of the WebRTCRtpTransceiver. See WebRTCRtpTransceiver.direction. */
@property(nonatomic) RTCRtpTransceiverDirection direction;

/** The added WebRTCRtpTransceiver will be added to these streams. */
@property(nonatomic) NSArray<NSString *> *streamIds;

/** TODO(bugs.webrtc.org/7600): Not implemented. */
@property(nonatomic) NSArray<WebRTCRtpEncodingParameters *> *sendEncodings;

@end

@class WebRTCRtpTransceiver;

/** The WebRTCRtpTransceiver maps to the WebRTCRtpTransceiver defined by the WebRTC
 *  specification. A transceiver represents a combination of an WebRTCRtpSender
 *  and an WebRTCRtpReceiver that share a common mid. As defined in JSEP, an
 *  WebRTCRtpTransceiver is said to be associated with a media description if its
 *  mid property is non-nil; otherwise, it is said to be disassociated.
 *  JSEP: https://tools.ietf.org/html/draft-ietf-rtcweb-jsep-24
 *
 *  Note that RTCRtpTransceivers are only supported when using
 *  WebRTCPeerConnection with Unified Plan SDP.
 *
 *  WebRTC specification for WebRTCRtpTransceiver, the JavaScript analog:
 *  https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiver
 */
RTC_OBJC_EXPORT
@protocol WebRTCRtpTransceiver <NSObject>

/** Media type of the transceiver. The sender and receiver will also have this
 *  type.
 */
@property(nonatomic, readonly) RTCRtpMediaType mediaType;

/** The mid attribute is the mid negotiated and present in the local and
 *  remote descriptions. Before negotiation is complete, the mid value may be
 *  nil. After rollbacks, the value may change from a non-nil value to nil.
 *  https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiver-mid
 */
@property(nonatomic, readonly) NSString *mid;

/** The sender attribute exposes the WebRTCRtpSender corresponding to the RTP
 *  media that may be sent with the transceiver's mid. The sender is always
 *  present, regardless of the direction of media.
 *  https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiver-sender
 */
@property(nonatomic, readonly) WebRTCRtpSender *sender;

/** The receiver attribute exposes the WebRTCRtpReceiver corresponding to the RTP
 *  media that may be received with the transceiver's mid. The receiver is
 *  always present, regardless of the direction of media.
 *  https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiver-receiver
 */
@property(nonatomic, readonly) WebRTCRtpReceiver *receiver;

/** The isStopped attribute indicates that the sender of this transceiver will
 *  no longer send, and that the receiver will no longer receive. It is true if
 *  either stop has been called or if setting the local or remote description
 *  has caused the WebRTCRtpTransceiver to be stopped.
 *  https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiver-stopped
 */
@property(nonatomic, readonly) BOOL isStopped;

/** The direction attribute indicates the preferred direction of this
 *  transceiver, which will be used in calls to createOffer and createAnswer.
 *  An update of directionality does not take effect immediately. Instead,
 *  future calls to createOffer and createAnswer mark the corresponding media
 *  descriptions as sendrecv, sendonly, recvonly, or inactive.
 *  https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiver-direction
 */
@property(nonatomic) RTCRtpTransceiverDirection direction;

/** The currentDirection attribute indicates the current direction negotiated
 *  for this transceiver. If this transceiver has never been represented in an
 *  offer/answer exchange, or if the transceiver is stopped, the value is not
 *  present and this method returns NO.
 *  https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiver-currentdirection
 */
- (BOOL)currentDirection:(RTCRtpTransceiverDirection *)currentDirectionOut;

/** The stop method irreversibly stops the WebRTCRtpTransceiver. The sender of
 *  this transceiver will no longer send, the receiver will no longer receive.
 *  https://w3c.github.io/webrtc-pc/#dom-rtcrtptransceiver-stop
 */
- (void)stop;

@end

RTC_OBJC_EXPORT
@interface WebRTCRtpTransceiver : NSObject <WebRTCRtpTransceiver>

- (instancetype)init NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END
