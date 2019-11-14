/*
 *  Copyright 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// This file contains interfaces for RtpReceivers
// http://w3c.github.io/webrtc-pc/#rtcrtpreceiver-interface

#ifndef API_RTP_RECEIVER_INTERFACE_H_
#define API_RTP_RECEIVER_INTERFACE_H_

#include <string>
#include <vector>

#include "api/crypto/frame_decryptor_interface.h"
#include "api/dtls_transport_interface.h"
#include "api/media_stream_interface.h"
#include "api/media_types.h"
#include "api/proxy.h"
#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "api/transport/rtp/rtp_source.h"
#include "rtc_base/deprecation.h"
#include "rtc_base/ref_count.h"
#include "rtc_base/system/rtc_export.h"

namespace webrtc {

class RtpReceiverObserverInterface {
 public:
  // Note: Currently if there are multiple RtpReceivers of the same media type,
  // they will all call OnFirstPacketReceived at once.
  //
  // In the future, it's likely that an RtpReceiver will only call
  // OnFirstPacketReceived when a packet is received specifically for its
  // SSRC/mid.
  virtual void OnFirstPacketReceived(cricket::MediaType media_type) = 0;

 protected:
  virtual ~RtpReceiverObserverInterface() {}
};

class RTC_EXPORT RtpReceiverInterface : public rtc::RefCountInterface {
 public:
  virtual rtc::scoped_refptr<MediaStreamTrackInterface> track() const = 0;

  // The dtlsTransport attribute exposes the DTLS transport on which the
  // media is received. It may be null.
  // https://w3c.github.io/webrtc-pc/#dom-rtcrtpreceiver-transport
  // TODO(https://bugs.webrtc.org/907849) remove default implementation
  virtual rtc::scoped_refptr<DtlsTransportInterface> dtls_transport() const;

  // The list of streams that |track| is associated with. This is the same as
  // the [[AssociatedRemoteMediaStreams]] internal slot in the spec.
  // https://w3c.github.io/webrtc-pc/#dfn-associatedremotemediastreams
  // TODO(hbos): Make pure virtual as soon as Chromium's mock implements this.
  // TODO(https://crbug.com/webrtc/9480): Remove streams() in favor of
  // stream_ids() as soon as downstream projects are no longer dependent on
  // stream objects.
  virtual std::vector<std::string> stream_ids() const;
  virtual std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams() const;

  // Audio or video receiver?
  virtual cricket::MediaType media_type() const = 0;

  // Not to be confused with "mid", this is a field we can temporarily use
  // to uniquely identify a receiver until we implement Unified Plan SDP.
  virtual std::string id() const = 0;

  // The WebRTC specification only defines RTCRtpParameters in terms of senders,
  // but this API also applies them to receivers, similar to ORTC:
  // http://ortc.org/wp-content/uploads/2016/03/ortc.html#rtcrtpparameters*.
  virtual RtpParameters GetParameters() const = 0;

  // Does not take ownership of observer.
  // Must call SetObserver(nullptr) before the observer is destroyed.
  virtual void SetObserver(RtpReceiverObserverInterface* observer) = 0;

  // Sets the jitter buffer minimum delay until media playout. Actual observed
  // delay may differ depending on the congestion control. |delay_seconds| is a
  // positive value including 0.0 measured in seconds. |nullopt| means default
  // value must be used.
  virtual void SetJitterBufferMinimumDelay(
      absl::optional<double> delay_seconds) = 0;

  // TODO(zhihuang): Remove the default implementation once the subclasses
  // implement this. Currently, the only relevant subclass is the
  // content::FakeRtpReceiver in Chromium.
  virtual std::vector<RtpSource> GetSources() const;

  // Sets a user defined frame decryptor that will decrypt the entire frame
  // before it is sent across the network. This will decrypt the entire frame
  // using the user provided decryption mechanism regardless of whether SRTP is
  // enabled or not.
  virtual void SetFrameDecryptor(
      rtc::scoped_refptr<FrameDecryptorInterface> frame_decryptor);

  // Returns a pointer to the frame decryptor set previously by the
  // user. This can be used to update the state of the object.
  virtual rtc::scoped_refptr<FrameDecryptorInterface> GetFrameDecryptor() const;

 protected:
  ~RtpReceiverInterface() override = default;
};

// Define proxy for RtpReceiverInterface.
// TODO(deadbeef): Move this to .cc file and out of api/. What threads methods
// are called on is an implementation detail.
BEGIN_SIGNALING_PROXY_MAP(RtpReceiver)
PROXY_SIGNALING_THREAD_DESTRUCTOR()
PROXY_CONSTMETHOD0(rtc::scoped_refptr<MediaStreamTrackInterface>, track)
PROXY_CONSTMETHOD0(rtc::scoped_refptr<DtlsTransportInterface>, dtls_transport)
PROXY_CONSTMETHOD0(std::vector<std::string>, stream_ids)
PROXY_CONSTMETHOD0(std::vector<rtc::scoped_refptr<MediaStreamInterface>>,
                   streams)
PROXY_CONSTMETHOD0(cricket::MediaType, media_type)
PROXY_CONSTMETHOD0(std::string, id)
PROXY_CONSTMETHOD0(RtpParameters, GetParameters)
PROXY_METHOD1(void, SetObserver, RtpReceiverObserverInterface*)
PROXY_METHOD1(void, SetJitterBufferMinimumDelay, absl::optional<double>)
PROXY_CONSTMETHOD0(std::vector<RtpSource>, GetSources)
PROXY_METHOD1(void,
              SetFrameDecryptor,
              rtc::scoped_refptr<FrameDecryptorInterface>)
PROXY_CONSTMETHOD0(rtc::scoped_refptr<FrameDecryptorInterface>,
                   GetFrameDecryptor)
END_PROXY_MAP()

}  // namespace webrtc

#endif  // API_RTP_RECEIVER_INTERFACE_H_
