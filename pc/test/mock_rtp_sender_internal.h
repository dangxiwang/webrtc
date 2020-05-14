/*
 *  Copyright 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef PC_TEST_MOCK_RTP_SENDER_INTERNAL_H_
#define PC_TEST_MOCK_RTP_SENDER_INTERNAL_H_

#include <string>
#include <vector>

#include "pc/rtp_sender.h"
#include "test/gmock.h"

namespace webrtc {

// The definition of MockRtpSender is copied in to avoid multiple inheritance.
class MockRtpSenderInternal : public RtpSenderInternal {
 public:
  // RtpSenderInterface methods.
  MOCK_METHOD(bool, SetTrack, (MediaStreamTrackInterface*), (override));
  MOCK_METHOD(rtc::scoped_refptr<MediaStreamTrackInterface>,
              track,
              (),
              (const, override));
  MOCK_METHOD(uint32_t, ssrc, (), (const, override));
  MOCK_METHOD(rtc::scoped_refptr<DtlsTransportInterface>,
              dtls_transport,
              (),
              (const, override));
  MOCK_METHOD(cricket::MediaType, media_type, (), (const, override));
  MOCK_METHOD(std::string, id, (), (const, override));
  MOCK_METHOD(std::vector<std::string>, stream_ids, (), (const, override));
  MOCK_METHOD(std::vector<RtpEncodingParameters>,
              init_send_encodings,
              (),
              (const, override));
  MOCK_METHOD(void,
              set_transport,
              (rtc::scoped_refptr<DtlsTransportInterface>),
              (override));
  MOCK_METHOD(RtpParameters, GetParameters, (), (const, override));
  MOCK_METHOD(RtpParameters, GetParametersInternal, (), (const, override));
  MOCK_METHOD(RTCError, SetParameters, (const RtpParameters&), (override));
  MOCK_METHOD(RTCError,
              SetParametersInternal,
              (const RtpParameters&),
              (override));
  MOCK_METHOD(rtc::scoped_refptr<DtmfSenderInterface>,
              GetDtmfSender,
              (),
              (const, override));
  MOCK_METHOD(void,
              SetFrameEncryptor,
              (rtc::scoped_refptr<FrameEncryptorInterface>),
              (override));
  MOCK_METHOD(rtc::scoped_refptr<FrameEncryptorInterface>,
              GetFrameEncryptor,
              (),
              (const, override));

  // RtpSenderInternal methods.
  MOCK_METHOD(void, SetMediaChannel, (cricket::MediaChannel*), (override));
  MOCK_METHOD(void, SetSsrc, (uint32_t), (override));
  MOCK_METHOD(void,
              set_stream_ids,
              (const std::vector<std::string>&),
              (override));
  MOCK_METHOD(void, SetStreams, (const std::vector<std::string>&), (override));
  MOCK_METHOD(void,
              set_init_send_encodings,
              (const std::vector<RtpEncodingParameters>&),
              (override));
  MOCK_METHOD(void, Stop, (), (override));
  MOCK_METHOD(int, AttachmentId, (), (const, override));
  MOCK_METHOD(RTCError,
              DisableEncodingLayers,
              (const std::vector<std::string>&),
              (override));
};

}  // namespace webrtc

#endif  // PC_TEST_MOCK_RTP_SENDER_INTERNAL_H_
