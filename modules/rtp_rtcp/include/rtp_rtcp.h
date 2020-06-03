/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_RTP_RTCP_INCLUDE_RTP_RTCP_H_
#define MODULES_RTP_RTCP_INCLUDE_RTP_RTCP_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/include/module.h"
#include "modules/rtp_rtcp/source/rtp_rtcp_interface.h"
#include "rtc_base/deprecation.h"

namespace webrtc {

// DEPRECATED. Do not use.
class DEPRECATED_RtpRtcp : public Module, public RtpRtcpInterface {
 public:
  // Instantiates a deprecated version of the RtpRtcp module.
  static std::unique_ptr<DEPRECATED_RtpRtcp> RTC_DEPRECATED
  Create(const Configuration& configuration) {
    return DEPRECATED_Create(configuration);
  }

  static std::unique_ptr<DEPRECATED_RtpRtcp> DEPRECATED_Create(
      const Configuration& configuration);

  // (De)registers RTP header extension type and id.
  // Returns -1 on failure else 0.
  RTC_DEPRECATED virtual int32_t RegisterSendRtpHeaderExtension(
      RTPExtensionType type,
      uint8_t id) = 0;

  // Requests new key frame.
  // using PLI, https://tools.ietf.org/html/rfc4585#section-6.3.1.1
  void SendPictureLossIndication() { SendRTCP(kRtcpPli); }
  // using FIR, https://tools.ietf.org/html/rfc5104#section-4.3.1.2
  void SendFullIntraRequest() { SendRTCP(kRtcpFir); }
};

typedef DEPRECATED_RtpRtcp RtpRtcp;

}  // namespace webrtc

#endif  // MODULES_RTP_RTCP_INCLUDE_RTP_RTCP_H_
