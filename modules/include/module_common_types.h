/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_INCLUDE_MODULE_COMMON_TYPES_H_
#define MODULES_INCLUDE_MODULE_COMMON_TYPES_H_

#include <stddef.h>
#include <stdint.h>
#include <vector>

#include "api/rtp_headers.h"
#include "api/video/video_frame_type.h"
#include "modules/include/module_common_types_public.h"
#include "modules/include/module_fec_types.h"
#include "modules/rtp_rtcp/source/rtp_video_header.h"
#include "rtc_base/system/rtc_export.h"

namespace webrtc {

class RTC_EXPORT RTPFragmentationHeader {
 public:
  RTPFragmentationHeader();
  RTPFragmentationHeader(const RTPFragmentationHeader&) = delete;
  RTPFragmentationHeader(RTPFragmentationHeader&& other);
  RTPFragmentationHeader& operator=(const RTPFragmentationHeader& other) =
      delete;
  RTPFragmentationHeader& operator=(RTPFragmentationHeader&& other);
  ~RTPFragmentationHeader();

  friend void swap(RTPFragmentationHeader& a, RTPFragmentationHeader& b);

  void CopyFrom(const RTPFragmentationHeader& src);
  void VerifyAndAllocateFragmentationHeader(size_t size) { Resize(size); }

  void Resize(size_t size);
  size_t Size() const { return fragmentationVectorSize; }

  size_t Offset(size_t index) const { return fragmentationOffset[index]; }
  size_t Length(size_t index) const { return fragmentationLength[index]; }

  // TODO(danilchap): Move all members to private section,
  // simplify by replacing 4 raw arrays with single std::vector<Fragment>
  uint16_t fragmentationVectorSize;  // Number of fragmentations
  size_t* fragmentationOffset;       // Offset of pointer to data for each
                                     // fragmentation
  size_t* fragmentationLength;       // Data size for each fragmentation
};

// Interface used by the CallStats class to distribute call statistics.
// Callbacks will be triggered as soon as the class has been registered to a
// CallStats object using RegisterStatsObserver.
class CallStatsObserver {
 public:
  virtual void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) = 0;

  virtual ~CallStatsObserver() {}
};

// Interface used by NackModule and JitterBuffer.
class NackSender {
 public:
  virtual void SendNack(const std::vector<uint16_t>& sequence_numbers) = 0;

 protected:
  virtual ~NackSender() {}
};

// Interface used by NackModule and JitterBuffer.
class KeyFrameRequestSender {
 public:
  virtual void RequestKeyFrame() = 0;

 protected:
  virtual ~KeyFrameRequestSender() {}
};

// Interface used by LossNotificationController to communicate to RtpRtcp.
// TODO(bugs.webrtc.org/10336): Hook up to RtpRtcp.
class LossNotificationSender {
 public:
  virtual ~LossNotificationSender() {}

  virtual void SendLossNotification(uint16_t last_decoded_seq_num,
                                    uint16_t last_received_seq_num,
                                    bool decodability_flag) = 0;
};

}  // namespace webrtc

#endif  // MODULES_INCLUDE_MODULE_COMMON_TYPES_H_
