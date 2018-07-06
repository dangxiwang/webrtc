/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef CONGESTION_WINDOW_PUSHBACK_CONTROLLER_H
#define CONGESTION_WINDOW_PUSHBACK_CONTROLLER_H

#include "api/transport/network_types.h"
#include "common_types.h"  // NOLINT(build/include)
#include "rtc_base/criticalsection.h"
#include "rtc_base/format_macros.h"

namespace webrtc {

// This class enables pushback from congestion window directly to video encoder.
// When the congestion window is filling up, the video encoder target bitrate
// will be reduced accordingly to accommodate the network changes. To avoid
// pausing video too frequently, a minimum encoder target bitrate threshold is
// used to prevent video pause due to a full congestion window.
class CongestionWindowPushbackController {
 public:
  CongestionWindowPushbackController();
  void UpdateOutstandingData(size_t outstanding_bytes);
  void UpdateMaxOutstandingData(size_t max_outstanding_bytes);
  uint32_t UpdateTargetBitrate(uint32_t bitrate_bps);
  void SetDataWindow(DataSize data_window);

 private:
  rtc::CriticalSection critsect_;
  absl::optional<DataSize> current_data_window_ RTC_GUARDED_BY(critsect_);
  size_t outstanding_bytes_ RTC_GUARDED_BY(critsect_) = 0;
  uint32_t min_pushback_target_bitrate_bps_ RTC_GUARDED_BY(critsect_);
  double encoding_rate_ratio_ RTC_GUARDED_BY(critsect_) = 1.0;
};

}  // namespace webrtc

#endif  // CONGESTION_WINDOW_PUSHBACK_CONTROLLER_H
