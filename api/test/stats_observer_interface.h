/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_TEST_STATS_OBSERVER_INTERFACE_H_
#define API_TEST_STATS_OBSERVER_INTERFACE_H_

#include <string>

#include "api/stats/rtc_stats_report.h"
#include "api/stats_types.h"

namespace webrtc {
namespace webrtc_pc_e2e {

// API is in development and can be changed without notice.
class StatsObserverInterface {
 public:
  virtual ~StatsObserverInterface() = default;

  // Method called when stats reports are available for the PeerConnection
  // identified by |pc_label|.
  virtual void OnStatsReports(const std::string& pc_label,
                              const StatsReports& reports) = 0;
  virtual void OnStatsReports(
      const std::string& pc_label,
      const rtc::scoped_refptr<const RTCStatsReport>& report) {}
};

}  // namespace webrtc_pc_e2e
}  // namespace webrtc

#endif  // API_TEST_STATS_OBSERVER_INTERFACE_H_
