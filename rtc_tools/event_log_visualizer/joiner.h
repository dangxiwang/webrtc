/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_TOOLS_EVENT_LOG_VISUALIZER_JOINER_H_
#define RTC_TOOLS_EVENT_LOG_VISUALIZER_JOINER_H_

#include "logging/rtc_event_log/rtc_event_log_parser_new.h"
#include "rtc_tools/event_log_visualizer/plot_base.h"

namespace webrtc {

class EventLogJoiner {
 public:
  EventLogJoiner(const ParsedRtcEventLogNew& log1,
                 const ParsedRtcEventLogNew& log2);

  void CreateIceSequenceGraph(PlotCollection* plot_collection);
  void CreateIceTransactionStateReached(PlotCollection* plot_collection);

  void CreateIceTransactionPlots(PlotCollection* plot_collection);
  void CreateActualIceSequenceDiagram(PlotCollection* plot_collection);

 private:
  static int64_t ToCallTimeUs(int64_t timestamp,
                              const ParsedRtcEventLogNew& log);
  static float ToCallTimeSec(int64_t timestamp,
                             const ParsedRtcEventLogNew& log);

  const ParsedRtcEventLogNew& GetLog(std::size_t index);

  std::map<uint32_t, std::string> log1_configs_;
  std::map<uint32_t, std::string> log2_configs_;

  const ParsedRtcEventLogNew& log1_;
  const ParsedRtcEventLogNew& log2_;
};

}  // namespace webrtc

#endif  // RTC_TOOLS_EVENT_LOG_VISUALIZER_JOINER_H_
