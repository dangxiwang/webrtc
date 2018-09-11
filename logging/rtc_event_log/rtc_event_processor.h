/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef LOGGING_RTC_EVENT_LOG_RTC_EVENT_PROCESSOR_H_
#define LOGGING_RTC_EVENT_LOG_RTC_EVENT_PROCESSOR_H_

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "rtc_base/function_view.h"

namespace webrtc {

class ProcessableEventListInterface {
 public:
  virtual void ProcessFirst() = 0;
  virtual bool IsEmpty() = 0;
  virtual int64_t GetTime() = 0;
  virtual ~ProcessableEventListInterface() = default;
};

template <typename T>
class ProcessableEventList : public ProcessableEventListInterface {
 public:
  // N.B. |f| is not owned by ProcessableEventList. The caller must ensure that
  // the function object or lambda outlives ProcessableEventList and
  // RtcEventProcessor.
  ProcessableEventList(typename std::vector<T>::const_iterator begin,
                       typename std::vector<T>::const_iterator end,
                       rtc::FunctionView<void(const T&)> f)
      : begin_(begin), end_(end), f_(f) {}
  void ProcessFirst() override {
    RTC_DCHECK(!IsEmpty());
    f_(*begin_);
    ++begin_;
  }
  bool IsEmpty() override { return begin_ == end_; }
  int64_t GetTime() override {
    RTC_DCHECK(!IsEmpty());
    return begin_->log_time_us();
  }

 private:
  typename std::vector<T>::const_iterator begin_;
  typename std::vector<T>::const_iterator end_;
  rtc::FunctionView<void(const T&)> f_;
};

// Helper class used to "merge" two or more lists of ordered RtcEventLog events
// so that they can be treated as a single ordered list. Since the individual
// lists may have different types,
//
// Usage example:
// ParsedRtcEventLogNew log;
// auto incoming_handler = [] (LoggedRtcpPacketIncoming elem) { ... };
// auto incoming_rtcp =
//     absl::make_unique<ProcessableEventList<LoggedRtcpPacketIncoming>>(
//         log.incoming_rtcp_packets().begin(),
//         log.incoming_rtcp_packets().end(),
//         incoming_handler);
// auto outgoing_handler = [] (LoggedRtcpPacketOutgoing elem) { ... };
// auto outgoing_rtcp =
//     absl::make_unique<ProcessableEventList<LoggedRtcpPacketOutgoing>>(
//         log.outgoing_rtcp_packets().begin(),
//         log.outgoing_rtcp_packets().end(),
//         outgoing_handler);
//
// RtcEventProcessor processor;
// processor.AddEvents(std::move(incoming_rtcp));
// processor.AddEvents(std::move(outgoing_rtcp));
// processor.ProcessEventsInOrder();
class RtcEventProcessor {
 public:
  void AddEvents(std::unique_ptr<ProcessableEventListInterface> events) {
    auto cmp = [](const std::unique_ptr<ProcessableEventListInterface>& a,
                  const std::unique_ptr<ProcessableEventListInterface>& b) {
      return a->GetTime() > b->GetTime();
    };
    if (!events->IsEmpty()) {
      event_lists.push_back(std::move(events));
      std::push_heap(event_lists.begin(), event_lists.end(), cmp);
    }
  }
  void ProcessEventsInOrder() {
    auto cmp = [](const std::unique_ptr<ProcessableEventListInterface>& a,
                  const std::unique_ptr<ProcessableEventListInterface>& b) {
      return a->GetTime() > b->GetTime();
    };
    while (!event_lists.empty()) {
      event_lists.front()->ProcessFirst();
      std::pop_heap(event_lists.begin(), event_lists.end(), cmp);
      if (event_lists.back()->IsEmpty()) {
        event_lists.pop_back();
      } else {
        std::push_heap(event_lists.begin(), event_lists.end(), cmp);
      }
    }
  }

 private:
  std::vector<std::unique_ptr<ProcessableEventListInterface>> event_lists;
};

}  // namespace webrtc

#endif  // LOGGING_RTC_EVENT_LOG_RTC_EVENT_PROCESSOR_H_
