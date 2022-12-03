/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef LOGGING_RTC_EVENT_LOG_RTC_EVENT_LOG_IMPL_H_
#define LOGGING_RTC_EVENT_LOG_RTC_EVENT_LOG_IMPL_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "api/rtc_event_log/rtc_event.h"
#include "api/rtc_event_log/rtc_event_log.h"
#include "api/rtc_event_log_output.h"
#include "api/sequence_checker.h"
#include "api/task_queue/task_queue_base.h"
#include "api/task_queue/task_queue_factory.h"
#include "logging/rtc_event_log/encoder/rtc_event_log_encoder.h"
#include "rtc_base/system/no_unique_address.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/task_utils/inline_task_queue.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

class RtcEventLogImpl final : public RtcEventLog {
 public:
  RtcEventLogImpl(EncodingType encoding_type,
                  TaskQueueFactory* task_queue_factory);
  RtcEventLogImpl(const RtcEventLogImpl&) = delete;
  RtcEventLogImpl& operator=(const RtcEventLogImpl&) = delete;

  ~RtcEventLogImpl() override;

  // TODO(eladalon): We should change these name to reflect that what we're
  // actually starting/stopping is the output of the log, not the log itself.
  bool StartLogging(std::unique_ptr<RtcEventLogOutput> output,
                    int64_t output_period_ms) override;
  void StopLogging() override;
  void StopLogging(std::function<void()> callback) override;

  void Log(std::unique_ptr<RtcEvent> event) override;

 private:
  void LogToMemory(std::unique_ptr<RtcEvent> event)
      RTC_RUN_ON(sequence_checker_);
  void LogEventsFromMemoryToOutput() RTC_RUN_ON(sequence_checker_);

  void StopOutput() RTC_RUN_ON(sequence_checker_);

  void WriteConfigsAndHistoryToOutput(absl::string_view encoded_configs,
                                      absl::string_view encoded_history)
      RTC_RUN_ON(sequence_checker_);
  void WriteToOutput(absl::string_view output_string)
      RTC_RUN_ON(sequence_checker_);

  void StopLoggingInternal() RTC_RUN_ON(sequence_checker_);

  void ScheduleOutput(TaskQueueBase* task_queue) RTC_RUN_ON(sequence_checker_);

  // History containing all past configuration events.
  std::deque<std::unique_ptr<RtcEvent>> config_history_
      RTC_GUARDED_BY(sequence_checker_);

  // History containing the most recent (non-configuration) events (~10s).
  std::deque<std::unique_ptr<RtcEvent>> history_
      RTC_GUARDED_BY(sequence_checker_);

  std::unique_ptr<RtcEventLogEncoder> event_encoder_
      RTC_GUARDED_BY(sequence_checker_);
  std::unique_ptr<RtcEventLogOutput> event_output_
      RTC_GUARDED_BY(sequence_checker_);

  size_t num_config_events_written_ RTC_GUARDED_BY(sequence_checker_);
  absl::optional<int64_t> output_period_ms_ RTC_GUARDED_BY(sequence_checker_);
  int64_t last_output_ms_ RTC_GUARDED_BY(sequence_checker_);
  bool output_scheduled_ RTC_GUARDED_BY(sequence_checker_);

  RTC_NO_UNIQUE_ADDRESS SequenceChecker logging_state_checker_;
  bool logging_state_started_ RTC_GUARDED_BY(logging_state_checker_);

  RTC_NO_UNIQUE_ADDRESS SequenceChecker sequence_checker_;

  // Since we are posting tasks bound to `this`,  it is critical that the event
  // log and its members outlive `task_queue_`. Keep the `task_queue_`
  // last to ensure it destructs first, or else tasks living on the queue might
  // access other members after they've been torn down.
  InlineTaskQueue task_queue_;
};

}  // namespace webrtc

#endif  //  LOGGING_RTC_EVENT_LOG_RTC_EVENT_LOG_IMPL_H_
