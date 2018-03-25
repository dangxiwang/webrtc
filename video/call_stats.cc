/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "video/call_stats.h"

#include <algorithm>

#include "modules/utility/include/process_thread.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructormagic.h"
#include "rtc_base/location.h"
#include "rtc_base/logging.h"
#include "rtc_base/task_queue.h"
#include "system_wrappers/include/metrics.h"

namespace webrtc {
namespace {

void RemoveOldReports(int64_t now, std::list<CallStats::RttTime>* reports) {
  static constexpr const int64_t kRttTimeoutMs = 1500;
  reports->remove_if(
      [&now](CallStats::RttTime& r) { return now - r.time > kRttTimeoutMs; });
}

int64_t GetMaxRttMs(const std::list<CallStats::RttTime>& reports) {
  int64_t max_rtt_ms = -1;
  for (const CallStats::RttTime& rtt_time : reports)
    max_rtt_ms = std::max(rtt_time.rtt, max_rtt_ms);
  return max_rtt_ms;
}

int64_t GetAvgRttMs(const std::list<CallStats::RttTime>& reports) {
  RTC_DCHECK(!reports.empty());
  int64_t sum = 0;
  for (std::list<CallStats::RttTime>::const_iterator it = reports.begin();
       it != reports.end(); ++it) {
    sum += it->rtt;
  }
  return sum / reports.size();
}

int64_t GetNewAvgRttMs(const std::list<CallStats::RttTime>& reports,
                       int64_t prev_avg_rtt) {
  if (reports.empty())
    return -1;  // Reset (invalid average).

  int64_t cur_rtt_ms = GetAvgRttMs(reports);
  if (prev_avg_rtt == -1)
    return cur_rtt_ms;  // New initial average value.

  // Weight factor to apply to the average rtt.
  // We weigh the old average at 70% against the new average (30%).
  constexpr const float kWeightFactor = 0.3f;
  return prev_avg_rtt * (1.0f - kWeightFactor) + cur_rtt_ms * kWeightFactor;
}
}  // namespace

class CallStats::TemporaryDeregistration {
 public:
  TemporaryDeregistration(Module* module,
                          ProcessThread* process_thread,
                          bool thread_running)
      : module_(module),
        process_thread_(process_thread),
        deregistered_(thread_running) {
    if (thread_running)
      process_thread_->DeRegisterModule(module_);
  }
  ~TemporaryDeregistration() {
    if (deregistered_)
      process_thread_->RegisterModule(module_, RTC_FROM_HERE);
  }

 private:
  Module* const module_;
  ProcessThread* const process_thread_;
  const bool deregistered_;
};

CallStats::CallStats(Clock* clock, ProcessThread* process_thread)
    : clock_(clock),
      last_process_time_(clock_->TimeInMilliseconds()),
      max_rtt_ms_(-1),
      avg_rtt_ms_(-1),
      sum_avg_rtt_ms_(0),
      num_avg_rtt_(0),
      time_of_first_rtt_ms_(-1),
      process_thread_(process_thread) {
  RTC_DCHECK(process_thread_);
  process_thread_checker_.DetachFromThread();
}

CallStats::~CallStats() {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  RTC_DCHECK(!process_thread_running_);
  RTC_DCHECK(observers_.empty());

  UpdateHistograms();
}

int64_t CallStats::TimeUntilNextProcess() {
  RTC_DCHECK_RUN_ON(&process_thread_checker_);
  return last_process_time_ + kUpdateIntervalMs - clock_->TimeInMilliseconds();
}

void CallStats::Process() {
  RTC_DCHECK_RUN_ON(&process_thread_checker_);
  int64_t now = clock_->TimeInMilliseconds();
  last_process_time_ = now;

  auto avg_rtt_ms = avg_rtt_ms_;
  RemoveOldReports(now, &reports_);
  max_rtt_ms_ = GetMaxRttMs(reports_);
  avg_rtt_ms = GetNewAvgRttMs(reports_, avg_rtt_ms);
  {
    rtc::CritScope lock(&avg_rtt_ms_lock_);
    avg_rtt_ms_ = avg_rtt_ms;
  }

  // If there is a valid rtt, update all observers with the max rtt.
  if (max_rtt_ms_ >= 0) {
    RTC_DCHECK_GE(avg_rtt_ms, 0);
    for (CallStatsObserver* observer : observers_)
      observer->OnRttUpdate(avg_rtt_ms, max_rtt_ms_);
    // Sum for Histogram of average RTT reported over the entire call.
    sum_avg_rtt_ms_ += avg_rtt_ms;
    ++num_avg_rtt_;
  }
}

void CallStats::ProcessThreadAttached(ProcessThread* process_thread) {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  RTC_DCHECK(!process_thread || process_thread_ == process_thread);
  process_thread_running_ = process_thread != nullptr;

  // Whether we just got attached or detached, we clear the
  // |process_thread_checker_| so that it can be used to protect variables
  // in either the process thread when it starts again, or UpdateHistograms()
  // (mutually exclusive).
  process_thread_checker_.DetachFromThread();
}

void CallStats::RegisterStatsObserver(CallStatsObserver* observer) {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  TemporaryDeregistration deregister(this, process_thread_,
                                     process_thread_running_);

  auto it = std::find(observers_.begin(), observers_.end(), observer);
  if (it == observers_.end())
    observers_.push_back(observer);
}

void CallStats::DeregisterStatsObserver(CallStatsObserver* observer) {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  TemporaryDeregistration deregister(this, process_thread_,
                                     process_thread_running_);
  observers_.remove(observer);
}

int64_t CallStats::LastProcessedRtt() const {
  rtc::CritScope cs(&avg_rtt_ms_lock_);
  return avg_rtt_ms_;
}

void CallStats::OnRttUpdate(int64_t rtt) {
  int64_t now_ms = clock_->TimeInMilliseconds();
  process_thread_->PostTask(rtc::NewClosure([rtt, now_ms, this]() {
    RTC_DCHECK_RUN_ON(&process_thread_checker_);
    reports_.push_back(RttTime(rtt, now_ms));
    if (time_of_first_rtt_ms_ == -1)
      time_of_first_rtt_ms_ = now_ms;

    process_thread_->WakeUp(this);
  }));
}

void CallStats::UpdateHistograms() {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  RTC_DCHECK(!process_thread_running_);

  // The extra scope is because we have two 'dcheck run on' thread checkers.
  // This is a special case since it's safe to access variables on the current
  // thread that normally are only touched on the process thread.
  // Since we're not attached to the process thread and/or the process thread
  // isn't running, it's OK to touch these variables here.
  {
    // This method is called on the ctor thread (usually from the dtor, unless
    // a test calls it). It's a requirement that the function be called when
    // the process thread is not running (a condition that's met at destruction
    // time), and thanks to that, we don't need a lock to synchronize against
    // it.
    RTC_DCHECK_RUN_ON(&process_thread_checker_);

    if (time_of_first_rtt_ms_ == -1 || num_avg_rtt_ < 1)
      return;

    int64_t elapsed_sec =
        (clock_->TimeInMilliseconds() - time_of_first_rtt_ms_) / 1000;
    if (elapsed_sec >= metrics::kMinRunTimeInSeconds) {
      int64_t avg_rtt_ms = (sum_avg_rtt_ms_ + num_avg_rtt_ / 2) / num_avg_rtt_;
      RTC_HISTOGRAM_COUNTS_10000(
          "WebRTC.Video.AverageRoundTripTimeInMilliseconds", avg_rtt_ms);
    }
  }
}

}  // namespace webrtc
