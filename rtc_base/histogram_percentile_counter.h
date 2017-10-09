/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_HISTOGRAM_PERCENTILE_COUNTER_H_
#define RTC_BASE_HISTOGRAM_PERCENTILE_COUNTER_H_

#include <stdint.h>
#include <map>
#include <vector>

#include "api/optional.h"

namespace rtc {
class HistogramPercentileCounter {
 public:
  // Values below |long_tail_boundary| are stored in the array.
  // Values above - in the map.
  explicit HistogramPercentileCounter(size_t long_tail_boundary);
  void Add(uint32_t value);
  void Add(uint32_t value, size_t count);
  void Add(const HistogramPercentileCounter& other);
  // Argument should be from 0 to 1.
  rtc::Optional<uint32_t> GetPercentile(float fraction);
  ~HistogramPercentileCounter();

 private:
  std::vector<size_t> histogram_low_;
  std::map<uint32_t, size_t> histogram_high_;
  const size_t long_tail_boundary_;
  size_t total_elements_;
  size_t total_elements_low_;
};
}  // namespace rtc
#endif  // RTC_BASE_HISTOGRAM_PERCENTILE_COUNTER_H_
