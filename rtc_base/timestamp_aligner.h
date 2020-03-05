/*
 *  Copyright (c) 2016 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_TIMESTAMP_ALIGNER_H_
#define RTC_BASE_TIMESTAMP_ALIGNER_H_

#include <stdint.h>

#include "rtc_base/constructor_magic.h"
#include "rtc_base/system/rtc_export.h"

namespace rtc {

// The TimestampAligner class helps translating timestamps of a capture system
// into the same timescale as is used by rtc::TimeMicros(). Some capture systems
// provide timestamps, which comes from the capturing hardware (camera or sound
// card) or stamped close to the capturing hardware. Such timestamps are more
// accurate (less jittery) than reading the system clock, but may have a
// different epoch and unknown clock drift. Frame timestamps in webrtc should
// use rtc::TimeMicros (system monotonic time), and this class provides a filter
// which lets us use the rtc::TimeMicros timescale, and at the same time take
// advantage of higher accuracy of the capturer's clock.

// This class is not thread safe, so all calls to it must be synchronized
// externally.
class RTC_EXPORT TimestampAligner {
 public:
  TimestampAligner();
  ~TimestampAligner();

 public:
  // Translates timestamps of a capture system to the same timescale as is used
  // by rtc::TimeMicros(). |capturer_time_us| is assumed to be accurate, but
  // with an unknown epoch and clock drift. |system_time_us| is
  // time according to rtc::TimeMicros(), preferably read as soon as
  // possible when the frame is captured. It may have poor accuracy
  // due to poor resolution or scheduling delays. Returns the
  // translated timestamp.
  int64_t TranslateTimestamp(int64_t capturer_time_us, int64_t system_time_us);

  // Returns the translated timestamp without updating the states.
  int64_t TranslateTimestamp(int64_t capturer_time_us) const;

 protected:
  // Update the estimated offset between capturer's time and system monotonic
  // time.
  int64_t UpdateOffset(int64_t capturer_time_us, int64_t system_time_us);

  // Clip timestamp, return value is always
  //    <= |system_time_us|, and
  //    >= min(|prev_translated_time_us_| + |kMinFrameIntervalUs|,
  //           |system_time_us|).
  int64_t ClipTimestamp(int64_t filtered_time_us, int64_t system_time_us);

 private:
  // State for the timestamp translation.
  int frames_seen_;
  // Estimated offset between capturer's time and system monotonic time.
  int64_t offset_us_;

  // State for the ClipTimestamp method, applied after the filter.
  // A large negative clock drift of the capturer tends to push translated
  // timestamps into the future. |clip_bias_us_| is subtracted from the
  // translated timestamps, to get them back from the future.
  int64_t clip_bias_us_;
  // Used to ensure that translated timestamps are monotonous.
  int64_t prev_translated_time_us_;
  RTC_DISALLOW_COPY_AND_ASSIGN(TimestampAligner);
};

}  // namespace rtc

#endif  // RTC_BASE_TIMESTAMP_ALIGNER_H_
