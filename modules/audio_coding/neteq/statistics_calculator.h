/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_CODING_NETEQ_STATISTICS_CALCULATOR_H_
#define MODULES_AUDIO_CODING_NETEQ_STATISTICS_CALCULATOR_H_

#include <deque>
#include <string>

#include "modules/audio_coding/neteq/include/neteq.h"
#include "rtc_base/constructormagic.h"

namespace webrtc {

class DelayManager;

// This class handles various network statistics in NetEq.
class StatisticsCalculator {
 public:
  StatisticsCalculator();

  virtual ~StatisticsCalculator();

  // Resets most of the counters.
  void Reset();

  // Resets the counters that are not handled by Reset().
  void ResetMcu();

  // Reports that |num_samples| samples were produced through expansion, and
  // that the expansion produced other than just noise samples.
  void ExpandedVoiceSamples(size_t num_samples, bool is_new_concealment_event);

  // Reports that |num_samples| samples were produced through expansion, and
  // that the expansion produced only noise samples.
  void ExpandedNoiseSamples(size_t num_samples, bool is_new_concealment_event);

  // Corrects the statistics for number of samples produced through non-noise
  // expansion by adding |num_samples| (negative or positive) to the current
  // value. The result is capped to zero to avoid negative values.
  void ExpandedVoiceSamplesCorrection(int num_samples);

  // Same as ExpandedVoiceSamplesCorrection but for noise samples.
  void ExpandedNoiseSamplesCorrection(int num_samples);

  // Reports that |num_samples| samples were produced through preemptive
  // expansion.
  void PreemptiveExpandedSamples(size_t num_samples);

  // Reports that |num_samples| samples were removed through accelerate.
  void AcceleratedSamples(size_t num_samples);

  // Reports that |num_samples| zeros were inserted into the output.
  void AddZeros(size_t num_samples);

  // Reports that |num_packets| packets were discarded.
  virtual void PacketsDiscarded(size_t num_packets);

  // Reports that |num_packets| packets samples were discarded.
  virtual void SecondaryPacketsDiscarded(size_t num_samples);

  // Reports that |num_samples| were lost.
  void LostSamples(size_t num_samples);

  // Increases the report interval counter with |num_samples| at a sample rate
  // of |fs_hz|. This is how the StatisticsCalculator gets notified that current
  // time is increasing.
  void IncreaseCounter(size_t num_samples, int fs_hz);

  // Update jitter buffer delay counter.
  void JitterBufferDelay(size_t num_samples, uint64_t waiting_time_ms);

  // Stores new packet waiting time in waiting time statistics.
  void StoreWaitingTime(int waiting_time_ms);

  // Reports that |num_samples| samples were decoded from secondary packets.
  void SecondaryDecodedSamples(int num_samples);

  // Rerport that the packet buffer was flushed.
  void FlushedPacketBuffer();

  // Logs a delayed packet outage event of |num_samples| expanded at a sample
  // rate of |fs_hz|. A delayed packet outage event is defined as an expand
  // period caused not by an actual packet loss, but by a delayed packet.
  virtual void LogDelayedPacketOutageEvent(int num_samples, int fs_hz);

  // Returns the current network statistics in |stats|. The current sample rate
  // is |fs_hz|, the total number of samples in packet buffer and sync buffer
  // yet to play out is |num_samples_in_buffers|, and the number of samples per
  // packet is |samples_per_packet|. The method does not populate
  // |preferred_buffer_size_ms|, |jitter_peaks_found| or |clockdrift_ppm|; use
  // the PopulateDelayManagerStats method for those.
  void GetNetworkStatistics(int fs_hz,
                            size_t num_samples_in_buffers,
                            size_t samples_per_packet,
                            NetEqNetworkStatistics* stats);

  // Populates |preferred_buffer_size_ms|, |jitter_peaks_found| and
  // |clockdrift_ppm| in |stats|. This is a convenience method, and does not
  // strictly have to be in the StatisticsCalculator class, but it makes sense
  // since all other stats fields are populated by that class.
  static void PopulateDelayManagerStats(int ms_per_packet,
                                        const DelayManager& delay_manager,
                                        NetEqNetworkStatistics* stats);

  // Returns a copy of this class's lifetime statistics. These statistics are
  // never reset.
  NetEqLifetimeStatistics GetLifetimeStatistics() const;

  NetEqOperationsAndState GetOperationsAndState() const;

 private:
  static const int kMaxReportPeriod = 60;  // Seconds before auto-reset.
  static const size_t kLenWaitingTimes = 100;

  class PeriodicUmaLogger {
   public:
    PeriodicUmaLogger(const std::string& uma_name,
                      int report_interval_ms,
                      int max_value);
    virtual ~PeriodicUmaLogger();
    void AdvanceClock(int step_ms);

   protected:
    void LogToUma(int value) const;
    virtual int Metric() const = 0;
    virtual void Reset() = 0;

    const std::string uma_name_;
    const int report_interval_ms_;
    const int max_value_;
    int timer_ = 0;
  };

  class PeriodicUmaCount final : public PeriodicUmaLogger {
   public:
    PeriodicUmaCount(const std::string& uma_name,
                     int report_interval_ms,
                     int max_value);
    ~PeriodicUmaCount() override;
    void RegisterSample();

   protected:
    int Metric() const override;
    void Reset() override;

   private:
    int counter_ = 0;
  };

  class PeriodicUmaAverage final : public PeriodicUmaLogger {
   public:
    PeriodicUmaAverage(const std::string& uma_name,
                       int report_interval_ms,
                       int max_value);
    ~PeriodicUmaAverage() override;
    void RegisterSample(int value);

   protected:
    int Metric() const override;
    void Reset() override;

   private:
    double sum_ = 0.0;
    int counter_ = 0;
  };

  // Corrects the concealed samples counter in lifetime_stats_. The value of
  // num_samples_ is added directly to the stat if the correction is positive.
  // If the correction is negative, it is cached and will be subtracted against
  // future additions to the counter. This is meant to be called from
  // Expanded{Voice,Noise}Samples{Correction}.
  void ConcealedSamplesCorrection(int num_samples, bool is_voice);

  // Calculates numerator / denominator, and returns the value in Q14.
  static uint16_t CalculateQ14Ratio(size_t numerator, uint32_t denominator);

  NetEqLifetimeStatistics lifetime_stats_;
  NetEqOperationsAndState operations_and_state_;
  size_t concealed_samples_correction_ = 0;
  size_t voice_concealed_samples_correction_ = 0;
  size_t preemptive_samples_;
  size_t accelerate_samples_;
  size_t added_zero_samples_;
  size_t expanded_speech_samples_;
  size_t expanded_noise_samples_;
  size_t discarded_packets_;
  size_t lost_timestamps_;
  uint32_t timestamps_since_last_report_;
  std::deque<int> waiting_times_;
  uint32_t secondary_decoded_samples_;
  size_t discarded_secondary_packets_;
  PeriodicUmaCount delayed_packet_outage_counter_;
  PeriodicUmaAverage excess_buffer_delay_;
  PeriodicUmaCount buffer_full_counter_;

  RTC_DISALLOW_COPY_AND_ASSIGN(StatisticsCalculator);
};

}  // namespace webrtc
#endif  // MODULES_AUDIO_CODING_NETEQ_STATISTICS_CALCULATOR_H_
