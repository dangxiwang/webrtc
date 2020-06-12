/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef VIDEO_ADAPTATION_VIDEO_STREAM_ENCODER_RESOURCE_MANAGER_H_
#define VIDEO_ADAPTATION_VIDEO_STREAM_ENCODER_RESOURCE_MANAGER_H_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "api/adaptation/resource.h"
#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "api/task_queue/task_queue_base.h"
#include "api/video/video_adaptation_counters.h"
#include "api/video/video_adaptation_reason.h"
#include "api/video/video_frame.h"
#include "api/video/video_source_interface.h"
#include "api/video/video_stream_encoder_observer.h"
#include "api/video_codecs/video_codec.h"
#include "api/video_codecs/video_encoder.h"
#include "api/video_codecs/video_encoder_config.h"
#include "call/adaptation/resource_adaptation_processor_interface.h"
#include "call/adaptation/video_stream_adapter.h"
#include "call/adaptation/video_stream_input_state_provider.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/experiments/quality_scaler_settings.h"
#include "rtc_base/ref_count.h"
#include "rtc_base/strings/string_builder.h"
#include "rtc_base/task_queue.h"
#include "system_wrappers/include/clock.h"
#include "video/adaptation/encode_usage_resource.h"
#include "video/adaptation/overuse_frame_detector.h"
#include "video/adaptation/quality_rampup_experiment_helper.h"
#include "video/adaptation/quality_scaler_resource.h"
#include "video/adaptation/video_stream_encoder_resource.h"

namespace webrtc {

// The assumed input frame size if we have not yet received a frame.
// TODO(hbos): This is 144p - why are we assuming super low quality? Seems like
// a bad heuristic.
extern const int kDefaultInputPixelsWidth;
extern const int kDefaultInputPixelsHeight;

// Owns adaptation-related Resources pertaining to a single VideoStreamEncoder
// and passes on the relevant input from the encoder to the resources. The
// resources provide resource usage states to the ResourceAdaptationProcessor
// which is responsible for reconfiguring streams in order not to overuse
// resources.
//
// The manager is also involved with various mitigations not part of the
// ResourceAdaptationProcessor code such as the inital frame dropping.
class VideoStreamEncoderResourceManager
    : public VideoSourceRestrictionsListener,
      public ResourceLimitationsListener,
      public QualityRampUpExperimentListener {
 public:
  VideoStreamEncoderResourceManager(
      VideoStreamInputStateProvider* input_state_provider,
      VideoStreamEncoderObserver* encoder_stats_observer,
      Clock* clock,
      bool experiment_cpu_load_estimator,
      std::unique_ptr<OveruseFrameDetector> overuse_detector);
  ~VideoStreamEncoderResourceManager() override;

  void Initialize(rtc::TaskQueue* encoder_queue,
                  rtc::TaskQueue* resource_adaptation_queue);
  void SetAdaptationProcessor(
      ResourceAdaptationProcessorInterface* adaptation_processor);

  // TODO(https://crbug.com/webrtc/11563): The degradation preference is a
  // setting of the Processor, it does not belong to the Manager - can we get
  // rid of this?
  void SetDegradationPreferences(DegradationPreference degradation_preference);
  DegradationPreference degradation_preference() const;

  // Starts the encode usage resource. The quality scaler resource is
  // automatically started on being configured.
  void StartEncodeUsageResource();
  // Stops the encode usage and quality scaler resources if not already stopped.
  void StopManagedResources();

  // Settings that affect the VideoStreamEncoder-specific resources.
  void SetEncoderSettings(EncoderSettings encoder_settings);
  void SetStartBitrate(DataRate start_bitrate);
  void SetTargetBitrate(DataRate target_bitrate);
  void SetEncoderRates(
      const VideoEncoder::RateControlParameters& encoder_rates);
  // TODO(https://crbug.com/webrtc/11338): This can be made private if we
  // configure on SetDegredationPreference and SetEncoderSettings.
  void ConfigureQualityScaler(const VideoEncoder::EncoderInfo& encoder_info);

  // Methods corresponding to different points in the encoding pipeline.
  void OnFrameDroppedDueToSize();
  void OnMaybeEncodeFrame();
  void OnEncodeStarted(const VideoFrame& cropped_frame,
                       int64_t time_when_first_seen_us);
  void OnEncodeCompleted(const EncodedImage& encoded_image,
                         int64_t time_sent_in_us,
                         absl::optional<int> encode_duration_us);
  void OnFrameDropped(EncodedImageCallback::DropReason reason);

  // Resources need to be mapped to an AdaptReason (kCpu or kQuality) in order
  // to update Legacy getStats().
  void MapResourceToReason(rtc::scoped_refptr<Resource> resource,
                           VideoAdaptationReason reason);
  std::vector<rtc::scoped_refptr<Resource>> MappedResources() const;
  std::vector<AdaptationConstraint*> AdaptationConstraints() const;
  std::vector<AdaptationListener*> AdaptationListeners() const;
  rtc::scoped_refptr<QualityScalerResource>
  quality_scaler_resource_for_testing();
  // If true, the VideoStreamEncoder should eexecute its logic to maybe drop
  // frames baseed on size and bitrate.
  bool DropInitialFrames() const;

  // VideoSourceRestrictionsListener implementation.
  // Updates |video_source_restrictions_|.
  void OnVideoSourceRestrictionsUpdated(
      VideoSourceRestrictions restrictions,
      const VideoAdaptationCounters& adaptation_counters,
      rtc::scoped_refptr<Resource> reason,
      const VideoSourceRestrictions& unfiltered_restrictions) override;
  void OnResourceLimitationChanged(
      rtc::scoped_refptr<Resource> resource,
      const std::map<rtc::scoped_refptr<Resource>, VideoAdaptationCounters>&
          resource_limitations) override;

  // QualityRampUpExperimentListener implementation.
  void OnQualityRampUp() override;

 private:
  class InitialFrameDropper;

  VideoAdaptationReason GetReasonFromResource(
      rtc::scoped_refptr<Resource> resource) const;

  CpuOveruseOptions GetCpuOveruseOptions() const;
  int LastInputFrameSizeOrDefault() const;

  // Calculates an up-to-date value of the target frame rate and informs the
  // |encode_usage_resource_| of the new value.
  void MaybeUpdateTargetFrameRate();

  // Use nullopt to disable quality scaling.
  void UpdateQualityScalerSettings(
      absl::optional<VideoEncoder::QpThresholds> qp_thresholds);

  void UpdateStatsAdaptationSettings() const;

  static std::string ActiveCountsToString(
      const std::map<VideoAdaptationReason, VideoAdaptationCounters>&
          active_counts);

  // TODO(hbos): Add tests for manager's constraints.
  // Does not trigger adaptations, only prevents adapting up resolution.
  class BitrateConstraint : public rtc::RefCountInterface,
                            public AdaptationConstraint {
   public:
    explicit BitrateConstraint(VideoStreamEncoderResourceManager* manager);
    ~BitrateConstraint() override = default;

    void SetAdaptationQueue(TaskQueueBase* resource_adaptation_queue);
    void OnEncoderSettingsUpdated(
        absl::optional<EncoderSettings> encoder_settings);
    void OnEncoderTargetBitrateUpdated(
        absl::optional<uint32_t> encoder_target_bitrate_bps);

    // AdaptationConstraint implementation.
    std::string Name() const override { return "BitrateConstraint"; }
    bool IsAdaptationUpAllowed(
        const VideoStreamInputState& input_state,
        const VideoSourceRestrictions& restrictions_before,
        const VideoSourceRestrictions& restrictions_after,
        rtc::scoped_refptr<Resource> reason_resource) const override;

   private:
    // The |manager_| must be alive as long as this resource is added to the
    // ResourceAdaptationProcessor, i.e. when IsAdaptationUpAllowed() is called.
    VideoStreamEncoderResourceManager* const manager_;
    TaskQueueBase* resource_adaptation_queue_;
    absl::optional<EncoderSettings> encoder_settings_
        RTC_GUARDED_BY(resource_adaptation_queue_);
    absl::optional<uint32_t> encoder_target_bitrate_bps_
        RTC_GUARDED_BY(resource_adaptation_queue_);
  };

  // Does not trigger adaptations, only prevents adapting up in BALANCED.
  class BalancedConstraint : public rtc::RefCountInterface,
                             public AdaptationConstraint {
   public:
    explicit BalancedConstraint(VideoStreamEncoderResourceManager* manager);
    ~BalancedConstraint() override = default;

    void SetAdaptationQueue(TaskQueueBase* resource_adaptation_queue);
    void SetAdaptationProcessor(
        ResourceAdaptationProcessorInterface* adaptation_processor);
    void OnEncoderTargetBitrateUpdated(
        absl::optional<uint32_t> encoder_target_bitrate_bps);

    // AdaptationConstraint implementation.
    std::string Name() const override { return "BalancedConstraint"; }
    bool IsAdaptationUpAllowed(
        const VideoStreamInputState& input_state,
        const VideoSourceRestrictions& restrictions_before,
        const VideoSourceRestrictions& restrictions_after,
        rtc::scoped_refptr<Resource> reason_resource) const override;

   private:
    // The |manager_| must be alive as long as this resource is added to the
    // ResourceAdaptationProcessor, i.e. when IsAdaptationUpAllowed() is called.
    VideoStreamEncoderResourceManager* const manager_;
    TaskQueueBase* resource_adaptation_queue_;
    ResourceAdaptationProcessorInterface* adaptation_processor_
        RTC_GUARDED_BY(resource_adaptation_queue_);
    absl::optional<uint32_t> encoder_target_bitrate_bps_
        RTC_GUARDED_BY(resource_adaptation_queue_);
  };

  const rtc::scoped_refptr<BitrateConstraint> bitrate_constraint_;
  const rtc::scoped_refptr<BalancedConstraint> balanced_constraint_;
  const rtc::scoped_refptr<EncodeUsageResource> encode_usage_resource_;
  const rtc::scoped_refptr<QualityScalerResource> quality_scaler_resource_;

  rtc::TaskQueue* encoder_queue_;
  rtc::TaskQueue* resource_adaptation_queue_;
  VideoStreamInputStateProvider* const input_state_provider_
      RTC_GUARDED_BY(encoder_queue_);
  ResourceAdaptationProcessorInterface* adaptation_processor_
      RTC_GUARDED_BY(resource_adaptation_queue_);
  // Thread-safe.
  VideoStreamEncoderObserver* const encoder_stats_observer_;

  DegradationPreference degradation_preference_ RTC_GUARDED_BY(encoder_queue_);
  VideoSourceRestrictions video_source_restrictions_
      RTC_GUARDED_BY(encoder_queue_);

  const BalancedDegradationSettings balanced_settings_;
  Clock* clock_ RTC_GUARDED_BY(encoder_queue_);
  const bool experiment_cpu_load_estimator_ RTC_GUARDED_BY(encoder_queue_);
  const std::unique_ptr<InitialFrameDropper> initial_frame_dropper_
      RTC_GUARDED_BY(encoder_queue_);
  const bool quality_scaling_experiment_enabled_ RTC_GUARDED_BY(encoder_queue_);
  absl::optional<uint32_t> encoder_target_bitrate_bps_
      RTC_GUARDED_BY(encoder_queue_);
  absl::optional<VideoEncoder::RateControlParameters> encoder_rates_
      RTC_GUARDED_BY(encoder_queue_);
  std::unique_ptr<QualityRampUpExperimentHelper> quality_rampup_experiment_
      RTC_GUARDED_BY(encoder_queue_);
  absl::optional<EncoderSettings> encoder_settings_
      RTC_GUARDED_BY(encoder_queue_);

  // Ties a resource to a reason for statistical reporting. This AdaptReason is
  // also used by this module to make decisions about how to adapt up/down.
  struct ResourceAndReason {
    ResourceAndReason(rtc::scoped_refptr<Resource> resource,
                      VideoAdaptationReason reason)
        : resource(resource), reason(reason) {}
    virtual ~ResourceAndReason() = default;

    const rtc::scoped_refptr<Resource> resource;
    const VideoAdaptationReason reason;
  };
  rtc::CriticalSection resource_lock_;
  std::vector<ResourceAndReason> resources_ RTC_GUARDED_BY(&resource_lock_);
};

}  // namespace webrtc

#endif  // VIDEO_ADAPTATION_VIDEO_STREAM_ENCODER_RESOURCE_MANAGER_H_
