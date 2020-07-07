/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "video/adaptation/video_stream_encoder_resource_manager.h"

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/base/macros.h"
#include "api/adaptation/resource.h"
#include "api/task_queue/task_queue_base.h"
#include "api/video/video_adaptation_reason.h"
#include "api/video/video_source_interface.h"
#include "call/adaptation/video_source_restrictions.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/safe_conversions.h"
#include "rtc_base/ref_counted_object.h"
#include "rtc_base/strings/string_builder.h"
#include "rtc_base/time_utils.h"

namespace webrtc {

const int kDefaultInputPixelsWidth = 176;
const int kDefaultInputPixelsHeight = 144;

namespace {

bool IsResolutionScalingEnabled(DegradationPreference degradation_preference) {
  return degradation_preference == DegradationPreference::MAINTAIN_FRAMERATE ||
         degradation_preference == DegradationPreference::BALANCED;
}

bool IsFramerateScalingEnabled(DegradationPreference degradation_preference) {
  return degradation_preference == DegradationPreference::MAINTAIN_RESOLUTION ||
         degradation_preference == DegradationPreference::BALANCED;
}

std::string ToString(VideoAdaptationReason reason) {
  switch (reason) {
    case VideoAdaptationReason::kQuality:
      return "quality";
    case VideoAdaptationReason::kCpu:
      return "cpu";
  }
}

}  // namespace

class VideoStreamEncoderResourceManager::InitialFrameDropper {
 public:
  explicit InitialFrameDropper(
      rtc::scoped_refptr<QualityScalerResource> quality_scaler_resource)
      : quality_scaler_resource_(quality_scaler_resource),
        quality_scaler_settings_(QualityScalerSettings::ParseFromFieldTrials()),
        has_seen_first_bwe_drop_(false),
        set_start_bitrate_(DataRate::Zero()),
        set_start_bitrate_time_ms_(0),
        initial_framedrop_(0) {
    RTC_DCHECK(quality_scaler_resource_);
  }

  // Output signal.
  bool DropInitialFrames() const {
    return initial_framedrop_ < kMaxInitialFramedrop;
  }

  // Input signals.
  void SetStartBitrate(DataRate start_bitrate, int64_t now_ms) {
    set_start_bitrate_ = start_bitrate;
    set_start_bitrate_time_ms_ = now_ms;
  }

  void SetTargetBitrate(DataRate target_bitrate, int64_t now_ms) {
    if (set_start_bitrate_ > DataRate::Zero() && !has_seen_first_bwe_drop_ &&
        quality_scaler_resource_->is_started() &&
        quality_scaler_settings_.InitialBitrateIntervalMs() &&
        quality_scaler_settings_.InitialBitrateFactor()) {
      int64_t diff_ms = now_ms - set_start_bitrate_time_ms_;
      if (diff_ms <
              quality_scaler_settings_.InitialBitrateIntervalMs().value() &&
          (target_bitrate <
           (set_start_bitrate_ *
            quality_scaler_settings_.InitialBitrateFactor().value()))) {
        RTC_LOG(LS_INFO) << "Reset initial_framedrop_. Start bitrate: "
                         << set_start_bitrate_.bps()
                         << ", target bitrate: " << target_bitrate.bps();
        initial_framedrop_ = 0;
        has_seen_first_bwe_drop_ = true;
      }
    }
  }

  void OnFrameDroppedDueToSize() { ++initial_framedrop_; }

  void OnMaybeEncodeFrame() { initial_framedrop_ = kMaxInitialFramedrop; }

  void OnQualityScalerSettingsUpdated() {
    if (quality_scaler_resource_->is_started()) {
      // Restart frame drops due to size.
      initial_framedrop_ = 0;
    } else {
      // Quality scaling disabled so we shouldn't drop initial frames.
      initial_framedrop_ = kMaxInitialFramedrop;
    }
  }

 private:
  // The maximum number of frames to drop at beginning of stream to try and
  // achieve desired bitrate.
  static const int kMaxInitialFramedrop = 4;

  const rtc::scoped_refptr<QualityScalerResource> quality_scaler_resource_;
  const QualityScalerSettings quality_scaler_settings_;
  bool has_seen_first_bwe_drop_;
  DataRate set_start_bitrate_;
  int64_t set_start_bitrate_time_ms_;
  // Counts how many frames we've dropped in the initial framedrop phase.
  int initial_framedrop_;
};

VideoStreamEncoderResourceManager::BitrateConstraint::BitrateConstraint(
    VideoStreamEncoderResourceManager* manager)
    : manager_(manager),
      resource_adaptation_queue_(nullptr),
      encoder_settings_(absl::nullopt),
      encoder_target_bitrate_bps_(absl::nullopt) {}

void VideoStreamEncoderResourceManager::BitrateConstraint::SetAdaptationQueue(
    TaskQueueBase* resource_adaptation_queue) {
  resource_adaptation_queue_ = resource_adaptation_queue;
}

void VideoStreamEncoderResourceManager::BitrateConstraint::
    OnEncoderSettingsUpdated(absl::optional<EncoderSettings> encoder_settings) {
  RTC_DCHECK_RUN_ON(manager_->encoder_queue_);
  resource_adaptation_queue_->PostTask(
      ToQueuedTask([this_ref = rtc::scoped_refptr<BitrateConstraint>(this),
                    encoder_settings] {
        RTC_DCHECK_RUN_ON(this_ref->resource_adaptation_queue_);
        this_ref->encoder_settings_ = std::move(encoder_settings);
      }));
}

void VideoStreamEncoderResourceManager::BitrateConstraint::
    OnEncoderTargetBitrateUpdated(
        absl::optional<uint32_t> encoder_target_bitrate_bps) {
  RTC_DCHECK_RUN_ON(manager_->encoder_queue_);
  resource_adaptation_queue_->PostTask(
      ToQueuedTask([this_ref = rtc::scoped_refptr<BitrateConstraint>(this),
                    encoder_target_bitrate_bps] {
        RTC_DCHECK_RUN_ON(this_ref->resource_adaptation_queue_);
        this_ref->encoder_target_bitrate_bps_ = encoder_target_bitrate_bps;
      }));
}

bool VideoStreamEncoderResourceManager::BitrateConstraint::
    IsAdaptationUpAllowed(const VideoStreamInputState& input_state,
                          const VideoSourceRestrictions& restrictions_before,
                          const VideoSourceRestrictions& restrictions_after,
                          rtc::scoped_refptr<Resource> reason_resource) const {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
  VideoAdaptationReason reason =
      manager_->GetReasonFromResource(reason_resource);
  // If increasing resolution due to kQuality, make sure bitrate limits are not
  // violated.
  // TODO(hbos): Why are we allowing violating bitrate constraints if adapting
  // due to CPU? Shouldn't this condition be checked regardless of reason?
  if (reason == VideoAdaptationReason::kQuality &&
      DidIncreaseResolution(restrictions_before, restrictions_after)) {
    uint32_t bitrate_bps = encoder_target_bitrate_bps_.value_or(0);
    absl::optional<VideoEncoder::ResolutionBitrateLimits> bitrate_limits =
        encoder_settings_.has_value()
            ? encoder_settings_->encoder_info()
                  .GetEncoderBitrateLimitsForResolution(
                      // Need some sort of expected resulting pixels to be used
                      // instead of unrestricted.
                      GetHigherResolutionThan(
                          input_state.frame_size_pixels().value()))
            : absl::nullopt;
    if (bitrate_limits.has_value() && bitrate_bps != 0) {
      RTC_DCHECK_GE(bitrate_limits->frame_size_pixels,
                    input_state.frame_size_pixels().value());
      return bitrate_bps >=
             static_cast<uint32_t>(bitrate_limits->min_start_bitrate_bps);
    }
  }
  return true;
}

VideoStreamEncoderResourceManager::BalancedConstraint::BalancedConstraint(
    VideoStreamEncoderResourceManager* manager,
    DegradationPreferenceProvider* degradation_preference_provider)
    : manager_(manager),
      resource_adaptation_queue_(nullptr),
      encoder_target_bitrate_bps_(absl::nullopt),
      degradation_preference_provider_(degradation_preference_provider) {
  RTC_DCHECK(manager_);
  RTC_DCHECK(degradation_preference_provider_);
}

void VideoStreamEncoderResourceManager::BalancedConstraint::SetAdaptationQueue(
    TaskQueueBase* resource_adaptation_queue) {
  resource_adaptation_queue_ = resource_adaptation_queue;
}

void VideoStreamEncoderResourceManager::BalancedConstraint::
    OnEncoderTargetBitrateUpdated(
        absl::optional<uint32_t> encoder_target_bitrate_bps) {
  RTC_DCHECK_RUN_ON(manager_->encoder_queue_);
  resource_adaptation_queue_->PostTask(
      ToQueuedTask([this_ref = rtc::scoped_refptr<BalancedConstraint>(this),
                    encoder_target_bitrate_bps] {
        RTC_DCHECK_RUN_ON(this_ref->resource_adaptation_queue_);
        this_ref->encoder_target_bitrate_bps_ = encoder_target_bitrate_bps;
      }));
}

bool VideoStreamEncoderResourceManager::BalancedConstraint::
    IsAdaptationUpAllowed(const VideoStreamInputState& input_state,
                          const VideoSourceRestrictions& restrictions_before,
                          const VideoSourceRestrictions& restrictions_after,
                          rtc::scoped_refptr<Resource> reason_resource) const {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
  VideoAdaptationReason reason =
      manager_->GetReasonFromResource(reason_resource);
  // Don't adapt if BalancedDegradationSettings applies and determines this will
  // exceed bitrate constraints.
  // TODO(hbos): Why are we allowing violating balanced settings if adapting due
  // CPU? Shouldn't this condition be checked regardless of reason?
  if (reason == VideoAdaptationReason::kQuality &&
      degradation_preference_provider_->degradation_preference() ==
          DegradationPreference::BALANCED &&
      !manager_->balanced_settings_.CanAdaptUp(
          input_state.video_codec_type(),
          input_state.frame_size_pixels().value(),
          encoder_target_bitrate_bps_.value_or(0))) {
    return false;
  }
  if (reason == VideoAdaptationReason::kQuality &&
      DidIncreaseResolution(restrictions_before, restrictions_after) &&
      !manager_->balanced_settings_.CanAdaptUpResolution(
          input_state.video_codec_type(),
          input_state.frame_size_pixels().value(),
          encoder_target_bitrate_bps_.value_or(0))) {
    return false;
  }
  return true;
}

VideoStreamEncoderResourceManager::VideoStreamEncoderResourceManager(
    VideoStreamInputStateProvider* input_state_provider,
    VideoStreamEncoderObserver* encoder_stats_observer,
    Clock* clock,
    bool experiment_cpu_load_estimator,
    std::unique_ptr<OveruseFrameDetector> overuse_detector,
    DegradationPreferenceProvider* degradation_preference_provider)
    : degradation_preference_provider_(degradation_preference_provider),
      bitrate_constraint_(new rtc::RefCountedObject<BitrateConstraint>(this)),
      balanced_constraint_(new rtc::RefCountedObject<BalancedConstraint>(
          this,
          degradation_preference_provider_)),
      encode_usage_resource_(
          EncodeUsageResource::Create(std::move(overuse_detector))),
      quality_scaler_resource_(
          QualityScalerResource::Create(degradation_preference_provider_,
                                        this)),
      encoder_queue_(nullptr),
      resource_adaptation_queue_(nullptr),
      input_state_provider_(input_state_provider),
      adaptation_processor_(nullptr),
      encoder_stats_observer_(encoder_stats_observer),
      degradation_preference_(DegradationPreference::DISABLED),
      video_source_restrictions_(),
      clock_(clock),
      experiment_cpu_load_estimator_(experiment_cpu_load_estimator),
      initial_frame_dropper_(
          std::make_unique<InitialFrameDropper>(quality_scaler_resource_)),
      quality_scaling_experiment_enabled_(QualityScalingExperiment::Enabled()),
      encoder_target_bitrate_bps_(absl::nullopt),
      quality_rampup_experiment_(
          QualityRampUpExperimentHelper::CreateIfEnabled(this, clock_)),
      encoder_settings_(absl::nullopt) {
  RTC_CHECK(degradation_preference_provider_);
  RTC_CHECK(encoder_stats_observer_);
  MapResourceToReason(encode_usage_resource_, VideoAdaptationReason::kCpu);
}

VideoStreamEncoderResourceManager::~VideoStreamEncoderResourceManager() =
    default;

void VideoStreamEncoderResourceManager::Initialize(
    rtc::TaskQueue* encoder_queue,
    rtc::TaskQueue* resource_adaptation_queue) {
  RTC_DCHECK(!encoder_queue_);
  RTC_DCHECK(encoder_queue);
  RTC_DCHECK(!resource_adaptation_queue_);
  RTC_DCHECK(resource_adaptation_queue);
  encoder_queue_ = encoder_queue;
  resource_adaptation_queue_ = resource_adaptation_queue;
  bitrate_constraint_->SetAdaptationQueue(resource_adaptation_queue_->Get());
  balanced_constraint_->SetAdaptationQueue(resource_adaptation_queue_->Get());
  encode_usage_resource_->RegisterEncoderTaskQueue(encoder_queue_->Get());
  encode_usage_resource_->RegisterAdaptationTaskQueue(
      resource_adaptation_queue_->Get());
  quality_scaler_resource_->RegisterEncoderTaskQueue(encoder_queue_->Get());
  quality_scaler_resource_->RegisterAdaptationTaskQueue(
      resource_adaptation_queue_->Get());
}

void VideoStreamEncoderResourceManager::SetAdaptationProcessor(
    ResourceAdaptationProcessorInterface* adaptation_processor,
    VideoStreamAdapter* stream_adapter) {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
  adaptation_processor_ = adaptation_processor;
  stream_adapter_ = stream_adapter;
}

void VideoStreamEncoderResourceManager::SetDegradationPreferences(
    DegradationPreference degradation_preference) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  degradation_preference_ = degradation_preference;
  UpdateStatsAdaptationSettings();
}

DegradationPreference
VideoStreamEncoderResourceManager::degradation_preference() const {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  return degradation_preference_;
}

void VideoStreamEncoderResourceManager::StartEncodeUsageResource() {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  RTC_DCHECK(!encode_usage_resource_->is_started());
  RTC_DCHECK(encoder_settings_.has_value());
  encode_usage_resource_->StartCheckForOveruse(GetCpuOveruseOptions());
}

void VideoStreamEncoderResourceManager::StopManagedResources() {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  encode_usage_resource_->StopCheckForOveruse();
  if (quality_scaler_resource_->is_started()) {
    quality_scaler_resource_->StopCheckForOveruse();
  }
}

void VideoStreamEncoderResourceManager::RemoveResource(
    const rtc::scoped_refptr<Resource>& resource) {
  rtc::CritScope crit(&resource_lock_);
  RTC_DCHECK(resource);

  auto resource_it = std::find_if(resources_.begin(), resources_.end(),
                                  [resource](const ResourceAndReason& r) {
                                    return r.resource == resource;
                                  });
  if (resource_it == resources_.end()) {
    RTC_LOG(INFO) << "Resource \"" << resource->Name() << "\" already removed";
    return;
  }
  resources_.erase(resource_it);

  resource_adaptation_queue_->PostTask([this, resource]() {
    RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
    if (adaptation_processor_) {
      adaptation_processor_->RemoveResource(resource);
    }
  });
}

void VideoStreamEncoderResourceManager::MapResourceToReason(
    rtc::scoped_refptr<Resource> resource,
    VideoAdaptationReason reason) {
  rtc::CritScope crit(&resource_lock_);
  RTC_DCHECK(resource);
  RTC_DCHECK(absl::c_find_if(resources_,
                             [resource](const ResourceAndReason& r) {
                               return r.resource == resource;
                             }) == resources_.end())
      << "Resource \"" << resource->Name() << "\" already was inserted";
  resources_.emplace_back(resource, reason);
}

std::vector<rtc::scoped_refptr<Resource>>
VideoStreamEncoderResourceManager::MappedResources() const {
  rtc::CritScope crit(&resource_lock_);
  std::vector<rtc::scoped_refptr<Resource>> resources;
  for (auto const& resource_and_reason : resources_) {
    resources.push_back(resource_and_reason.resource);
  }
  return resources;
}

std::vector<AdaptationConstraint*>
VideoStreamEncoderResourceManager::AdaptationConstraints() const {
  return {bitrate_constraint_, balanced_constraint_};
}

std::vector<AdaptationListener*>
VideoStreamEncoderResourceManager::AdaptationListeners() const {
  return {quality_scaler_resource_};
}

rtc::scoped_refptr<QualityScalerResource>
VideoStreamEncoderResourceManager::quality_scaler_resource_for_testing() {
  rtc::CritScope crit(&resource_lock_);
  return quality_scaler_resource_;
}

void VideoStreamEncoderResourceManager::SetEncoderSettings(
    EncoderSettings encoder_settings) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  encoder_settings_ = std::move(encoder_settings);
  bitrate_constraint_->OnEncoderSettingsUpdated(encoder_settings_);
  MaybeUpdateTargetFrameRate();
}

void VideoStreamEncoderResourceManager::SetStartBitrate(
    DataRate start_bitrate) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  if (!start_bitrate.IsZero()) {
    encoder_target_bitrate_bps_ = start_bitrate.bps();
    bitrate_constraint_->OnEncoderTargetBitrateUpdated(
        encoder_target_bitrate_bps_);
    balanced_constraint_->OnEncoderTargetBitrateUpdated(
        encoder_target_bitrate_bps_);
  }
  initial_frame_dropper_->SetStartBitrate(start_bitrate,
                                          clock_->TimeInMicroseconds());
}

void VideoStreamEncoderResourceManager::SetTargetBitrate(
    DataRate target_bitrate) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  if (!target_bitrate.IsZero()) {
    encoder_target_bitrate_bps_ = target_bitrate.bps();
    bitrate_constraint_->OnEncoderTargetBitrateUpdated(
        encoder_target_bitrate_bps_);
    balanced_constraint_->OnEncoderTargetBitrateUpdated(
        encoder_target_bitrate_bps_);
  }
  initial_frame_dropper_->SetTargetBitrate(target_bitrate,
                                           clock_->TimeInMilliseconds());
}

void VideoStreamEncoderResourceManager::SetEncoderRates(
    const VideoEncoder::RateControlParameters& encoder_rates) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  encoder_rates_ = encoder_rates;
}

void VideoStreamEncoderResourceManager::OnFrameDroppedDueToSize() {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  // The VideoStreamEncoder makes the manager outlive the adaptation queue. This
  // means that if the task gets executed, |this| has not been freed yet.
  // TODO(https://crbug.com/webrtc/11565): When the manager no longer outlives
  // the adaptation queue, add logic to prevent use-after-free on |this|.
  resource_adaptation_queue_->PostTask([this] {
    RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
    if (!adaptation_processor_) {
      // The processor nulled before this task had a chance to execute. This
      // happens if the processor is destroyed. No action needed.
      return;
    }
    Adaptation reduce_resolution = stream_adapter_->GetAdaptDownResolution();
    if (reduce_resolution.status() == Adaptation::Status::kValid) {
      stream_adapter_->ApplyAdaptation(reduce_resolution,
                                       quality_scaler_resource_);
    }
  });
  initial_frame_dropper_->OnFrameDroppedDueToSize();
}

void VideoStreamEncoderResourceManager::OnEncodeStarted(
    const VideoFrame& cropped_frame,
    int64_t time_when_first_seen_us) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  encode_usage_resource_->OnEncodeStarted(cropped_frame,
                                          time_when_first_seen_us);
}

void VideoStreamEncoderResourceManager::OnEncodeCompleted(
    const EncodedImage& encoded_image,
    int64_t time_sent_in_us,
    absl::optional<int> encode_duration_us) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  // Inform |encode_usage_resource_| of the encode completed event.
  uint32_t timestamp = encoded_image.Timestamp();
  int64_t capture_time_us =
      encoded_image.capture_time_ms_ * rtc::kNumMicrosecsPerMillisec;
  encode_usage_resource_->OnEncodeCompleted(
      timestamp, time_sent_in_us, capture_time_us, encode_duration_us);
  // Inform |quality_scaler_resource_| of the encode completed event.
  quality_scaler_resource_->OnEncodeCompleted(encoded_image, time_sent_in_us);
}

void VideoStreamEncoderResourceManager::OnFrameDropped(
    EncodedImageCallback::DropReason reason) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  quality_scaler_resource_->OnFrameDropped(reason);
}

bool VideoStreamEncoderResourceManager::DropInitialFrames() const {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  return initial_frame_dropper_->DropInitialFrames();
}

void VideoStreamEncoderResourceManager::OnMaybeEncodeFrame() {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  initial_frame_dropper_->OnMaybeEncodeFrame();
  if (quality_rampup_experiment_) {
    DataRate bandwidth = encoder_rates_.has_value()
                             ? encoder_rates_->bandwidth_allocation
                             : DataRate::Zero();
    quality_rampup_experiment_->PerformQualityRampupExperiment(
        quality_scaler_resource_, bandwidth,
        DataRate::BitsPerSec(encoder_target_bitrate_bps_.value_or(0)),
        DataRate::KilobitsPerSec(encoder_settings_->video_codec().maxBitrate),
        LastInputFrameSizeOrDefault());
  }
}

void VideoStreamEncoderResourceManager::OnQualityScalerStopped() {
  RemoveResource(quality_scaler_resource_);
}

void VideoStreamEncoderResourceManager::OnQualityScalerStarted() {
  MapResourceToReason(quality_scaler_resource_,
                      VideoAdaptationReason::kQuality);
  resource_adaptation_queue_->PostTask([this]() {
    RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
    if (adaptation_processor_) {
      adaptation_processor_->AddResource(quality_scaler_resource_);
    }
  });
}

void VideoStreamEncoderResourceManager::UpdateQualityScalerSettings(
    absl::optional<VideoEncoder::QpThresholds> qp_thresholds) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  if (qp_thresholds.has_value()) {
    if (quality_scaler_resource_->is_started()) {
      quality_scaler_resource_->SetQpThresholds(qp_thresholds.value());
    } else {
      quality_scaler_resource_->StartCheckForOveruse(qp_thresholds.value());
    }
  } else {
    if (quality_scaler_resource_->is_started()) {
      quality_scaler_resource_->StopCheckForOveruse();
    }
  }
  initial_frame_dropper_->OnQualityScalerSettingsUpdated();
}

void VideoStreamEncoderResourceManager::ConfigureQualityScaler(
    const VideoEncoder::EncoderInfo& encoder_info) {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  const auto scaling_settings = encoder_info.scaling_settings;
  auto degradation_preference =
      degradation_preference_provider_->degradation_preference();
  const bool quality_scaling_allowed =
      IsResolutionScalingEnabled(degradation_preference) &&
      scaling_settings.thresholds;

  // TODO(https://crbug.com/webrtc/11222): Should this move to
  // QualityScalerResource?
  if (quality_scaling_allowed) {
    if (!quality_scaler_resource_->is_started()) {
      // Quality scaler has not already been configured.

      // Use experimental thresholds if available.
      absl::optional<VideoEncoder::QpThresholds> experimental_thresholds;
      if (quality_scaling_experiment_enabled_) {
        experimental_thresholds = QualityScalingExperiment::GetQpThresholds(
            GetVideoCodecTypeOrGeneric(encoder_settings_));
      }
      UpdateQualityScalerSettings(experimental_thresholds
                                      ? *experimental_thresholds
                                      : *(scaling_settings.thresholds));
    }
  } else {
    UpdateQualityScalerSettings(absl::nullopt);
  }

  // Set the qp-thresholds to the balanced settings if balanced mode.
  if (degradation_preference == DegradationPreference::BALANCED &&
      quality_scaler_resource_->is_started()) {
    absl::optional<VideoEncoder::QpThresholds> thresholds =
        balanced_settings_.GetQpThresholds(
            GetVideoCodecTypeOrGeneric(encoder_settings_),
            LastInputFrameSizeOrDefault());
    if (thresholds) {
      quality_scaler_resource_->SetQpThresholds(*thresholds);
    }
  }
  UpdateStatsAdaptationSettings();
}

VideoAdaptationReason VideoStreamEncoderResourceManager::GetReasonFromResource(
    rtc::scoped_refptr<Resource> resource) const {
  rtc::CritScope crit(&resource_lock_);
  const auto& registered_resource =
      absl::c_find_if(resources_, [&resource](const ResourceAndReason& r) {
        return r.resource == resource;
      });
  RTC_DCHECK(registered_resource != resources_.end())
      << resource->Name() << " not found.";
  return registered_resource->reason;
}

// TODO(pbos): Lower these thresholds (to closer to 100%) when we handle
// pipelining encoders better (multiple input frames before something comes
// out). This should effectively turn off CPU adaptations for systems that
// remotely cope with the load right now.
CpuOveruseOptions VideoStreamEncoderResourceManager::GetCpuOveruseOptions()
    const {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  // This is already ensured by the only caller of this method:
  // StartResourceAdaptation().
  RTC_DCHECK(encoder_settings_.has_value());
  CpuOveruseOptions options;
  // Hardware accelerated encoders are assumed to be pipelined; give them
  // additional overuse time.
  if (encoder_settings_->encoder_info().is_hardware_accelerated) {
    options.low_encode_usage_threshold_percent = 150;
    options.high_encode_usage_threshold_percent = 200;
  }
  if (experiment_cpu_load_estimator_) {
    options.filter_time_ms = 5 * rtc::kNumMillisecsPerSec;
  }
  return options;
}

int VideoStreamEncoderResourceManager::LastInputFrameSizeOrDefault() const {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  return input_state_provider_->InputState().frame_size_pixels().value_or(
      kDefaultInputPixelsWidth * kDefaultInputPixelsHeight);
}

void VideoStreamEncoderResourceManager::OnVideoSourceRestrictionsUpdated(
    VideoSourceRestrictions restrictions,
    const VideoAdaptationCounters& adaptation_counters,
    rtc::scoped_refptr<Resource> reason,
    const VideoSourceRestrictions& unfiltered_restrictions) {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
  // TODO(bugs.webrtc.org/11553) Remove reason parameter and add reset callback.
  if (!reason && adaptation_counters.Total() == 0) {
    // Adaptation was manually reset - clear the per-reason counters too.
    encoder_stats_observer_->ClearAdaptationStats();
  }

  // The VideoStreamEncoder makes the manager outlive the encoder queue. This
  // means that if the task gets executed, |this| has not been freed yet.
  encoder_queue_->PostTask([this, restrictions] {
    RTC_DCHECK_RUN_ON(encoder_queue_);
    video_source_restrictions_ = FilterRestrictionsByDegradationPreference(
        restrictions, degradation_preference_);
    MaybeUpdateTargetFrameRate();
  });
}

void VideoStreamEncoderResourceManager::OnResourceLimitationChanged(
    rtc::scoped_refptr<Resource> resource,
    const std::map<rtc::scoped_refptr<Resource>, VideoAdaptationCounters>&
        resource_limitations) {
  RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
  if (!resource) {
    encoder_stats_observer_->ClearAdaptationStats();
    return;
  }

  std::map<VideoAdaptationReason, VideoAdaptationCounters> limitations;
  for (auto& resource_counter : resource_limitations) {
    std::map<VideoAdaptationReason, VideoAdaptationCounters>::iterator it;
    bool inserted;
    std::tie(it, inserted) = limitations.emplace(
        GetReasonFromResource(resource_counter.first), resource_counter.second);
    if (!inserted && it->second.Total() < resource_counter.second.Total()) {
      it->second = resource_counter.second;
    }
  }

  VideoAdaptationReason adaptation_reason = GetReasonFromResource(resource);
  encoder_stats_observer_->OnAdaptationChanged(
      adaptation_reason, limitations[VideoAdaptationReason::kCpu],
      limitations[VideoAdaptationReason::kQuality]);

  encoder_queue_->PostTask(ToQueuedTask(
      [cpu_limited = limitations.at(VideoAdaptationReason::kCpu).Total() > 0,
       qp_resolution_adaptations =
           limitations.at(VideoAdaptationReason::kQuality)
               .resolution_adaptations,
       this]() {
        RTC_DCHECK_RUN_ON(encoder_queue_);
        if (quality_rampup_experiment_) {
          quality_rampup_experiment_->cpu_adapted(cpu_limited);
          quality_rampup_experiment_->qp_resolution_adaptations(
              qp_resolution_adaptations);
        }
      }));

  RTC_LOG(LS_INFO) << ActiveCountsToString(limitations);
}

void VideoStreamEncoderResourceManager::MaybeUpdateTargetFrameRate() {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  absl::optional<double> codec_max_frame_rate =
      encoder_settings_.has_value()
          ? absl::optional<double>(
                encoder_settings_->video_codec().maxFramerate)
          : absl::nullopt;
  // The current target framerate is the maximum frame rate as specified by
  // the current codec configuration or any limit imposed by the adaptation
  // module. This is used to make sure overuse detection doesn't needlessly
  // trigger in low and/or variable framerate scenarios.
  absl::optional<double> target_frame_rate =
      video_source_restrictions_.max_frame_rate();
  if (!target_frame_rate.has_value() ||
      (codec_max_frame_rate.has_value() &&
       codec_max_frame_rate.value() < target_frame_rate.value())) {
    target_frame_rate = codec_max_frame_rate;
  }
  encode_usage_resource_->SetTargetFrameRate(target_frame_rate);
}

void VideoStreamEncoderResourceManager::UpdateStatsAdaptationSettings() const {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  VideoStreamEncoderObserver::AdaptationSettings cpu_settings(
      IsResolutionScalingEnabled(degradation_preference_),
      IsFramerateScalingEnabled(degradation_preference_));

  VideoStreamEncoderObserver::AdaptationSettings quality_settings =
      quality_scaler_resource_->is_started()
          ? cpu_settings
          : VideoStreamEncoderObserver::AdaptationSettings();
  encoder_stats_observer_->UpdateAdaptationSettings(cpu_settings,
                                                    quality_settings);
}

// static
std::string VideoStreamEncoderResourceManager::ActiveCountsToString(
    const std::map<VideoAdaptationReason, VideoAdaptationCounters>&
        active_counts) {
  rtc::StringBuilder ss;

  ss << "Downgrade counts: fps: {";
  for (auto& reason_count : active_counts) {
    ss << ToString(reason_count.first) << ":";
    ss << reason_count.second.fps_adaptations;
  }
  ss << "}, resolution {";
  for (auto& reason_count : active_counts) {
    ss << ToString(reason_count.first) << ":";
    ss << reason_count.second.resolution_adaptations;
  }
  ss << "}";

  return ss.Release();
}

void VideoStreamEncoderResourceManager::OnQualityRampUp() {
  RTC_DCHECK_RUN_ON(encoder_queue_);
  // The VideoStreamEncoder makes the manager outlive the adaptation queue.
  // This means that if the task gets executed, |this| has not been freed yet.
  // TODO(https://crbug.com/webrtc/11565): When the manager no longer outlives
  // the adaptation queue, add logic to prevent use-after-free on |this|.
  resource_adaptation_queue_->PostTask([this] {
    RTC_DCHECK_RUN_ON(resource_adaptation_queue_);
    if (!stream_adapter_) {
      // The processor nulled before this task had a chance to execute. This
      // happens if the processor is destroyed. No action needed.
      return;
    }
    stream_adapter_->ClearRestrictions();
  });
  quality_rampup_experiment_.reset();
}
}  // namespace webrtc
