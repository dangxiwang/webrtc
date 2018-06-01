/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "video/video_send_stream_impl.h"

#include <algorithm>
#include <string>
#include <utility>

#include "call/rtp_transport_controller_send_interface.h"
#include "modules/pacing/packet_router.h"
#include "modules/rtp_rtcp/include/rtp_rtcp.h"
#include "modules/rtp_rtcp/source/rtp_sender.h"
#include "rtc_base/checks.h"
#include "rtc_base/experiments/alr_experiment.h"
#include "rtc_base/file.h"
#include "rtc_base/location.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/safe_conversions.h"
#include "rtc_base/trace_event.h"
#include "system_wrappers/include/field_trial.h"

namespace webrtc {
namespace internal {
namespace {
static const int kMinSendSidePacketHistorySize = 600;

// Assume an average video stream has around 3 packets per frame (1 mbps / 30
// fps / 1400B) A sequence number set with size 5500 will be able to store
// packet sequence number for at least last 60 seconds.
static const int kSendSideSeqNumSetMaxSize = 5500;

// We don't do MTU discovery, so assume that we have the standard ethernet MTU.
const size_t kPathMTU = 1500;

std::vector<RtpRtcp*> CreateRtpRtcpModules(
    const VideoSendStream::Config& config,
    RtcpIntraFrameObserver* intra_frame_callback,
    RtcpBandwidthObserver* bandwidth_callback,
    RtpTransportControllerSendInterface* transport,
    RtcpRttStats* rtt_stats,
    FlexfecSender* flexfec_sender,
    SendStatisticsProxy* stats_proxy,
    SendDelayStats* send_delay_stats,
    RtcEventLog* event_log,
    RateLimiter* retransmission_rate_limiter,
    OverheadObserver* overhead_observer,
    RtpKeepAliveConfig keepalive_config) {
  RTC_DCHECK_GT(config.rtp.ssrcs.size(), 0);
  RtpRtcp::Configuration configuration;
  configuration.audio = false;
  configuration.receiver_only = false;
  configuration.outgoing_transport = config.send_transport;
  configuration.intra_frame_callback = intra_frame_callback;
  configuration.bandwidth_callback = bandwidth_callback;
  configuration.transport_feedback_callback =
      transport->transport_feedback_observer();
  configuration.rtt_stats = rtt_stats;
  configuration.rtcp_packet_type_counter_observer = stats_proxy;
  configuration.paced_sender = transport->packet_sender();
  configuration.transport_sequence_number_allocator =
      transport->packet_router();
  configuration.send_bitrate_observer = stats_proxy;
  configuration.send_frame_count_observer = stats_proxy;
  configuration.send_side_delay_observer = stats_proxy;
  configuration.send_packet_observer = send_delay_stats;
  configuration.event_log = event_log;
  configuration.retransmission_rate_limiter = retransmission_rate_limiter;
  configuration.overhead_observer = overhead_observer;
  configuration.keepalive_config = keepalive_config;
  configuration.rtcp_interval_config.video_interval_ms =
      config.rtcp.video_report_interval_ms;
  configuration.rtcp_interval_config.audio_interval_ms =
      config.rtcp.audio_report_interval_ms;
  std::vector<RtpRtcp*> modules;
  const std::vector<uint32_t>& flexfec_protected_ssrcs =
      config.rtp.flexfec.protected_media_ssrcs;
  for (uint32_t ssrc : config.rtp.ssrcs) {
    bool enable_flexfec = flexfec_sender != nullptr &&
                          std::find(flexfec_protected_ssrcs.begin(),
                                    flexfec_protected_ssrcs.end(),
                                    ssrc) != flexfec_protected_ssrcs.end();
    configuration.flexfec_sender = enable_flexfec ? flexfec_sender : nullptr;
    RtpRtcp* rtp_rtcp = RtpRtcp::CreateRtpRtcp(configuration);
    rtp_rtcp->SetSendingStatus(false);
    rtp_rtcp->SetSendingMediaStatus(false);
    rtp_rtcp->SetRTCPStatus(RtcpMode::kCompound);
    modules.push_back(rtp_rtcp);
  }
  return modules;
}

// TODO(brandtr): Update this function when we support multistream protection.
std::unique_ptr<FlexfecSender> MaybeCreateFlexfecSender(
    const VideoSendStream::Config& config,
    const std::map<uint32_t, RtpState>& suspended_ssrcs) {
  if (config.rtp.flexfec.payload_type < 0) {
    return nullptr;
  }
  RTC_DCHECK_GE(config.rtp.flexfec.payload_type, 0);
  RTC_DCHECK_LE(config.rtp.flexfec.payload_type, 127);
  if (config.rtp.flexfec.ssrc == 0) {
    RTC_LOG(LS_WARNING) << "FlexFEC is enabled, but no FlexFEC SSRC given. "
                           "Therefore disabling FlexFEC.";
    return nullptr;
  }
  if (config.rtp.flexfec.protected_media_ssrcs.empty()) {
    RTC_LOG(LS_WARNING)
        << "FlexFEC is enabled, but no protected media SSRC given. "
           "Therefore disabling FlexFEC.";
    return nullptr;
  }

  if (config.rtp.flexfec.protected_media_ssrcs.size() > 1) {
    RTC_LOG(LS_WARNING)
        << "The supplied FlexfecConfig contained multiple protected "
           "media streams, but our implementation currently only "
           "supports protecting a single media stream. "
           "To avoid confusion, disabling FlexFEC completely.";
    return nullptr;
  }

  const RtpState* rtp_state = nullptr;
  auto it = suspended_ssrcs.find(config.rtp.flexfec.ssrc);
  if (it != suspended_ssrcs.end()) {
    rtp_state = &it->second;
  }

  RTC_DCHECK_EQ(1U, config.rtp.flexfec.protected_media_ssrcs.size());
  return rtc::MakeUnique<FlexfecSender>(
      config.rtp.flexfec.payload_type, config.rtp.flexfec.ssrc,
      config.rtp.flexfec.protected_media_ssrcs[0], config.rtp.mid,
      config.rtp.extensions, RTPSender::FecExtensionSizes(), rtp_state,
      Clock::GetRealTimeClock());
}

bool TransportSeqNumExtensionConfigured(const VideoSendStream::Config& config) {
  const std::vector<RtpExtension>& extensions = config.rtp.extensions;
  return std::find_if(
             extensions.begin(), extensions.end(), [](const RtpExtension& ext) {
               return ext.uri == RtpExtension::kTransportSequenceNumberUri;
             }) != extensions.end();
}

const char kForcedFallbackFieldTrial[] =
    "WebRTC-VP8-Forced-Fallback-Encoder-v2";

rtc::Optional<int> GetFallbackMinBpsFromFieldTrial() {
  if (!webrtc::field_trial::IsEnabled(kForcedFallbackFieldTrial))
    return rtc::nullopt;

  std::string group =
      webrtc::field_trial::FindFullName(kForcedFallbackFieldTrial);
  if (group.empty())
    return rtc::nullopt;

  int min_pixels;
  int max_pixels;
  int min_bps;
  if (sscanf(group.c_str(), "Enabled-%d,%d,%d", &min_pixels, &max_pixels,
             &min_bps) != 3) {
    return rtc::nullopt;
  }

  if (min_bps <= 0)
    return rtc::nullopt;

  return min_bps;
}

int GetEncoderMinBitrateBps() {
  const int kDefaultEncoderMinBitrateBps = 30000;
  return GetFallbackMinBpsFromFieldTrial().value_or(
      kDefaultEncoderMinBitrateBps);
}

bool PayloadTypeSupportsSkippingFecPackets(const std::string& payload_name) {
  const VideoCodecType codecType = PayloadStringToCodecType(payload_name);
  if (codecType == kVideoCodecVP8 || codecType == kVideoCodecVP9) {
    return true;
  }
  return false;
}

int CalculateMaxPadBitrateBps(std::vector<VideoStream> streams,
                              int min_transmit_bitrate_bps,
                              bool pad_to_min_bitrate) {
  int pad_up_to_bitrate_bps = 0;
  // Calculate max padding bitrate for a multi layer codec.
  if (streams.size() > 1) {
    // Pad to min bitrate of the highest layer.
    pad_up_to_bitrate_bps = streams[streams.size() - 1].min_bitrate_bps;
    // Add target_bitrate_bps of the lower layers.
    for (size_t i = 0; i < streams.size() - 1; ++i)
      pad_up_to_bitrate_bps += streams[i].target_bitrate_bps;
  } else if (pad_to_min_bitrate) {
    pad_up_to_bitrate_bps = streams[0].min_bitrate_bps;
  }

  pad_up_to_bitrate_bps =
      std::max(pad_up_to_bitrate_bps, min_transmit_bitrate_bps);

  return pad_up_to_bitrate_bps;
}

uint32_t CalculateOverheadRateBps(int packets_per_second,
                                  size_t overhead_bytes_per_packet,
                                  uint32_t max_overhead_bps) {
  uint32_t overhead_bps =
      static_cast<uint32_t>(8 * overhead_bytes_per_packet * packets_per_second);
  return std::min(overhead_bps, max_overhead_bps);
}

int CalculatePacketRate(uint32_t bitrate_bps, size_t packet_size_bytes) {
  size_t packet_size_bits = 8 * packet_size_bytes;
  // Ceil for int value of bitrate_bps / packet_size_bits.
  return static_cast<int>((bitrate_bps + packet_size_bits - 1) /
                          packet_size_bits);
}

}  // namespace

// CheckEncoderActivityTask is used for tracking when the encoder last produced
// and encoded video frame. If the encoder has not produced anything the last
// kEncoderTimeOutMs we also want to stop sending padding.
class VideoSendStreamImpl::CheckEncoderActivityTask : public rtc::QueuedTask {
 public:
  static const int kEncoderTimeOutMs = 2000;
  explicit CheckEncoderActivityTask(
      const rtc::WeakPtr<VideoSendStreamImpl>& send_stream)
      : activity_(0), send_stream_(std::move(send_stream)), timed_out_(false) {}

  void Stop() {
    RTC_CHECK(task_checker_.CalledSequentially());
    send_stream_.reset();
  }

  void UpdateEncoderActivity() {
    // UpdateEncoderActivity is called from VideoSendStreamImpl::Encoded on
    // whatever thread the real encoder implementation run on. In the case of
    // hardware encoders, there might be several encoders
    // running in parallel on different threads.
    rtc::AtomicOps::ReleaseStore(&activity_, 1);
  }

 private:
  bool Run() override {
    RTC_CHECK(task_checker_.CalledSequentially());
    if (!send_stream_)
      return true;
    if (!rtc::AtomicOps::AcquireLoad(&activity_)) {
      if (!timed_out_) {
        send_stream_->SignalEncoderTimedOut();
      }
      timed_out_ = true;
    } else if (timed_out_) {
      send_stream_->SignalEncoderActive();
      timed_out_ = false;
    }
    rtc::AtomicOps::ReleaseStore(&activity_, 0);

    rtc::TaskQueue::Current()->PostDelayedTask(
        std::unique_ptr<rtc::QueuedTask>(this), kEncoderTimeOutMs);
    // Return false to prevent this task from being deleted. Ownership has been
    // transferred to the task queue when PostDelayedTask was called.
    return false;
  }
  volatile int activity_;

  rtc::SequencedTaskChecker task_checker_;
  rtc::WeakPtr<VideoSendStreamImpl> send_stream_;
  bool timed_out_;
};

VideoSendStreamImpl::VideoSendStreamImpl(
    SendStatisticsProxy* stats_proxy,
    rtc::TaskQueue* worker_queue,
    CallStats* call_stats,
    RtpTransportControllerSendInterface* transport,
    BitrateAllocatorInterface* bitrate_allocator,
    SendDelayStats* send_delay_stats,
    VideoStreamEncoderInterface* video_stream_encoder,
    RtcEventLog* event_log,
    const VideoSendStream::Config* config,
    int initial_encoder_max_bitrate,
    double initial_encoder_bitrate_priority,
    std::map<uint32_t, RtpState> suspended_ssrcs,
    std::map<uint32_t, RtpPayloadState> suspended_payload_states,
    VideoEncoderConfig::ContentType content_type,
    std::unique_ptr<FecController> fec_controller,
    RateLimiter* retransmission_limiter)
    : send_side_bwe_with_overhead_(
          webrtc::field_trial::IsEnabled("WebRTC-SendSideBwe-WithOverhead")),
      stats_proxy_(stats_proxy),
      config_(config),
      suspended_ssrcs_(std::move(suspended_ssrcs)),
      fec_controller_(std::move(fec_controller)),
      module_process_thread_(nullptr),
      worker_queue_(worker_queue),
      check_encoder_activity_task_(nullptr),
      call_stats_(call_stats),
      transport_(transport),
      bitrate_allocator_(bitrate_allocator),
      flexfec_sender_(MaybeCreateFlexfecSender(*config_, suspended_ssrcs_)),
      max_padding_bitrate_(0),
      encoder_min_bitrate_bps_(0),
      encoder_target_rate_bps_(0),
      encoder_bitrate_priority_(initial_encoder_bitrate_priority),
      has_packet_feedback_(false),
      video_stream_encoder_(video_stream_encoder),
      encoder_feedback_(Clock::GetRealTimeClock(),
                        config_->rtp.ssrcs,
                        video_stream_encoder),
      bandwidth_observer_(transport->GetBandwidthObserver()),
      rtp_rtcp_modules_(CreateRtpRtcpModules(*config_,
                                             &encoder_feedback_,
                                             bandwidth_observer_,
                                             transport,
                                             call_stats,
                                             flexfec_sender_.get(),
                                             stats_proxy_,
                                             send_delay_stats,
                                             event_log,
                                             retransmission_limiter,
                                             this,
                                             transport->keepalive_config())),
      payload_router_(rtp_rtcp_modules_,
                      config_->rtp.ssrcs,
                      config_->rtp.payload_type,
                      suspended_payload_states),
      weak_ptr_factory_(this),
      overhead_bytes_per_packet_(0),
      transport_overhead_bytes_per_packet_(0) {
  RTC_DCHECK_RUN_ON(worker_queue_);
  RTC_LOG(LS_INFO) << "VideoSendStreamInternal: " << config_->ToString();
  weak_ptr_ = weak_ptr_factory_.GetWeakPtr();
  module_process_thread_checker_.DetachFromThread();

  RTC_DCHECK(!config_->rtp.ssrcs.empty());
  RTC_DCHECK(call_stats_);
  RTC_DCHECK(transport_);
  RTC_DCHECK_NE(initial_encoder_max_bitrate, 0);

  if (initial_encoder_max_bitrate > 0) {
    encoder_max_bitrate_bps_ =
        rtc::dchecked_cast<uint32_t>(initial_encoder_max_bitrate);
  } else {
    // TODO(srte): Make sure max bitrate is not set to negative values. We don't
    // have any way to handle unset values in downstream code, such as the
    // bitrate allocator. Previously -1 was implicitly casted to UINT32_MAX, a
    // behaviour that is not safe. Converting to 10 Mbps should be safe for
    // reasonable use cases as it allows adding the max of multiple streams
    // without wrappping around.
    const int kFallbackMaxBitrateBps = 10000000;
    RTC_DLOG(LS_ERROR) << "ERROR: Initial encoder max bitrate = "
                       << initial_encoder_max_bitrate << " which is <= 0!";
    RTC_DLOG(LS_INFO) << "Using default encoder max bitrate = 10 Mbps";
    encoder_max_bitrate_bps_ = kFallbackMaxBitrateBps;
  }

  RTC_CHECK(AlrExperimentSettings::MaxOneFieldTrialEnabled());
  // If send-side BWE is enabled, check if we should apply updated probing and
  // pacing settings.
  if (TransportSeqNumExtensionConfigured(*config_)) {
    has_packet_feedback_ = true;

    rtc::Optional<AlrExperimentSettings> alr_settings;
    if (content_type == VideoEncoderConfig::ContentType::kScreen) {
      alr_settings = AlrExperimentSettings::CreateFromFieldTrial(
          AlrExperimentSettings::kScreenshareProbingBweExperimentName);
    } else {
      alr_settings = AlrExperimentSettings::CreateFromFieldTrial(
          AlrExperimentSettings::kStrictPacingAndProbingExperimentName);
    }
    if (alr_settings) {
      transport->EnablePeriodicAlrProbing(true);
      transport->SetPacingFactor(alr_settings->pacing_factor);
      configured_pacing_factor_ = alr_settings->pacing_factor;
      transport->SetQueueTimeLimit(alr_settings->max_paced_queue_time);
    } else {
      transport->EnablePeriodicAlrProbing(false);
      transport->SetPacingFactor(PacedSender::kDefaultPaceMultiplier);
      configured_pacing_factor_ = PacedSender::kDefaultPaceMultiplier;
      transport->SetQueueTimeLimit(PacedSender::kMaxQueueLengthMs);
    }
  }

  if (config_->periodic_alr_bandwidth_probing) {
    transport->EnablePeriodicAlrProbing(true);
  }

  // RTP/RTCP initialization.

  // We add the highest spatial layer first to ensure it'll be prioritized
  // when sending padding, with the hope that the packet rate will be smaller,
  // and that it's more important to protect than the lower layers.
  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
    constexpr bool remb_candidate = true;
    transport->packet_router()->AddSendRtpModule(rtp_rtcp, remb_candidate);
  }

  for (size_t i = 0; i < config_->rtp.extensions.size(); ++i) {
    const std::string& extension = config_->rtp.extensions[i].uri;
    int id = config_->rtp.extensions[i].id;
    // One-byte-extension local identifiers are in the range 1-14 inclusive.
    RTC_DCHECK_GE(id, 1);
    RTC_DCHECK_LE(id, 14);
    RTC_DCHECK(RtpExtension::IsSupportedForVideo(extension));
    for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
      RTC_CHECK_EQ(0, rtp_rtcp->RegisterSendRtpHeaderExtension(
                          StringToRtpExtensionType(extension), id));
    }
  }

  ConfigureProtection();
  ConfigureSsrcs();

  if (!config_->rtp.mid.empty()) {
    for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
      rtp_rtcp->SetMid(config_->rtp.mid);
    }
  }

  // TODO(pbos): Should we set CNAME on all RTP modules?
  rtp_rtcp_modules_.front()->SetCNAME(config_->rtp.c_name.c_str());

  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
    rtp_rtcp->RegisterRtcpStatisticsCallback(stats_proxy_);
    rtp_rtcp->RegisterSendChannelRtpStatisticsCallback(stats_proxy_);
    rtp_rtcp->SetMaxRtpPacketSize(config_->rtp.max_packet_size);
    rtp_rtcp->RegisterVideoSendPayload(config_->rtp.payload_type,
                                       config_->rtp.payload_name.c_str());
  }

  fec_controller_->SetProtectionCallback(this);
  // Signal congestion controller this object is ready for OnPacket* callbacks.
  if (fec_controller_->UseLossVectorMask()) {
    transport_->RegisterPacketFeedbackObserver(this);
  }

  RTC_DCHECK_GE(config_->rtp.payload_type, 0);
  RTC_DCHECK_LE(config_->rtp.payload_type, 127);

  video_stream_encoder_->SetStartBitrate(
      bitrate_allocator_->GetStartBitrate(this));

  // Only request rotation at the source when we positively know that the remote
  // side doesn't support the rotation extension. This allows us to prepare the
  // encoder in the expectation that rotation is supported - which is the common
  // case.
  bool rotation_applied =
      std::find_if(config_->rtp.extensions.begin(),
                   config_->rtp.extensions.end(),
                   [](const RtpExtension& extension) {
                     return extension.uri == RtpExtension::kVideoRotationUri;
                   }) == config_->rtp.extensions.end();

  video_stream_encoder_->SetSink(this, rotation_applied);
}

void VideoSendStreamImpl::RegisterProcessThread(
    ProcessThread* module_process_thread) {
  RTC_DCHECK_RUN_ON(&module_process_thread_checker_);
  RTC_DCHECK(!module_process_thread_);
  module_process_thread_ = module_process_thread;

  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_)
    module_process_thread_->RegisterModule(rtp_rtcp, RTC_FROM_HERE);
}

void VideoSendStreamImpl::DeRegisterProcessThread() {
  RTC_DCHECK_RUN_ON(&module_process_thread_checker_);
  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_)
    module_process_thread_->DeRegisterModule(rtp_rtcp);
}

VideoSendStreamImpl::~VideoSendStreamImpl() {
  RTC_DCHECK_RUN_ON(worker_queue_);
  RTC_DCHECK(!payload_router_.IsActive())
      << "VideoSendStreamImpl::Stop not called";
  RTC_LOG(LS_INFO) << "~VideoSendStreamInternal: " << config_->ToString();
  if (fec_controller_->UseLossVectorMask()) {
    transport_->DeRegisterPacketFeedbackObserver(this);
  }
  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
    transport_->packet_router()->RemoveSendRtpModule(rtp_rtcp);
    delete rtp_rtcp;
  }
}

bool VideoSendStreamImpl::DeliverRtcp(const uint8_t* packet, size_t length) {
  // Runs on a network thread.
  RTC_DCHECK(!worker_queue_->IsCurrent());
  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_)
    rtp_rtcp->IncomingRtcpPacket(packet, length);
  return true;
}

void VideoSendStreamImpl::UpdateActiveSimulcastLayers(
    const std::vector<bool> active_layers) {
  RTC_DCHECK_RUN_ON(worker_queue_);
  RTC_DCHECK_EQ(rtp_rtcp_modules_.size(), active_layers.size());
  RTC_LOG(LS_INFO) << "VideoSendStream::UpdateActiveSimulcastLayers";
  bool previously_active = payload_router_.IsActive();
  payload_router_.SetActiveModules(active_layers);
  if (!payload_router_.IsActive() && previously_active) {
    // Payload router switched from active to inactive.
    StopVideoSendStream();
  } else if (payload_router_.IsActive() && !previously_active) {
    // Payload router switched from inactive to active.
    StartupVideoSendStream();
  }
}

void VideoSendStreamImpl::Start() {
  RTC_DCHECK_RUN_ON(worker_queue_);
  RTC_LOG(LS_INFO) << "VideoSendStream::Start";
  if (payload_router_.IsActive())
    return;
  TRACE_EVENT_INSTANT0("webrtc", "VideoSendStream::Start");
  payload_router_.SetActive(true);
  StartupVideoSendStream();
}

void VideoSendStreamImpl::StartupVideoSendStream() {
  RTC_DCHECK_RUN_ON(worker_queue_);
  bitrate_allocator_->AddObserver(
      this,
      MediaStreamAllocationConfig{
          static_cast<uint32_t>(encoder_min_bitrate_bps_),
          encoder_max_bitrate_bps_, static_cast<uint32_t>(max_padding_bitrate_),
          !config_->suspend_below_min_bitrate, config_->track_id,
          encoder_bitrate_priority_, has_packet_feedback_});
  // Start monitoring encoder activity.
  {
    rtc::CritScope lock(&encoder_activity_crit_sect_);
    RTC_DCHECK(!check_encoder_activity_task_);
    check_encoder_activity_task_ = new CheckEncoderActivityTask(weak_ptr_);
    worker_queue_->PostDelayedTask(
        std::unique_ptr<rtc::QueuedTask>(check_encoder_activity_task_),
        CheckEncoderActivityTask::kEncoderTimeOutMs);
  }

  video_stream_encoder_->SendKeyFrame();
}

void VideoSendStreamImpl::Stop() {
  RTC_DCHECK_RUN_ON(worker_queue_);
  RTC_LOG(LS_INFO) << "VideoSendStream::Stop";
  if (!payload_router_.IsActive())
    return;
  TRACE_EVENT_INSTANT0("webrtc", "VideoSendStream::Stop");
  payload_router_.SetActive(false);
  StopVideoSendStream();
}

void VideoSendStreamImpl::StopVideoSendStream() {
  bitrate_allocator_->RemoveObserver(this);
  {
    rtc::CritScope lock(&encoder_activity_crit_sect_);
    check_encoder_activity_task_->Stop();
    check_encoder_activity_task_ = nullptr;
  }
  video_stream_encoder_->OnBitrateUpdated(0, 0, 0);
  stats_proxy_->OnSetEncoderTargetRate(0);
}

void VideoSendStreamImpl::SignalEncoderTimedOut() {
  RTC_DCHECK_RUN_ON(worker_queue_);
  // If the encoder has not produced anything the last kEncoderTimeOutMs and it
  // is supposed to, deregister as BitrateAllocatorObserver. This can happen
  // if a camera stops producing frames.
  if (encoder_target_rate_bps_ > 0) {
    RTC_LOG(LS_INFO) << "SignalEncoderTimedOut, Encoder timed out.";
    bitrate_allocator_->RemoveObserver(this);
  }
}

void VideoSendStreamImpl::OnBitrateAllocationUpdated(
    const VideoBitrateAllocation& allocation) {
  payload_router_.OnBitrateAllocationUpdated(allocation);
}

void VideoSendStreamImpl::SignalEncoderActive() {
  RTC_DCHECK_RUN_ON(worker_queue_);
  RTC_LOG(LS_INFO) << "SignalEncoderActive, Encoder is active.";
  bitrate_allocator_->AddObserver(
      this,
      MediaStreamAllocationConfig{
          static_cast<uint32_t>(encoder_min_bitrate_bps_),
          encoder_max_bitrate_bps_, static_cast<uint32_t>(max_padding_bitrate_),
          !config_->suspend_below_min_bitrate, config_->track_id,
          encoder_bitrate_priority_, has_packet_feedback_});
}

void VideoSendStreamImpl::OnEncoderConfigurationChanged(
    std::vector<VideoStream> streams,
    int min_transmit_bitrate_bps) {
  if (!worker_queue_->IsCurrent()) {
    rtc::WeakPtr<VideoSendStreamImpl> send_stream = weak_ptr_;
    worker_queue_->PostTask([send_stream, streams, min_transmit_bitrate_bps]() {
      if (send_stream)
        send_stream->OnEncoderConfigurationChanged(std::move(streams),
                                                   min_transmit_bitrate_bps);
    });
    return;
  }
  RTC_DCHECK_GE(config_->rtp.ssrcs.size(), streams.size());
  TRACE_EVENT0("webrtc", "VideoSendStream::OnEncoderConfigurationChanged");
  RTC_DCHECK_GE(config_->rtp.ssrcs.size(), streams.size());
  RTC_DCHECK_RUN_ON(worker_queue_);

  encoder_min_bitrate_bps_ =
      std::max(streams[0].min_bitrate_bps, GetEncoderMinBitrateBps());
  encoder_max_bitrate_bps_ = 0;
  double stream_bitrate_priority_sum = 0;
  for (const auto& stream : streams) {
    // We don't want to allocate more bitrate than needed to inactive streams.
    encoder_max_bitrate_bps_ += stream.active ? stream.max_bitrate_bps : 0;
    if (stream.bitrate_priority) {
      RTC_DCHECK_GT(*stream.bitrate_priority, 0);
      stream_bitrate_priority_sum += *stream.bitrate_priority;
    }
  }
  RTC_DCHECK_GT(stream_bitrate_priority_sum, 0);
  encoder_bitrate_priority_ = stream_bitrate_priority_sum;
  encoder_max_bitrate_bps_ =
      std::max(static_cast<uint32_t>(encoder_min_bitrate_bps_),
               encoder_max_bitrate_bps_);

  const VideoCodecType codec_type =
      PayloadStringToCodecType(config_->rtp.payload_name);
  if (codec_type == kVideoCodecVP9 && 0) {
    max_padding_bitrate_ = streams[0].target_bitrate_bps;
  } else {
    max_padding_bitrate_ = CalculateMaxPadBitrateBps(
        streams, min_transmit_bitrate_bps, config_->suspend_below_min_bitrate);
  }

  // Clear stats for disabled layers.
  for (size_t i = streams.size(); i < config_->rtp.ssrcs.size(); ++i) {
    stats_proxy_->OnInactiveSsrc(config_->rtp.ssrcs[i]);
  }

  const size_t num_temporal_layers =
      streams.back().num_temporal_layers.value_or(1);
  fec_controller_->SetEncodingData(streams[0].width, streams[0].height,
                                   num_temporal_layers,
                                   config_->rtp.max_packet_size);

  if (payload_router_.IsActive()) {
    // The send stream is started already. Update the allocator with new bitrate
    // limits.
    bitrate_allocator_->AddObserver(
        this, MediaStreamAllocationConfig{
                  static_cast<uint32_t>(encoder_min_bitrate_bps_),
                  encoder_max_bitrate_bps_,
                  static_cast<uint32_t>(max_padding_bitrate_),
                  !config_->suspend_below_min_bitrate, config_->track_id,
                  encoder_bitrate_priority_, has_packet_feedback_});
  }
}

EncodedImageCallback::Result VideoSendStreamImpl::OnEncodedImage(
    const EncodedImage& encoded_image,
    const CodecSpecificInfo* codec_specific_info,
    const RTPFragmentationHeader* fragmentation) {
  // Encoded is called on whatever thread the real encoder implementation run
  // on. In the case of hardware encoders, there might be several encoders
  // running in parallel on different threads.
  size_t simulcast_idx = 0;
  if (codec_specific_info->codecType == kVideoCodecVP8) {
    simulcast_idx = codec_specific_info->codecSpecific.VP8.simulcastIdx;
  }
  if (config_->post_encode_callback) {
    config_->post_encode_callback->EncodedFrameCallback(EncodedFrame(
        encoded_image._buffer, encoded_image._length, encoded_image._frameType,
        simulcast_idx, encoded_image._timeStamp));
  }
  {
    rtc::CritScope lock(&encoder_activity_crit_sect_);
    if (check_encoder_activity_task_)
      check_encoder_activity_task_->UpdateEncoderActivity();
  }

  fec_controller_->UpdateWithEncodedData(encoded_image._length,
                                         encoded_image._frameType);
  EncodedImageCallback::Result result = payload_router_.OnEncodedImage(
      encoded_image, codec_specific_info, fragmentation);

  RTC_DCHECK(codec_specific_info);

  int layer = codec_specific_info->codecType == kVideoCodecVP8
                  ? codec_specific_info->codecSpecific.VP8.simulcastIdx
                  : 0;
  {
    rtc::CritScope lock(&ivf_writers_crit_);
    if (file_writers_[layer].get()) {
      bool ok = file_writers_[layer]->WriteFrame(
          encoded_image, codec_specific_info->codecType);
      RTC_DCHECK(ok);
    }
  }

  return result;
}

void VideoSendStreamImpl::ConfigureProtection() {
  RTC_DCHECK_RUN_ON(worker_queue_);

  // Consistency of FlexFEC parameters is checked in MaybeCreateFlexfecSender.
  const bool flexfec_enabled = (flexfec_sender_ != nullptr);

  // Consistency of NACK and RED+ULPFEC parameters is checked in this function.
  const bool nack_enabled = config_->rtp.nack.rtp_history_ms > 0;
  int red_payload_type = config_->rtp.ulpfec.red_payload_type;
  int ulpfec_payload_type = config_->rtp.ulpfec.ulpfec_payload_type;

  // Shorthands.
  auto IsRedEnabled = [&]() { return red_payload_type >= 0; };
  auto DisableRed = [&]() { red_payload_type = -1; };
  auto IsUlpfecEnabled = [&]() { return ulpfec_payload_type >= 0; };
  auto DisableUlpfec = [&]() { ulpfec_payload_type = -1; };

  if (webrtc::field_trial::IsEnabled("WebRTC-DisableUlpFecExperiment")) {
    RTC_LOG(LS_INFO) << "Experiment to disable sending ULPFEC is enabled.";
    DisableUlpfec();
  }

  // If enabled, FlexFEC takes priority over RED+ULPFEC.
  if (flexfec_enabled) {
    // We can safely disable RED here, because if the remote supports FlexFEC,
    // we know that it has a receiver without the RED/RTX workaround.
    // See http://crbug.com/webrtc/6650 for more information.
    if (IsRedEnabled()) {
      RTC_LOG(LS_INFO) << "Both FlexFEC and RED are configured. Disabling RED.";
      DisableRed();
    }
    if (IsUlpfecEnabled()) {
      RTC_LOG(LS_INFO)
          << "Both FlexFEC and ULPFEC are configured. Disabling ULPFEC.";
      DisableUlpfec();
    }
  }

  // Payload types without picture ID cannot determine that a stream is complete
  // without retransmitting FEC, so using ULPFEC + NACK for H.264 (for instance)
  // is a waste of bandwidth since FEC packets still have to be transmitted.
  // Note that this is not the case with FlexFEC.
  if (nack_enabled && IsUlpfecEnabled() &&
      !PayloadTypeSupportsSkippingFecPackets(config_->rtp.payload_name)) {
    RTC_LOG(LS_WARNING)
        << "Transmitting payload type without picture ID using "
           "NACK+ULPFEC is a waste of bandwidth since ULPFEC packets "
           "also have to be retransmitted. Disabling ULPFEC.";
    DisableUlpfec();
  }

  // Verify payload types.
  //
  // Due to how old receivers work, we need to always send RED if it has been
  // negotiated. This is a remnant of an old RED/RTX workaround, see
  // https://codereview.webrtc.org/2469093003.
  // TODO(brandtr): This change went into M56, so we can remove it in ~M59.
  // At that time, we can disable RED whenever ULPFEC is disabled, as there is
  // no point in using RED without ULPFEC.
  if (IsRedEnabled()) {
    RTC_DCHECK_GE(red_payload_type, 0);
    RTC_DCHECK_LE(red_payload_type, 127);
  }
  if (IsUlpfecEnabled()) {
    RTC_DCHECK_GE(ulpfec_payload_type, 0);
    RTC_DCHECK_LE(ulpfec_payload_type, 127);
    if (!IsRedEnabled()) {
      RTC_LOG(LS_WARNING)
          << "ULPFEC is enabled but RED is disabled. Disabling ULPFEC.";
      DisableUlpfec();
    }
  }

  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
    // Set NACK.
    rtp_rtcp->SetStorePacketsStatus(true, kMinSendSidePacketHistorySize);
    // Set RED/ULPFEC information.
    rtp_rtcp->SetUlpfecConfig(red_payload_type, ulpfec_payload_type);
  }

  // Currently, both ULPFEC and FlexFEC use the same FEC rate calculation logic,
  // so enable that logic if either of those FEC schemes are enabled.
  fec_controller_->SetProtectionMethod(flexfec_enabled || IsUlpfecEnabled(),
                                       nack_enabled);
}

void VideoSendStreamImpl::ConfigureSsrcs() {
  RTC_DCHECK_RUN_ON(worker_queue_);
  // Configure regular SSRCs.
  for (size_t i = 0; i < config_->rtp.ssrcs.size(); ++i) {
    uint32_t ssrc = config_->rtp.ssrcs[i];
    RtpRtcp* const rtp_rtcp = rtp_rtcp_modules_[i];
    rtp_rtcp->SetSSRC(ssrc);

    // Restore RTP state if previous existed.
    VideoSendStream::RtpStateMap::iterator it = suspended_ssrcs_.find(ssrc);
    if (it != suspended_ssrcs_.end())
      rtp_rtcp->SetRtpState(it->second);
  }

  // Set up RTX if available.
  if (config_->rtp.rtx.ssrcs.empty())
    return;

  // Configure RTX SSRCs.
  RTC_DCHECK_EQ(config_->rtp.rtx.ssrcs.size(), config_->rtp.ssrcs.size());
  for (size_t i = 0; i < config_->rtp.rtx.ssrcs.size(); ++i) {
    uint32_t ssrc = config_->rtp.rtx.ssrcs[i];
    RtpRtcp* const rtp_rtcp = rtp_rtcp_modules_[i];
    rtp_rtcp->SetRtxSsrc(ssrc);
    VideoSendStream::RtpStateMap::iterator it = suspended_ssrcs_.find(ssrc);
    if (it != suspended_ssrcs_.end())
      rtp_rtcp->SetRtxState(it->second);
  }

  // Configure RTX payload types.
  RTC_DCHECK_GE(config_->rtp.rtx.payload_type, 0);
  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
    rtp_rtcp->SetRtxSendPayloadType(config_->rtp.rtx.payload_type,
                                    config_->rtp.payload_type);
    rtp_rtcp->SetRtxSendStatus(kRtxRetransmitted | kRtxRedundantPayloads);
  }
  if (config_->rtp.ulpfec.red_payload_type != -1 &&
      config_->rtp.ulpfec.red_rtx_payload_type != -1) {
    for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
      rtp_rtcp->SetRtxSendPayloadType(config_->rtp.ulpfec.red_rtx_payload_type,
                                      config_->rtp.ulpfec.red_payload_type);
    }
  }
}

std::map<uint32_t, RtpState> VideoSendStreamImpl::GetRtpStates() const {
  RTC_DCHECK_RUN_ON(worker_queue_);
  std::map<uint32_t, RtpState> rtp_states;

  for (size_t i = 0; i < config_->rtp.ssrcs.size(); ++i) {
    uint32_t ssrc = config_->rtp.ssrcs[i];
    RTC_DCHECK_EQ(ssrc, rtp_rtcp_modules_[i]->SSRC());
    rtp_states[ssrc] = rtp_rtcp_modules_[i]->GetRtpState();
  }

  for (size_t i = 0; i < config_->rtp.rtx.ssrcs.size(); ++i) {
    uint32_t ssrc = config_->rtp.rtx.ssrcs[i];
    rtp_states[ssrc] = rtp_rtcp_modules_[i]->GetRtxState();
  }

  if (flexfec_sender_) {
    uint32_t ssrc = config_->rtp.flexfec.ssrc;
    rtp_states[ssrc] = flexfec_sender_->GetRtpState();
  }

  return rtp_states;
}

std::map<uint32_t, RtpPayloadState> VideoSendStreamImpl::GetRtpPayloadStates()
    const {
  RTC_DCHECK_RUN_ON(worker_queue_);
  return payload_router_.GetRtpPayloadStates();
}

void VideoSendStreamImpl::SignalNetworkState(NetworkState state) {
  RTC_DCHECK_RUN_ON(worker_queue_);
  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
    rtp_rtcp->SetRTCPStatus(state == kNetworkUp ? config_->rtp.rtcp_mode
                                                : RtcpMode::kOff);
  }
}

uint32_t VideoSendStreamImpl::OnBitrateUpdated(uint32_t bitrate_bps,
                                               uint8_t fraction_loss,
                                               int64_t rtt,
                                               int64_t probing_interval_ms) {
  RTC_DCHECK_RUN_ON(worker_queue_);
  RTC_DCHECK(payload_router_.IsActive())
      << "VideoSendStream::Start has not been called.";

  // Substract overhead from bitrate.
  rtc::CritScope lock(&overhead_bytes_per_packet_crit_);
  uint32_t payload_bitrate_bps = bitrate_bps;
  if (send_side_bwe_with_overhead_) {
    payload_bitrate_bps -= CalculateOverheadRateBps(
        CalculatePacketRate(bitrate_bps,
                            config_->rtp.max_packet_size +
                                transport_overhead_bytes_per_packet_),
        overhead_bytes_per_packet_ + transport_overhead_bytes_per_packet_,
        bitrate_bps);
  }

  // Get the encoder target rate. It is the estimated network rate -
  // protection overhead.
  encoder_target_rate_bps_ = fec_controller_->UpdateFecRates(
      payload_bitrate_bps, stats_proxy_->GetSendFrameRate(), fraction_loss,
      loss_mask_vector_, rtt);
  loss_mask_vector_.clear();

  uint32_t encoder_overhead_rate_bps =
      send_side_bwe_with_overhead_
          ? CalculateOverheadRateBps(
                CalculatePacketRate(encoder_target_rate_bps_,
                                    config_->rtp.max_packet_size +
                                        transport_overhead_bytes_per_packet_ -
                                        overhead_bytes_per_packet_),
                overhead_bytes_per_packet_ +
                    transport_overhead_bytes_per_packet_,
                bitrate_bps - encoder_target_rate_bps_)
          : 0;

  // When the field trial "WebRTC-SendSideBwe-WithOverhead" is enabled
  // protection_bitrate includes overhead.
  uint32_t protection_bitrate =
      bitrate_bps - (encoder_target_rate_bps_ + encoder_overhead_rate_bps);

  encoder_target_rate_bps_ =
      std::min(encoder_max_bitrate_bps_, encoder_target_rate_bps_);
  video_stream_encoder_->OnBitrateUpdated(encoder_target_rate_bps_,
                                          fraction_loss, rtt);
  stats_proxy_->OnSetEncoderTargetRate(encoder_target_rate_bps_);
  return protection_bitrate;
}

void VideoSendStreamImpl::EnableEncodedFrameRecording(
    const std::vector<rtc::PlatformFile>& files,
    size_t byte_limit) {
  {
    rtc::CritScope lock(&ivf_writers_crit_);
    for (unsigned int i = 0; i < kMaxSimulcastStreams; ++i) {
      if (i < files.size()) {
        file_writers_[i] = IvfFileWriter::Wrap(rtc::File(files[i]), byte_limit);
      } else {
        file_writers_[i].reset();
      }
    }
  }

  if (!files.empty()) {
    // Make a keyframe appear as early as possible in the logs, to give actually
    // decodable output.
    video_stream_encoder_->SendKeyFrame();
  }
}

int VideoSendStreamImpl::ProtectionRequest(
    const FecProtectionParams* delta_params,
    const FecProtectionParams* key_params,
    uint32_t* sent_video_rate_bps,
    uint32_t* sent_nack_rate_bps,
    uint32_t* sent_fec_rate_bps) {
  RTC_DCHECK_RUN_ON(worker_queue_);
  *sent_video_rate_bps = 0;
  *sent_nack_rate_bps = 0;
  *sent_fec_rate_bps = 0;
  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
    uint32_t not_used = 0;
    uint32_t module_video_rate = 0;
    uint32_t module_fec_rate = 0;
    uint32_t module_nack_rate = 0;
    rtp_rtcp->SetFecParameters(*delta_params, *key_params);
    rtp_rtcp->BitrateSent(&not_used, &module_video_rate, &module_fec_rate,
                          &module_nack_rate);
    *sent_video_rate_bps += module_video_rate;
    *sent_nack_rate_bps += module_nack_rate;
    *sent_fec_rate_bps += module_fec_rate;
  }
  return 0;
}

void VideoSendStreamImpl::OnOverheadChanged(size_t overhead_bytes_per_packet) {
  rtc::CritScope lock(&overhead_bytes_per_packet_crit_);
  overhead_bytes_per_packet_ = overhead_bytes_per_packet;
}

void VideoSendStreamImpl::SetTransportOverhead(
    size_t transport_overhead_bytes_per_packet) {
  if (transport_overhead_bytes_per_packet >= static_cast<int>(kPathMTU)) {
    RTC_LOG(LS_ERROR) << "Transport overhead exceeds size of ethernet frame";
    return;
  }

  transport_overhead_bytes_per_packet_ = transport_overhead_bytes_per_packet;

  size_t rtp_packet_size =
      std::min(config_->rtp.max_packet_size,
               kPathMTU - transport_overhead_bytes_per_packet_);

  for (RtpRtcp* rtp_rtcp : rtp_rtcp_modules_) {
    rtp_rtcp->SetMaxRtpPacketSize(rtp_packet_size);
  }
}

void VideoSendStreamImpl::OnPacketAdded(uint32_t ssrc, uint16_t seq_num) {
  if (!worker_queue_->IsCurrent()) {
    auto ptr = weak_ptr_;
    worker_queue_->PostTask([=] {
      if (!ptr.get())
        return;
      ptr->OnPacketAdded(ssrc, seq_num);
    });
    return;
  }
  const auto ssrcs = config_->rtp.ssrcs;
  if (std::find(ssrcs.begin(), ssrcs.end(), ssrc) != ssrcs.end()) {
    feedback_packet_seq_num_set_.insert(seq_num);
    if (feedback_packet_seq_num_set_.size() > kSendSideSeqNumSetMaxSize) {
      RTC_LOG(LS_WARNING) << "Feedback packet sequence number set exceed it's "
                             "max size', will get reset.";
      feedback_packet_seq_num_set_.clear();
    }
  }
}

void VideoSendStreamImpl::OnPacketFeedbackVector(
    const std::vector<PacketFeedback>& packet_feedback_vector) {
  if (!worker_queue_->IsCurrent()) {
    auto ptr = weak_ptr_;
    worker_queue_->PostTask([=] {
      if (!ptr.get())
        return;
      ptr->OnPacketFeedbackVector(packet_feedback_vector);
    });
    return;
  }
  // Lost feedbacks are not considered to be lost packets.
  for (const PacketFeedback& packet : packet_feedback_vector) {
    if (auto it = feedback_packet_seq_num_set_.find(packet.sequence_number) !=
                  feedback_packet_seq_num_set_.end()) {
      const bool lost = packet.arrival_time_ms == PacketFeedback::kNotReceived;
      loss_mask_vector_.push_back(lost);
      feedback_packet_seq_num_set_.erase(it);
    }
  }
}
}  // namespace internal
}  // namespace webrtc
