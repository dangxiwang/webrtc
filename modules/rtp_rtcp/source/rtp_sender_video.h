/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_RTP_RTCP_SOURCE_RTP_SENDER_VIDEO_H_
#define MODULES_RTP_RTCP_SOURCE_RTP_SENDER_VIDEO_H_

#include <map>
#include <memory>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "api/array_view.h"
#include "api/frame_transformer_interface.h"
#include "api/rtp_headers.h"
#include "api/scoped_refptr.h"
#include "api/task_queue/task_queue_base.h"
#include "api/transport/rtp/dependency_descriptor.h"
#include "api/video/video_codec_type.h"
#include "api/video/video_frame_type.h"
#include "modules/include/module_common_types.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/absolute_capture_time_sender.h"
#include "modules/rtp_rtcp/source/rtp_rtcp_config.h"
#include "modules/rtp_rtcp/source/rtp_sender.h"
#include "modules/rtp_rtcp/source/rtp_sender_video_frame_transformer_delegate.h"
#include "modules/rtp_rtcp/source/rtp_video_header.h"
#include "modules/rtp_rtcp/source/video_fec_generator.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/one_time_event.h"
#include "rtc_base/race_checker.h"
#include "rtc_base/rate_statistics.h"
#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

class FrameEncryptorInterface;
class RtpPacketizer;
class RtpPacketToSend;

// kConditionallyRetransmitHigherLayers allows retransmission of video frames
// in higher layers if either the last frame in that layer was too far back in
// time, or if we estimate that a new frame will be available in a lower layer
// in a shorter time than it would take to request and receive a retransmission.
enum RetransmissionMode : uint8_t {
  kRetransmitOff = 0x0,
  kRetransmitBaseLayer = 0x2,
  kRetransmitHigherLayers = 0x4,
  kRetransmitAllLayers = 0x6,
  kConditionallyRetransmitHigherLayers = 0x8
};

class RTPSenderVideo {
 public:
  static constexpr int64_t kTLRateWindowSizeMs = 2500;

  struct Config {
    Config() = default;
    Config(const Config&) = delete;
    Config(Config&&) = default;

    // All members of this struct, with the exception of |field_trials|, are
    // expected to outlive the RTPSenderVideo object they are passed to.
    Clock* clock = nullptr;
    RTPSender* rtp_sender = nullptr;
    FlexfecSender* flexfec_sender = nullptr;
    VideoFecGenerator* fec_generator = nullptr;
    // Some FEC data is duplicated here in preparation of moving FEC to
    // the egress stage.
    absl::optional<VideoFecGenerator::FecType> fec_type;
    size_t fec_overhead_bytes = 0;  // Per packet max FEC overhead.
    FrameEncryptorInterface* frame_encryptor = nullptr;
    bool require_frame_encryption = false;
    bool enable_retransmit_all_layers = false;
    absl::optional<int> red_payload_type;
    const WebRtcKeyValueConfig* field_trials = nullptr;
    rtc::scoped_refptr<FrameTransformerInterface> frame_transformer;
    TaskQueueBase* send_transport_queue = nullptr;
  };

  explicit RTPSenderVideo(const Config& config);

  virtual ~RTPSenderVideo();

  // expected_retransmission_time_ms.has_value() -> retransmission allowed.
  // Calls to this method is assumed to be externally serialized.
  bool SendVideo(int payload_type,
                 absl::optional<VideoCodecType> codec_type,
                 uint32_t rtp_timestamp,
                 int64_t capture_time_ms,
                 rtc::ArrayView<const uint8_t> payload,
                 const RTPFragmentationHeader* fragmentation,
                 RTPVideoHeader video_header,
                 absl::optional<int64_t> expected_retransmission_time_ms);

  bool SendEncodedImage(
      int payload_type,
      absl::optional<VideoCodecType> codec_type,
      uint32_t rtp_timestamp,
      const EncodedImage& encoded_image,
      const RTPFragmentationHeader* fragmentation,
      RTPVideoHeader video_header,
      absl::optional<int64_t> expected_retransmission_time_ms);

  // Configures video structures produced by encoder to send using the
  // dependency descriptor rtp header extension. Next call to SendVideo should
  // have video_header.frame_type == kVideoFrameKey.
  // All calls to SendVideo after this call must use video_header compatible
  // with the video_structure.
  void SetVideoStructure(const FrameDependencyStructure* video_structure);
  void SetVideoStructureUnderLock(
      const FrameDependencyStructure* video_structure);

  uint32_t VideoBitrateSent() const;

  // Returns the current packetization overhead rate, in bps. Note that this is
  // the payload overhead, eg the VP8 payload headers, not the RTP headers
  // or extension/
  uint32_t PacketizationOverheadBps() const;

 protected:
  static uint8_t GetTemporalId(const RTPVideoHeader& header);
  bool AllowRetransmission(uint8_t temporal_id,
                           int32_t retransmission_settings,
                           int64_t expected_retransmission_time_ms);

 private:
  struct TemporalLayerStats {
    TemporalLayerStats()
        : frame_rate_fp1000s(kTLRateWindowSizeMs, 1000 * 1000),
          last_frame_time_ms(0) {}
    // Frame rate, in frames per 1000 seconds. This essentially turns the fps
    // value into a fixed point value with three decimals. Improves precision at
    // low frame rates.
    RateStatistics frame_rate_fp1000s;
    int64_t last_frame_time_ms;
  };

  void AddRtpHeaderExtensions(
      const RTPVideoHeader& video_header,
      const absl::optional<AbsoluteCaptureTime>& absolute_capture_time,
      bool first_packet,
      bool last_packet,
      RtpPacketToSend* packet) const
      RTC_EXCLUSIVE_LOCKS_REQUIRED(send_checker_);

  size_t FecPacketOverhead() const RTC_EXCLUSIVE_LOCKS_REQUIRED(send_checker_);

  void LogAndSendToNetwork(
      std::vector<std::unique_ptr<RtpPacketToSend>> packets,
      size_t unpacketized_payload_size);

  bool red_enabled() const { return red_payload_type_.has_value(); }

  bool UpdateConditionalRetransmit(uint8_t temporal_id,
                                   int64_t expected_retransmission_time_ms)
      RTC_EXCLUSIVE_LOCKS_REQUIRED(stats_crit_);

  void MaybeUpdateCurrentPlayoutDelay(const RTPVideoHeader& header)
      RTC_EXCLUSIVE_LOCKS_REQUIRED(send_checker_);

  RTPSender* const rtp_sender_;
  Clock* const clock_;

  const int32_t retransmission_settings_;

  // These members should only be accessed from within SendVideo() to avoid
  // potential race conditions.
  rtc::RaceChecker send_checker_;
  VideoRotation last_rotation_ RTC_GUARDED_BY(send_checker_);
  absl::optional<ColorSpace> last_color_space_ RTC_GUARDED_BY(send_checker_);
  bool transmit_color_space_next_frame_ RTC_GUARDED_BY(send_checker_);
  std::unique_ptr<FrameDependencyStructure> video_structure_
      RTC_GUARDED_BY(send_checker_);

  // Current target playout delay.
  PlayoutDelay current_playout_delay_ RTC_GUARDED_BY(send_checker_);
  // Flag indicating if we need to propagate |current_playout_delay_| in order
  // to guarantee it gets delivered.
  bool playout_delay_pending_;

  // Should never be held when calling out of this class.
  rtc::CriticalSection crit_;

  const absl::optional<int> red_payload_type_;
  VideoFecGenerator* const fec_generator_;
  absl::optional<VideoFecGenerator::FecType> fec_type_;
  const size_t fec_overhead_bytes_;  // Per packet max FEC overhead.

  rtc::CriticalSection stats_crit_;
  // Bitrate used for video payload and RTP headers.
  RateStatistics video_bitrate_ RTC_GUARDED_BY(stats_crit_);
  RateStatistics packetization_overhead_bitrate_ RTC_GUARDED_BY(stats_crit_);

  std::map<int, TemporalLayerStats> frame_stats_by_temporal_layer_
      RTC_GUARDED_BY(stats_crit_);

  OneTimeEvent first_frame_sent_;

  // E2EE Custom Video Frame Encryptor (optional)
  FrameEncryptorInterface* const frame_encryptor_ = nullptr;
  // If set to true will require all outgoing frames to pass through an
  // initialized frame_encryptor_ before being sent out of the network.
  // Otherwise these payloads will be dropped.
  const bool require_frame_encryption_;
  // Set to true if the generic descriptor should be authenticated.
  const bool generic_descriptor_auth_experiment_;

  AbsoluteCaptureTimeSender absolute_capture_time_sender_;

  const rtc::scoped_refptr<RTPSenderVideoFrameTransformerDelegate>
      frame_transformer_delegate_;
};

}  // namespace webrtc

#endif  // MODULES_RTP_RTCP_SOURCE_RTP_SENDER_VIDEO_H_
