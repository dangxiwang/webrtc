//
//  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
//
//  Use of this source code is governed by a BSD-style license
//  that can be found in the LICENSE file in the root of the source
//  tree. An additional intellectual property rights grant can be found
//  in the file PATENTS.  All contributing project authors may
//  be found in the AUTHORS file in the root of the source tree.
//

#include "audio/voip/audio_ingress.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "api/audio_codecs/audio_format.h"
#include "audio/utility/audio_frame_operations.h"
#include "modules/audio_coding/include/audio_coding_module.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/safe_minmax.h"

namespace webrtc {

namespace {

constexpr double kAudioSampleDurationSeconds = 0.01;

}  // namespace

AudioIngress::AudioIngress(
    std::shared_ptr<RtpRtcp> rtp_rtcp,
    Clock* clock,
    rtc::scoped_refptr<AudioDecoderFactory> decoder_factory,
    std::unique_ptr<ReceiveStatistics> receive_statistics)
    : rtp_rtcp_(rtp_rtcp),
      rtp_receive_statistics_(std::move(receive_statistics)),
      ntp_estimator_(std::make_unique<RemoteNtpTimeEstimator>(clock)),
      rtp_ts_wraparound_handler_(
          std::make_unique<rtc::TimestampWrapAroundHandler>()) {
  AudioCodingModule::Config acm_config;
  acm_config.neteq_config.enable_muted_state = true;
  acm_config.decoder_factory = decoder_factory;
  acm_receiver_ = std::make_unique<acm2::AcmReceiver>(acm_config);

  acm_receiver_->ResetInitialDelay();
  acm_receiver_->SetMinimumDelay(0);
  acm_receiver_->SetMaximumDelay(0);
  acm_receiver_->FlushBuffers();

  output_audio_level_.ResetLevelFullRange();
}

AudioIngress::~AudioIngress() = default;

void AudioIngress::StartPlay() {
  RTC_DCHECK_RUN_ON(&worker_thread_checker_);
  playing_ = true;
}

void AudioIngress::StopPlay() {
  RTC_DCHECK_RUN_ON(&worker_thread_checker_);
  playing_ = false;
  output_audio_level_.ResetLevelFullRange();
}

AudioMixer::Source::AudioFrameInfo AudioIngress::GetAudioFrameWithInfo(
    int sample_rate_hz,
    AudioFrame* audio_frame) {
  audio_frame->sample_rate_hz_ = sample_rate_hz;

  // Get 10ms raw PCM data from the ACM (mixer limits output frequency)
  bool muted;
  if (acm_receiver_->GetAudio(audio_frame->sample_rate_hz_, audio_frame,
                              &muted) == -1) {
    RTC_DLOG(LS_ERROR)
        << "ChannelReceive::GetAudioFrame() PlayoutData10Ms() failed!";
    // In all likelihood, the audio in this frame is garbage. We return an
    // error so that the audio mixer module doesn't add it to the mix. As
    // a result, it won't be played out and the actions skipped here are
    // irrelevant.
    return AudioMixer::Source::AudioFrameInfo::kError;
  }

  if (muted) {
    // TODO(henrik.lundin): We should be able to do better than this. But we
    // will have to go through all the cases below where the audio samples may
    // be used, and handle the muted case in some way.
    AudioFrameOperations::Mute(audio_frame);
  }

  // Measure audio level (0-9)
  // TODO(henrik.lundin) Use the |muted| information here too.
  // TODO(deadbeef): Use RmsLevel for |_outputAudioLevel| (see
  // https://crbug.com/webrtc/7517).
  output_audio_level_.ComputeLevel(*audio_frame, kAudioSampleDurationSeconds);

  if (capture_start_rtp_time_stamp_ < 0 && audio_frame->timestamp_ != 0) {
    // The first frame with a valid rtp timestamp.
    capture_start_rtp_time_stamp_ = audio_frame->timestamp_;
  }

  if (capture_start_rtp_time_stamp_ >= 0) {
    // audio_frame.timestamp_ should be valid from now on.

    // Compute elapsed time.
    int64_t unwrap_timestamp =
        rtp_ts_wraparound_handler_->Unwrap(audio_frame->timestamp_);
    audio_frame->elapsed_time_ms_ =
        (unwrap_timestamp - capture_start_rtp_time_stamp_) /
        (GetRtpTimestampRateHz() / 1000);
    // Get ntp time.
    {
      rtc::CritScope lock(&ntp_lock_);
      audio_frame->ntp_time_ms_ =
          ntp_estimator_->Estimate(audio_frame->timestamp_);
    }
  }

  return muted ? AudioMixer::Source::AudioFrameInfo::kMuted
               : AudioMixer::Source::AudioFrameInfo::kNormal;
}

int AudioIngress::Ssrc() const {
  return static_cast<int>(remote_ssrc_);
}

int AudioIngress::PreferredSampleRate() const {
  // Return the bigger of playout and receive frequency in the ACM.
  return std::max(acm_receiver_->last_packet_sample_rate_hz().value_or(0),
                  acm_receiver_->last_output_sample_rate_hz());
}

void AudioIngress::SetReceiveCodecs(
    const std::map<int, SdpAudioFormat>& codecs) {
  {
    rtc::CritScope scoped(&payload_type_lock_);
    for (const auto& kv : codecs) {
      RTC_DCHECK_GE(kv.second.clockrate_hz, 1000);
      payload_type_frequencies_[kv.first] = kv.second.clockrate_hz;
    }
  }
  acm_receiver_->SetCodecs(codecs);
}

void AudioIngress::ReceivedRTPPacket(const uint8_t* data, size_t length) {
  if (!Playing()) {
    return;
  }

  RtpPacketReceived rtp_packet;
  rtp_packet.Parse(data, length);
  rtp_receive_statistics_->OnRtpPacket(rtp_packet);

  {
    rtc::CritScope scoped(&payload_type_lock_);
    const auto& it = payload_type_frequencies_.find(rtp_packet.PayloadType());
    if (it == payload_type_frequencies_.end()) {
      return;
    }
    rtp_packet.set_payload_type_frequency(it->second);
  }

  RTPHeader header;
  rtp_packet.GetHeader(&header);

  size_t packet_length = rtp_packet.size();
  const uint8_t* payload = rtp_packet.data() + header.headerLength;
  assert(packet_length >= header.headerLength);
  size_t payload_length = packet_length - header.headerLength;
  size_t payload_data_length = payload_length - header.paddingLength;

  // Push the incoming payload (parsed and ready for decoding) into the ACM
  if (acm_receiver_->InsertPacket(
          header,
          rtc::ArrayView<const uint8_t>(payload, payload_data_length)) != 0) {
    RTC_DLOG(LS_ERROR) << "AudioIngress::ReceivedRTPPacket() unable to "
                          "push data to the ACM";
  }
}

void AudioIngress::ReceivedRTCPPacket(const uint8_t* data, size_t length) {
  // Deliver RTCP packet to RTP/RTCP module for parsing
  rtp_rtcp_->IncomingRtcpPacket(data, length);

  int64_t rtt = GetRTT();
  if (rtt == 0) {
    // Waiting for valid RTT.
    return;
  }

  uint32_t ntp_secs = 0, ntp_frac = 0, rtp_timestamp = 0;
  if (rtp_rtcp_->RemoteNTP(&ntp_secs, &ntp_frac, nullptr, nullptr,
                           &rtp_timestamp) != 0) {
    // Waiting for RTCP.
    return;
  }

  {
    rtc::CritScope lock(&ntp_lock_);
    ntp_estimator_->UpdateRtcpTimestamp(rtt, ntp_secs, ntp_frac, rtp_timestamp);
  }
}

int64_t AudioIngress::GetRTT() {
  std::vector<RTCPReportBlock> report_blocks;
  rtp_rtcp_->RemoteRTCPStat(&report_blocks);

  if (report_blocks.empty()) {
    return 0;
  }

  // TODO(natim): handle the case where remote end is changing ssrc and update
  // accordingly here.
  //
  // We don't know in advance the remote ssrc used by the other end's receiver
  // reports, so use the SSRC of the first report block as remote ssrc for now.
  if (report_blocks[0].sender_ssrc != remote_ssrc_) {
    remote_ssrc_ = report_blocks[0].sender_ssrc;
    rtp_rtcp_->SetRemoteSSRC(remote_ssrc_);
  }

  int64_t rtt = 0;
  int64_t avg_rtt = 0;
  int64_t max_rtt = 0;
  int64_t min_rtt = 0;

  if (rtp_rtcp_->RTT(remote_ssrc_, &rtt, &avg_rtt, &min_rtt, &max_rtt) != 0) {
    return 0;
  }
  return rtt;
}

int AudioIngress::GetSpeechOutputLevelFullRange() const {
  RTC_DCHECK_RUN_ON(&worker_thread_checker_);
  return output_audio_level_.LevelFullRange();
}

bool AudioIngress::Playing() const {
  return playing_;
}

NetworkStatistics AudioIngress::GetNetworkStatistics() const {
  RTC_DCHECK_RUN_ON(&worker_thread_checker_);
  NetworkStatistics stats;
  acm_receiver_->GetNetworkStatistics(&stats);
  return stats;
}

AudioDecodingCallStats AudioIngress::GetDecodingCallStatistics() const {
  RTC_DCHECK_RUN_ON(&worker_thread_checker_);
  AudioDecodingCallStats stats;
  acm_receiver_->GetDecodingCallStatistics(&stats);
  return stats;
}

int AudioIngress::GetRtpTimestampRateHz() const {
  const auto decoder = acm_receiver_->LastDecoder();
  // Default to the playout frequency if we've not gotten any packets yet.
  // TODO(ossu): Zero clockrate can only happen if we've added an external
  // decoder for a format we don't support internally. Remove once that way of
  // adding decoders is gone!
  // TODO(kwiberg): `decoder->second.clockrate_hz` is an RTP clockrate as it
  // should, but `acm_receiver_->last_output_sample_rate_hz()` is a codec sample
  // rate, which is not always the same thing.
  return (decoder && decoder->second.clockrate_hz != 0)
             ? decoder->second.clockrate_hz
             : acm_receiver_->last_output_sample_rate_hz();
}

}  // namespace webrtc
