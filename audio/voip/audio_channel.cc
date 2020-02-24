//
//  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
//
//  Use of this source code is governed by a BSD-style license
//  that can be found in the LICENSE file in the root of the source
//  tree. An additional intellectual property rights grant can be found
//  in the file PATENTS.  All contributing project authors may
//  be found in the AUTHORS file in the root of the source tree.
//

#include "audio/voip/audio_channel.h"

#include <utility>
#include <vector>

#include "api/audio_codecs/audio_format.h"
#include "api/task_queue/task_queue_factory.h"
#include "modules/rtp_rtcp/include/receive_statistics.h"
#include "rtc_base/checks.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/location.h"
#include "rtc_base/logging.h"

namespace webrtc {

AudioChannel::AudioChannel(
    std::shared_ptr<RtpRtcp> rtp_rtcp,
    Clock* clock,
    TaskQueueFactory* task_queue_factory,
    ProcessThread* process_thread,
    AudioMixer* audio_mixer,
    rtc::scoped_refptr<AudioDecoderFactory> decoder_factory,
    std::unique_ptr<ReceiveStatistics> receive_statistics)
    : AudioIngress(rtp_rtcp,
                   clock,
                   decoder_factory,
                   std::move(receive_statistics)),
      AudioEgress(rtp_rtcp, clock, task_queue_factory),
      audio_mixer_(audio_mixer),
      process_thread_(process_thread),
      rtp_rtcp_(rtp_rtcp) {
  RTC_DCHECK(task_queue_factory);
  RTC_DCHECK(process_thread);
  RTC_DCHECK(audio_mixer);

  rtp_rtcp_->SetSendingMediaStatus(false);
  rtp_rtcp_->SetRTCPStatus(RtcpMode::kCompound);

  // Process thread periodically services rtp stack for rtcp
  process_thread_->RegisterModule(rtp_rtcp_.get(), RTC_FROM_HERE);

  // Add this instance of audio ingress to be part of audio mixer for ADM to
  // fetch audio samples to play.
  audio_mixer_->AddSource(this);
}

AudioChannel::~AudioChannel() {
  audio_mixer_->RemoveSource(this);
  process_thread_->DeRegisterModule(rtp_rtcp_.get());
}

void AudioChannel::StartSend() {
  // Start sending media.
  AudioEgress::StartSend();

  // Start sending with rtp stack if not sending yet
  if (!rtp_rtcp_->Sending()) {
    if (rtp_rtcp_->SetSendingStatus(true) != 0) {
      RTC_DLOG(LS_ERROR) << "StartSend() RTP/RTCP failed to start sending";
    }
  }
}

void AudioChannel::StopSend() {
  // Stop sending media.
  AudioEgress::StopSend();

  // If we are not playing and rtp is sending then turn it off.
  if (!Playing() && rtp_rtcp_->Sending()) {
    // Thiw will reset sending SSRC and sequence number and
    // triggers direct transmission of RTCP BYE
    if (rtp_rtcp_->SetSendingStatus(false) == -1) {
      RTC_DLOG(LS_ERROR) << "StopSend() RTP/RTCP failed to stop sending";
    }
  }
}

void AudioChannel::StartPlay() {
  AudioIngress::StartPlay();

  // Even in recv-only mode, RTCP receiver report is required to be sent back.
  if (!rtp_rtcp_->Sending()) {
    if (rtp_rtcp_->SetSendingStatus(true) != 0) {
      RTC_DLOG(LS_ERROR) << "StartPlay() RTP/RTCP failed to start sending";
    }
  }
}

void AudioChannel::StopPlay() {
  AudioIngress::StopPlay();

  // when stop playing and sending then turn off rtp stack
  if (!Sending() && rtp_rtcp_->Sending()) {
    if (rtp_rtcp_->SetSendingStatus(false) != 0) {
      RTC_DLOG(LS_ERROR) << "StopPlay() RTP/RTCP failed to stop sending";
    }
  }
}

}  // namespace webrtc
