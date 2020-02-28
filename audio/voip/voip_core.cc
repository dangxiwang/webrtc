//
//  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
//
//  Use of this source code is governed by a BSD-style license
//  that can be found in the LICENSE file in the root of the source
//  tree. An additional intellectual property rights grant can be found
//  in the file PATENTS.  All contributing project authors may
//  be found in the AUTHORS file in the root of the source tree.
//

#include "audio/voip/voip_core.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "api/audio_codecs/audio_format.h"
#include "api/task_queue/default_task_queue_factory.h"
#include "modules/rtp_rtcp/include/receive_statistics.h"
#include "modules/rtp_rtcp/include/rtp_rtcp.h"
#include "rtc_base/checks.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/logging.h"

namespace webrtc {

namespace {

constexpr int kRtcpReportIntervalMs = 5000;

}  // namespace

VoipBase* VoipCore::Base() {
  return this;
}

VoipNetwork* VoipCore::Network() {
  return this;
}

VoipCodec* VoipCore::Codec() {
  return this;
}

bool VoipCore::Init(std::unique_ptr<TaskQueueFactory> task_queue_factory,
                    std::unique_ptr<AudioProcessing> audio_processing,
                    rtc::scoped_refptr<AudioDeviceModule> audio_device_module,
                    rtc::scoped_refptr<AudioEncoderFactory> encoder_factory,
                    rtc::scoped_refptr<AudioDecoderFactory> decoder_factory) {
  audio_processing_ = std::move(audio_processing);
  task_queue_factory_ = std::move(task_queue_factory);
  audio_device_module_ = std::move(audio_device_module);

  encoder_factory_ = encoder_factory;
  decoder_factory_ = decoder_factory;

  process_thread_ = ProcessThread::Create("ModuleProcessThread");
  audio_mixer_ = AudioMixerImpl::Create();

  AudioProcessing::Config apm_config = audio_processing_->GetConfig();
  apm_config.echo_canceller.enabled = true;
  audio_processing_->ApplyConfig(apm_config);

  // audio transport depends on mixer and APM
  audio_transport_ = std::make_unique<AudioTransportImpl>(
      audio_mixer_.get(), audio_processing_.get());

  if (audio_device_module_->Init() != 0) {
    RTC_LOG(LS_ERROR) << "Failed to initialize the ADM.";
    return false;
  }

  if (audio_device_module_->SetPlayoutDevice(0) != 0) {
    RTC_LOG(LS_ERROR) << "Unable to set playout device.";
    return false;
  }
  if (audio_device_module_->InitSpeaker() != 0) {
    RTC_LOG(LS_ERROR) << "Unable to access speaker.";
  }

  // Set number of channels
  bool available = false;
  if (audio_device_module_->StereoPlayoutIsAvailable(&available) != 0) {
    RTC_LOG(LS_ERROR) << "Failed to query stereo playout.";
  }
  if (audio_device_module_->SetStereoPlayout(available) != 0) {
    RTC_LOG(LS_ERROR) << "Failed to set stereo playout mode.";
  }

  // Recording device.
  if (audio_device_module_->SetRecordingDevice(0) != 0) {
    RTC_LOG(LS_ERROR) << "Unable to set recording device.";
    return false;
  }
  if (audio_device_module_->InitMicrophone() != 0) {
    RTC_LOG(LS_ERROR) << "Unable to access microphone.";
  }

  // Set number of channels
  available = false;
  if (audio_device_module_->StereoRecordingIsAvailable(&available) != 0) {
    RTC_LOG(LS_ERROR) << "Failed to query stereo recording.";
  }
  if (audio_device_module_->SetStereoRecording(available) != 0) {
    RTC_LOG(LS_ERROR) << "Failed to set stereo recording mode.";
  }

  int ret = audio_device_module_->RegisterAudioCallback(audio_transport_.get());

  return (ret == 0);
}

int VoipCore::CreateChannel(const VoipBase::Config& config) {
  int channel = -1;

  auto clock = Clock::GetRealTimeClock();
  auto receive_statistics = ReceiveStatistics::Create(clock);

  RtpRtcp::Configuration rtp_config;
  rtp_config.clock = clock;
  rtp_config.audio = true;
  rtp_config.receive_statistics = receive_statistics.get();
  rtp_config.rtcp_report_interval_ms = kRtcpReportIntervalMs;
  rtp_config.outgoing_transport = config.transport;
  rtp_config.local_media_ssrc = config.local_ssrc;

  auto audio_channel = std::make_shared<AudioChannel>(
      RtpRtcp::Create(rtp_config), clock, task_queue_factory_.get(),
      process_thread_.get(), audio_mixer_.get(), decoder_factory_,
      std::move(receive_statistics));

  {
    rtc::CritScope scoped(&lock_);
    if (!idle_.empty()) {
      channel = idle_.front();
      idle_.pop();
    } else {
      channel = channels_.size();
      // Make sure we have a room for a new audio channel
      channels_.push_back(nullptr);
    }
    channels_[channel] = audio_channel;
  }
  return channel;
}

bool VoipCore::ReleaseChannel(int channel) {
  // Destroy channel outside of the lock
  std::shared_ptr<AudioChannel> audio_channel;
  {
    rtc::CritScope scoped(&lock_);
    if (channel < 0 || channel >= static_cast<int>(channels_.size())) {
      RTC_LOG(LS_ERROR) << "channel out of range " << channel;
      return false;
    }
    audio_channel = std::move(channels_[channel]);
    if (audio_channel) {
      idle_.push(channel);
      return true;
    }
  }
  return false;
}

std::shared_ptr<AudioChannel> VoipCore::GetChannel(int channel) {
  rtc::CritScope scoped(&lock_);
  if (channel < 0 || channel >= static_cast<int>(channels_.size())) {
    RTC_LOG(LS_ERROR) << "channel out of range " << channel;
    return nullptr;
  }
  return channels_[channel];
}

bool VoipCore::UpdateAudioTransportWithSenders() {
  std::vector<AudioSender*> audio_senders;
  int max_sample_rate_hz = 8000;
  size_t max_num_channels = 1;

  {
    rtc::CritScope scoped(&lock_);
    for (auto channel : channels_) {
      if (channel->Sending()) {
        audio_senders.push_back(channel.get());
        max_sample_rate_hz =
            std::max(max_sample_rate_hz, channel->EncoderSampleRate());
        max_num_channels =
            std::max(max_num_channels, channel->EncoderNumChannel());
      }
    }
    audio_transport_->UpdateAudioSenders(audio_senders, max_sample_rate_hz,
                                         max_num_channels);
  }

  // Depending on availability of senders, turn on or off ADM recording
  if (!audio_senders.empty()) {
    if (!audio_device_module_->Recording()) {
      if (audio_device_module_->InitRecording() != 0) {
        RTC_LOG(LS_ERROR) << "InitRecording failed";
        return false;
      }
      if (audio_device_module_->StartRecording() != 0) {
        RTC_LOG(LS_ERROR) << "StartRecording failed";
        return false;
      }
    }
  } else {
    if (audio_device_module_->Recording()) {
      if (audio_device_module_->StopRecording() != 0) {
        RTC_LOG(LS_ERROR) << "StopRecording failed";
        return false;
      }
    }
  }
  return true;
}

bool VoipCore::StartSend(int channel) {
  auto audio_channel = GetChannel(channel);
  if (!audio_channel) {
    return false;
  }

  audio_channel->StartSend();

  return UpdateAudioTransportWithSenders();
}

bool VoipCore::StopSend(int channel) {
  auto audio_channel = GetChannel(channel);
  if (!audio_channel) {
    return false;
  }

  audio_channel->StopSend();

  return UpdateAudioTransportWithSenders();
}

bool VoipCore::StartPlayout(int channel) {
  auto audio_channel = GetChannel(channel);
  if (!audio_channel) {
    return false;
  }

  audio_channel->StartPlay();

  if (!audio_device_module_->Playing()) {
    if (audio_device_module_->InitPlayout() != 0) {
      RTC_LOG(LS_ERROR) << "InitPlayout failed";
      return false;
    }
    if (audio_device_module_->StartPlayout() != 0) {
      RTC_LOG(LS_ERROR) << "StartPlayout failed";
      return false;
    }
  }
  return true;
}

bool VoipCore::StopPlayout(int channel) {
  auto audio_channel = GetChannel(channel);
  if (!audio_channel) {
    return false;
  }

  audio_channel->StopPlay();

  bool stop_device = true;
  {
    rtc::CritScope scoped(&lock_);
    for (auto channel : channels_) {
      if (channel->Playing()) {
        stop_device = false;
        break;
      }
    }
  }

  if (stop_device && audio_device_module_->Playing()) {
    if (audio_device_module_->StopPlayout() != 0) {
      RTC_LOG(LS_ERROR) << "StopPlayout failed";
      return false;
    }
  }
  return true;
}

bool VoipCore::ReceivedRTPPacket(int channel,
                                 const uint8_t* data,
                                 size_t length) {
  auto audio_channel = GetChannel(channel);
  if (!audio_channel) {
    return false;
  }
  audio_channel->ReceivedRTPPacket(data, length);
  return true;
}

bool VoipCore::ReceivedRTCPPacket(int channel,
                                  const uint8_t* data,
                                  size_t length) {
  auto audio_channel = GetChannel(channel);
  if (!audio_channel) {
    return false;
  }
  audio_channel->ReceivedRTCPPacket(data, length);
  return true;
}

bool VoipCore::SetSendCodec(int channel,
                            int payload_type,
                            const SdpAudioFormat& encoder_format) {
  auto audio_channel = GetChannel(channel);
  if (!audio_channel) {
    return false;
  }
  auto encoder = encoder_factory_->MakeAudioEncoder(
      payload_type, encoder_format, absl::nullopt);
  audio_channel->SetEncoder(payload_type, encoder_format, std::move(encoder));
  return true;
}

bool VoipCore::SetReceiveCodecs(
    int channel,
    const std::map<int, SdpAudioFormat>& decoder_specs) {
  auto audio_channel = GetChannel(channel);
  if (!audio_channel) {
    return false;
  }
  audio_channel->SetReceiveCodecs(decoder_specs);
  return true;
}

}  // namespace webrtc
