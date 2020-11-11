/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "audio/voip/voip_core.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "api/audio_codecs/audio_format.h"
#include "rtc_base/logging.h"

namespace webrtc {

namespace {

// For Windows, use specific enum type to initialize default audio device as
// defined in AudioDeviceModule::WindowsDeviceType.
#if defined(WEBRTC_WIN)
constexpr AudioDeviceModule::WindowsDeviceType kAudioDeviceId =
    AudioDeviceModule::WindowsDeviceType::kDefaultCommunicationDevice;
#else
constexpr uint16_t kAudioDeviceId = 0;
#endif  // defined(WEBRTC_WIN)

// Maximum value range limit on ChannelId. This can be increased without any
// side effect and only set at this moderate value for better readability for
// logging.
static constexpr int kMaxChannelId = 100000;

}  // namespace

bool VoipCore::Init(rtc::scoped_refptr<AudioEncoderFactory> encoder_factory,
                    rtc::scoped_refptr<AudioDecoderFactory> decoder_factory,
                    std::unique_ptr<TaskQueueFactory> task_queue_factory,
                    rtc::scoped_refptr<AudioDeviceModule> audio_device_module,
                    rtc::scoped_refptr<AudioProcessing> audio_processing,
                    std::unique_ptr<ProcessThread> process_thread) {
  encoder_factory_ = std::move(encoder_factory);
  decoder_factory_ = std::move(decoder_factory);
  task_queue_factory_ = std::move(task_queue_factory);
  audio_device_module_ = std::move(audio_device_module);
  audio_processing_ = std::move(audio_processing);
  process_thread_ = std::move(process_thread);

  if (!process_thread_) {
    process_thread_ = ProcessThread::Create("ModuleProcessThread");
  }
  audio_mixer_ = AudioMixerImpl::Create();

  // AudioTransportImpl depends on audio mixer and audio processing instances.
  audio_transport_ = std::make_unique<AudioTransportImpl>(
      audio_mixer_.get(), audio_processing_.get(), nullptr);

  // Initialize ADM.
  if (audio_device_module_->Init() != 0) {
    RTC_LOG(LS_ERROR) << "Failed to initialize the ADM.";
    return false;
  }

  // Note that failures on initializing default recording/speaker devices are
  // not considered to be fatal here. In certain case, caller may not care about
  // recording device functioning (e.g webinar where only speaker is available).
  // It's also possible that there are other audio devices available that may
  // work.
  // TODO(natim@webrtc.org): consider moving this part out of initialization.

  // Initialize default speaker device.
  if (audio_device_module_->SetPlayoutDevice(kAudioDeviceId) != 0) {
    RTC_LOG(LS_WARNING) << "Unable to set playout device.";
  }
  if (audio_device_module_->InitSpeaker() != 0) {
    RTC_LOG(LS_WARNING) << "Unable to access speaker.";
  }

  // Initialize default recording device.
  if (audio_device_module_->SetRecordingDevice(kAudioDeviceId) != 0) {
    RTC_LOG(LS_WARNING) << "Unable to set recording device.";
  }
  if (audio_device_module_->InitMicrophone() != 0) {
    RTC_LOG(LS_WARNING) << "Unable to access microphone.";
  }

  // Set number of channels on speaker device.
  bool available = false;
  if (audio_device_module_->StereoPlayoutIsAvailable(&available) != 0) {
    RTC_LOG(LS_WARNING) << "Unable to query stereo playout.";
  }
  if (audio_device_module_->SetStereoPlayout(available) != 0) {
    RTC_LOG(LS_WARNING) << "Unable to set mono/stereo playout mode.";
  }

  // Set number of channels on recording device.
  available = false;
  if (audio_device_module_->StereoRecordingIsAvailable(&available) != 0) {
    RTC_LOG(LS_WARNING) << "Unable to query stereo recording.";
  }
  if (audio_device_module_->SetStereoRecording(available) != 0) {
    RTC_LOG(LS_WARNING) << "Unable to set stereo recording mode.";
  }

  if (audio_device_module_->RegisterAudioCallback(audio_transport_.get()) !=
      0) {
    RTC_LOG(LS_WARNING) << "Unable to register audio callback.";
  }

  return true;
}

absl::optional<ChannelId> VoipCore::CreateChannel(
    Transport* transport,
    absl::optional<uint32_t> local_ssrc) {
  absl::optional<ChannelId> channel_id;

  // Set local ssrc to random if not set by caller.
  if (!local_ssrc) {
    Random random(rtc::TimeMicros());
    local_ssrc = random.Rand<uint32_t>();
  }

  rtc::scoped_refptr<AudioChannel> channel =
      new rtc::RefCountedObject<AudioChannel>(
          transport, local_ssrc.value(), task_queue_factory_.get(),
          process_thread_.get(), audio_mixer_.get(), decoder_factory_);

  // Check if we need to start the process thread.
  bool start_process_thread = false;

  {
    MutexLock lock(&lock_);

    // Start process thread if the channel is the first one.
    start_process_thread = channels_.empty();

    channel_id = static_cast<ChannelId>(next_channel_id_);
    channels_[*channel_id] = channel;
    next_channel_id_++;
    if (next_channel_id_ >= kMaxChannelId) {
      next_channel_id_ = 0;
    }
  }

  // Set ChannelId in audio channel for logging/debugging purpose.
  channel->SetId(*channel_id);

  if (start_process_thread) {
    process_thread_->Start();
  }

  return channel_id;
}

void VoipCore::ReleaseChannel(ChannelId channel_id) {
  // Destroy channel outside of the lock.
  rtc::scoped_refptr<AudioChannel> channel;

  bool no_channels_after_release = false;

  {
    MutexLock lock(&lock_);

    auto iter = channels_.find(channel_id);
    if (iter != channels_.end()) {
      channel = std::move(iter->second);
      channels_.erase(iter);
    }

    no_channels_after_release = channels_.empty();
  }

  if (!channel) {
    RTC_LOG(LS_WARNING) << "Channel " << channel_id << " not found";
  }

  if (no_channels_after_release) {
    // Release audio channel first to have it DeRegisterModule first.
    channel = nullptr;
    process_thread_->Stop();

    // Make sure to stop playout on ADM if it is playing.
    if (audio_device_module_->Playing()) {
      if (audio_device_module_->StopPlayout() != 0) {
        RTC_LOG(LS_WARNING) << "StopPlayout failed";
      }
    }
  }
}

rtc::scoped_refptr<AudioChannel> VoipCore::GetChannel(ChannelId channel_id) {
  rtc::scoped_refptr<AudioChannel> channel;
  {
    MutexLock lock(&lock_);
    auto iter = channels_.find(channel_id);
    if (iter != channels_.end()) {
      channel = iter->second;
    }
  }
  if (!channel) {
    RTC_LOG(LS_ERROR) << "Channel " << channel_id << " not found";
  }
  return channel;
}

bool VoipCore::UpdateAudioTransportWithSenders() {
  std::vector<AudioSender*> audio_senders;

  // Gather a list of audio channel that are currently sending along with
  // highest sampling rate and channel numbers to configure into audio
  // transport.
  int max_sampling_rate = 8000;
  size_t max_num_channels = 1;
  {
    MutexLock lock(&lock_);
    // Reserve to prevent run time vector re-allocation.
    audio_senders.reserve(channels_.size());
    for (auto kv : channels_) {
      rtc::scoped_refptr<AudioChannel>& channel = kv.second;
      if (channel->IsSendingMedia()) {
        auto encoder_format = channel->GetEncoderFormat();
        if (!encoder_format) {
          RTC_LOG(LS_ERROR)
              << "channel " << channel->GetId() << " encoder is not set";
          continue;
        }
        audio_senders.push_back(channel->GetAudioSender());
        max_sampling_rate =
            std::max(max_sampling_rate, encoder_format->clockrate_hz);
        max_num_channels =
            std::max(max_num_channels, encoder_format->num_channels);
      }
    }
  }

  audio_transport_->UpdateAudioSenders(audio_senders, max_sampling_rate,
                                       max_num_channels);

  // Depending on availability of senders, turn on or off ADM recording.
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
    if (audio_device_module_->Recording() &&
        audio_device_module_->StopRecording() != 0) {
      RTC_LOG(LS_ERROR) << "StopRecording failed";
      return false;
    }
  }
  return true;
}

bool VoipCore::StartSend(ChannelId channel_id) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (!channel || !channel->StartSend()) {
    return false;
  }

  return UpdateAudioTransportWithSenders();
}

bool VoipCore::StopSend(ChannelId channel_id) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (!channel) {
    return false;
  }

  channel->StopSend();

  return UpdateAudioTransportWithSenders();
}

bool VoipCore::StartPlayout(ChannelId channel_id) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (!channel) {
    return false;
  }

  if (channel->IsPlaying()) {
    return true;
  }

  if (!channel->StartPlay()) {
    return false;
  }

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

bool VoipCore::StopPlayout(ChannelId channel_id) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (!channel) {
    return false;
  }

  channel->StopPlay();

  return true;
}

void VoipCore::ReceivedRTPPacket(ChannelId channel_id,
                                 rtc::ArrayView<const uint8_t> rtp_packet) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (channel) {
    channel->ReceivedRTPPacket(rtp_packet);
  }
}

void VoipCore::ReceivedRTCPPacket(ChannelId channel_id,
                                  rtc::ArrayView<const uint8_t> rtcp_packet) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (channel) {
    channel->ReceivedRTCPPacket(rtcp_packet);
  }
}

void VoipCore::SetSendCodec(ChannelId channel_id,
                            int payload_type,
                            const SdpAudioFormat& encoder_format) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (channel) {
    auto encoder = encoder_factory_->MakeAudioEncoder(
        payload_type, encoder_format, absl::nullopt);
    channel->SetEncoder(payload_type, encoder_format, std::move(encoder));
  }
}

void VoipCore::SetReceiveCodecs(
    ChannelId channel_id,
    const std::map<int, SdpAudioFormat>& decoder_specs) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (channel) {
    channel->SetReceiveCodecs(decoder_specs);
  }
}

void VoipCore::RegisterTelephoneEventType(ChannelId channel_id,
                                          int rtp_payload_type,
                                          int sample_rate_hz) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (channel) {
    channel->RegisterTelephoneEventType(rtp_payload_type, sample_rate_hz);
  }
}

bool VoipCore::SendDtmfEvent(ChannelId channel_id,
                             DtmfEvent dtmf_event,
                             int duration_ms) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (channel) {
    return channel->SendTelephoneEvent(static_cast<int>(dtmf_event),
                                       duration_ms);
  }
  return false;
}

absl::optional<IngressStatistics> VoipCore::GetIngressStatistics(
    ChannelId channel_id) {
  rtc::scoped_refptr<AudioChannel> channel = GetChannel(channel_id);

  if (channel) {
    return channel->GetIngressStatistics();
  }
  return absl::nullopt;
}

}  // namespace webrtc
