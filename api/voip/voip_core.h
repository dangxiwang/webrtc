//
//  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
//
//  Use of this source code is governed by a BSD-style license
//  that can be found in the LICENSE file in the root of the source
//  tree. An additional intellectual property rights grant can be found
//  in the file PATENTS.  All contributing project authors may
//  be found in the AUTHORS file in the root of the source tree.
//

#ifndef API_VOIP_VOIP_CORE_H_
#define API_VOIP_VOIP_CORE_H_

#include <map>
#include <memory>
#include <queue>
#include <vector>

#include "api/audio_codecs/audio_decoder_factory.h"
#include "api/audio_codecs/audio_encoder_factory.h"
#include "api/scoped_refptr.h"
#include "api/task_queue/task_queue_factory.h"
#include "api/voip/audio_channel.h"
#include "api/voip/voip_engine.h"
#include "api/voip/voip_network.h"
#include "audio/audio_transport_impl.h"
#include "modules/audio_device/include/audio_device.h"
#include "modules/audio_mixer/audio_mixer_impl.h"
#include "modules/audio_processing/include/audio_processing.h"
#include "modules/utility/include/process_thread.h"
#include "rtc_base/critical_section.h"

namespace webrtc {

class VoipCore : public VoipEngine,
                 public VoipChannel,
                 public VoipNetwork,
                 public VoipCodec {
 public:
  ~VoipCore() override = default;

  bool Init(std::unique_ptr<TaskQueueFactory> task_queue_factory,
            std::unique_ptr<AudioProcessing> audio_processing,
            rtc::scoped_refptr<AudioDeviceModule> audio_device_module,
            rtc::scoped_refptr<AudioEncoderFactory> encoder_factory,
            rtc::scoped_refptr<AudioDecoderFactory> decoder_factory);

  // VoipEngine interfaces
  VoipNetwork* NetworkInterface() override;
  VoipCodec* CodecInterface() override;
  VoipChannel* ChannelInterface() override;

  // VoipChannel interfaces
  int CreateChannel(const VoipChannel::Config& config) override;
  bool ReleaseChannel(int channel) override;
  bool StartSend(int channel) override;
  bool StopSend(int channel) override;
  bool StartPlayout(int channel) override;
  bool StopPlayout(int channel) override;

  // VoipNetwork interfaces
  bool ReceivedRTPPacket(int channel,
                         const uint8_t* data,
                         size_t length) override;
  bool ReceivedRTCPPacket(int channel,
                          const uint8_t* data,
                          size_t length) override;

  // VoipCodec interfaces
  bool SetSendCodec(int channel,
                    int payload_type,
                    const SdpAudioFormat& encoder_format) override;
  bool SetReceiveCodecs(
      int channel,
      const std::map<int, SdpAudioFormat>& decoder_specs) override;

 private:
  std::shared_ptr<AudioChannel> GetChannel(int channel);

  bool UpdateAudioTransportWithSenders();

  // Listed in order for safe destruction of voip core object.
  std::unique_ptr<AudioTransportImpl> audio_transport_;
  std::unique_ptr<AudioProcessing> audio_processing_;
  rtc::scoped_refptr<AudioMixer> audio_mixer_;
  rtc::scoped_refptr<AudioEncoderFactory> encoder_factory_;
  rtc::scoped_refptr<AudioDecoderFactory> decoder_factory_;
  rtc::scoped_refptr<AudioDeviceModule> audio_device_module_;
  std::unique_ptr<TaskQueueFactory> task_queue_factory_;
  std::unique_ptr<ProcessThread> process_thread_;

  rtc::CriticalSection lock_;
  std::vector<std::shared_ptr<AudioChannel>> channels_ RTC_GUARDED_BY(lock_);
  std::queue<int> idle_ RTC_GUARDED_BY(lock_);
};

}  // namespace webrtc

#endif  // API_VOIP_VOIP_CORE_H_
