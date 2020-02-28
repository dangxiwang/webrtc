//
//  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
//
//  Use of this source code is governed by a BSD-style license
//  that can be found in the LICENSE file in the root of the source
//  tree. An additional intellectual property rights grant can be found
//  in the file PATENTS.  All contributing project authors may
//  be found in the AUTHORS file in the root of the source tree.
//

#ifndef AUDIO_VOIP_AUDIO_CHANNEL_H_
#define AUDIO_VOIP_AUDIO_CHANNEL_H_

#include <memory>
#include <queue>

#include "api/task_queue/task_queue_factory.h"
#include "audio/voip/audio_egress.h"
#include "audio/voip/audio_ingress.h"
#include "modules/rtp_rtcp/include/receive_statistics.h"
#include "modules/rtp_rtcp/include/rtp_rtcp.h"
#include "modules/utility/include/process_thread.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/critical_section.h"

namespace webrtc {

// AudioChannel is made up of two implementation detail classes;
// namely AudioIngress and AudioEgress.
//
//  - AudioIngress mainly handles incoming RTP/RTCP packets from remote
//    endpoint through its own acm2::AcmReceiver. Its role is similar
//    to ChannelReceive in that it is AudioMixer::Source and will
//    provide audio sample to play as requested by AudioDeviceModule.
//    Though its usage on RtpRtcp is limited, it provides clarity and
//    better logical division that it has the shared RtpRtcp instance.
//
//  - AudioEgress is the implementation that receives the process
//    input samples from AudioDeviceModule through AudioTransportImpl.
//    Once it encodes the sample via selected encoder, resulted data
//    will be packetized by RTP stack, resulting in ready-to-send RTP
//    packet to remote endpoint.
//
// Note that the instance of RtpRtcp is shared among all three classes.
// Though send and receive logic can be nicely divided into two separate
// classes, there still is some logic that would require orchestration
// between the two. In media recv-only use case, RTCP is still expected
// to be transmitted and thus the rtp stack needs to be set to sending
// in recv-only mode; that is AudioEgress is not active but only AudioIngress
// is. AudioChannel has the shared instance of rtp stack to control it
// as it inhertits both implementations. In the similar reason, we have
// overridden Start* and Stop* method from these parents. Note also
// that since RtpRtcp is shared among three classes, it's easier to
// maintain the destruction sequence by using std::shared_ptr.
//
// It's also possible to make this composite relationship instead but
// it would require AudioChannel to open up APIs for all AudioIngress/Egress
// resulting in noisy interfaces. Since AudioIngress/Egress are indeed
// just implementation details, it's more fitting to regard this as
// 'as-is' relationship.
//
class AudioChannel : public AudioIngress, public AudioEgress {
 public:
  AudioChannel(std::shared_ptr<RtpRtcp> rtp_rtcp,
               Clock* clock,
               TaskQueueFactory* task_queue_factory,
               ProcessThread* process_thread,
               AudioMixer* audio_mixer,
               rtc::scoped_refptr<AudioDecoderFactory> decoder_factory,
               std::unique_ptr<ReceiveStatistics> receive_statistics);
  ~AudioChannel() override;

  // Channel needs to orchestrate between ingress and egress.
  void StartSend() override;
  void StopSend() override;
  void StartPlay() override;
  void StopPlay() override;

 private:
  AudioMixer* audio_mixer_ = nullptr;
  ProcessThread* process_thread_ = nullptr;

  // Shares the common rtp stack instance with AudioIngress/Egress.
  std::shared_ptr<RtpRtcp> rtp_rtcp_;

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(AudioChannel);
};

}  // namespace webrtc

#endif  // AUDIO_VOIP_AUDIO_CHANNEL_H_
