//
//  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
//
//  Use of this source code is governed by a BSD-style license
//  that can be found in the LICENSE file in the root of the source
//  tree. An additional intellectual property rights grant can be found
//  in the file PATENTS.  All contributing project authors may
//  be found in the AUTHORS file in the root of the source tree.
//

#ifndef AUDIO_VOIP_AUDIO_INGRESS_H_
#define AUDIO_VOIP_AUDIO_INGRESS_H_

#include <map>
#include <memory>
#include <utility>

#include "api/array_view.h"
#include "api/audio/audio_mixer.h"
#include "api/call/transport.h"
#include "api/rtp_headers.h"
#include "api/scoped_refptr.h"
#include "audio/audio_level.h"
#include "modules/audio_coding/acm2/acm_receiver.h"
#include "modules/audio_coding/include/audio_coding_module.h"
#include "modules/rtp_rtcp/include/receive_statistics.h"
#include "modules/rtp_rtcp/include/remote_ntp_time_estimator.h"
#include "modules/rtp_rtcp/include/rtp_rtcp.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/thread_checker.h"
#include "rtc_base/time_utils.h"

namespace webrtc {

class AudioIngress : public AudioMixer::Source {
 public:
  AudioIngress(std::shared_ptr<RtpRtcp> rtp_rtcp,
               Clock* clock,
               rtc::scoped_refptr<AudioDecoderFactory> decoder_factory,
               std::unique_ptr<ReceiveStatistics> receive_statistics);
  ~AudioIngress() override;

  // These are virtual as expected to be controlled by AudioChannel.
  virtual void StartPlay();
  virtual void StopPlay();

  bool Playing() const;

  void SetReceiveCodecs(const std::map<int, SdpAudioFormat>& codecs);

  void ReceivedRTPPacket(const uint8_t* data, size_t length);
  void ReceivedRTCPPacket(const uint8_t* data, size_t length);

  int GetSpeechOutputLevelFullRange() const;
  int64_t GetRTT();
  NetworkStatistics GetNetworkStatistics() const;
  AudioDecodingCallStats GetDecodingCallStatistics() const;

 private:
  int GetRtpTimestampRateHz() const;

  // AudioMixer::Source interface
  AudioMixer::Source::AudioFrameInfo GetAudioFrameWithInfo(
      int sample_rate_hz,
      AudioFrame* audio_frame) override;
  int Ssrc() const override;
  int PreferredSampleRate() const override;

  rtc::ThreadChecker worker_thread_checker_;

  bool playing_ = false;
  // The rtp timestamp of the first played out audio frame.
  int64_t capture_start_rtp_time_stamp_ = -1;
  uint32_t remote_ssrc_ = 0;

  // Indexed by payload type.
  rtc::CriticalSection payload_type_lock_;
  std::map<uint8_t, int> payload_type_frequencies_
      RTC_GUARDED_BY(payload_type_lock_);

  std::shared_ptr<RtpRtcp> rtp_rtcp_;
  std::unique_ptr<ReceiveStatistics> rtp_receive_statistics_;
  std::unique_ptr<RemoteNtpTimeEstimator> ntp_estimator_
      RTC_GUARDED_BY(ntp_lock_);
  rtc::CriticalSection ntp_lock_;

  std::unique_ptr<acm2::AcmReceiver> acm_receiver_;
  voe::AudioLevel output_audio_level_;
  std::unique_ptr<rtc::TimestampWrapAroundHandler> rtp_ts_wraparound_handler_;

  friend class AudioIngressTest;
};

}  // namespace webrtc

#endif  // AUDIO_VOIP_AUDIO_INGRESS_H_
