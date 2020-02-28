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
#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "api/call/transport.h"
#include "api/task_queue/default_task_queue_factory.h"
#include "modules/audio_mixer/audio_mixer_impl.h"
#include "modules/audio_mixer/sine_wave_generator.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/utility/include/process_thread.h"
#include "rtc_base/logging.h"
#include "test/gmock.h"
#include "test/gtest.h"
#include "test/mock_transport.h"
#include "third_party/abseil-cpp/absl/synchronization/notification.h"

namespace webrtc {

using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Unused;

namespace {

constexpr uint64_t kStartTime = 123456789;
constexpr uint32_t kLocalSsrc = 0xdeadc0de;
constexpr int16_t kAudioLevel = 3004;  // used for sine wave level
constexpr int kPcmuPayload = 0;
const SdpAudioFormat kPcmuFormat = {"pcmu", 8000, 1};

std::unique_ptr<RtpRtcp> CreateRtpStack(Clock* clock,
                                        Transport* transport,
                                        ReceiveStatistics* receive_statistics) {
  RtpRtcp::Configuration rtp_config;
  rtp_config.clock = clock;
  rtp_config.audio = true;
  rtp_config.receive_statistics = receive_statistics;
  rtp_config.rtcp_report_interval_ms = 5000;
  rtp_config.outgoing_transport = transport;
  rtp_config.local_media_ssrc = kLocalSsrc;
  auto rtp_rtcp = RtpRtcp::Create(rtp_config);
  rtp_rtcp->SetSendingMediaStatus(false);
  rtp_rtcp->SetRTCPStatus(RtcpMode::kCompound);
  return rtp_rtcp;
}

}  // namespace

class AudioChannelTest : public ::testing::Test {
 public:
  AudioChannelTest()
      : fake_clock_(kStartTime), wave_generator_(1000.0, kAudioLevel) {
    process_thread_ = ProcessThread::Create("ModuleProcessThread");
    audio_mixer_ = AudioMixerImpl::Create();
    task_queue_factory_ = CreateDefaultTaskQueueFactory();
    encoder_factory_ = CreateBuiltinAudioEncoderFactory();
    decoder_factory_ = CreateBuiltinAudioDecoderFactory();
  }

  std::unique_ptr<AudioFrame> GetAudioFrame(uint32_t order) {
    auto frame = std::make_unique<AudioFrame>();
    frame->sample_rate_hz_ = kPcmuFormat.clockrate_hz;
    frame->samples_per_channel_ = kPcmuFormat.clockrate_hz / 100;  // 10 ms.
    frame->num_channels_ = kPcmuFormat.num_channels;
    frame->timestamp_ = frame->samples_per_channel_ * order;
    wave_generator_.GenerateNextFrame(frame.get());
    return frame;
  }

  void InsertPackets(size_t num_packets) {
    // Two 10 ms audio frames will result in rtp packet with ptime 20.
    for (size_t i = 0; i < (num_packets * 2); i++) {
      audio_channel_->SendAudioData(GetAudioFrame(i));
      fake_clock_.AdvanceTimeMilliseconds(10);
    }
  }

  // SimulatedClock doesn't directly affect this testcase as the the
  // AudioFrame's timestamp is driven by GetAudioFrame.
  SimulatedClock fake_clock_;
  SineWaveGenerator wave_generator_;
  NiceMock<MockTransport> transport_;
  std::unique_ptr<TaskQueueFactory> task_queue_factory_;
  rtc::scoped_refptr<AudioMixer> audio_mixer_;
  rtc::scoped_refptr<AudioDecoderFactory> decoder_factory_;
  rtc::scoped_refptr<AudioEncoderFactory> encoder_factory_;
  std::unique_ptr<ProcessThread> process_thread_;
  std::unique_ptr<AudioChannel> audio_channel_;
};

TEST_F(AudioChannelTest, PlayRtpByLocalLoop) {
  auto receive_statistics = ReceiveStatistics::Create(&fake_clock_);
  audio_channel_ = std::make_unique<AudioChannel>(
      CreateRtpStack(&fake_clock_, &transport_, receive_statistics.get()),
      &fake_clock_, task_queue_factory_.get(), process_thread_.get(),
      audio_mixer_.get(), decoder_factory_, std::move(receive_statistics));
  audio_channel_->SetEncoder(kPcmuPayload, kPcmuFormat,
                             encoder_factory_->MakeAudioEncoder(
                                 kPcmuPayload, kPcmuFormat, absl::nullopt));
  audio_channel_->SetReceiveCodecs({{kPcmuPayload, kPcmuFormat}});
  audio_channel_->StartSend();
  audio_channel_->StartPlay();

  absl::Notification rtp_done;
  auto loop_rtp = [&](const uint8_t* packet, size_t length, Unused) {
    audio_channel_->ReceivedRTPPacket(packet, length);
    rtp_done.Notify();
    return true;
  };
  EXPECT_CALL(transport_, SendRtp).WillOnce(Invoke(loop_rtp));

  InsertPackets(1);

  rtp_done.WaitForNotificationWithTimeout(absl::Milliseconds(1000));
  EXPECT_TRUE(rtp_done.HasBeenNotified());

  AudioFrame empty_frame, audio_frame;
  empty_frame.Mute();
  empty_frame.mutable_data();  // this will zero out the data
  audio_frame.CopyFrom(empty_frame);
  audio_mixer_->Mix(/*number_of_channels*/ 1, &audio_frame);

  // we expect now audio frame to pick up something
  EXPECT_NE(memcmp(empty_frame.data(), audio_frame.data(),
                   AudioFrame::kMaxDataSizeBytes),
            0);
  audio_channel_->StopPlay();
  audio_channel_->StopSend();
}

}  // namespace webrtc
