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
#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "api/call/transport.h"
#include "api/task_queue/default_task_queue_factory.h"
#include "audio/voip/audio_egress.h"
#include "modules/audio_mixer/sine_wave_generator.h"
#include "rtc_base/logging.h"
#include "test/gmock.h"
#include "test/gtest.h"
#include "test/mock_transport.h"
#include "third_party/abseil-cpp/absl/synchronization/notification.h"

namespace webrtc {

using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Unused;

namespace {

constexpr uint64_t kStartTime = 123456789;
constexpr uint32_t kRemoteSsrc = 0xDEADBEEF;
constexpr int16_t kAudioLevel = 3004;  // used for sine wave level
constexpr int kPcmuPayload = 0;
const SdpAudioFormat kPcmuFormat = {"pcmu", 8000, 1};

std::unique_ptr<RtpRtcp> CreateRtpStack(Clock* clock, Transport* transport) {
  RtpRtcp::Configuration rtp_config;
  rtp_config.clock = clock;
  rtp_config.audio = true;
  rtp_config.rtcp_report_interval_ms = 5000;
  rtp_config.outgoing_transport = transport;
  rtp_config.local_media_ssrc = kRemoteSsrc;
  auto rtp_rtcp = RtpRtcp::Create(rtp_config);
  rtp_rtcp->SetSendingMediaStatus(false);
  rtp_rtcp->SetRTCPStatus(RtcpMode::kCompound);
  return rtp_rtcp;
}

}  // namespace

class AudioIngressTest : public ::testing::Test {
 public:
  AudioIngressTest()
      : fake_clock_(kStartTime), wave_generator_(1000.0, kAudioLevel) {
    rtp_rtcp_ = CreateRtpStack(&fake_clock_, &transport_);
    task_queue_factory_ = CreateDefaultTaskQueueFactory();
    encoder_factory_ = CreateBuiltinAudioEncoderFactory();
    decoder_factory_ = CreateBuiltinAudioDecoderFactory();
  }

  void SetUp() override {
    ingress_ = std::make_unique<AudioIngress>(
        rtp_rtcp_, &fake_clock_, decoder_factory_,
        ReceiveStatistics::Create(&fake_clock_));
    ingress_->SetReceiveCodecs({{kPcmuPayload, kPcmuFormat}});

    egress_ = std::make_unique<AudioEgress>(rtp_rtcp_, &fake_clock_,
                                            task_queue_factory_.get());
    egress_->SetEncoder(kPcmuPayload, kPcmuFormat,
                        encoder_factory_->MakeAudioEncoder(
                            kPcmuPayload, kPcmuFormat, absl::nullopt));
    egress_->StartSend();
    ingress_->StartPlay();
    rtp_rtcp_->SetSendingStatus(true);
  }

  void TearDown() override {
    rtp_rtcp_->SetSendingStatus(false);
    ingress_->StopPlay();
    egress_->StopSend();
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

  void InsertPackets(size_t num_packets, absl::Notification* notify) {
    auto handle_rtp = [&](const uint8_t* packet, size_t length, Unused) {
      ingress_->ReceivedRTPPacket(packet, length);
      if (--num_packets == 0) {
        notify->Notify();
      }
      return true;
    };

    EXPECT_CALL(transport_, SendRtp).WillRepeatedly(Invoke(handle_rtp));

    // Two 10 ms audio frame will result in rtp packet with ptime 20.
    for (size_t i = 0, num_samples = (num_packets * 2); i < num_samples; i++) {
      egress_->SendAudioData(GetAudioFrame(i));
      fake_clock_.AdvanceTimeMilliseconds(10);
    }
    notify->WaitForNotificationWithTimeout(absl::Milliseconds(1000));
  }

  AudioMixer::Source::AudioFrameInfo GetAudioFrameWithInfo(
      int sample_rate_hz,
      AudioFrame* audio_frame) {
    return ingress_->GetAudioFrameWithInfo(sample_rate_hz, audio_frame);
  }

  int PreferredSampleRate() const { return ingress_->PreferredSampleRate(); }

  SimulatedClock fake_clock_;
  SineWaveGenerator wave_generator_;
  NiceMock<MockTransport> transport_;
  std::unique_ptr<AudioIngress> ingress_;
  rtc::scoped_refptr<AudioDecoderFactory> decoder_factory_;
  // members used to drive the input to ingress
  std::unique_ptr<AudioEgress> egress_;
  std::unique_ptr<TaskQueueFactory> task_queue_factory_;
  std::shared_ptr<RtpRtcp> rtp_rtcp_;
  rtc::scoped_refptr<AudioEncoderFactory> encoder_factory_;
};

TEST_F(AudioIngressTest, CorrectPlayingAfterStartAndStop) {
  EXPECT_EQ(ingress_->Playing(), true);
  ingress_->StopPlay();
  EXPECT_EQ(ingress_->Playing(), false);
}

TEST_F(AudioIngressTest, GetAudioFrameAfterRtpReceived) {
  absl::Notification done;
  InsertPackets(1, &done);
  ASSERT_TRUE(done.HasBeenNotified());

  AudioFrame audio_frame;
  EXPECT_EQ(GetAudioFrameWithInfo(kPcmuFormat.clockrate_hz, &audio_frame),
            AudioMixer::Source::AudioFrameInfo::kNormal);
  EXPECT_FALSE(audio_frame.muted());
  EXPECT_EQ(audio_frame.num_channels_, 1u);
  EXPECT_EQ(audio_frame.samples_per_channel_,
            static_cast<size_t>(kPcmuFormat.clockrate_hz / 100));
  EXPECT_EQ(audio_frame.sample_rate_hz_, kPcmuFormat.clockrate_hz);
  EXPECT_NE(audio_frame.timestamp_, 0u);
  EXPECT_EQ(audio_frame.elapsed_time_ms_, 0);
}

TEST_F(AudioIngressTest, GetSpeechOutputLevelFullRange) {
  // Per audio_level's kUpdateFrequency, we need 11 rtp to get audio level
  constexpr size_t kNumRtp = 11;
  absl::Notification done;
  InsertPackets(kNumRtp, &done);
  ASSERT_TRUE(done.HasBeenNotified());

  for (size_t i = 0; i < kNumRtp; ++i) {
    AudioFrame audio_frame;
    EXPECT_EQ(GetAudioFrameWithInfo(kPcmuFormat.clockrate_hz, &audio_frame),
              AudioMixer::Source::AudioFrameInfo::kNormal);
  }
  EXPECT_EQ(ingress_->GetSpeechOutputLevelFullRange(), kAudioLevel);
}

TEST_F(AudioIngressTest, PreferredSampleRate) {
  absl::Notification done;
  InsertPackets(1, &done);
  ASSERT_TRUE(done.HasBeenNotified());

  AudioFrame audio_frame;
  EXPECT_EQ(GetAudioFrameWithInfo(kPcmuFormat.clockrate_hz, &audio_frame),
            AudioMixer::Source::AudioFrameInfo::kNormal);
  EXPECT_EQ(PreferredSampleRate(), kPcmuFormat.clockrate_hz);
}

}  // namespace webrtc
