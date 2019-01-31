/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "test/pc/e2e/peerconnection_quality_test.h"

#include <set>
#include <utility>

#include "absl/memory/memory.h"
#include "api/peer_connection_interface.h"
#include "api/scoped_refptr.h"
#include "api/units/time_delta.h"
#include "rtc_base/bind.h"
#include "rtc_base/gunit.h"
#include "test/pc/e2e/analyzer/video/example_video_quality_analyzer.h"
#include "test/pc/e2e/api/video_quality_analyzer_interface.h"

namespace webrtc {
namespace test {
namespace {

constexpr int kDefaultTimeoutMs = 10000;
constexpr char kSignalThreadName[] = "signaling_thread";

}  // namespace

PeerConnectionE2EQualityTest::PeerConnectionE2EQualityTest(
    std::unique_ptr<InjectableComponents> alice_components,
    std::unique_ptr<Params> alice_params,
    std::unique_ptr<InjectableComponents> bob_components,
    std::unique_ptr<Params> bob_params,
    std::unique_ptr<Analyzers> analyzers)
    : clock_(Clock::GetRealTimeClock()),
      signaling_thread_(rtc::Thread::Create()) {
  signaling_thread_->SetName(kSignalThreadName, nullptr);
  signaling_thread_->Start();

  // Create default video quality analyzer. We will always create analyzer,
  // even if there are no video streams, because it will be installed into video
  // encoder/decoder factories.
  if (analyzers->video_quality_analyzer == nullptr) {
    analyzers->video_quality_analyzer =
        absl::make_unique<ExampleVideoQualityAnalyzer>();
  }
  encoded_image_id_controller_ =
      absl::make_unique<SingleProcessEncodedImageIdInjector>();
  video_quality_analyzer_injection_helper_ =
      absl::make_unique<VideoQualityAnalyzerInjectionHelper>(
          std::move(analyzers->video_quality_analyzer),
          encoded_image_id_controller_.get(),
          encoded_image_id_controller_.get());

  // Create call participants: Alice and Bob.
  absl::optional<std::string> alice_audio_output_filename =
      bob_params->audio_config ? bob_params->audio_config->output_dump_file_name
                               : absl::nullopt;
  absl::optional<std::string> bob_audio_output_filename =
      alice_params->audio_config
          ? alice_params->audio_config->output_dump_file_name
          : absl::nullopt;
  alice_ = TestPeer::CreateTestPeer(
      std::move(alice_components), std::move(alice_params),
      video_quality_analyzer_injection_helper_.get(), signaling_thread_.get(),
      alice_audio_output_filename);
  bob_ = TestPeer::CreateTestPeer(
      std::move(bob_components), std::move(bob_params),
      video_quality_analyzer_injection_helper_.get(), signaling_thread_.get(),
      bob_audio_output_filename);
}

void PeerConnectionE2EQualityTest::Run(RunParams run_params) {
  SetMissedVideoStreamLabels({alice_->params(), bob_->params()});
  ValidateParams({alice_->params(), bob_->params()});
  signaling_thread_->Invoke<void>(
      RTC_FROM_HERE,
      rtc::Bind(&PeerConnectionE2EQualityTest::Run_s, this, run_params));
}

void PeerConnectionE2EQualityTest::SetMissedVideoStreamLabels(
    std::vector<Params*> params) {
  int counter = 0;
  for (auto* p : params) {
    for (auto& video_config : p->video_configs) {
      if (!video_config.stream_label) {
        video_config.stream_label =
            "_auto_video_stream_label_" + std::to_string(counter);
        counter++;
      }
    }
  }
}

void PeerConnectionE2EQualityTest::ValidateParams(std::vector<Params*> params) {
  std::set<std::string> video_labels;
  for (Params* p : params) {
    // Validate that each video config has exactly one of |generator|,
    // |input_file_name| or |screen_share_config| set. Also validate that all
    // video stream labels are unique.
    for (auto& video_config : p->video_configs) {
      RTC_CHECK(video_config.stream_label);
      bool inserted =
          video_labels.insert(video_config.stream_label.value()).second;
      RTC_CHECK(inserted) << "Duplicate video stream label: "
                          << video_config.stream_label.value();
      RTC_CHECK(video_config.generator || video_config.input_file_name ||
                video_config.screen_share_config);
      if (video_config.generator) {
        RTC_CHECK(!video_config.input_file_name);
        RTC_CHECK(!video_config.screen_share_config);
      }
      if (video_config.input_file_name) {
        RTC_CHECK(!video_config.generator);
        RTC_CHECK(!video_config.screen_share_config);
      }
      if (video_config.screen_share_config) {
        RTC_CHECK(!video_config.generator);
        RTC_CHECK(!video_config.input_file_name);
      }
    }
    if (p->audio_config) {
      // Check that if mode input file name specified only if mode is kFile.
      if (p->audio_config.value().mode == AudioConfig::Mode::kGenerated) {
        RTC_CHECK(!p->audio_config.value().input_file_name);
      }
      if (p->audio_config.value().mode == AudioConfig::Mode::kFile) {
        RTC_CHECK(p->audio_config.value().input_file_name);
      }
    }
  }
}

void PeerConnectionE2EQualityTest::Run_s(RunParams run_params) {
  AddMedia(alice_.get());
  AddMedia(bob_.get());

  SetupCall(alice_.get(), bob_.get());

  WaitForTransceiversSetup(alice_->params(), bob_.get());
  WaitForTransceiversSetup(bob_->params(), alice_.get());
  SetupVideoSink(alice_->params(), bob_.get());
  SetupVideoSink(bob_->params(), alice_.get());

  StartVideo();

  rtc::Event done;
  done.Wait(static_cast<int>(run_params.run_duration.ms()));

  TearDownCall();
  video_quality_analyzer_injection_helper_->Stop();
}

void PeerConnectionE2EQualityTest::AddMedia(TestPeer* peer) {
  AddVideo(peer);
  if (peer->params()->audio_config) {
    AddAudio(peer);
  }
}

void PeerConnectionE2EQualityTest::AddVideo(TestPeer* peer) {
  Params* params = peer->params();
  for (auto video_config : params->video_configs) {
    // Params here valid because of pre run validation.
    std::unique_ptr<FrameGenerator> frame_generator = nullptr;
    if (video_config.generator) {
      absl::optional<FrameGenerator::OutputType> frame_generator_type =
          absl::nullopt;
      if (video_config.generator == VideoGeneratorType::kDefault) {
      } else if (video_config.generator == VideoGeneratorType::kI420A) {
        frame_generator_type = FrameGenerator::OutputType::I420A;
      } else if (video_config.generator == VideoGeneratorType::kI010) {
        frame_generator_type = FrameGenerator::OutputType::I010;
      }
      frame_generator = FrameGenerator::CreateSquareGenerator(
          static_cast<int>(video_config.width),
          static_cast<int>(video_config.height), frame_generator_type,
          absl::nullopt);
    } else if (video_config.input_file_name) {
      frame_generator = FrameGenerator::CreateFromYuvFile(
          std::vector<std::string>(1, video_config.input_file_name.value()),
          video_config.width, video_config.height, 1);
    } else if (video_config.screen_share_config) {
      // TODO(titovartem) implement screen share support
      RTC_NOTREACHED() << "Screen share is not implemented";
    }
    RTC_CHECK(frame_generator);

    VideoFrameWriter* writer = nullptr;
    if (video_config.input_dump_file_name) {
      auto video_writer = absl::make_unique<VideoFrameWriter>(
          video_config.input_dump_file_name.value(), video_config.width,
          video_config.height, video_config.fps);
      writer = video_writer.get();
      input_dump_video_writers_.push_back(std::move(video_writer));
    }
    frame_generator =
        video_quality_analyzer_injection_helper_->WrapFrameGenerator(
            video_config.stream_label.value(), std::move(frame_generator),
            writer);

    std::unique_ptr<FrameGeneratorCapturer> capturer =
        absl::WrapUnique(FrameGeneratorCapturer::Create(
            std::move(frame_generator), video_config.fps, clock_));

    rtc::scoped_refptr<FrameGeneratorCapturerVideoTrackSource> source =
        new rtc::RefCountedObject<FrameGeneratorCapturerVideoTrackSource>(
            move(capturer));
    video_sources_.push_back(source);
    RTC_LOG(INFO) << "Adding video with label "
                  << video_config.stream_label.value();
    rtc::scoped_refptr<VideoTrackInterface> track =
        peer->pc_factory()->CreateVideoTrack(video_config.stream_label.value(),
                                             source);
    peer->AddTransceiver(track);
  }
}

void PeerConnectionE2EQualityTest::AddAudio(TestPeer* peer) {
  RTC_CHECK(peer->params()->audio_config);
  rtc::scoped_refptr<webrtc::AudioSourceInterface> source =
      peer->pc_factory()->CreateAudioSource(
          peer->params()->audio_config->audio_options);
  rtc::scoped_refptr<AudioTrackInterface> track =
      peer->pc_factory()->CreateAudioTrack("audio", source);
  peer->AddTransceiver(track);
}

void PeerConnectionE2EQualityTest::SetupCall(TestPeer* alice, TestPeer* bob) {
  // Connect peers.
  ASSERT_TRUE(alice->ExchangeOfferAnswerWith(bob));
  // Do the SDP negotiation, and also exchange ice candidates.
  ASSERT_TRUE_WAIT(alice->signaling_state() == PeerConnectionInterface::kStable,
                   kDefaultTimeoutMs);
  ASSERT_TRUE_WAIT(alice->IsIceGatheringDone(), kDefaultTimeoutMs);
  ASSERT_TRUE_WAIT(bob->IsIceGatheringDone(), kDefaultTimeoutMs);

  // Connect an ICE candidate pairs.
  ASSERT_TRUE(bob->AddIceCandidates(alice->observer()->GetAllCandidates()));
  ASSERT_TRUE(alice->AddIceCandidates(bob->observer()->GetAllCandidates()));
  // This means that ICE and DTLS are connected.
  ASSERT_TRUE_WAIT(bob->IsIceConnected(), kDefaultTimeoutMs);
  ASSERT_TRUE_WAIT(alice->IsIceConnected(), kDefaultTimeoutMs);
}

void PeerConnectionE2EQualityTest::WaitForTransceiversSetup(
    Params* params,
    TestPeer* remote_peer) {
  uint64_t expected_remote_transceivers =
      params->video_configs.size() + (params->audio_config ? 1 : 0);
  ASSERT_TRUE_WAIT(remote_peer->observer()->on_track_transceivers_.size() ==
                       expected_remote_transceivers,
                   kDefaultTimeoutMs);
}

void PeerConnectionE2EQualityTest::SetupVideoSink(Params* params,
                                                  TestPeer* remote_peer) {
  if (params->video_configs.empty()) {
    return;
  }
  std::map<std::string, VideoConfig*> video_configs_by_label;
  for (auto& video_config : params->video_configs) {
    video_configs_by_label.insert(std::pair<std::string, VideoConfig*>(
        video_config.stream_label.value(), &video_config));
  }

  for (const auto& transceiver :
       remote_peer->observer()->on_track_transceivers_) {
    const rtc::scoped_refptr<MediaStreamTrackInterface>& track =
        transceiver->receiver()->track();
    if (track->kind() != MediaStreamTrackInterface::kVideoKind) {
      continue;
    }

    auto it = video_configs_by_label.find(track->id());
    RTC_CHECK(it != video_configs_by_label.end());
    VideoConfig* video_config = it->second;

    VideoFrameWriter* writer = nullptr;
    if (video_config->output_dump_file_name) {
      auto video_writer = absl::make_unique<VideoFrameWriter>(
          video_config->output_dump_file_name.value(), video_config->width,
          video_config->height, video_config->fps);
      writer = video_writer.get();
      output_dump_video_writers_.push_back(std::move(video_writer));
    }
    // It is safe to cast here, because it is checked above that track->kind()
    // is kVideoKind.
    auto* video_track = static_cast<VideoTrackInterface*>(track.get());
    auto video_sink =
        video_quality_analyzer_injection_helper_->CreateVideoSink(writer);
    video_track->AddOrUpdateSink(video_sink.get(), rtc::VideoSinkWants());
    output_video_sinks_.push_back(std::move(video_sink));
  }
}

void PeerConnectionE2EQualityTest::StartVideo() {
  for (const auto& video_source : video_sources_) {
    video_source->Start();
  }
}

void PeerConnectionE2EQualityTest::TearDownCall() {
  for (auto& video_source : video_sources_) {
    video_source->Stop();
  }

  alice_->pc()->Close();
  bob_->pc()->Close();

  for (auto& video_writer : input_dump_video_writers_) {
    video_writer->Close();
  }
  for (auto& video_writer : output_dump_video_writers_) {
    video_writer->Close();
  }
}

}  // namespace test
}  // namespace webrtc
