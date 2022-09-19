/*
 *  Copyright (c) 2022 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "api/media_stream_interface.h"
#include "api/test/create_network_emulation_manager.h"
#include "api/test/create_peer_connection_quality_test_frame_generator.h"
#include "api/test/create_peerconnection_quality_test_fixture.h"
#include "api/test/create_two_network_links.h"
#include "api/test/frame_generator_interface.h"
#include "api/test/network_emulation_manager.h"
#include "api/test/peerconnection_quality_test_fixture.h"
#include "api/test/simulated_network.h"
#include "api/test/time_controller.h"
#include "api/video_codecs/vp9_profile.h"
#include "call/simulated_network.h"
#include "modules/video_coding/codecs/vp9/include/vp9.h"
#include "modules/video_coding/svc/scalability_mode_util.h"
#include "pc/test/svc_video_quality_analyzer.h"
#include "system_wrappers/include/field_trial.h"
#include "test/field_trial.h"
#include "test/gmock.h"
#include "test/gtest.h"
#include "test/pc/e2e/network_quality_metrics_reporter.h"
#include "test/testsupport/file_utils.h"

namespace webrtc {
namespace {

using PeerConfigurer = ::webrtc::webrtc_pc_e2e::
    PeerConnectionE2EQualityTestFixture::PeerConfigurer;
using RunParams =
    ::webrtc::webrtc_pc_e2e::PeerConnectionE2EQualityTestFixture::RunParams;
using VideoConfig =
    ::webrtc::webrtc_pc_e2e::PeerConnectionE2EQualityTestFixture::VideoConfig;
using ScreenShareConfig = ::webrtc::webrtc_pc_e2e::
    PeerConnectionE2EQualityTestFixture::ScreenShareConfig;
using VideoCodecConfig = ::webrtc::webrtc_pc_e2e::
    PeerConnectionE2EQualityTestFixture::VideoCodecConfig;
using ::cricket::kAv1CodecName;
using ::cricket::kVp8CodecName;
using ::cricket::kVp9CodecName;
using ::testing::Combine;
using ::testing::UnitTest;
using ::testing::Values;
using ::testing::ValuesIn;

std::unique_ptr<webrtc_pc_e2e::PeerConnectionE2EQualityTestFixture>
CreateTestFixture(absl::string_view test_case_name,
                  TimeController& time_controller,
                  std::pair<EmulatedNetworkManagerInterface*,
                            EmulatedNetworkManagerInterface*> network_links,
                  rtc::FunctionView<void(PeerConfigurer*)> alice_configurer,
                  rtc::FunctionView<void(PeerConfigurer*)> bob_configurer,
                  std::unique_ptr<VideoQualityAnalyzerInterface>
                      video_quality_analyzer = nullptr) {
  auto fixture = webrtc_pc_e2e::CreatePeerConnectionE2EQualityTestFixture(
      std::string(test_case_name), time_controller, nullptr,
      std::move(video_quality_analyzer));
  fixture->AddPeer(network_links.first->network_dependencies(),
                   alice_configurer);
  fixture->AddPeer(network_links.second->network_dependencies(),
                   bob_configurer);
  return fixture;
}

// Takes the current active field trials set, and appends some new trials.
std::string AppendFieldTrials(std::string new_trial_string) {
  return std::string(field_trial::GetFieldTrialString()) + new_trial_string;
}

enum class UseDependencyDescriptor {
  Enabled,
  Disabled,
};

struct SvcTestParameters {
  static SvcTestParameters Create(const std::string& codec_name,
                                  const std::string& scalability_mode_str) {
    absl::optional<ScalabilityMode> scalability_mode =
        ScalabilityModeFromString(scalability_mode_str);
    RTC_CHECK(scalability_mode.has_value())
        << "Unsupported scalability mode: " << scalability_mode_str;

    int num_spatial_layers =
        ScalabilityModeToNumSpatialLayers(*scalability_mode);
    int num_temporal_layers =
        ScalabilityModeToNumTemporalLayers(*scalability_mode);

    return SvcTestParameters{codec_name, scalability_mode_str,
                             num_spatial_layers, num_temporal_layers};
  }

  std::string codec_name;
  std::string scalability_mode;
  int expected_spatial_layers;
  int expected_temporal_layers;
};

class SvcTest : public testing::TestWithParam<
                    std::tuple<SvcTestParameters, UseDependencyDescriptor>> {
 public:
  SvcTest()
      : video_codec_config(ToVideoCodecConfig(SvcTestParameters().codec_name)) {
  }

  static VideoCodecConfig ToVideoCodecConfig(absl::string_view codec) {
    if (codec == cricket::kVp9CodecName) {
      return VideoCodecConfig(
          cricket::kVp9CodecName,
          {{kVP9FmtpProfileId, VP9ProfileToString(VP9Profile::kProfile0)}});
    }

    return VideoCodecConfig(std::string(codec));
  }

  const SvcTestParameters& SvcTestParameters() const {
    return std::get<0>(GetParam());
  }

  bool UseDependencyDescriptor() const {
    return std::get<1>(GetParam()) == UseDependencyDescriptor::Enabled;
  }

 protected:
  VideoCodecConfig video_codec_config;
};

std::string SvcTestNameGenerator(
    const testing::TestParamInfo<SvcTest::ParamType>& info) {
  return std::get<0>(info.param).scalability_mode +
         (std::get<1>(info.param) == UseDependencyDescriptor::Enabled ? "_DD"
                                                                      : "");
}

}  // namespace

TEST_P(SvcTest, ScalabilityModeSupported) {
  std::string trials;
  if (UseDependencyDescriptor()) {
    trials += "WebRTC-DependencyDescriptorAdvertised/Enabled/";
  }
  webrtc::test::ScopedFieldTrials override_trials(AppendFieldTrials(trials));
  std::unique_ptr<NetworkEmulationManager> network_emulation_manager =
      CreateNetworkEmulationManager(webrtc::TimeMode::kSimulated);
  auto analyzer = std::make_unique<SvcVideoQualityAnalyzer>(
      network_emulation_manager->time_controller()->GetClock());
  SvcVideoQualityAnalyzer* analyzer_ptr = analyzer.get();
  auto fixture = CreateTestFixture(
      UnitTest::GetInstance()->current_test_info()->name(),
      *network_emulation_manager->time_controller(),
      CreateTwoNetworkLinks(network_emulation_manager.get(),
                            BuiltInNetworkBehaviorConfig()),
      [this](PeerConfigurer* alice) {
        VideoConfig video(/*stream_label=*/"alice-video", /*width=*/1850,
                          /*height=*/1110, /*fps=*/30);
        RtpEncodingParameters parameters;
        parameters.scalability_mode = SvcTestParameters().scalability_mode;
        video.encoding_params.push_back(parameters);
        alice->AddVideoConfig(
            std::move(video),
            CreateScreenShareFrameGenerator(
                video, ScreenShareConfig(TimeDelta::Seconds(5))));
        alice->SetVideoCodecs({video_codec_config});
      },
      [](PeerConfigurer* bob) {}, std::move(analyzer));
  fixture->Run(RunParams(TimeDelta::Seconds(5)));
  EXPECT_THAT(analyzer_ptr->encoder_layers_seen(),
              HasSpatialAndTemporalLayers(
                  SvcTestParameters().expected_spatial_layers,
                  SvcTestParameters().expected_temporal_layers));
  EXPECT_THAT(analyzer_ptr->decoder_layers_seen(),
              HasSpatialAndTemporalLayers(
                  SvcTestParameters().expected_spatial_layers,
                  SvcTestParameters().expected_temporal_layers));
  RTC_LOG(LS_INFO) << "Encoder layers seen: "
                   << analyzer_ptr->encoder_layers_seen().size();
  for (auto& [spatial_index, temporal_layers] :
       analyzer_ptr->encoder_layers_seen()) {
    for (auto& [temporal_index, frame_count] : temporal_layers) {
      RTC_LOG(LS_INFO) << "  Layer: " << spatial_index << "," << temporal_index
                       << " frames: " << frame_count;
    }
  }
  RTC_LOG(LS_INFO) << "Decoder layers seen: "
                   << analyzer_ptr->decoder_layers_seen().size();
  for (auto& [spatial_index, temporal_layers] :
       analyzer_ptr->decoder_layers_seen()) {
    for (auto& [temporal_index, frame_count] : temporal_layers) {
      RTC_LOG(LS_INFO) << "  Layer: " << spatial_index << "," << temporal_index
                       << " frames: " << frame_count;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    SvcTestVP8,
    SvcTest,
    Combine(Values(SvcTestParameters::Create(kVp8CodecName, "L1T1"),
                   SvcTestParameters::Create(kVp8CodecName, "L1T2"),
                   SvcTestParameters::Create(kVp8CodecName, "L1T3")),
            Values(UseDependencyDescriptor::Disabled,
                   UseDependencyDescriptor::Enabled)),
    SvcTestNameGenerator);
#if RTC_ENABLE_VP9
INSTANTIATE_TEST_SUITE_P(
    SvcTestVP9,
    SvcTest,
    Combine(
        // TODO(bugs.webrtc.org/13960): Fix and enable remaining VP9 modes
        ValuesIn({
            SvcTestParameters::Create(kVp9CodecName, "L1T1"),
            SvcTestParameters::Create(kVp9CodecName, "L1T2"),
            SvcTestParameters::Create(kVp9CodecName, "L1T3"),
            SvcTestParameters::Create(kVp9CodecName, "L2T1"),
            SvcTestParameters::Create(kVp9CodecName, "L2T1h"),
            SvcTestParameters::Create(kVp9CodecName, "L2T1_KEY"),
            SvcTestParameters::Create(kVp9CodecName, "L2T2"),
            SvcTestParameters::Create(kVp9CodecName, "L2T2h"),
            SvcTestParameters::Create(kVp9CodecName, "L2T2_KEY"),
            SvcTestParameters::Create(kVp9CodecName, "L2T2_KEY_SHIFT"),
            SvcTestParameters::Create(kVp9CodecName, "L2T3"),
            SvcTestParameters::Create(kVp9CodecName, "L2T3h"),
            SvcTestParameters::Create(kVp9CodecName, "L2T3_KEY"),
            // SvcTestParameters::Create(kVp9CodecName, "L2T3_KEY_SHIFT"),
            SvcTestParameters::Create(kVp9CodecName, "L3T1"),
            SvcTestParameters::Create(kVp9CodecName, "L3T1h"),
            SvcTestParameters::Create(kVp9CodecName, "L3T1_KEY"),
            SvcTestParameters::Create(kVp9CodecName, "L3T2"),
            SvcTestParameters::Create(kVp9CodecName, "L3T2h"),
            SvcTestParameters::Create(kVp9CodecName, "L3T2_KEY"),
            // SvcTestParameters::Create(kVp9CodecName, "L3T2_KEY_SHIFT"),
            SvcTestParameters::Create(kVp9CodecName, "L3T3"),
            SvcTestParameters::Create(kVp9CodecName, "L3T3h"),
            SvcTestParameters::Create(kVp9CodecName, "L3T3_KEY"),
            // SvcTestParameters::Create(kVp9CodecName, "L3T3_KEY_SHIFT"),
            // SvcTestParameters::Create(kVp9CodecName, "S2T1"),
            // SvcTestParameters::Create(kVp9CodecName, "S2T1h"),
            // SvcTestParameters::Create(kVp9CodecName, "S2T2"),
            // SvcTestParameters::Create(kVp9CodecName, "S2T2h"),
            SvcTestParameters::Create(kVp9CodecName, "S2T3"),
            // SvcTestParameters::Create(kVp9CodecName, "S2T3h"),
            // SvcTestParameters::Create(kVp9CodecName, "S3T1"),
            // SvcTestParameters::Create(kVp9CodecName, "S3T1h"),
            // SvcTestParameters::Create(kVp9CodecName, "S3T2"),
            // SvcTestParameters::Create(kVp9CodecName, "S3T2h"),
            // SvcTestParameters::Create(kVp9CodecName, "S3T3"),
            // SvcTestParameters::Create(kVp9CodecName, "S3T3h"),
        }),
        Values(UseDependencyDescriptor::Disabled,
               UseDependencyDescriptor::Enabled)),
    SvcTestNameGenerator);

INSTANTIATE_TEST_SUITE_P(
    SvcTestAV1,
    SvcTest,
    Combine(ValuesIn({
                SvcTestParameters::Create(kAv1CodecName, "L1T1"),
                SvcTestParameters::Create(kAv1CodecName, "L1T2"),
                SvcTestParameters::Create(kAv1CodecName, "L1T3"),
                SvcTestParameters::Create(kAv1CodecName, "L2T1"),
                SvcTestParameters::Create(kAv1CodecName, "L2T1h"),
                SvcTestParameters::Create(kAv1CodecName, "L2T1_KEY"),
                SvcTestParameters::Create(kAv1CodecName, "L2T2"),
                SvcTestParameters::Create(kAv1CodecName, "L2T2h"),
                SvcTestParameters::Create(kAv1CodecName, "L2T2_KEY"),
                SvcTestParameters::Create(kAv1CodecName, "L2T2_KEY_SHIFT"),
                SvcTestParameters::Create(kAv1CodecName, "L2T3"),
                SvcTestParameters::Create(kAv1CodecName, "L2T3h"),
                SvcTestParameters::Create(kAv1CodecName, "L2T3_KEY"),
                // SvcTestParameters::Create(kAv1CodecName, "L2T3_KEY_SHIFT"),
                SvcTestParameters::Create(kAv1CodecName, "L3T1"),
                SvcTestParameters::Create(kAv1CodecName, "L3T1h"),
                SvcTestParameters::Create(kAv1CodecName, "L3T1_KEY"),
                SvcTestParameters::Create(kAv1CodecName, "L3T2"),
                SvcTestParameters::Create(kAv1CodecName, "L3T2h"),
                SvcTestParameters::Create(kAv1CodecName, "L3T2_KEY"),
                // SvcTestParameters::Create(kAv1CodecName, "L3T2_KEY_SHIFT"),
                SvcTestParameters::Create(kAv1CodecName, "L3T3"),
                SvcTestParameters::Create(kAv1CodecName, "L3T3h"),
                SvcTestParameters::Create(kAv1CodecName, "L3T3_KEY"),
                // SvcTestParameters::Create(kAv1CodecName, "L3T3_KEY_SHIFT"),
                // SvcTestParameters::Create(kAv1CodecName, "S2T1"),
                // SvcTestParameters::Create(kAv1CodecName, "S2T1h"),
                // SvcTestParameters::Create(kAv1CodecName, "S2T2"),
                // SvcTestParameters::Create(kAv1CodecName, "S2T2h"),
                // SvcTestParameters::Create(kAv1CodecName, "S2T3"),
                // SvcTestParameters::Create(kAv1CodecName, "S2T3h"),
                // SvcTestParameters::Create(kAv1CodecName, "S3T1"),
                // SvcTestParameters::Create(kAv1CodecName, "S3T1h"),
                // SvcTestParameters::Create(kAv1CodecName, "S3T2"),
                // SvcTestParameters::Create(kAv1CodecName, "S3T2h"),
                // SvcTestParameters::Create(kAv1CodecName, "S3T3"),
                // SvcTestParameters::Create(kAv1CodecName, "S3T3h"),
            }),
            Values(UseDependencyDescriptor::Enabled)),
    SvcTestNameGenerator);

#endif

}  // namespace webrtc
