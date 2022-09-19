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
using VideoSimulcastConfig = ::webrtc::webrtc_pc_e2e::
    PeerConnectionE2EQualityTestFixture::VideoSimulcastConfig;
using ::cricket::kAv1CodecName;
using ::cricket::kVp8CodecName;
using ::cricket::kVp9CodecName;
using ::testing::Combine;
using ::testing::UnitTest;
using ::testing::Values;
using ::testing::ValuesIn;

constexpr int kInputWidth = 1850;
constexpr int kInputHeight = 1150;

// Helper methods for concatenating nested vectors of vectors.
template <typename T, typename... Args>
void AppendVectors(std::vector<T>& res, const Args&... args) {
  using std::begin;
  using std::end;

  (res.insert(res.end(), begin(args), end(args)), ...);
}

template <typename... Args>
auto ConcatVectors(const Args&... args) {
  using T =
      typename std::tuple_element<0, std::tuple<Args...>>::type::value_type;
  std::vector<T> res;
  res.reserve((... + args.size()));
  AppendVectors(res, args...);
  return res;
}

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

struct SvcSelectLayerTestParameters {
  std::string codec_name;
  std::string scalability_mode;
  int target_spatial_index;
  int target_temporal_index;
};

static std::vector<SvcSelectLayerTestParameters> CreateAllLayerParameters(
    const std::string& codec_name,
    const std::string& scalability_mode_str) {
  std::vector<SvcSelectLayerTestParameters> result;
  absl::optional<ScalabilityMode> scalability_mode =
      ScalabilityModeFromString(scalability_mode_str);
  RTC_CHECK(scalability_mode.has_value())
      << "Unsupported scalability mode: " << scalability_mode_str;
  int num_spatial_layers = ScalabilityModeToNumSpatialLayers(*scalability_mode);
  int num_temporal_layers =
      ScalabilityModeToNumTemporalLayers(*scalability_mode);
  for (int spatial_layer = 0; spatial_layer < num_spatial_layers;
       ++spatial_layer) {
    for (int temporal_layer = 0; temporal_layer < num_temporal_layers;
         ++temporal_layer) {
      result.push_back(SvcSelectLayerTestParameters{
          codec_name, scalability_mode_str, spatial_layer, temporal_layer});
    }
  }
  return result;
}

class SvcSelectLayerTest
    : public testing::TestWithParam<
          std::tuple<SvcSelectLayerTestParameters, UseDependencyDescriptor>> {
 public:
  SvcSelectLayerTest()
      : video_codec_config(
            ToVideoCodecConfig(SvcSelectLayerTestParameters().codec_name)) {}

  static VideoCodecConfig ToVideoCodecConfig(absl::string_view codec) {
    if (codec == cricket::kVp9CodecName) {
      return VideoCodecConfig(
          cricket::kVp9CodecName,
          {{kVP9FmtpProfileId, VP9ProfileToString(VP9Profile::kProfile0)}});
    }

    return VideoCodecConfig(std::string(codec));
  }

  const SvcSelectLayerTestParameters& SvcSelectLayerTestParameters() const {
    return std::get<0>(GetParam());
  }

  bool UseDependencyDescriptor() const {
    return std::get<1>(GetParam()) == UseDependencyDescriptor::Enabled;
  }

 protected:
  VideoCodecConfig video_codec_config;
};

std::string SvcSelectLayerTestNameGenerator(
    const testing::TestParamInfo<std::tuple<SvcSelectLayerTestParameters,
                                            UseDependencyDescriptor>>& info) {
  rtc::StringBuilder ost;
  ost << std::get<0>(info.param).scalability_mode;
  ost << "_" << std::get<0>(info.param).target_spatial_index << "_"
      << std::get<0>(info.param).target_temporal_index;
  if (std::get<1>(info.param) == UseDependencyDescriptor::Enabled) {
    ost << "_DD";
  }
  return ost.str();
}

}  // namespace

// Tests whether the SVC encoded data can be successfully decoded in the
// target spatial/temporal layer by checking the number of video frames
// and output video size using an emulated Selective Forwarding Unit (SFU).
TEST_P(SvcSelectLayerTest, CheckDecodeSelectedLayer) {
  // WebRTC-VideoFrameTrackingIdAdvertised Field trial is not compatible
  // with this test.
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

  std::string scalability_mode_str =
      SvcSelectLayerTestParameters().scalability_mode;
  absl::optional<ScalabilityMode> scalability_mode =
      ScalabilityModeFromString(scalability_mode_str);
  ASSERT_TRUE(scalability_mode.has_value());

  int target_spatial_index =
      SvcSelectLayerTestParameters().target_spatial_index;
  int target_temporal_index =
      SvcSelectLayerTestParameters().target_temporal_index;
  int num_spatial_layers = ScalabilityModeToNumSpatialLayers(*scalability_mode);
  int num_temporal_layers =
      ScalabilityModeToNumTemporalLayers(*scalability_mode);
  ASSERT_GE(target_spatial_index, 0);
  ASSERT_LT(target_spatial_index, num_spatial_layers);
  ASSERT_GE(target_temporal_index, 0);
  ASSERT_LT(target_temporal_index, num_temporal_layers);

  VideoSimulcastConfig simulcast_config(
      /*is_svc=*/true, num_spatial_layers, target_spatial_index,
      target_temporal_index);

  auto fixture = CreateTestFixture(
      UnitTest::GetInstance()->current_test_info()->name(),
      *network_emulation_manager->time_controller(),
      CreateTwoNetworkLinks(network_emulation_manager.get(),
                            BuiltInNetworkBehaviorConfig()),
      [this, simulcast_config, scalability_mode_str](PeerConfigurer* alice) {
        VideoConfig video(/*stream_label=*/"alice-video", /*width=*/1850,
                          /*height=*/1110, /*fps=*/30);
        video.simulcast_config = simulcast_config;
        RtpEncodingParameters parameters;
        parameters.scalability_mode = scalability_mode_str;
        video.encoding_params.push_back(parameters);
        video.degradation_preference =
            DegradationPreference::MAINTAIN_RESOLUTION;
        alice->AddVideoConfig(
            std::move(video),
            CreateScreenShareFrameGenerator(
                video, ScreenShareConfig(TimeDelta::Seconds(5))));
        alice->SetVideoCodecs({video_codec_config});
      },
      [](PeerConfigurer* bob) {}, std::move(analyzer));
  fixture->Run(RunParams(TimeDelta::Seconds(5)));

  absl::optional<ScalabilityModeResolutionRatio> resolution_ratio =
      ScalabilityModeToResolutionRatio(*scalability_mode);

  int expected_width = kInputWidth;
  int expected_height = kInputHeight;
  if (!resolution_ratio) {
    RTC_DCHECK_EQ(num_spatial_layers, target_spatial_index + 1);
  } else if (*resolution_ratio == ScalabilityModeResolutionRatio::kTwoToOne) {
    const int shift = (num_spatial_layers - 1 - target_spatial_index);
    expected_width = kInputWidth >> shift;
    expected_height = kInputHeight >> shift;
  } else if (*resolution_ratio == ScalabilityModeResolutionRatio::kThreeToTwo) {
    const float ratio = std::pow(
        2.0 / 3.0,
        static_cast<float>(num_spatial_layers - 1 - target_spatial_index));
    expected_width = static_cast<float>(kInputWidth) * ratio;
    expected_height = static_cast<float>(kInputHeight) * ratio;
  }

  const int decoder_input_frames =
      SvcVideoQualityAnalyzer::CountDecodedFramesForLayer(
          analyzer_ptr->decoder_layers_seen(), target_spatial_index,
          target_temporal_index);
  int decoded_frames = 0;
  for (const auto& [frame_size, count] : analyzer_ptr->frames_decoded()) {
    // Some configurations may not return the correct frame size due to pixel
    // alignment.
    if (std::abs(frame_size.width * frame_size.height -
                 expected_width * expected_height) <
        0.05 * (expected_width * expected_height)) {
      decoded_frames = count;
      break;
    }
  }

  EXPECT_GT(decoder_input_frames, 0);
  EXPECT_GT(decoded_frames, 0);
  EXPECT_NEAR(decoder_input_frames, decoded_frames, 2);

  RTC_LOG(LS_INFO) << "Encoder layers: "
                   << analyzer_ptr->encoder_layers_seen().size();
  for (auto& [spatial_index, temporal_layers] :
       analyzer_ptr->encoder_layers_seen()) {
    for (auto& [temporal_index, frame_count] : temporal_layers) {
      RTC_LOG(LS_INFO) << "  Layer: " << spatial_index << "," << temporal_index
                       << " frames: " << frame_count;
    }
  }
  RTC_LOG(LS_INFO) << "Decoder layers: "
                   << analyzer_ptr->decoder_layers_seen().size();
  for (auto& [spatial_index, temporal_layers] :
       analyzer_ptr->decoder_layers_seen()) {
    for (auto& [temporal_index, frame_count] : temporal_layers) {
      RTC_LOG(LS_INFO) << "  Layer: " << spatial_index << "," << temporal_index
                       << " frames: " << frame_count;
    }
  }

  RTC_LOG(LS_INFO) << "Decoder Decoded ";
  for (auto& [frame_size, count] : analyzer_ptr->frames_decoded()) {
    RTC_LOG(LS_INFO) << "  Size: " << frame_size.width << "x"
                     << frame_size.height << " frames: " << count;
  }
}

INSTANTIATE_TEST_SUITE_P(
    SvcSelectLayerTestVP8,
    SvcSelectLayerTest,
    Combine(ValuesIn(
                ConcatVectors(CreateAllLayerParameters(kVp8CodecName, "L1T1"),
                              CreateAllLayerParameters(kVp8CodecName, "L1T2"),
                              CreateAllLayerParameters(kVp8CodecName, "L1T3"))),
            Values(UseDependencyDescriptor::Disabled,
                   UseDependencyDescriptor::Enabled)),
    SvcSelectLayerTestNameGenerator);

#if RTC_ENABLE_VP9
INSTANTIATE_TEST_SUITE_P(
    SvcSelectLayerTestVP9,
    SvcSelectLayerTest,
    Combine(ValuesIn(ConcatVectors(
                CreateAllLayerParameters(kVp9CodecName, "L1T1"),
                CreateAllLayerParameters(kVp9CodecName, "L1T2"),
                CreateAllLayerParameters(kVp9CodecName, "L1T3"),
                CreateAllLayerParameters(kVp9CodecName, "L2T1"),
                CreateAllLayerParameters(kVp9CodecName, "L2T1h"),
                CreateAllLayerParameters(kVp9CodecName, "L2T1_KEY"),
                CreateAllLayerParameters(kVp9CodecName, "L2T2"),
                CreateAllLayerParameters(kVp9CodecName, "L2T2h"),
                CreateAllLayerParameters(kVp9CodecName, "L2T2_KEY"),
                CreateAllLayerParameters(kVp9CodecName, "L2T2_KEY_SHIFT"),
                CreateAllLayerParameters(kVp9CodecName, "L2T3"),
                CreateAllLayerParameters(kVp9CodecName, "L2T3h"),
                CreateAllLayerParameters(kVp9CodecName, "L2T3_KEY"),
                // CreateAllLayerParameters(kVp9CodecName, "L2T3_KEY_SHIFT"),
                CreateAllLayerParameters(kVp9CodecName, "L3T1"),
                CreateAllLayerParameters(kVp9CodecName, "L3T1h"),
                CreateAllLayerParameters(kVp9CodecName, "L3T1_KEY"),
                CreateAllLayerParameters(kVp9CodecName, "L3T2"),
                CreateAllLayerParameters(kVp9CodecName, "L3T2h"),
                CreateAllLayerParameters(kVp9CodecName, "L3T2_KEY"),
                // CreateAllLayerParameters(kVp9CodecName, "L3T2_KEY_SHIFT"),
                CreateAllLayerParameters(kVp9CodecName, "L3T3"),
                CreateAllLayerParameters(kVp9CodecName, "L3T3h"),
                CreateAllLayerParameters(kVp9CodecName, "L3T3_KEY"),
                // CreateAllLayerParameters(kVp9CodecName, "L3T3_KEY_SHIFT"),
                CreateAllLayerParameters(kVp9CodecName, "S2T1"),
                CreateAllLayerParameters(kVp9CodecName, "S2T1h"),
                CreateAllLayerParameters(kVp9CodecName, "S2T2"),
                CreateAllLayerParameters(kVp9CodecName, "S2T2h"),
                CreateAllLayerParameters(kVp9CodecName, "S2T3"),
                CreateAllLayerParameters(kVp9CodecName, "S2T3h"),
                CreateAllLayerParameters(kVp9CodecName, "S3T1"),
                CreateAllLayerParameters(kVp9CodecName, "S3T1h"),
                CreateAllLayerParameters(kVp9CodecName, "S3T2"),
                CreateAllLayerParameters(kVp9CodecName, "S3T2h"),
                CreateAllLayerParameters(kVp9CodecName, "S3T3"),
                CreateAllLayerParameters(kVp9CodecName, "S3T3h"))),
            Values(UseDependencyDescriptor::Disabled,
                   UseDependencyDescriptor::Enabled)),
    SvcSelectLayerTestNameGenerator);
#endif

INSTANTIATE_TEST_SUITE_P(
    SvcSelectLayerTestAV1,
    SvcSelectLayerTest,
    Combine(ValuesIn(ConcatVectors(
                CreateAllLayerParameters(kAv1CodecName, "L1T1"),
                CreateAllLayerParameters(kAv1CodecName, "L1T2"),
                CreateAllLayerParameters(kAv1CodecName, "L1T3"),
                CreateAllLayerParameters(kAv1CodecName, "L2T1"),
                CreateAllLayerParameters(kAv1CodecName, "L2T1h"),
                CreateAllLayerParameters(kAv1CodecName, "L2T1_KEY"),
                CreateAllLayerParameters(kAv1CodecName, "L2T2"),
                CreateAllLayerParameters(kAv1CodecName, "L2T2h"),
                CreateAllLayerParameters(kAv1CodecName, "L2T2_KEY"),
                CreateAllLayerParameters(kAv1CodecName, "L2T2_KEY_SHIFT"),
                CreateAllLayerParameters(kAv1CodecName, "L2T3"),
                CreateAllLayerParameters(kAv1CodecName, "L2T3h"),
                CreateAllLayerParameters(kAv1CodecName, "L2T3_KEY"),
                // CreateAllLayerParameters(kAv1CodecName, "L2T3_KEY_SHIFT"),
                CreateAllLayerParameters(kAv1CodecName, "L3T1"),
                CreateAllLayerParameters(kAv1CodecName, "L3T1h"),
                CreateAllLayerParameters(kAv1CodecName, "L3T1_KEY"),
                CreateAllLayerParameters(kAv1CodecName, "L3T2"),
                CreateAllLayerParameters(kAv1CodecName, "L3T2h"),
                CreateAllLayerParameters(kAv1CodecName, "L3T2_KEY"),
                // CreateAllLayerParameters(kAv1CodecName, "L3T2_KEY_SHIFT"),
                CreateAllLayerParameters(kAv1CodecName, "L3T3"),
                CreateAllLayerParameters(kAv1CodecName, "L3T3h"),
                CreateAllLayerParameters(kAv1CodecName, "L3T3_KEY"),
                // CreateAllLayerParameters(kAv1CodecName, "L3T3_KEY_SHIFT"),
                CreateAllLayerParameters(kAv1CodecName, "S2T1"),
                CreateAllLayerParameters(kAv1CodecName, "S2T1h"),
                CreateAllLayerParameters(kAv1CodecName, "S2T2"),
                CreateAllLayerParameters(kAv1CodecName, "S2T2h"),
                CreateAllLayerParameters(kAv1CodecName, "S2T3"),
                CreateAllLayerParameters(kAv1CodecName, "S2T3h"),
                CreateAllLayerParameters(kAv1CodecName, "S3T1"),
                CreateAllLayerParameters(kAv1CodecName, "S3T1h"),
                CreateAllLayerParameters(kAv1CodecName, "S3T2"),
                CreateAllLayerParameters(kAv1CodecName, "S3T2h"),
                CreateAllLayerParameters(kAv1CodecName, "S3T3"),
                CreateAllLayerParameters(kAv1CodecName, "S3T3h"))),
            Values(UseDependencyDescriptor::Enabled)),
    SvcSelectLayerTestNameGenerator);

}  // namespace webrtc
