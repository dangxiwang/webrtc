/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "media/engine/null_webrtc_video_engine.h"

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "api/task_queue/default_task_queue_factory.h"
#include "api/task_queue/task_queue_factory.h"
#include "media/engine/webrtc_voice_engine.h"
#include "modules/audio_device/include/mock_audio_device.h"
#include "modules/audio_processing/include/audio_processing.h"
#include "test/gtest.h"
#include "test/mock_audio_decoder_factory.h"
#include "test/mock_audio_encoder_factory.h"

namespace cricket {

// Simple test to check if NullWebRtcVideoEngine implements the methods
// required by CompositeMediaEngine.
TEST(NullWebRtcVideoEngineTest, CheckInterface) {
  std::unique_ptr<webrtc::TaskQueueFactory> task_queue_factory =
      webrtc::CreateDefaultTaskQueueFactory();
  ::testing::NiceMock<webrtc::test::MockAudioDeviceModule> adm;
  auto audio_engine = absl::make_unique<WebRtcVoiceEngine>(
      task_queue_factory.get(), &adm,
      webrtc::MockAudioEncoderFactory::CreateUnusedFactory(),
      webrtc::MockAudioDecoderFactory::CreateUnusedFactory(), nullptr,
      webrtc::AudioProcessingBuilder().Create());

  CompositeMediaEngine engine(std::move(audio_engine),
                              absl::make_unique<NullWebRtcVideoEngine>());

  EXPECT_TRUE(engine.Init());
}

}  // namespace cricket
