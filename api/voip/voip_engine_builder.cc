//
//  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
//
//  Use of this source code is governed by a BSD-style license
//  that can be found in the LICENSE file in the root of the source
//  tree. An additional intellectual property rights grant can be found
//  in the file PATENTS.  All contributing project authors may
//  be found in the AUTHORS file in the root of the source tree.
//

#include "api/voip/voip_engine_builder.h"

#include <memory>
#include <utility>

#include "api/audio_codecs/audio_format.h"
#include "api/task_queue/default_task_queue_factory.h"
#include "audio/voip/voip_core.h"
#include "modules/audio_device/include/audio_device.h"
#include "modules/audio_processing/include/audio_processing.h"
#include "rtc_base/logging.h"

namespace webrtc {

VoipEngineBuilder& VoipEngineBuilder::SetTaskQueueFactory(
    std::unique_ptr<TaskQueueFactory> task_queue_factory) {
  task_queue_factory_ = std::move(task_queue_factory);
  return *this;
}

VoipEngineBuilder& VoipEngineBuilder::SetAudioProcessing(
    std::unique_ptr<AudioProcessing> audio_processing) {
  audio_processing_ = std::move(audio_processing);
  return *this;
}

VoipEngineBuilder& VoipEngineBuilder::SetAudioDeviceModule(
    rtc::scoped_refptr<AudioDeviceModule> audio_device_module) {
  audio_device_module_ = audio_device_module;
  return *this;
}

VoipEngineBuilder& VoipEngineBuilder::SetAudioEncoderFactory(
    rtc::scoped_refptr<AudioEncoderFactory> encoder_factory) {
  encoder_factory_ = encoder_factory;
  return *this;
}

VoipEngineBuilder& VoipEngineBuilder::SetAudioDecoderFactory(
    rtc::scoped_refptr<AudioDecoderFactory> decoder_factory) {
  decoder_factory_ = decoder_factory;
  return *this;
}

std::unique_ptr<VoipEngine> VoipEngineBuilder::Create() {
  // In order to trim the size of unused codecs, application
  // need to set audio codec factories.
  RTC_DCHECK(encoder_factory_);
  RTC_DCHECK(decoder_factory_);

  if (!task_queue_factory_) {
    task_queue_factory_ = CreateDefaultTaskQueueFactory();
  }

  if (!audio_processing_) {
    RTC_LOG(INFO) << "Creating default APM.";
    audio_processing_.reset(AudioProcessingBuilder().Create());
  }

  if (!audio_device_module_) {
    RTC_LOG(INFO) << "Creating default ADM.";
    audio_device_module_ = AudioDeviceModule::Create(
        AudioDeviceModule::kPlatformDefaultAudio, task_queue_factory_.get());
  }

  auto voip_core = std::make_unique<VoipCore>();

  voip_core->Init(std::move(task_queue_factory_), std::move(audio_processing_),
                  audio_device_module_, encoder_factory_, decoder_factory_);
  return voip_core;
}

void VoipEngineBuilder::SetLogLevel(absl::string_view log_level) {
  RTC_DCHECK(log_level.size() > 0);
  rtc::LogMessage::ConfigureLogging(log_level.data());
}

}  // namespace webrtc
