/*
 *  Copyright (c) 2004 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MEDIA_BASE_MEDIAENGINE_H_
#define MEDIA_BASE_MEDIAENGINE_H_

#if defined(WEBRTC_MAC) && !defined(WEBRTC_IOS)
#include <CoreAudio/CoreAudio.h>
#endif

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "api/audio_codecs/audio_decoder_factory.h"
#include "api/audio_codecs/audio_encoder_factory.h"
#include "api/rtpparameters.h"
#include "call/audio_state.h"
#include "media/base/codec.h"
#include "media/base/mediachannel.h"
#include "media/base/videocommon.h"
#include "rtc_base/platform_file.h"

namespace webrtc {
class AudioDeviceModule;
class AudioMixer;
class AudioProcessing;
class Call;
}  // namespace webrtc

namespace cricket {

webrtc::RTCError ValidateRtpParameters(
    const webrtc::RtpParameters& old_parameters,
    const webrtc::RtpParameters& new_parameters);

struct RtpCapabilities {
  RtpCapabilities();
  ~RtpCapabilities();
  std::vector<webrtc::RtpExtension> header_extensions;
};

class AudioEngineInterface {
 public:
  AudioEngineInterface() = default;
  virtual ~AudioEngineInterface() = default;
  RTC_DISALLOW_COPY_AND_ASSIGN(AudioEngineInterface);

  // Initialization
  // Starts the engine.
  virtual void Init() = 0;

  // TODO(solenberg): Remove once VoE API refactoring is done.
  virtual rtc::scoped_refptr<webrtc::AudioState> GetAudioState() const = 0;

  // MediaChannel creation
  // Creates a voice media channel. Returns NULL on failure.
  virtual VoiceMediaChannel* CreateChannel(webrtc::Call* call,
                                           const MediaConfig& config,
                                           const AudioOptions& options) = 0;

  virtual const std::vector<AudioCodec>& send_codecs() const = 0;
  virtual const std::vector<AudioCodec>& recv_codecs() const = 0;
  virtual RtpCapabilities GetCapabilities() const = 0;

  // Starts AEC dump using existing file, a maximum file size in bytes can be
  // specified. Logging is stopped just before the size limit is exceeded.
  // If max_size_bytes is set to a value <= 0, no limit will be used.
  virtual bool StartAecDump(rtc::PlatformFile file, int64_t max_size_bytes) = 0;

  // Stops recording AEC dump.
  virtual void StopAecDump() = 0;
};

class VideoEngineInterface {
 public:
  VideoEngineInterface() = default;
  virtual ~VideoEngineInterface() = default;
  RTC_DISALLOW_COPY_AND_ASSIGN(VideoEngineInterface);

  // Creates a video media channel, paired with the specified voice channel.
  // Returns NULL on failure.
  virtual VideoMediaChannel* CreateChannel(webrtc::Call* call,
                                           const MediaConfig& config,
                                           const VideoOptions& options) = 0;

  virtual std::vector<VideoCodec> codecs() const = 0;
  virtual RtpCapabilities GetCapabilities() const = 0;
};

// MediaEngineInterface is an abstraction of a media engine which can be
// subclassed to support different media componentry backends.
// It supports voice and video operations in the same class to facilitate
// proper synchronization between both media types.
class MediaEngineInterface {
 public:
  virtual ~MediaEngineInterface() {}
  // Initialization
  // Starts the engine.
  virtual bool Init() = 0;
  // TODO(solenberg): Remove once VoE API refactoring is done.
  virtual rtc::scoped_refptr<webrtc::AudioState> GetAudioState() const = 0;

  // MediaChannel creation
  // Creates a voice media channel. Returns NULL on failure.
  virtual VoiceMediaChannel* CreateChannel(webrtc::Call* call,
                                           const MediaConfig& config,
                                           const AudioOptions& options) = 0;
  // Creates a video media channel, paired with the specified voice channel.
  // Returns NULL on failure.
  virtual VideoMediaChannel* CreateVideoChannel(
      webrtc::Call* call,
      const MediaConfig& config,
      const VideoOptions& options) = 0;

  virtual const std::vector<AudioCodec>& audio_send_codecs() = 0;
  virtual const std::vector<AudioCodec>& audio_recv_codecs() = 0;
  virtual RtpCapabilities GetAudioCapabilities() = 0;
  virtual std::vector<VideoCodec> video_codecs() = 0;
  virtual RtpCapabilities GetVideoCapabilities() = 0;

  // Starts AEC dump using existing file, a maximum file size in bytes can be
  // specified. Logging is stopped just before the size limit is exceeded.
  // If max_size_bytes is set to a value <= 0, no limit will be used.
  virtual bool StartAecDump(rtc::PlatformFile file, int64_t max_size_bytes) = 0;

  // Stops recording AEC dump.
  virtual void StopAecDump() = 0;
};

// CompositeMediaEngine constructs a MediaEngine from separate
// voice and video engine classes.
class CompositeMediaEngine : public MediaEngineInterface {
 public:
  CompositeMediaEngine(std::unique_ptr<AudioEngineInterface> audio_engine,
                       std::unique_ptr<VideoEngineInterface> video_engine);
  ~CompositeMediaEngine() override;

  bool Init() override;

  rtc::scoped_refptr<webrtc::AudioState> GetAudioState() const override;
  VoiceMediaChannel* CreateChannel(webrtc::Call* call,
                                   const MediaConfig& config,
                                   const AudioOptions& options) override;
  VideoMediaChannel* CreateVideoChannel(webrtc::Call* call,
                                        const MediaConfig& config,
                                        const VideoOptions& options) override;

  const std::vector<AudioCodec>& audio_send_codecs() override;
  const std::vector<AudioCodec>& audio_recv_codecs() override;
  RtpCapabilities GetAudioCapabilities() override;
  std::vector<VideoCodec> video_codecs() override;
  RtpCapabilities GetVideoCapabilities() override;

  bool StartAecDump(rtc::PlatformFile file, int64_t max_size_bytes) override;

  void StopAecDump() override;

 protected:
  AudioEngineInterface& voice();
  VideoEngineInterface& video();
  const AudioEngineInterface& voice() const;
  const VideoEngineInterface& video() const;

 private:
  std::pair<std::unique_ptr<AudioEngineInterface>,
            std::unique_ptr<VideoEngineInterface>>
      engines_;
};

enum DataChannelType { DCT_NONE = 0, DCT_RTP = 1, DCT_SCTP = 2 };

class DataEngineInterface {
 public:
  virtual ~DataEngineInterface() {}
  virtual DataMediaChannel* CreateChannel(const MediaConfig& config) = 0;
  virtual const std::vector<DataCodec>& data_codecs() = 0;
};

webrtc::RtpParameters CreateRtpParametersWithOneEncoding();
webrtc::RtpParameters CreateRtpParametersWithEncodings(StreamParams sp);

}  // namespace cricket

#endif  // MEDIA_BASE_MEDIAENGINE_H_
