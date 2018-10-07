/*
 *  Copyright 2018 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MEDIA_TRANSPORT_TEST_FAKE_MEDIA_TRANSPORT_H_
#define MEDIA_TRANSPORT_TEST_FAKE_MEDIA_TRANSPORT_H_

#include <memory>

#include "api/media_transport_interface.h"

namespace webrtc {

// TODO(sukhanov): For now fake media transport does nothing and used only
// in jsepcontroller unittests. In the future we should implement fake media
// transport, which forwards frames to another fake media transport, so we
// could unit test audio / video integration.
class FakeMediaTransport : public MediaTransportInterface {
 public:
  explicit FakeMediaTransport(bool is_caller) : is_caller_(is_caller) {}
  ~FakeMediaTransport() = default;

  RTCError SendAudioFrame(uint64_t channel_id,
                          MediaTransportEncodedAudioFrame frame) override {
    return RTCError::OK();
  }

  RTCError SendVideoFrame(
      uint64_t channel_id,
      const MediaTransportEncodedVideoFrame& frame) override {
    return RTCError::OK();
  }

  RTCError RequestKeyFrame(uint64_t channel_id) override {
    return RTCError::OK();
  };

  void SetReceiveAudioSink(MediaTransportAudioSinkInterface* sink) override {}
  void SetReceiveVideoSink(MediaTransportVideoSinkInterface* sink) override {}

  bool TestOnly_is_caller() const { return is_caller_; }

 private:
  const bool is_caller_;
};

// Fake media transport factory creates fake media transport.
class FakeMediaTransportFactory : public MediaTransportFactory {
 public:
  FakeMediaTransportFactory() = default;
  ~FakeMediaTransportFactory() = default;

  RTCErrorOr<std::unique_ptr<MediaTransportInterface>> CreateMediaTransport(
      rtc::PacketTransportInternal* packet_transport,
      rtc::Thread* network_thread,
      bool is_caller) override {
    RTC_CHECK(network_thread != nullptr);
    RTC_CHECK(packet_transport != nullptr);

    std::unique_ptr<MediaTransportInterface> media_transport =
        absl::MakeUnique<FakeMediaTransport>(is_caller);
    return media_transport;
  }
};

}  // namespace webrtc

#endif  // MEDIA_TRANSPORT_TEST_FAKE_MEDIA_TRANSPORT_H_
