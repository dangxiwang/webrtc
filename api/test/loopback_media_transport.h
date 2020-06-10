/*
 *  Copyright 2018 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_TEST_LOOPBACK_MEDIA_TRANSPORT_H_
#define API_TEST_LOOPBACK_MEDIA_TRANSPORT_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "api/transport/media/media_transport_interface.h"
#include "rtc_base/async_invoker.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/thread.h"
#include "rtc_base/thread_checker.h"

namespace webrtc {

// Wrapper used to hand out unique_ptrs to loopback media
// transport without ownership changes to the underlying
// transport.
// It works in two modes:
// It can either wrap a factory, or it can wrap an existing interface.
// In the former mode, it delegates the work to the wrapped factory.
// In the latter mode, it always returns static instance of the transport
// interface.
//
// Example use:
// Factory wrap_static_interface = Wrapper(media_transport_interface);
// Factory wrap_factory = Wrapper(wrap_static_interface);
// The second factory may be created multiple times, and ownership may be passed
// to the client. The first factory counts the number of invocations of
// CreateMediaTransport();
class WrapperMediaTransportFactory : public MediaTransportFactory {
 public:
  explicit WrapperMediaTransportFactory(MediaTransportFactory* wrapped);

  RTCErrorOr<std::unique_ptr<MediaTransportInterface>> CreateMediaTransport(
      rtc::PacketTransportInternal* packet_transport,
      rtc::Thread* network_thread,
      const MediaTransportSettings& settings) override;

  RTCErrorOr<std::unique_ptr<MediaTransportInterface>> CreateMediaTransport(
      rtc::Thread* network_thread,
      const MediaTransportSettings& settings) override;

  std::string GetTransportName() const override;

  int created_transport_count() const;

 private:
  MediaTransportFactory* wrapped_factory_ = nullptr;
  int created_transport_count_ = 0;
};

// Contains two MediaTransportsInterfaces that are connected to each other.
// Currently supports audio only.
class MediaTransportPair {
 public:
  struct Stats {
    int sent_audio_frames = 0;
    int received_audio_frames = 0;
    int sent_video_frames = 0;
    int received_video_frames = 0;
  };

#if 0
  explicit MediaTransportPair(rtc::Thread* thread);
#endif
  ~MediaTransportPair();

  std::unique_ptr<MediaTransportFactory> first_factory() {
    return std::make_unique<WrapperMediaTransportFactory>(&first_factory_);
  }

  std::unique_ptr<MediaTransportFactory> second_factory() {
    return std::make_unique<WrapperMediaTransportFactory>(&second_factory_);
  }

  int first_factory_transport_count() const {
    return first_factory_.created_transport_count();
  }

  int second_factory_transport_count() const {
    return second_factory_.created_transport_count();
  }

 private:
  class LoopbackDataChannelTransport : public DataChannelTransportInterface {
   public:
    explicit LoopbackDataChannelTransport(rtc::Thread* thread);
    ~LoopbackDataChannelTransport() override;

    void Connect(LoopbackDataChannelTransport* other);

    RTCError OpenChannel(int channel_id) override;

    RTCError SendData(int channel_id,
                      const SendDataParams& params,
                      const rtc::CopyOnWriteBuffer& buffer) override;

    RTCError CloseChannel(int channel_id) override;

    bool IsReadyToSend() const override;

    void SetDataSink(DataChannelSink* sink) override;

    void OnReadyToSend(bool ready_to_send);

    void FlushAsyncInvokes();

   private:
    void OnData(int channel_id,
                DataMessageType type,
                const rtc::CopyOnWriteBuffer& buffer);

    void OnRemoteCloseChannel(int channel_id);

    rtc::Thread* const thread_;
    rtc::CriticalSection sink_lock_;
    DataChannelSink* data_sink_ RTC_GUARDED_BY(sink_lock_) = nullptr;

    bool ready_to_send_ RTC_GUARDED_BY(sink_lock_) = false;

    LoopbackDataChannelTransport* other_;

    rtc::AsyncInvoker invoker_;
  };

  WrapperMediaTransportFactory first_factory_;
  WrapperMediaTransportFactory second_factory_;
};

}  // namespace webrtc

#endif  // API_TEST_LOOPBACK_MEDIA_TRANSPORT_H_
