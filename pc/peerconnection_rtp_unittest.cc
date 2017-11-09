/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <memory>
#include <vector>

#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "api/jsep.h"
#include "api/mediastreaminterface.h"
#include "api/peerconnectioninterface.h"
#include "pc/mediastream.h"
#include "pc/mediastreamtrack.h"
#include "pc/peerconnectionwrapper.h"
#include "pc/test/fakeaudiocapturemodule.h"
#include "pc/test/mockpeerconnectionobservers.h"
#include "rtc_base/checks.h"
#include "rtc_base/gunit.h"
#include "rtc_base/ptr_util.h"
#include "rtc_base/refcountedobject.h"
#include "rtc_base/scoped_ref_ptr.h"
#include "rtc_base/thread.h"

// This file contains tests for RTP Media API-related behavior of
// |webrtc::PeerConnection|, see https://w3c.github.io/webrtc-pc/#rtp-media-api.

namespace {

class PeerConnectionRtpTest : public testing::Test {
 public:
  PeerConnectionRtpTest()
      : pc_factory_(webrtc::CreatePeerConnectionFactory(
            rtc::Thread::Current(),
            rtc::Thread::Current(),
            rtc::Thread::Current(),
            FakeAudioCaptureModule::Create(),
            webrtc::CreateBuiltinAudioEncoderFactory(),
            webrtc::CreateBuiltinAudioDecoderFactory(),
            nullptr,
            nullptr)) {}

  std::unique_ptr<webrtc::PeerConnectionWrapper> CreatePeerConnection() {
    webrtc::PeerConnectionInterface::RTCConfiguration config;
    auto observer = rtc::MakeUnique<webrtc::MockPeerConnectionObserver>();
    auto pc = pc_factory_->CreatePeerConnection(config, nullptr, nullptr,
                                                observer.get());
    return std::unique_ptr<webrtc::PeerConnectionWrapper>(
        new webrtc::PeerConnectionWrapper(pc_factory_, pc,
                                          std::move(observer)));
  }

 protected:
  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> pc_factory_;
};

// These tests cover |webrtc::PeerConnectionObserver| callbacks firing upon
// setting the remote description.
class PeerConnectionRtpCallbacksTest : public PeerConnectionRtpTest {};

TEST_F(PeerConnectionRtpCallbacksTest, AddTrackWithoutStreamFiresOnAddTrack) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
      pc_factory_->CreateAudioTrack("audio_track", nullptr));
  EXPECT_TRUE(caller->pc()->AddTrack(audio_track.get(), {}));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal()));

  ASSERT_EQ(1u, callee->observer()->add_track_events_.size());
  // TODO(deadbeef): When no stream is handled correctly we would expect
  // |add_track_events_[0].streams| to be empty. https://crbug.com/webrtc/7933
  ASSERT_EQ(1u, callee->observer()->add_track_events_[0].streams.size());
  EXPECT_TRUE(
      callee->observer()->add_track_events_[0].streams[0]->FindAudioTrack(
          "audio_track"));
}

TEST_F(PeerConnectionRtpCallbacksTest, AddTrackWithStreamFiresOnAddTrack) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
      pc_factory_->CreateAudioTrack("audio_track", nullptr));
  auto stream = webrtc::MediaStream::Create("audio_stream");
  EXPECT_TRUE(caller->pc()->AddTrack(audio_track.get(), {stream.get()}));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal()));

  ASSERT_EQ(1u, callee->observer()->add_track_events_.size());
  ASSERT_EQ(1u, callee->observer()->add_track_events_[0].streams.size());
  EXPECT_EQ("audio_stream",
            callee->observer()->add_track_events_[0].streams[0]->label());
  EXPECT_TRUE(
      callee->observer()->add_track_events_[0].streams[0]->FindAudioTrack(
          "audio_track"));
}

TEST_F(PeerConnectionRtpCallbacksTest,
       RemoveTrackWithoutStreamFiresOnRemoveTrack) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
      pc_factory_->CreateAudioTrack("audio_track", nullptr));
  auto sender = caller->pc()->AddTrack(audio_track.get(), {});
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal()));
  ASSERT_EQ(1u, callee->observer()->add_track_events_.size());
  EXPECT_TRUE(caller->pc()->RemoveTrack(sender));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal()));

  ASSERT_EQ(1u, callee->observer()->add_track_events_.size());
  EXPECT_EQ(callee->observer()->GetAddTrackReceivers(),
            callee->observer()->remove_track_events_);
}

TEST_F(PeerConnectionRtpCallbacksTest,
       RemoveTrackWithStreamFiresOnRemoveTrack) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
      pc_factory_->CreateAudioTrack("audio_track", nullptr));
  auto stream = webrtc::MediaStream::Create("audio_stream");
  auto sender = caller->pc()->AddTrack(audio_track.get(), {stream.get()});
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal()));
  ASSERT_EQ(1u, callee->observer()->add_track_events_.size());
  EXPECT_TRUE(caller->pc()->RemoveTrack(sender));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal()));

  ASSERT_EQ(1u, callee->observer()->add_track_events_.size());
  EXPECT_EQ(callee->observer()->GetAddTrackReceivers(),
            callee->observer()->remove_track_events_);
}

TEST_F(PeerConnectionRtpCallbacksTest,
       RemoveTrackWithSharedStreamFiresOnRemoveTrack) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track1(
      pc_factory_->CreateAudioTrack("audio_track1", nullptr));
  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track2(
      pc_factory_->CreateAudioTrack("audio_track2", nullptr));
  auto stream = webrtc::MediaStream::Create("shared_audio_stream");
  std::vector<webrtc::MediaStreamInterface*> streams{stream.get()};
  auto sender1 = caller->pc()->AddTrack(audio_track1.get(), streams);
  auto sender2 = caller->pc()->AddTrack(audio_track2.get(), streams);
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal()));

  ASSERT_EQ(2u, callee->observer()->add_track_events_.size());

  // Remove "audio_track1".
  EXPECT_TRUE(caller->pc()->RemoveTrack(sender1));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal()));
  ASSERT_EQ(2u, callee->observer()->add_track_events_.size());
  EXPECT_EQ(
      std::vector<rtc::scoped_refptr<webrtc::RtpReceiverInterface>>{
          callee->observer()->add_track_events_[0].receiver},
      callee->observer()->remove_track_events_);

  // Remove "audio_track2".
  EXPECT_TRUE(caller->pc()->RemoveTrack(sender2));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal()));
  ASSERT_EQ(2u, callee->observer()->add_track_events_.size());
  EXPECT_EQ(callee->observer()->GetAddTrackReceivers(),
            callee->observer()->remove_track_events_);
}

// These tests cover |webrtc::SetRemoteDescriptionObserver| callbacks with state
// changes firing upon setting the remote description.
class PeerConnectionRtpObserverTest : public PeerConnectionRtpTest {};

TEST_F(PeerConnectionRtpObserverTest, AddSenderWithoutStreamAddsReceiver) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();
  webrtc::SetRemoteDescriptionObserver::StateChanges state_changes;

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
      pc_factory_->CreateAudioTrack("audio_track", nullptr));
  EXPECT_TRUE(caller->pc()->AddTrack(audio_track.get(), {}));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal(),
                                           &state_changes, nullptr));

  EXPECT_EQ(1u, state_changes.receivers_added.size());
  webrtc::ReceiverWithStreams receiver_added = state_changes.receivers_added[0];
  EXPECT_EQ("audio_track", receiver_added.receiver->track()->id());
  // TODO(hbos): ...
  EXPECT_EQ(1u, receiver_added.streams.size());
  EXPECT_TRUE(receiver_added.streams[0]->FindAudioTrack("audio_track"));
}

TEST_F(PeerConnectionRtpObserverTest, AddSenderWithStreamAddsReceiver) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();
  webrtc::SetRemoteDescriptionObserver::StateChanges state_changes;

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
      pc_factory_->CreateAudioTrack("audio_track", nullptr));
  auto stream = webrtc::MediaStream::Create("audio_stream");
  EXPECT_TRUE(caller->pc()->AddTrack(audio_track.get(), {stream}));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal(),
                                           &state_changes, nullptr));

  EXPECT_EQ(1u, state_changes.receivers_added.size());
  webrtc::ReceiverWithStreams receiver_added = state_changes.receivers_added[0];
  EXPECT_EQ("audio_track", receiver_added.receiver->track()->id());
  EXPECT_EQ(1u, receiver_added.streams.size());
  EXPECT_EQ("audio_stream", receiver_added.streams[0]->label());
  EXPECT_TRUE(receiver_added.streams[0]->FindAudioTrack("audio_track"));
}

TEST_F(PeerConnectionRtpObserverTest,
       RemoveSenderWithoutStreamRemovesReceiver) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();
  webrtc::SetRemoteDescriptionObserver::StateChanges state_changes;

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
      pc_factory_->CreateAudioTrack("audio_track", nullptr));
  auto sender = caller->pc()->AddTrack(audio_track.get(), {});
  ASSERT_TRUE(sender);
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal(),
                                           &state_changes, nullptr));
  ASSERT_EQ(1u, state_changes.receivers_added.size());
  rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver =
      state_changes.receivers_added[0].receiver;
  ASSERT_TRUE(caller->pc()->RemoveTrack(sender));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal(),
                                           &state_changes, nullptr));

  EXPECT_EQ(1u, state_changes.receivers_removed.size());
  EXPECT_EQ(receiver, state_changes.receivers_removed[0]);
}

TEST_F(PeerConnectionRtpObserverTest, RemoveSenderWithStreamRemovesReceiver) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();
  webrtc::SetRemoteDescriptionObserver::StateChanges state_changes;

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
      pc_factory_->CreateAudioTrack("audio_track", nullptr));
  auto stream = webrtc::MediaStream::Create("audio_stream");
  auto sender = caller->pc()->AddTrack(audio_track.get(), {stream});
  ASSERT_TRUE(sender);
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal(),
                                           &state_changes, nullptr));
  ASSERT_EQ(1u, state_changes.receivers_added.size());
  rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver =
      state_changes.receivers_added[0].receiver;
  ASSERT_TRUE(caller->pc()->RemoveTrack(sender));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal(),
                                           &state_changes, nullptr));

  EXPECT_EQ(1u, state_changes.receivers_removed.size());
  EXPECT_EQ(receiver, state_changes.receivers_removed[0]);
}

TEST_F(PeerConnectionRtpObserverTest,
       RemoveSenderWithSharedStreamRemovesReceiver) {
  auto caller = CreatePeerConnection();
  auto callee = CreatePeerConnection();
  webrtc::SetRemoteDescriptionObserver::StateChanges state_changes;

  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track1(
      pc_factory_->CreateAudioTrack("audio_track1", nullptr));
  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track2(
      pc_factory_->CreateAudioTrack("audio_track2", nullptr));
  auto stream = webrtc::MediaStream::Create("shared_audio_stream");
  std::vector<webrtc::MediaStreamInterface*> streams{stream.get()};
  auto sender1 = caller->pc()->AddTrack(audio_track1.get(), streams);
  auto sender2 = caller->pc()->AddTrack(audio_track2.get(), streams);
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal(),
                                           &state_changes, nullptr));

  ASSERT_EQ(2u, state_changes.receivers_added.size());
  rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver1;
  rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver2;
  if (state_changes.receivers_added[0].receiver->track()->id() ==
      "audio_track1") {
    receiver1 = state_changes.receivers_added[0].receiver;
    receiver2 = state_changes.receivers_added[1].receiver;
  } else {
    receiver1 = state_changes.receivers_added[1].receiver;
    receiver2 = state_changes.receivers_added[0].receiver;
  }
  EXPECT_EQ("audio_track1", receiver1->track()->id());
  EXPECT_EQ("audio_track2", receiver2->track()->id());

  // Remove "audio_track1".
  EXPECT_TRUE(caller->pc()->RemoveTrack(sender1));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal(),
                                           &state_changes, nullptr));
  EXPECT_EQ(1u, state_changes.receivers_removed.size());
  EXPECT_EQ(receiver1, state_changes.receivers_removed[0]);

  // Remove "audio_track2".
  EXPECT_TRUE(caller->pc()->RemoveTrack(sender2));
  ASSERT_TRUE(callee->SetRemoteDescription(caller->CreateOfferAndSetAsLocal(),
                                           &state_changes, nullptr));
  EXPECT_EQ(1u, state_changes.receivers_removed.size());
  EXPECT_EQ(receiver2, state_changes.receivers_removed[0]);
}

}  // namespace
