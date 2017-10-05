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

namespace {

const int kDefaultTimeoutMs = 10000;

class FakeAudioMediaStreamTrack
    : public rtc::RefCountedObject<
          webrtc::MediaStreamTrack<webrtc::AudioTrackInterface>> {
 public:
  FakeAudioMediaStreamTrack(const std::string& id)
      : rtc::RefCountedObject<
            webrtc::MediaStreamTrack<webrtc::AudioTrackInterface>>(id) {}

  std::string kind() const override {
    return webrtc::MediaStreamTrackInterface::kAudioKind;
  }

  webrtc::AudioSourceInterface* GetSource() const override { return nullptr; }

  void AddSink(webrtc::AudioTrackSinkInterface* sink) override {}

  void RemoveSink(webrtc::AudioTrackSinkInterface* sink) override {}

  bool GetSignalLevel(int* level) override {
    RTC_NOTREACHED();
    return false;
  }

  rtc::scoped_refptr<webrtc::AudioProcessorInterface> GetAudioProcessor()
      override {
    RTC_NOTREACHED();
    return nullptr;
  }
};

class PeerConnectionRtpTest : public testing::Test {
 public:
  // TODO(hbos): Use multiple threads just like the real thing.
  PeerConnectionRtpTest()
      : pc_factory_(webrtc::CreatePeerConnectionFactory(
            rtc::Thread::Current(),
            rtc::Thread::Current(),
            rtc::Thread::Current(),
            FakeAudioCaptureModule::Create(),
            nullptr,
            nullptr)) {
    webrtc::PeerConnectionInterface::RTCConfiguration config;
    auto observer1 = rtc::MakeUnique<webrtc::MockPeerConnectionObserver>();
    auto observer2 = rtc::MakeUnique<webrtc::MockPeerConnectionObserver>();
    auto pc1 = pc_factory_->CreatePeerConnection(config, nullptr, nullptr,
                                                 observer1.get());
    auto pc2 = pc_factory_->CreatePeerConnection(config, nullptr, nullptr,
                                                 observer2.get());
    pc1_.reset(new webrtc::PeerConnectionWrapper(pc_factory_, pc1,
                                                 std::move(observer1)));
    pc2_.reset(new webrtc::PeerConnectionWrapper(pc_factory_, pc2,
                                                 std::move(observer2)));
  }

  webrtc::PeerConnectionInterface* pc1() { return pc1_->pc(); }
  webrtc::PeerConnectionInterface* pc2() { return pc2_->pc(); }
  webrtc::MockPeerConnectionObserver* observer1() { return pc1_->observer(); }
  webrtc::MockPeerConnectionObserver* observer2() { return pc2_->observer(); }

  void CreateOfferAndSetLocalAndRemoteDescriptions() {
    // pc1: Create offer.
    rtc::scoped_refptr<webrtc::MockCreateSessionDescriptionObserver>
        create_offer_observer(new rtc::RefCountedObject<
                              webrtc::MockCreateSessionDescriptionObserver>());
    pc1()->CreateOffer(
        create_offer_observer,
        webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
    EXPECT_TRUE_WAIT(create_offer_observer->called(), kDefaultTimeoutMs);
    EXPECT_TRUE(create_offer_observer->result());
    std::unique_ptr<webrtc::SessionDescriptionInterface> offer =
        create_offer_observer->MoveDescription();
    std::string offer_sdp;
    offer->ToString(&offer_sdp);

    // pc1: Set local description with offer.
    rtc::scoped_refptr<webrtc::MockSetSessionDescriptionObserver>
        set_local_description_observer(
            new rtc::RefCountedObject<
                webrtc::MockSetSessionDescriptionObserver>());
    pc1()->SetLocalDescription(
        set_local_description_observer,
        webrtc::CreateSessionDescription(offer->type(), offer_sdp, nullptr));
    EXPECT_TRUE_WAIT(set_local_description_observer->called(),
                     kDefaultTimeoutMs);
    EXPECT_TRUE(set_local_description_observer->result());

    // pc2: Set remote description with offer.
    rtc::scoped_refptr<webrtc::MockSetSessionDescriptionObserver>
        set_remote_description_observer(
            new rtc::RefCountedObject<
                webrtc::MockSetSessionDescriptionObserver>());
    pc2()->SetRemoteDescription(set_remote_description_observer,
                                offer.release());
    EXPECT_TRUE_WAIT(set_remote_description_observer->called(),
                     kDefaultTimeoutMs);
    EXPECT_TRUE(set_remote_description_observer->result());
  }

 protected:
  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> pc_factory_;
  std::unique_ptr<webrtc::PeerConnectionWrapper> pc1_;
  std::unique_ptr<webrtc::PeerConnectionWrapper> pc2_;
};

TEST_F(PeerConnectionRtpTest, AddTrackWithoutStreamFiresOnAddTrack) {
  rtc::scoped_refptr<FakeAudioMediaStreamTrack> audio_track(
      new FakeAudioMediaStreamTrack("audio_track"));
  EXPECT_TRUE(pc1()->AddTrack(audio_track.get(),
                              std::vector<webrtc::MediaStreamInterface*>()));

  CreateOfferAndSetLocalAndRemoteDescriptions();

  EXPECT_EQ(1u, observer2()->add_track_events_.size());
  // TODO(deadbeef): When no stream is handled correctly we would expect
  // |add_track_events_[0].streams| to be empty. https://crbug.com/webrtc/7933
  EXPECT_EQ(1u, observer2()->add_track_events_[0].streams.size());
  EXPECT_TRUE(observer2()->add_track_events_[0].streams[0]->FindAudioTrack(
      "audio_track"));
}

TEST_F(PeerConnectionRtpTest, AddTrackWithStreamFiresOnAddTrack) {
  rtc::scoped_refptr<FakeAudioMediaStreamTrack> audio_track(
      new FakeAudioMediaStreamTrack("audio_track"));
  auto stream = webrtc::MediaStream::Create("audio_stream");
  EXPECT_TRUE(pc1()->AddTrack(
      audio_track.get(),
      std::vector<webrtc::MediaStreamInterface*>{stream.get()}));
  CreateOfferAndSetLocalAndRemoteDescriptions();

  EXPECT_EQ(1u, observer2()->add_track_events_.size());
  EXPECT_EQ(1u, observer2()->add_track_events_[0].streams.size());
  EXPECT_EQ("audio_stream",
            observer2()->add_track_events_[0].streams[0]->label());
  EXPECT_TRUE(observer2()->add_track_events_[0].streams[0]->FindAudioTrack(
      "audio_track"));
}

TEST_F(PeerConnectionRtpTest, RemoveTrackWithoutStreamFiresOnRemoveTrack) {
  rtc::scoped_refptr<FakeAudioMediaStreamTrack> audio_track(
      new FakeAudioMediaStreamTrack("audio_track"));
  auto sender = pc1()->AddTrack(audio_track.get(),
                                std::vector<webrtc::MediaStreamInterface*>());
  CreateOfferAndSetLocalAndRemoteDescriptions();
  ASSERT_EQ(1u, observer2()->add_track_events_.size());
  EXPECT_TRUE(pc1()->RemoveTrack(sender));
  CreateOfferAndSetLocalAndRemoteDescriptions();

  ASSERT_EQ(1u, observer2()->add_track_events_.size());
  EXPECT_EQ(observer2()->GetAddTrackReceivers(),
            observer2()->remove_track_events_);
}

TEST_F(PeerConnectionRtpTest, RemoveTrackWithStreamFiresOnRemoveTrack) {
  rtc::scoped_refptr<FakeAudioMediaStreamTrack> audio_track(
      new FakeAudioMediaStreamTrack("audio_track"));
  auto stream = webrtc::MediaStream::Create("audio_stream");
  auto sender =
      pc1()->AddTrack(audio_track.get(),
                      std::vector<webrtc::MediaStreamInterface*>{stream.get()});
  CreateOfferAndSetLocalAndRemoteDescriptions();
  ASSERT_EQ(1u, observer2()->add_track_events_.size());
  EXPECT_TRUE(pc1()->RemoveTrack(sender));
  CreateOfferAndSetLocalAndRemoteDescriptions();

  ASSERT_EQ(1u, observer2()->add_track_events_.size());
  EXPECT_EQ(observer2()->GetAddTrackReceivers(),
            observer2()->remove_track_events_);
}

TEST_F(PeerConnectionRtpTest, RemoveTrackWithSharedStreamFiresOnRemoveTrack) {
  rtc::scoped_refptr<FakeAudioMediaStreamTrack> audio_track1(
      new FakeAudioMediaStreamTrack("audio_track1"));
  rtc::scoped_refptr<FakeAudioMediaStreamTrack> audio_track2(
      new FakeAudioMediaStreamTrack("audio_track2"));
  auto stream = webrtc::MediaStream::Create("shared_audio_stream");
  std::vector<webrtc::MediaStreamInterface*> streams{stream.get()};
  auto sender1 = pc1()->AddTrack(audio_track1.get(), streams);
  auto sender2 = pc1()->AddTrack(audio_track2.get(), streams);
  CreateOfferAndSetLocalAndRemoteDescriptions();
  ASSERT_EQ(2u, observer2()->add_track_events_.size());

  // Remove "audio_track1".
  EXPECT_TRUE(pc1()->RemoveTrack(sender1));
  CreateOfferAndSetLocalAndRemoteDescriptions();
  ASSERT_EQ(2u, observer2()->add_track_events_.size());
  EXPECT_EQ(
      std::vector<rtc::scoped_refptr<webrtc::RtpReceiverInterface>>{
          observer2()->add_track_events_[0].receiver},
      observer2()->remove_track_events_);

  // Remove "audio_track2".
  EXPECT_TRUE(pc1()->RemoveTrack(sender2));
  CreateOfferAndSetLocalAndRemoteDescriptions();
  ASSERT_EQ(2u, observer2()->add_track_events_.size());
  EXPECT_EQ(observer2()->GetAddTrackReceivers(),
            observer2()->remove_track_events_);
}

}  // namespace
