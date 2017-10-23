/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef PC_PEERCONNECTIONWRAPPER_H_
#define PC_PEERCONNECTIONWRAPPER_H_

#include <functional>
#include <memory>
#include <string>

#include "api/peerconnectioninterface.h"
#include "pc/test/mockpeerconnectionobservers.h"

namespace webrtc {

// Class that wraps a PeerConnection so that it is easier to use in unit tests.
// Namely, gives a synchronous API for the event-callback-based API of
// PeerConnection and provides an observer object that stores information from
// PeerConnectionObserver callbacks.
//
// This is intended to be subclassed if additional information needs to be
// stored with the PeerConnection (e.g., fake PeerConnection parameters so that
// tests can be written against those interactions). The base
// PeerConnectionWrapper should only have helper methods that are broadly
// useful. More specific helper methods should be created in the test-specific
// subclass.
//
// The wrapper is intended to be constructed by specialized factory methods on
// a test fixture class then used as a local variable in each test case.
class PeerConnectionWrapper {
 public:
  // Constructs a PeerConnectionWrapper from the given PeerConnection.
  // The given PeerConnectionFactory should be the factory that created the
  // PeerConnection and the MockPeerConnectionObserver should be the observer
  // that is watching the PeerConnection.
  PeerConnectionWrapper(
      rtc::scoped_refptr<PeerConnectionFactoryInterface> pc_factory,
      rtc::scoped_refptr<PeerConnectionInterface> pc,
      std::unique_ptr<MockPeerConnectionObserver> observer);
  virtual ~PeerConnectionWrapper();

  PeerConnectionFactoryInterface* pc_factory();
  PeerConnectionInterface* pc();
  MockPeerConnectionObserver* observer();

  // Calls the underlying PeerConnection's CreateOffer method and returns the
  // resulting SessionDescription once it is available. If the method call
  // failed, null is returned.
  std::unique_ptr<SessionDescriptionInterface> CreateOffer(
      const PeerConnectionInterface::RTCOfferAnswerOptions& options,
      std::string* error_out = nullptr);
  // Calls CreateOffer with default options.
  std::unique_ptr<SessionDescriptionInterface> CreateOffer();
  // Calls CreateOffer and sets a copy of the offer as the local description.
  std::unique_ptr<SessionDescriptionInterface> CreateOfferAndSetAsLocal(
      const PeerConnectionInterface::RTCOfferAnswerOptions& options);
  // Calls CreateOfferAndSetAsLocal with default options.
  std::unique_ptr<SessionDescriptionInterface> CreateOfferAndSetAsLocal();

  // Calls the underlying PeerConnection's CreateAnswer method and returns the
  // resulting SessionDescription once it is available. If the method call
  // failed, null is returned.
  std::unique_ptr<SessionDescriptionInterface> CreateAnswer(
      const PeerConnectionInterface::RTCOfferAnswerOptions& options,
      std::string* error_out = nullptr);
  // Calls CreateAnswer with the default options.
  std::unique_ptr<SessionDescriptionInterface> CreateAnswer();
  // Calls CreateAnswer and sets a copy of the offer as the local description.
  std::unique_ptr<SessionDescriptionInterface> CreateAnswerAndSetAsLocal(
      const PeerConnectionInterface::RTCOfferAnswerOptions& options);
  // Calls CreateAnswerAndSetAsLocal with default options.
  std::unique_ptr<SessionDescriptionInterface> CreateAnswerAndSetAsLocal();

  // Calls the underlying PeerConnection's SetLocalDescription method with the
  // given session description and waits for the success/failure response.
  // Returns true if the description was successfully set.
  bool SetLocalDescription(std::unique_ptr<SessionDescriptionInterface> desc,
                           std::string* error_out = nullptr);
  // Calls the underlying PeerConnection's SetRemoteDescription method with the
  // given session description and waits for the success/failure response.
  // Returns true if the description was successfully set.
  bool SetRemoteDescription(std::unique_ptr<SessionDescriptionInterface> desc,
                            std::string* error_out = nullptr);

  // Calls the underlying PeerConnection's AddTrack method with an audio media
  // stream track not bound to any source.
  rtc::scoped_refptr<RtpSenderInterface> AddAudioTrack(
      const std::string& track_label,
      std::vector<MediaStreamInterface*> streams = {});

  // Calls the underlying PeerConnection's AddTrack method with a video media
  // stream track fed by a fake video capturer.
  rtc::scoped_refptr<RtpSenderInterface> AddVideoTrack(
      const std::string& track_label,
      std::vector<MediaStreamInterface*> streams = {});

  // Returns the signaling state of the underlying PeerConnection.
  PeerConnectionInterface::SignalingState signaling_state();

  // Returns true if ICE has finished gathering candidates.
  bool IsIceGatheringDone();

  // Returns true if ICE has established a connection.
  bool IsIceConnected();

  // Calls GetStats() on the underlying PeerConnection and returns the resulting
  // report. If GetStats() fails, this method returns null and fails the test.
  rtc::scoped_refptr<const RTCStatsReport> GetStats();

 private:
  std::unique_ptr<SessionDescriptionInterface> CreateSdp(
      std::function<void(CreateSessionDescriptionObserver*)> fn,
      std::string* error_out);
  bool SetSdp(std::function<void(SetSessionDescriptionObserver*)> fn,
              std::string* error_out);

  rtc::scoped_refptr<PeerConnectionFactoryInterface> pc_factory_;
  std::unique_ptr<MockPeerConnectionObserver> observer_;
  rtc::scoped_refptr<PeerConnectionInterface> pc_;
};

}  // namespace webrtc

#endif  // PC_PEERCONNECTIONWRAPPER_H_
