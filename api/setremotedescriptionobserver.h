/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_SETREMOTEDESCRIPTIONOBSERVER_H_
#define API_SETREMOTEDESCRIPTIONOBSERVER_H_

#include <utility>
#include <vector>

#include "api/failurereason.h"
#include "api/jsep.h"
#include "api/mediastreaminterface.h"
#include "api/rtpreceiverinterface.h"
#include "rtc_base/bind.h"
#include "rtc_base/messagehandler.h"
#include "rtc_base/refcount.h"
#include "rtc_base/refcountedobject.h"
#include "rtc_base/scoped_ref_ptr.h"
#include "rtc_base/thread.h"

namespace webrtc {

struct ReceiverWithStreams {
  ReceiverWithStreams(
      rtc::scoped_refptr<RtpReceiverInterface> receiver,
      std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams)
      : receiver(std::move(receiver)),
        streams(std::move(streams)) {}

  rtc::scoped_refptr<RtpReceiverInterface> receiver;
  std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams;
};

class SetRemoteDescriptionObserver : public rtc::RefCountInterface {
 public:
  // Describes what changes occurred as a result of processing the remote
  // description of the SetRemoteDescription call.
  struct StateChanges {
    // Receivers that were created for new remote tracks.
    // https://w3c.github.io/webrtc-pc/#process-remote-track-addition
    std::vector<ReceiverWithStreams> receivers_added;
    // Receivers that were completely removed due to the removal of remote
    // tracks. This is current (Plan B SDP) behavior. When Unified Plan SDP is
    // supported, transceivers can change direction (and receivers stopped) but
    // receivers are never removed.
    // https://w3c.github.io/webrtc-pc/#process-remote-track-removal
    // TODO(hbos,deadbeef): When Unified Plan SDP is supported, deprecate and
    // remove this.
    std::vector<rtc::scoped_refptr<RtpReceiverInterface>> receivers_removed;

    // TODO(hbos): Add a list of receivers that had their associated set of
    // streams updated. https://crbug.com/webrtc/8315
    // This may be blocked on supporting multiple streams per sender or else
    // this may count as the removal and addition of a track.
    // https://crbug.com/webrtc/7932
  };

  virtual void OnSuccess(StateChanges state_changes) = 0;
  virtual void OnFailure(FailureReason reason) = 0;
};

// Upon success/failure, posts a task to execute the callback of the
// SetSessionDescriptionObserver asynchronously on the same thread.
class SetRemoteDescriptionSessionObserverWrapper
    : public rtc::RefCountedObject<SetRemoteDescriptionObserver>,
      public rtc::MessageHandler {
 public:
  SetRemoteDescriptionSessionObserverWrapper(
      rtc::scoped_refptr<SetSessionDescriptionObserver> set_desc_observer);

  void OnSuccess(StateChanges state_changes) override;
  void OnFailure(FailureReason reason) override;

  void OnMessage(rtc::Message* msg) override;

 private:
  class MessageData;

  rtc::scoped_refptr<SetSessionDescriptionObserver> set_desc_observer_;
};

}  // namespace webrtc

#endif  // API_SETREMOTEDESCRIPTIONOBSERVER_H_
