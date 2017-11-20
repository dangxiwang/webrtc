/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/setremotedescriptionobserver.h"

#include <string>

namespace webrtc {

// The message keeps the observer alive through reference counting.
class SetRemoteDescriptionSessionObserverWrapper::MessageData
    : public rtc::MessageData {
 public:
  static MessageData* CreateOnSuccess(
      rtc::scoped_refptr<SetRemoteDescriptionSessionObserverWrapper> observer) {
    return new MessageData(std::move(observer), rtc::Optional<std::string>());
  }

  static MessageData* CreateOnFailure(
      rtc::scoped_refptr<SetRemoteDescriptionSessionObserverWrapper> observer,
      std::string failure_message) {
    return new MessageData(
        std::move(observer),
        rtc::Optional<std::string>(std::move(failure_message)));
  }

  bool OnSuccess() const { return !failure_message_; }

  std::string failure_message() const { return *failure_message_; }

 private:
  MessageData(
      rtc::scoped_refptr<SetRemoteDescriptionSessionObserverWrapper> observer,
      rtc::Optional<std::string> failure_message)
      : observer_(std::move(observer)),
        failure_message_(std::move(failure_message)) {}

  rtc::scoped_refptr<SetRemoteDescriptionSessionObserverWrapper> observer_;
  rtc::Optional<std::string> failure_message_;
};

SetRemoteDescriptionSessionObserverWrapper::
    SetRemoteDescriptionSessionObserverWrapper(
        rtc::scoped_refptr<SetSessionDescriptionObserver> set_desc_observer)
    : set_desc_observer_(std::move(set_desc_observer)) {}

void SetRemoteDescriptionSessionObserverWrapper::OnSuccess() {
  // MessageData keeps a reference to |this|, ensuring it is not deleted until
  // OnMessage().
  rtc::Thread::Current()->Post(RTC_FROM_HERE, this, 0,
                               MessageData::CreateOnSuccess(this));
}

void SetRemoteDescriptionSessionObserverWrapper::OnFailure(
    FailureReason reason) {
  // MessageData keeps a reference to |this|, ensuring it is not deleted until
  // OnMessage().
  rtc::Thread::Current()->Post(
      RTC_FROM_HERE, this, 0,
      MessageData::CreateOnFailure(this,
                                   reason.message.value_or(std::string())));
}

void SetRemoteDescriptionSessionObserverWrapper::OnMessage(rtc::Message* msg) {
  MessageData* message = static_cast<MessageData*>(msg->pdata);
  if (message->OnSuccess())
    set_desc_observer_->OnSuccess();
  else
    set_desc_observer_->OnFailure(message->failure_message());
  // Delete the message data, this may delete |this|. Don't use member variables
  // after this line.
  delete message;
}

}  // namespace webrtc
