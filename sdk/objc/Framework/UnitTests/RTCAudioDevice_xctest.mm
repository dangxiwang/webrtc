/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import <XCTest/XCTest.h>
#import "sdk/objc/Framework/Native/src/audio/audio_device_ios.h"
#import "sdk/objc/Framework/Classes/Audio/RTCAudioSession+Private.h"

@interface RTCAudioDevice: XCTestCase
@end

@implementation RTCAudioDevice

// Verifies that the AudioDeviceIOS is_interrupted_ flag is reset correctly
// after an iOS AVAudioSessionInterruptionTypeEnded notification event.
// AudioDeviceIOS listens to RTCAudioSession interrupted notifications by:
// - In AudioDeviceIOS.InitPlayOrRecord registers its audio_session_observer_
//   callback with RTCAudioSession's delegate list.
// - When RTCAudioSession receives an iOS audio interrupted notification, it
//   passes the notification to callbacks in its delegate list which sets
//   AudioDeviceIOS's is_interrupted_ flag to true.
// - When AudioDeviceIOS.ShutdownPlayOrRecord is called, its
//   audio_session_observer_ callback is removed from RTCAudioSessions's
//   delegate list.
//   So if RTCAudioSession receives an iOS end audio interruption notification,
//   AudioDeviceIOS is not notified as its callback is not in RTCAudioSession's
//   delegate list. This causes AudioDeviceIOS's is_interrupted_ flag to be in
//   the wrong (true) state and the audio session will ignore audio changes.
// As RTCAudioSession keeps its own interrupted state, the fix is to initialize
// AudioDeviceIOS's is_interrupted_ flag to RTCAudioSession's isInterrupted
// flag in AudioDeviceIOS.InitPlayOrRecord.
- (void)testInterruptedAudioSession {
  RTCAudioSession *session = [RTCAudioSession sharedInstance];
  std::unique_ptr<webrtc::ios_adm::AudioDeviceIOS> audio_device;
  audio_device.reset(new webrtc::ios_adm::AudioDeviceIOS());
  std::unique_ptr<webrtc::AudioDeviceBuffer> audio_buffer;
  audio_buffer.reset(new webrtc::AudioDeviceBuffer());
  audio_device->AttachAudioBuffer(audio_buffer.get());
  audio_device->Init();
  audio_device->InitPlayout();
  audio_device->StartPlayout();

  // Force interruption.
  [session notifyDidBeginInterruption];

  // Wait for notification to propagate.
  rtc::MessageQueueManager::ProcessAllMessageQueues();
  XCTAssertTrue(audio_device->IsInterrupted());

  // Force it for testing.
  audio_device->StopPlayout();

  [session notifyDidEndInterruptionWithShouldResumeSession:YES];
  // Wait for notification to propagate.
  rtc::MessageQueueManager::ProcessAllMessageQueues();
  XCTAssertTrue(audio_device->IsInterrupted());

  audio_device->Init();
  audio_device->InitPlayout();
  XCTAssertFalse(audio_device->IsInterrupted());
}

@end
