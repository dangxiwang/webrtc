/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_CREATE_MODULAR_PEERCONNECTION_FACTORY_H_
#define API_CREATE_MODULAR_PEERCONNECTION_FACTORY_H_

#include <memory>

#include "api/call/callfactoryinterface.h"
#include "api/fec_controller.h"
#include "api/peerconnectioninterface.h"
#include "api/scoped_refptr.h"
#include "api/transport/network_control.h"

// This is a lower-level version of the CreatePeerConnectionFactory functions.
//
// If an application knows it will only require certain modules, it can reduce
// webrtc's impact on its binary size by  using
// CreateModularPeerConnectionFactory instead of one of the
// CreatePeerConnectionFactory methods.
//
// For example, if an application only uses WebRTC for audio, it can pass in
// null pointers for the video-specific interfaces, and omit the corresponding
// modules from its build.
//
// If |network_thread| or |worker_thread| are null, the PeerConnectionFactory
// will create the necessary thread internally. If |signaling_thread| is null,
// the PeerConnectionFactory will use the thread on which this method is called
// as the signaling thread, wrapping it in an rtc::Thread object if needed.
//
// If non-null, a reference is added to |default_adm|, and ownership of
// |video_encoder_factory| and |video_decoder_factory| is transferred to the
// returned factory.
//
// If |audio_mixer| is null, an internal audio mixer will be created and used.

namespace rtc {
class Thread;
}  // namespace rtc

namespace cricket {
class MediaEngineInterface;
}  // namespace cricket

namespace webrtc {

class RtcEventLogFactoryInterface;

// TODO(bugs.webrtc.org/10044): Remove this overload.
rtc::scoped_refptr<PeerConnectionFactoryInterface>
CreateModularPeerConnectionFactory(
    rtc::Thread* network_thread,
    rtc::Thread* worker_thread,
    rtc::Thread* signaling_thread,
    std::unique_ptr<cricket::MediaEngineInterface> media_engine,
    std::unique_ptr<CallFactoryInterface> call_factory,
    std::unique_ptr<RtcEventLogFactoryInterface> event_log_factory);

// TODO(bugs.webrtc.org/10044): Remove this overload.
rtc::scoped_refptr<PeerConnectionFactoryInterface>
CreateModularPeerConnectionFactory(
    rtc::Thread* network_thread,
    rtc::Thread* worker_thread,
    rtc::Thread* signaling_thread,
    std::unique_ptr<cricket::MediaEngineInterface> media_engine,
    std::unique_ptr<CallFactoryInterface> call_factory,
    std::unique_ptr<RtcEventLogFactoryInterface> event_log_factory,
    std::unique_ptr<FecControllerFactoryInterface> fec_controller_factory,
    std::unique_ptr<NetworkControllerFactoryInterface>
        network_controller_factory = nullptr);

rtc::scoped_refptr<PeerConnectionFactoryInterface>
CreateModularPeerConnectionFactory(
    PeerConnectionFactoryDependencies dependencies);

}  // namespace webrtc

#endif  // API_CREATE_MODULAR_PEERCONNECTION_FACTORY_H_
