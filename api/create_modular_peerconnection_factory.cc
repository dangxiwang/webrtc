/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/create_modular_peerconnection_factory.h"

#include <memory>
#include <utility>

#include "api/peerconnectionfactoryproxy.h"
#include "logging/rtc_event_log/rtc_event_log_factory_interface.h"
#include "media/base/mediaengine.h"
#include "pc/peerconnectionfactory.h"
#include "rtc_base/refcountedobject.h"
#include "rtc_base/thread.h"

namespace webrtc {

rtc::scoped_refptr<PeerConnectionFactoryInterface>
CreateModularPeerConnectionFactory(
    rtc::Thread* network_thread,
    rtc::Thread* worker_thread,
    rtc::Thread* signaling_thread,
    std::unique_ptr<cricket::MediaEngineInterface> media_engine,
    std::unique_ptr<CallFactoryInterface> call_factory,
    std::unique_ptr<RtcEventLogFactoryInterface> event_log_factory) {
  PeerConnectionFactoryDependencies dependencies;
  dependencies.network_thread = network_thread;
  dependencies.worker_thread = worker_thread;
  dependencies.signaling_thread = signaling_thread;
  dependencies.media_engine = std::move(media_engine);
  dependencies.call_factory = std::move(call_factory);
  dependencies.event_log_factory = std::move(event_log_factory);
  return CreateModularPeerConnectionFactory(std::move(dependencies));
}

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
        network_controller_factory) {
  PeerConnectionFactoryDependencies dependencies;
  dependencies.network_thread = network_thread;
  dependencies.worker_thread = worker_thread;
  dependencies.signaling_thread = signaling_thread;
  dependencies.media_engine = std::move(media_engine);
  dependencies.call_factory = std::move(call_factory);
  dependencies.event_log_factory = std::move(event_log_factory);
  dependencies.fec_controller_factory = std::move(fec_controller_factory);
  dependencies.network_controller_factory =
      std::move(network_controller_factory);
  return CreateModularPeerConnectionFactory(std::move(dependencies));
}

rtc::scoped_refptr<PeerConnectionFactoryInterface>
CreateModularPeerConnectionFactory(
    PeerConnectionFactoryDependencies dependencies) {
  rtc::scoped_refptr<PeerConnectionFactory> pc_factory(
      new rtc::RefCountedObject<PeerConnectionFactory>(
          std::move(dependencies)));
  // Call Initialize synchronously but make sure it is executed on
  // |signaling_thread|.
  MethodCall0<PeerConnectionFactory, bool> call(
      pc_factory.get(), &PeerConnectionFactory::Initialize);
  bool result = call.Marshal(RTC_FROM_HERE, pc_factory->signaling_thread());

  if (!result) {
    return nullptr;
  }
  return PeerConnectionFactoryProxy::Create(pc_factory->signaling_thread(),
                                            pc_factory);
}

}  // namespace webrtc
