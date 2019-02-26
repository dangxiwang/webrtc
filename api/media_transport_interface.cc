/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// This is EXPERIMENTAL interface for media transport.
//
// The goal is to refactor WebRTC code so that audio and video frames
// are sent / received through the media transport interface. This will
// enable different media transport implementations, including QUIC-based
// media transport.

#include "api/media_transport_interface.h"

#include <cstdint>
#include <utility>

namespace webrtc {

MediaTransportSettings::MediaTransportSettings() = default;
MediaTransportSettings::MediaTransportSettings(const MediaTransportSettings&) =
    default;
MediaTransportSettings& MediaTransportSettings::operator=(
    const MediaTransportSettings&) = default;
MediaTransportSettings::~MediaTransportSettings() = default;


SendDataParams::SendDataParams() = default;
SendDataParams::SendDataParams(const SendDataParams&) = default;

RTCErrorOr<std::unique_ptr<MediaTransportInterface>>
MediaTransportFactory::CreateMediaTransport(
    rtc::PacketTransportInternal* packet_transport,
    rtc::Thread* network_thread,
    const MediaTransportSettings& settings) {
  return std::unique_ptr<MediaTransportInterface>(nullptr);
}

RTCErrorOr<std::unique_ptr<MediaTransportInterface>>
MediaTransportFactory::CreateMediaTransport(
    rtc::Thread* network_thread,
    const MediaTransportSettings& settings) {
  return std::unique_ptr<MediaTransportInterface>(nullptr);
}

std::string MediaTransportFactory::GetTransportName() const {
  return "";
}

MediaTransportInterface::MediaTransportInterface() = default;
MediaTransportInterface::~MediaTransportInterface() = default;

absl::optional<std::string>
MediaTransportInterface::GetTransportParametersOffer() const {
  return absl::nullopt;
}

void MediaTransportInterface::Connect(
    rtc::PacketTransportInternal* packet_transport) {}

void MediaTransportInterface::SetKeyFrameRequestCallback(
    MediaTransportKeyFrameRequestCallback* callback) {}

absl::optional<TargetTransferRate>
MediaTransportInterface::GetLatestTargetTransferRate() {
  return absl::nullopt;
}

void MediaTransportInterface::AddNetworkChangeCallback(
    MediaTransportNetworkChangeCallback* callback) {}

void MediaTransportInterface::RemoveNetworkChangeCallback(
    MediaTransportNetworkChangeCallback* callback) {}

void MediaTransportInterface::SetFirstAudioPacketReceivedObserver(
    AudioPacketReceivedObserver* observer) {}

void MediaTransportInterface::AddTargetTransferRateObserver(
    TargetTransferRateObserver* observer) {}
void MediaTransportInterface::RemoveTargetTransferRateObserver(
    TargetTransferRateObserver* observer) {}

void MediaTransportInterface::AddRttObserver(
    MediaTransportRttObserver* observer) {}
void MediaTransportInterface::RemoveRttObserver(
    MediaTransportRttObserver* observer) {}

size_t MediaTransportInterface::GetAudioPacketOverhead() const {
  return 0;
}

void MediaTransportInterface::SetAllocatedBitrateLimits(
    const MediaTransportAllocatedBitrateLimits& limits) {}

// TODO(mellem):  Delete when all implementations support it.
RTCError MediaTransportInterface::OpenChannel(int channel_id) {
  // NB: This must return OK to avoid breaking existing implementations, which
  // do not require calling OpenChannel.
  return RTCError::OK();
}

}  // namespace webrtc
