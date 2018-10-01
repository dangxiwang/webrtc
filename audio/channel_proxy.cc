/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "audio/channel_proxy.h"

#include <utility>

#include "api/call/audio_sink.h"
#include "call/rtp_transport_controller_send_interface.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/safe_minmax.h"

namespace webrtc {
namespace voe {
ChannelProxy::ChannelProxy() {}

ChannelProxy::ChannelProxy(std::unique_ptr<Channel> channel)
    : channel_(std::move(channel)) {
  RTC_DCHECK(channel_);
  module_process_thread_checker_.DetachFromThread();
}

ChannelProxy::~ChannelProxy() {}

bool ChannelProxy::SetEncoder(int payload_type,
                              std::unique_ptr<AudioEncoder> encoder) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  return channel_->SetEncoder(payload_type, std::move(encoder));
}

void ChannelProxy::ModifyEncoder(
    rtc::FunctionView<void(std::unique_ptr<AudioEncoder>*)> modifier) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->ModifyEncoder(modifier);
}

void ChannelProxy::SetRTCPStatus(bool enable) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->SetRTCPStatus(enable);
}

void ChannelProxy::SetLocalSSRC(uint32_t ssrc) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  int error = channel_->SetLocalSSRC(ssrc);
  RTC_DCHECK_EQ(0, error);
}

void ChannelProxy::SetMid(const std::string& mid, int extension_id) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->SetMid(mid, extension_id);
}

void ChannelProxy::SetRTCP_CNAME(const std::string& c_name) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  // Note: VoERTP_RTCP::SetRTCP_CNAME() accepts a char[256] array.
  std::string c_name_limited = c_name.substr(0, 255);
  int error = channel_->SetRTCP_CNAME(c_name_limited.c_str());
  RTC_DCHECK_EQ(0, error);
}

void ChannelProxy::SetNACKStatus(bool enable, int max_packets) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->SetNACKStatus(enable, max_packets);
}

void ChannelProxy::SetSendAudioLevelIndicationStatus(bool enable, int id) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  int error = channel_->SetSendAudioLevelIndicationStatus(enable, id);
  RTC_DCHECK_EQ(0, error);
}

void ChannelProxy::EnableSendTransportSequenceNumber(int id) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->EnableSendTransportSequenceNumber(id);
}

void ChannelProxy::RegisterSenderCongestionControlObjects(
    RtpTransportControllerSendInterface* transport,
    RtcpBandwidthObserver* bandwidth_observer) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->RegisterSenderCongestionControlObjects(transport,
                                                   bandwidth_observer);
}

void ChannelProxy::ResetSenderCongestionControlObjects() {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->ResetSenderCongestionControlObjects();
}

CallStatistics ChannelProxy::GetRTCPStatistics() const {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  CallStatistics stats = {0};
  int error = channel_->GetRTPStatistics(stats);
  RTC_DCHECK_EQ(0, error);
  return stats;
}

std::vector<ReportBlock> ChannelProxy::GetRemoteRTCPReportBlocks() const {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  std::vector<webrtc::ReportBlock> blocks;
  int error = channel_->GetRemoteRTCPReportBlocks(&blocks);
  RTC_DCHECK_EQ(0, error);
  return blocks;
}

NetworkStatistics ChannelProxy::GetNetworkStatistics() const {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  NetworkStatistics stats = {0};
  int error = channel_->GetNetworkStatistics(stats);
  RTC_DCHECK_EQ(0, error);
  return stats;
}

AudioDecodingCallStats ChannelProxy::GetDecodingCallStatistics() const {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  AudioDecodingCallStats stats;
  channel_->GetDecodingCallStatistics(&stats);
  return stats;
}

ANAStats ChannelProxy::GetANAStatistics() const {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  return channel_->GetANAStatistics();
}

uint32_t ChannelProxy::GetDelayEstimate() const {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread() ||
             module_process_thread_checker_.CalledOnValidThread());
  return channel_->GetDelayEstimate();
}

bool ChannelProxy::SetSendTelephoneEventPayloadType(int payload_type,
                                                    int payload_frequency) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  return channel_->SetSendTelephoneEventPayloadType(payload_type,
                                                    payload_frequency) == 0;
}

bool ChannelProxy::SendTelephoneEventOutband(int event, int duration_ms) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  return channel_->SendTelephoneEventOutband(event, duration_ms) == 0;
}

void ChannelProxy::SetBitrate(int bitrate_bps, int64_t probing_interval_ms) {
  // This method can be called on the worker thread, module process thread
  // or on a TaskQueue via VideoSendStreamImpl::OnEncoderConfigurationChanged.
  // TODO(solenberg): Figure out a good way to check this or enforce calling
  // rules.
  // RTC_DCHECK(worker_thread_checker_.CalledOnValidThread() ||
  //            module_process_thread_checker_.CalledOnValidThread());
  channel_->SetBitRate(bitrate_bps, probing_interval_ms);
}

void ChannelProxy::SetInputMute(bool muted) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->SetInputMute(muted);
}

void ChannelProxy::RegisterTransport(Transport* transport) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->RegisterTransport(transport);
}

bool ChannelProxy::ReceivedRTCPPacket(const uint8_t* packet, size_t length) {
  // May be called on either worker thread or network thread.
  return channel_->ReceivedRTCPPacket(packet, length) == 0;
}

int ChannelProxy::PreferredSampleRate() const {
  RTC_DCHECK_RUNS_SERIALIZED(&audio_thread_race_checker_);
  return channel_->PreferredSampleRate();
}

void ChannelProxy::ProcessAndEncodeAudio(
    std::unique_ptr<AudioFrame> audio_frame) {
  RTC_DCHECK_RUNS_SERIALIZED(&audio_thread_race_checker_);
  return channel_->ProcessAndEncodeAudio(std::move(audio_frame));
}

void ChannelProxy::SetTransportOverhead(int transport_overhead_per_packet) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->SetTransportOverhead(transport_overhead_per_packet);
}

RtpRtcp* ChannelProxy::GetRtpRtcp() const {
  RTC_DCHECK(module_process_thread_checker_.CalledOnValidThread());
  return channel_->GetRtpRtcp();
}

int64_t ChannelProxy::GetRTT() const {
  RTC_DCHECK(module_process_thread_checker_.CalledOnValidThread());
  return channel_->GetRTT();
}

uint32_t ChannelProxy::GetPlayoutTimestamp() const {
  RTC_DCHECK_RUNS_SERIALIZED(&video_capture_thread_race_checker_);
  unsigned int timestamp = 0;
  int error = channel_->GetPlayoutTimestamp(timestamp);
  RTC_DCHECK(!error || timestamp == 0);
  return timestamp;
}

void ChannelProxy::SetMinimumPlayoutDelay(int delay_ms) {
  RTC_DCHECK(module_process_thread_checker_.CalledOnValidThread());
  // Limit to range accepted by both VoE and ACM, so we're at least getting as
  // close as possible, instead of failing.
  delay_ms = rtc::SafeClamp(delay_ms, 0, 10000);
  int error = channel_->SetMinimumPlayoutDelay(delay_ms);
  if (0 != error) {
    RTC_LOG(LS_WARNING) << "Error setting minimum playout delay.";
  }
}

void ChannelProxy::OnTwccBasedUplinkPacketLossRate(float packet_loss_rate) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->OnTwccBasedUplinkPacketLossRate(packet_loss_rate);
}

void ChannelProxy::OnRecoverableUplinkPacketLossRate(
    float recoverable_packet_loss_rate) {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->OnRecoverableUplinkPacketLossRate(recoverable_packet_loss_rate);
}

void ChannelProxy::StartSend() {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  int error = channel_->StartSend();
  RTC_DCHECK_EQ(0, error);
}

void ChannelProxy::StopSend() {
  RTC_DCHECK(worker_thread_checker_.CalledOnValidThread());
  channel_->StopSend();
}

}  // namespace voe
}  // namespace webrtc
