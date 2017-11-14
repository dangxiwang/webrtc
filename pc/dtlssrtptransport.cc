/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "pc/dtlssrtptransport.h"

#include <memory>
#include <string>
#include <utility>

#include "media/base/rtputils.h"
#include "rtc_base/sslstreamadapter.h"

namespace {
// Value specified in RFC 5764.
static const char kDtlsSrtpExporterLabel[] = "EXTRACTOR-dtls_srtp";
}  // namespace

namespace webrtc {

DtlsSrtpTransport::DtlsSrtpTransport(
    std::unique_ptr<webrtc::SrtpTransport> srtp_transport)
    : RtpTransportInternalAdapter(srtp_transport.get()) {
  srtp_transport_ = std::move(srtp_transport);
  RTC_DCHECK(srtp_transport_);
  srtp_transport_->SignalPacketReceived.connect(
      this, &DtlsSrtpTransport::OnPacketReceived);
  srtp_transport_->SignalReadyToSend.connect(this,
                                             &DtlsSrtpTransport::OnReadyToSend);
}

void DtlsSrtpTransport::SetDtlsTransports(
    cricket::DtlsTransportInternal* rtp_dtls_transport,
    cricket::DtlsTransportInternal* rtcp_dtls_transport) {
  // Transport names should be the same.
  if (rtp_dtls_transport && rtcp_dtls_transport) {
    RTC_DCHECK(rtp_dtls_transport->transport_name() ==
               rtcp_dtls_transport->transport_name());
  }

  // When using DTLS-SRTP, we must reset the SrtpTransport every time the
  // DtlsTransport changes and wait until the DTLS handshake is complete to set
  // the newly negotiated parameters.
  if (IsActive()) {
    srtp_transport_->ResetParams();
  }

  if (rtcp_dtls_transport) {
    // This would only be possible if using BUNDLE but not rtcp-mux, which isn't
    // allowed according to the BUNDLE spec.
    RTC_CHECK(!(IsActive()))
        << "Setting RTCP for DTLS/SRTP after the DTLS is active "
        << "should never happen.";

    RTC_LOG(LS_INFO) << "Setting RTCP Transport on "
                     << rtcp_dtls_transport->transport_name() << " transport "
                     << rtcp_dtls_transport;
    SetRtcpDtlsTransport(rtcp_dtls_transport);
    SetRtcpPacketTransport(rtcp_dtls_transport);
  }

  RTC_LOG(LS_INFO) << "Setting RTP Transport on "
                   << rtp_dtls_transport->transport_name() << " transport "
                   << rtp_dtls_transport;
  SetRtpDtlsTransport(rtp_dtls_transport);
  SetRtpPacketTransport(rtp_dtls_transport);

  UpdateWritableStateAndMaybeSetupDtlsSrtp();
}

void DtlsSrtpTransport::SetRtcpMuxEnabled(bool enable) {
  srtp_transport_->SetRtcpMuxEnabled(enable);
  if (enable) {
    MaybeSetupDtlsSrtp();
  }
}

void DtlsSrtpTransport::SetSendEncryptedHeaderExtensionIds(
    const std::vector<int>& send_extension_ids) {
  send_extension_ids_.emplace(send_extension_ids);
  // Reset the crypto parameters to update the send_extension IDs.
  SetupRtpDtlsSrtp();
}

void DtlsSrtpTransport::SetRecvEncryptedHeaderExtensionIds(
    const std::vector<int>& recv_extension_ids) {
  recv_extension_ids_.emplace(recv_extension_ids);
  // Reset the crypto parameters to update the send_extension IDs.
  SetupRtpDtlsSrtp();
}

bool DtlsSrtpTransport::IsDtlsActive() {
  auto rtcp_dtls_transport =
      rtcp_mux_enabled() ? nullptr : rtcp_dtls_transport_;
  return (rtp_dtls_transport_ && rtp_dtls_transport_->IsDtlsActive() &&
          (!rtcp_dtls_transport || rtcp_dtls_transport->IsDtlsActive()));
}

bool DtlsSrtpTransport::IsDtlsConnected() {
  auto rtcp_dtls_transport =
      rtcp_mux_enabled() ? nullptr : rtcp_dtls_transport_;
  return (rtp_dtls_transport_ &&
          rtp_dtls_transport_->dtls_state() ==
              cricket::DTLS_TRANSPORT_CONNECTED &&
          (!rtcp_dtls_transport || rtcp_dtls_transport->dtls_state() ==
                                       cricket::DTLS_TRANSPORT_CONNECTED));
}

bool DtlsSrtpTransport::IsDtlsWritable() {
  auto rtp_packet_transport = srtp_transport_->rtp_packet_transport();
  auto rtcp_packet_transport =
      rtcp_mux_enabled() ? nullptr : srtp_transport_->rtcp_packet_transport();
  return rtp_packet_transport && rtp_packet_transport->writable() &&
         (!rtcp_packet_transport || rtcp_packet_transport->writable());
}

bool DtlsSrtpTransport::DtlsHandshakeCompleted() {
  return IsDtlsActive() && IsDtlsConnected();
}

void DtlsSrtpTransport::MaybeSetupDtlsSrtp() {
  if (IsActive() || !DtlsHandshakeCompleted()) {
    return;
  }

  SetupRtpDtlsSrtp();

  if (!rtcp_mux_enabled() && rtcp_dtls_transport_) {
    SetupRtcpDtlsSrtp();
  }
}

void DtlsSrtpTransport::SetupRtpDtlsSrtp() {
  // Use an empty encrypted header extension ID vector if not set. This could
  // happen when the DTLS handshake is completed before processing the
  // Offer/Answer which contains the encrypted header extension IDs.
  std::vector<int> send_extension_ids;
  std::vector<int> recv_extension_ids;
  if (send_extension_ids_) {
    send_extension_ids = *send_extension_ids_;
  }
  if (recv_extension_ids_) {
    recv_extension_ids = *recv_extension_ids_;
  }

  int selected_crypto_suite;
  std::vector<unsigned char> send_key;
  std::vector<unsigned char> recv_key;

  if (!ExtractParams(rtp_dtls_transport_, &selected_crypto_suite, &send_key,
                     &recv_key) ||
      !srtp_transport_->SetRtpParams(
          selected_crypto_suite, &send_key[0],
          static_cast<int>(send_key.size()), send_extension_ids,
          selected_crypto_suite, &recv_key[0],
          static_cast<int>(recv_key.size()), recv_extension_ids)) {
    SignalDtlsSrtpSetupFailure(this, /*rtcp=*/false);
    RTC_LOG(LS_WARNING) << "DTLS-SRTP key installation for RTP failed";
  }
}

void DtlsSrtpTransport::SetupRtcpDtlsSrtp() {
  // Return if the DTLS-SRTP is active because the encrypted header extension
  // IDs don't need to be updated for RTCP and the crypto params don't need to
  // be reset.
  if (IsActive()) {
    return;
  }

  std::vector<int> send_extension_ids;
  std::vector<int> recv_extension_ids;
  if (send_extension_ids_) {
    send_extension_ids = *send_extension_ids_;
  }
  if (recv_extension_ids_) {
    recv_extension_ids = *recv_extension_ids_;
  }

  int selected_crypto_suite;
  std::vector<unsigned char> rtcp_send_key;
  std::vector<unsigned char> rtcp_recv_key;
  if (!ExtractParams(rtcp_dtls_transport_, &selected_crypto_suite,
                     &rtcp_send_key, &rtcp_recv_key) ||
      !srtp_transport_->SetRtcpParams(
          selected_crypto_suite, &rtcp_send_key[0],
          static_cast<int>(rtcp_send_key.size()), send_extension_ids,
          selected_crypto_suite, &rtcp_recv_key[0],
          static_cast<int>(rtcp_recv_key.size()), recv_extension_ids)) {
    SignalDtlsSrtpSetupFailure(this, /*rtcp=*/true);
    RTC_LOG(LS_WARNING) << "DTLS-SRTP key installation for RTCP failed";
  }
}

bool DtlsSrtpTransport::ExtractParams(
    cricket::DtlsTransportInternal* dtls_transport,
    int* selected_crypto_suite,
    std::vector<unsigned char>* send_key,
    std::vector<unsigned char>* recv_key) {
  if (!dtls_transport || !dtls_transport->IsDtlsActive()) {
    return false;
  }

  if (!dtls_transport->GetSrtpCryptoSuite(selected_crypto_suite)) {
    RTC_LOG(LS_ERROR) << "No DTLS-SRTP selected crypto suite";
    return false;
  }

  RTC_LOG(LS_INFO) << "Extracting keys from transport: "
                   << dtls_transport->transport_name();

  int key_len;
  int salt_len;
  if (!rtc::GetSrtpKeyAndSaltLengths((*selected_crypto_suite), &key_len,
                                     &salt_len)) {
    RTC_LOG(LS_ERROR) << "Unknown DTLS-SRTP crypto suite"
                      << selected_crypto_suite;
    return false;
  }

  // OK, we're now doing DTLS (RFC 5764)
  std::vector<unsigned char> dtls_buffer(key_len * 2 + salt_len * 2);

  // RFC 5705 exporter using the RFC 5764 parameters
  if (!dtls_transport->ExportKeyingMaterial(kDtlsSrtpExporterLabel, NULL, 0,
                                            false, &dtls_buffer[0],
                                            dtls_buffer.size())) {
    RTC_LOG(LS_WARNING) << "DTLS-SRTP key export failed";
    RTC_NOTREACHED();  // This should never happen
    return false;
  }

  // Sync up the keys with the DTLS-SRTP interface
  std::vector<unsigned char> client_write_key(key_len + salt_len);
  std::vector<unsigned char> server_write_key(key_len + salt_len);
  size_t offset = 0;
  memcpy(&client_write_key[0], &dtls_buffer[offset], key_len);
  offset += key_len;
  memcpy(&server_write_key[0], &dtls_buffer[offset], key_len);
  offset += key_len;
  memcpy(&client_write_key[key_len], &dtls_buffer[offset], salt_len);
  offset += salt_len;
  memcpy(&server_write_key[key_len], &dtls_buffer[offset], salt_len);

  rtc::SSLRole role;
  if (!dtls_transport->GetSslRole(&role)) {
    RTC_LOG(LS_WARNING) << "GetSslRole failed";
    return false;
  }

  if (role == rtc::SSL_SERVER) {
    *send_key = server_write_key;
    *recv_key = client_write_key;
  } else {
    *send_key = client_write_key;
    *recv_key = server_write_key;
  }

  return true;
}

void DtlsSrtpTransport::SetDtlsTransport(
    cricket::DtlsTransportInternal* new_dtls_transport,
    cricket::DtlsTransportInternal** old_dtls_transport) {
  if (*old_dtls_transport) {
    (*old_dtls_transport)->SignalDtlsState.disconnect(this);
    (*old_dtls_transport)->SignalWritableState.disconnect(this);
  }

  *old_dtls_transport = new_dtls_transport;

  if (new_dtls_transport) {
    new_dtls_transport->SignalDtlsState.connect(
        this, &DtlsSrtpTransport::OnDtlsState);
    new_dtls_transport->SignalWritableState.connect(
        this, &DtlsSrtpTransport::OnWritableState);
  }
}

void DtlsSrtpTransport::SetRtpDtlsTransport(
    cricket::DtlsTransportInternal* rtp_dtls_transport) {
  SetDtlsTransport(rtp_dtls_transport, &rtp_dtls_transport_);
}

void DtlsSrtpTransport::SetRtcpDtlsTransport(
    cricket::DtlsTransportInternal* rtcp_dtls_transport) {
  SetDtlsTransport(rtcp_dtls_transport, &rtcp_dtls_transport_);
}

void DtlsSrtpTransport::UpdateWritableStateAndMaybeSetupDtlsSrtp() {
  bool writable = IsDtlsWritable();
  SetWritable(writable);
  if (writable) {
    MaybeSetupDtlsSrtp();
  }
}

void DtlsSrtpTransport::SetWritable(bool writable) {
  // Only fire the signal if the writable state changes.
  if (writable_ != writable) {
    writable_ = writable;
    SignalWritableState(writable_);
  }
}

void DtlsSrtpTransport::OnDtlsState(cricket::DtlsTransportInternal* transport,
                                    cricket::DtlsTransportState state) {
  RTC_DCHECK(transport == rtp_dtls_transport_ ||
             transport == rtcp_dtls_transport_);

  if (state != cricket::DTLS_TRANSPORT_CONNECTED) {
    srtp_transport_->ResetParams();
    return;
  }

  MaybeSetupDtlsSrtp();
}

void DtlsSrtpTransport::OnWritableState(
    rtc::PacketTransportInternal* transport) {
  RTC_DCHECK(transport == srtp_transport_->rtp_packet_transport() ||
             transport == srtp_transport_->rtcp_packet_transport());
  UpdateWritableStateAndMaybeSetupDtlsSrtp();
}

void DtlsSrtpTransport::OnPacketReceived(bool rtcp,
                                         rtc::CopyOnWriteBuffer* packet,
                                         const rtc::PacketTime& packet_time) {
  SignalPacketReceived(rtcp, packet, packet_time);
}

void DtlsSrtpTransport::OnReadyToSend(bool ready) {
  SignalReadyToSend(ready);
}

}  // namespace webrtc
