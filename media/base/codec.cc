/*
 *  Copyright (c) 2004 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "media/base/codec.h"

#include "absl/algorithm/container.h"
#include "absl/strings/match.h"
#include "media/base/h264_profile_level_id.h"
#include "media/base/vp9_profile.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/string_encode.h"
#include "rtc_base/strings/string_builder.h"

namespace cricket {

FeedbackParams::FeedbackParams() = default;
FeedbackParams::~FeedbackParams() = default;

bool FeedbackParam::operator==(const FeedbackParam& other) const {
  return absl::EqualsIgnoreCase(other.id(), id()) &&
         absl::EqualsIgnoreCase(other.param(), param());
}

bool FeedbackParams::operator==(const FeedbackParams& other) const {
  return params_ == other.params_;
}

bool FeedbackParams::Has(const FeedbackParam& param) const {
  return absl::c_linear_search(params_, param);
}

void FeedbackParams::Add(const FeedbackParam& param) {
  if (param.id().empty()) {
    return;
  }
  if (Has(param)) {
    // Param already in |this|.
    return;
  }
  params_.push_back(param);
  RTC_CHECK(!HasDuplicateEntries());
}

void FeedbackParams::Intersect(const FeedbackParams& from) {
  std::vector<FeedbackParam>::iterator iter_to = params_.begin();
  while (iter_to != params_.end()) {
    if (!from.Has(*iter_to)) {
      iter_to = params_.erase(iter_to);
    } else {
      ++iter_to;
    }
  }
}

bool FeedbackParams::HasDuplicateEntries() const {
  for (std::vector<FeedbackParam>::const_iterator iter = params_.begin();
       iter != params_.end(); ++iter) {
    for (std::vector<FeedbackParam>::const_iterator found = iter + 1;
         found != params_.end(); ++found) {
      if (*found == *iter) {
        return true;
      }
    }
  }
  return false;
}

Codec::Codec(int id, const std::string& name, int clockrate)
    : id(id), name(name), clockrate(clockrate) {}

Codec::Codec() : id(0), clockrate(0) {}

Codec::Codec(const Codec& c) = default;
Codec::Codec(Codec&& c) = default;
Codec::~Codec() = default;
Codec& Codec::operator=(const Codec& c) = default;
Codec& Codec::operator=(Codec&& c) = default;

bool Codec::operator==(const Codec& c) const {
  return this->id == c.id &&  // id is reserved in objective-c
         name == c.name && clockrate == c.clockrate && params == c.params &&
         feedback_params == c.feedback_params;
}

bool Codec::Matches(const Codec& codec) const {
  // Match the codec id/name based on the typical static/dynamic name rules.
  // Matching is case-insensitive.
  const int kMaxStaticPayloadId = 95;
  return (id <= kMaxStaticPayloadId || codec.id <= kMaxStaticPayloadId)
             ? (id == codec.id)
             : (absl::EqualsIgnoreCase(name, codec.name));
}

bool Codec::MatchesCapability(
    const webrtc::RtpCodecCapability& codec_capability) const {
  webrtc::RtpCodecParameters codec_parameters = ToCodecParameters();

  return codec_parameters.name == codec_capability.name &&
         codec_parameters.kind == codec_capability.kind &&
         (codec_parameters.name == cricket::kRtxCodecName ||
          (codec_parameters.num_channels == codec_capability.num_channels &&
           codec_parameters.clock_rate == codec_capability.clock_rate &&
           codec_parameters.parameters == codec_capability.parameters));
}

bool Codec::GetParam(const std::string& name, std::string* out) const {
  CodecParameterMap::const_iterator iter = params.find(name);
  if (iter == params.end())
    return false;
  *out = iter->second;
  return true;
}

bool Codec::GetParam(const std::string& name, int* out) const {
  CodecParameterMap::const_iterator iter = params.find(name);
  if (iter == params.end())
    return false;
  return rtc::FromString(iter->second, out);
}

void Codec::SetParam(const std::string& name, const std::string& value) {
  params[name] = value;
}

void Codec::SetParam(const std::string& name, int value) {
  params[name] = rtc::ToString(value);
}

bool Codec::RemoveParam(const std::string& name) {
  return params.erase(name) == 1;
}

void Codec::AddFeedbackParam(const FeedbackParam& param) {
  feedback_params.Add(param);
}

bool Codec::HasFeedbackParam(const FeedbackParam& param) const {
  return feedback_params.Has(param);
}

void Codec::IntersectFeedbackParams(const Codec& other) {
  feedback_params.Intersect(other.feedback_params);
}

webrtc::RtpCodecParameters Codec::ToCodecParameters() const {
  webrtc::RtpCodecParameters codec_params;
  codec_params.payload_type = id;
  codec_params.name = name;
  codec_params.clock_rate = clockrate;
  codec_params.parameters.insert(params.begin(), params.end());
  return codec_params;
}

AudioCodec::AudioCodec(int id,
                       const std::string& name,
                       int clockrate,
                       int bitrate,
                       size_t channels)
    : Codec(id, name, clockrate), bitrate(bitrate), channels(channels) {}

AudioCodec::AudioCodec() : Codec(), bitrate(0), channels(0) {}

AudioCodec::AudioCodec(const AudioCodec& c) = default;
AudioCodec::AudioCodec(AudioCodec&& c) = default;
AudioCodec& AudioCodec::operator=(const AudioCodec& c) = default;
AudioCodec& AudioCodec::operator=(AudioCodec&& c) = default;

bool AudioCodec::operator==(const AudioCodec& c) const {
  return bitrate == c.bitrate && channels == c.channels && Codec::operator==(c);
}

bool AudioCodec::Matches(const AudioCodec& codec) const {
  // If a nonzero clockrate is specified, it must match the actual clockrate.
  // If a nonzero bitrate is specified, it must match the actual bitrate,
  // unless the codec is VBR (0), where we just force the supplied value.
  // The number of channels must match exactly, with the exception
  // that channels=0 is treated synonymously as channels=1, per RFC
  // 4566 section 6: " [The channels] parameter is OPTIONAL and may be
  // omitted if the number of channels is one."
  // Preference is ignored.
  // TODO(juberti): Treat a zero clockrate as 8000Hz, the RTP default clockrate.
  return Codec::Matches(codec) &&
         ((codec.clockrate == 0 /*&& clockrate == 8000*/) ||
          clockrate == codec.clockrate) &&
         (codec.bitrate == 0 || bitrate <= 0 || bitrate == codec.bitrate) &&
         ((codec.channels < 2 && channels < 2) || channels == codec.channels);
}

std::string AudioCodec::ToString() const {
  char buf[256];
  rtc::SimpleStringBuilder sb(buf);
  sb << "AudioCodec[" << id << ":" << name << ":" << clockrate << ":" << bitrate
     << ":" << channels << "]";
  return sb.str();
}

webrtc::RtpCodecParameters AudioCodec::ToCodecParameters() const {
  webrtc::RtpCodecParameters codec_params = Codec::ToCodecParameters();
  codec_params.num_channels = static_cast<int>(channels);
  codec_params.kind = MEDIA_TYPE_AUDIO;
  return codec_params;
}

std::string VideoCodec::ToString() const {
  char buf[256];
  rtc::SimpleStringBuilder sb(buf);
  sb << "VideoCodec[" << id << ":" << name << "]";
  return sb.str();
}

webrtc::RtpCodecParameters VideoCodec::ToCodecParameters() const {
  webrtc::RtpCodecParameters codec_params = Codec::ToCodecParameters();
  codec_params.kind = MEDIA_TYPE_VIDEO;
  return codec_params;
}

VideoCodec::VideoCodec(int id, const std::string& name)
    : Codec(id, name, kVideoCodecClockrate) {
  SetDefaultParameters();
}

VideoCodec::VideoCodec(const std::string& name) : VideoCodec(0 /* id */, name) {
  SetDefaultParameters();
}

VideoCodec::VideoCodec() : Codec() {
  clockrate = kVideoCodecClockrate;
}

VideoCodec::VideoCodec(const webrtc::SdpVideoFormat& c)
    : Codec(0 /* id */, c.name, kVideoCodecClockrate) {
  params = c.parameters;
}

VideoCodec::VideoCodec(const VideoCodec& c) = default;
VideoCodec::VideoCodec(VideoCodec&& c) = default;
VideoCodec& VideoCodec::operator=(const VideoCodec& c) = default;
VideoCodec& VideoCodec::operator=(VideoCodec&& c) = default;

void VideoCodec::SetDefaultParameters() {
  if (absl::EqualsIgnoreCase(kH264CodecName, name)) {
    // This default is set for all H.264 codecs created because
    // that was the default before packetization mode support was added.
    // TODO(hta): Move this to the places that create VideoCodecs from
    // SDP or from knowledge of implementation capabilities.
    SetParam(kH264FmtpPacketizationMode, "1");
  }
}

bool VideoCodec::operator==(const VideoCodec& c) const {
  return Codec::operator==(c);
}

static bool IsSameH264PacketizationMode(const CodecParameterMap& ours,
                                        const CodecParameterMap& theirs) {
  // If packetization-mode is not present, default to "0".
  // https://tools.ietf.org/html/rfc6184#section-6.2
  std::string our_packetization_mode = "0";
  std::string their_packetization_mode = "0";
  auto ours_it = ours.find(kH264FmtpPacketizationMode);
  if (ours_it != ours.end()) {
    our_packetization_mode = ours_it->second;
  }
  auto theirs_it = theirs.find(kH264FmtpPacketizationMode);
  if (theirs_it != theirs.end()) {
    their_packetization_mode = theirs_it->second;
  }
  return our_packetization_mode == their_packetization_mode;
}

bool VideoCodec::Matches(const VideoCodec& other) const {
  if (!Codec::Matches(other))
    return false;
  if (absl::EqualsIgnoreCase(name, kH264CodecName))
    return webrtc::H264::IsSameH264Profile(params, other.params) &&
           IsSameH264PacketizationMode(params, other.params);
  if (absl::EqualsIgnoreCase(name, kVp9CodecName))
    return webrtc::IsSameVP9Profile(params, other.params);
  return true;
}

VideoCodec VideoCodec::CreateRtxCodec(int rtx_payload_type,
                                      int associated_payload_type) {
  VideoCodec rtx_codec(rtx_payload_type, kRtxCodecName);
  rtx_codec.SetParam(kCodecParamAssociatedPayloadType, associated_payload_type);
  return rtx_codec;
}

VideoCodec::CodecType VideoCodec::GetCodecType() const {
  const char* payload_name = name.c_str();
  if (absl::EqualsIgnoreCase(payload_name, kRedCodecName)) {
    return CODEC_RED;
  }
  if (absl::EqualsIgnoreCase(payload_name, kUlpfecCodecName)) {
    return CODEC_ULPFEC;
  }
  if (absl::EqualsIgnoreCase(payload_name, kFlexfecCodecName)) {
    return CODEC_FLEXFEC;
  }
  if (absl::EqualsIgnoreCase(payload_name, kRtxCodecName)) {
    return CODEC_RTX;
  }

  return CODEC_VIDEO;
}

bool VideoCodec::ValidateCodecFormat() const {
  if (id < 0 || id > 127) {
    RTC_LOG(LS_ERROR) << "Codec with invalid payload type: " << ToString();
    return false;
  }
  if (GetCodecType() != CODEC_VIDEO) {
    return true;
  }

  // Video validation from here on.
  int min_bitrate = -1;
  int max_bitrate = -1;
  if (GetParam(kCodecParamMinBitrate, &min_bitrate) &&
      GetParam(kCodecParamMaxBitrate, &max_bitrate)) {
    if (max_bitrate < min_bitrate) {
      RTC_LOG(LS_ERROR) << "Codec with max < min bitrate: " << ToString();
      return false;
    }
  }
  return true;
}

RtpDataCodec::RtpDataCodec(int id, const std::string& name)
    : Codec(id, name, kDataCodecClockrate) {}

RtpDataCodec::RtpDataCodec() : Codec() {
  clockrate = kDataCodecClockrate;
}

RtpDataCodec::RtpDataCodec(const RtpDataCodec& c) = default;
RtpDataCodec::RtpDataCodec(RtpDataCodec&& c) = default;
RtpDataCodec& RtpDataCodec::operator=(const RtpDataCodec& c) = default;
RtpDataCodec& RtpDataCodec::operator=(RtpDataCodec&& c) = default;

std::string RtpDataCodec::ToString() const {
  char buf[256];
  rtc::SimpleStringBuilder sb(buf);
  sb << "DataCodec[" << id << ":" << name << "]";
  return sb.str();
}

bool HasNack(const Codec& codec) {
  return codec.HasFeedbackParam(
      FeedbackParam(kRtcpFbParamNack, kParamValueEmpty));
}

bool HasRemb(const Codec& codec) {
  return codec.HasFeedbackParam(
      FeedbackParam(kRtcpFbParamRemb, kParamValueEmpty));
}

bool HasRrtr(const Codec& codec) {
  return codec.HasFeedbackParam(
      FeedbackParam(kRtcpFbParamRrtr, kParamValueEmpty));
}

bool HasTransportCc(const Codec& codec) {
  return codec.HasFeedbackParam(
      FeedbackParam(kRtcpFbParamTransportCc, kParamValueEmpty));
}

const VideoCodec* FindMatchingCodec(
    const std::vector<VideoCodec>& supported_codecs,
    const VideoCodec& codec) {
  for (const VideoCodec& supported_codec : supported_codecs) {
    if (IsSameCodec(codec.name, codec.params, supported_codec.name,
                    supported_codec.params)) {
      return &supported_codec;
    }
  }
  return nullptr;
}

bool IsSameCodec(const std::string& name1,
                 const CodecParameterMap& params1,
                 const std::string& name2,
                 const CodecParameterMap& params2) {
  // If different names (case insensitive), then not same formats.
  if (!absl::EqualsIgnoreCase(name1, name2))
    return false;
  // For every format besides H264 and VP9, comparing names is enough.
  if (absl::EqualsIgnoreCase(name1, kH264CodecName))
    return webrtc::H264::IsSameH264Profile(params1, params2);
  if (absl::EqualsIgnoreCase(name1, kVp9CodecName))
    return webrtc::IsSameVP9Profile(params1, params2);
  return true;
}

}  // namespace cricket
