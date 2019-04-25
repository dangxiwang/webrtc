/*
 *  Copyright 2010 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "pc/session_description.h"

#include <algorithm>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/memory/memory.h"
#include "pc/media_protocol_names.h"
#include "rtc_base/checks.h"

namespace cricket {
namespace {

ContentInfo* FindContentInfoByName(ContentInfos* contents,
                                   const std::string& name) {
  RTC_DCHECK(contents);
  for (ContentInfo& content : *contents) {
    if (content.name == name) {
      return &content;
    }
  }
  return nullptr;
}

}  // namespace

const ContentInfo* FindContentInfoByName(const ContentInfos& contents,
                                         const std::string& name) {
  for (ContentInfos::const_iterator content = contents.begin();
       content != contents.end(); ++content) {
    if (content->name == name) {
      return &(*content);
    }
  }
  return NULL;
}

const ContentInfo* FindContentInfoByType(const ContentInfos& contents,
                                         MediaProtocolType type) {
  for (const auto& content : contents) {
    if (content.type == type) {
      return &content;
    }
  }
  return nullptr;
}

ContentGroup::ContentGroup(const std::string& semantics)
    : semantics_(semantics) {}

ContentGroup::ContentGroup(const ContentGroup&) = default;
ContentGroup::ContentGroup(ContentGroup&&) = default;
ContentGroup& ContentGroup::operator=(const ContentGroup&) = default;
ContentGroup& ContentGroup::operator=(ContentGroup&&) = default;
ContentGroup::~ContentGroup() = default;

const std::string* ContentGroup::FirstContentName() const {
  return (!content_names_.empty()) ? &(*content_names_.begin()) : NULL;
}

bool ContentGroup::HasContentName(const std::string& content_name) const {
  return absl::c_linear_search(content_names_, content_name);
}

void ContentGroup::AddContentName(const std::string& content_name) {
  if (!HasContentName(content_name)) {
    content_names_.push_back(content_name);
  }
}

bool ContentGroup::RemoveContentName(const std::string& content_name) {
  ContentNames::iterator iter = absl::c_find(content_names_, content_name);
  if (iter == content_names_.end()) {
    return false;
  }
  content_names_.erase(iter);
  return true;
}

SessionDescription::SessionDescription() = default;
SessionDescription::SessionDescription(const SessionDescription&) = default;

SessionDescription::~SessionDescription() {
  for (ContentInfos::iterator content = contents_.begin();
       content != contents_.end(); ++content) {
    delete content->description;
  }
}

std::unique_ptr<SessionDescription> SessionDescription::Clone() const {
  // Copy the non-special portions using the private copy constructor.
  auto copy = absl::WrapUnique(new SessionDescription(*this));
  // Copy all ContentDescriptions.
  for (ContentInfos::iterator content = copy->contents_.begin();
       content != copy->contents().end(); ++content) {
    content->description = content->description->Copy();
  }
  return copy;
}

SessionDescription* SessionDescription::Copy() const {
  return Clone().release();
}

const ContentInfo* SessionDescription::GetContentByName(
    const std::string& name) const {
  return FindContentInfoByName(contents_, name);
}

ContentInfo* SessionDescription::GetContentByName(const std::string& name) {
  return FindContentInfoByName(&contents_, name);
}

const MediaContentDescription* SessionDescription::GetContentDescriptionByName(
    const std::string& name) const {
  const ContentInfo* cinfo = FindContentInfoByName(contents_, name);
  if (cinfo == NULL) {
    return NULL;
  }

  return cinfo->media_description();
}

MediaContentDescription* SessionDescription::GetContentDescriptionByName(
    const std::string& name) {
  ContentInfo* cinfo = FindContentInfoByName(&contents_, name);
  if (cinfo == NULL) {
    return NULL;
  }

  return cinfo->media_description();
}

const ContentInfo* SessionDescription::FirstContentByType(
    MediaProtocolType type) const {
  return FindContentInfoByType(contents_, type);
}

const ContentInfo* SessionDescription::FirstContent() const {
  return (contents_.empty()) ? NULL : &(*contents_.begin());
}

void SessionDescription::AddContent(const std::string& name,
                                    MediaProtocolType type,
                                    MediaContentDescription* description) {
  ContentInfo content(type);
  content.name = name;
  content.description = description;
  AddContent(&content);
}

void SessionDescription::AddContent(const std::string& name,
                                    MediaProtocolType type,
                                    bool rejected,
                                    MediaContentDescription* description) {
  ContentInfo content(type);
  content.name = name;
  content.rejected = rejected;
  content.description = description;
  AddContent(&content);
}

void SessionDescription::AddContent(const std::string& name,
                                    MediaProtocolType type,
                                    bool rejected,
                                    bool bundle_only,
                                    MediaContentDescription* description) {
  ContentInfo content(type);
  content.name = name;
  content.rejected = rejected;
  content.bundle_only = bundle_only;
  content.description = description;
  AddContent(&content);
}

void SessionDescription::AddContent(ContentInfo* content) {
  if (extmap_allow_mixed()) {
    // Mixed support on session level overrides setting on media level.
    content->description->set_extmap_allow_mixed_enum(
        MediaContentDescription::kSession);
  }
  contents_.push_back(std::move(*content));
}

bool SessionDescription::RemoveContentByName(const std::string& name) {
  for (ContentInfos::iterator content = contents_.begin();
       content != contents_.end(); ++content) {
    if (content->name == name) {
      delete content->description;
      contents_.erase(content);
      return true;
    }
  }

  return false;
}

void SessionDescription::AddTransportInfo(const TransportInfo& transport_info) {
  transport_infos_.push_back(transport_info);
}

bool SessionDescription::RemoveTransportInfoByName(const std::string& name) {
  for (TransportInfos::iterator transport_info = transport_infos_.begin();
       transport_info != transport_infos_.end(); ++transport_info) {
    if (transport_info->content_name == name) {
      transport_infos_.erase(transport_info);
      return true;
    }
  }
  return false;
}

const TransportInfo* SessionDescription::GetTransportInfoByName(
    const std::string& name) const {
  for (TransportInfos::const_iterator iter = transport_infos_.begin();
       iter != transport_infos_.end(); ++iter) {
    if (iter->content_name == name) {
      return &(*iter);
    }
  }
  return NULL;
}

TransportInfo* SessionDescription::GetTransportInfoByName(
    const std::string& name) {
  for (TransportInfos::iterator iter = transport_infos_.begin();
       iter != transport_infos_.end(); ++iter) {
    if (iter->content_name == name) {
      return &(*iter);
    }
  }
  return NULL;
}

void SessionDescription::RemoveGroupByName(const std::string& name) {
  for (ContentGroups::iterator iter = content_groups_.begin();
       iter != content_groups_.end(); ++iter) {
    if (iter->semantics() == name) {
      content_groups_.erase(iter);
      break;
    }
  }
}

bool SessionDescription::HasGroup(const std::string& name) const {
  for (ContentGroups::const_iterator iter = content_groups_.begin();
       iter != content_groups_.end(); ++iter) {
    if (iter->semantics() == name) {
      return true;
    }
  }
  return false;
}

const ContentGroup* SessionDescription::GetGroupByName(
    const std::string& name) const {
  for (ContentGroups::const_iterator iter = content_groups_.begin();
       iter != content_groups_.end(); ++iter) {
    if (iter->semantics() == name) {
      return &(*iter);
    }
  }
  return NULL;
}

// DataContentDescription shenanigans
DataContentDescription* RtpDataContentDescription::as_data() {
  if (!shim_) {
    shim_.reset(new DataContentDescription(this));
  }
  return shim_.get();
}

const DataContentDescription* RtpDataContentDescription::as_data() const {
  return const_cast<RtpDataContentDescription*>(this)->as_data();
}

DataContentDescription* SctpDataContentDescription::as_data() {
  if (!shim_) {
    shim_.reset(new DataContentDescription(this));
  }
  return shim_.get();
}

const DataContentDescription* SctpDataContentDescription::as_data() const {
  return const_cast<SctpDataContentDescription*>(this)->as_data();
}

DataContentDescription::DataContentDescription() : is_sctp_(false) {
  // In this case, we will initialize owned_description_ as soon as
  // we are told what protocol to use.
}

DataContentDescription::DataContentDescription(
    SctpDataContentDescription* wrapped)
    : real_description_(wrapped), is_sctp_(true) {}

DataContentDescription::DataContentDescription(
    RtpDataContentDescription* wrapped)
    : real_description_(wrapped), is_sctp_(false) {}

void DataContentDescription::set_protocol(const std::string& protocol) {
  if (real_description_) {
    real_description_->set_protocol(protocol);
  } else {
    RTC_CHECK(!owned_description_.get());
    // We used to not know what protocol we were going to use. Now we know.
    if (IsSctpProtocol(protocol)) {
      owned_description_ = absl::make_unique<SctpDataContentDescription>();
    } else {
      owned_description_ = absl::make_unique<RtpDataContentDescription>();
    }
    real_description_ = owned_description_.get();
  }
}

const std::vector<DataCodec>& DataContentDescription::codecs() const {
  if (is_sctp_) {
    return null_array_;
  }
  return real_description_->as_rtp_data()->codecs();
}

void DataContentDescription::set_codecs(const std::vector<DataCodec>& codecs) {
  if (!is_sctp_) {
    real_description_->as_rtp_data()->set_codecs(codecs);
  }
}

bool DataContentDescription::has_codecs() const {
  if (is_sctp_) {
    return false;
  }
  return real_description_->as_rtp_data()->has_codecs();
}

bool DataContentDescription::HasCodec(int id) {
  if (is_sctp_) {
    return false;
  }
  return real_description_->as_rtp_data()->HasCodec(id);
}

void DataContentDescription::AddCodec(const DataCodec& codec) {
  if (!is_sctp_) {
    real_description_->as_rtp_data()->AddCodec(codec);
  }
}

void DataContentDescription::AddOrReplaceCodec(const DataCodec& codec) {
  if (!is_sctp_) {
    real_description_->as_rtp_data()->AddOrReplaceCodec(codec);
  }
}

void DataContentDescription::AddCodecs(const std::vector<DataCodec>& codecs) {
  if (!is_sctp_) {
    real_description_->as_rtp_data()->AddCodecs(codecs);
  }
}

}  // namespace cricket
