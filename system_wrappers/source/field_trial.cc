// Copyright (c) 2014 The WebRTC project authors. All Rights Reserved.
//
// Use of this source code is governed by a BSD-style license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.
//

#include "system_wrappers/include/field_trial.h"

#include <stddef.h>

#include <map>
#include <string>

#include "absl/strings/string_view.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/string_encode.h"

// Simple field trial implementation, which allows client to
// specify desired flags in InitFieldTrialsFromString.
namespace webrtc {
namespace field_trial {

static const char* trials_init_string = NULL;

#ifndef WEBRTC_EXCLUDE_FIELD_TRIAL_DEFAULT
namespace {
constexpr char kPersistentStringSeparator = '/';
// Validates the given field trial string.
//  E.g.:
//    "WebRTC-experimentFoo/Enabled/WebRTC-experimentBar/Enabled100kbps/"
//    Assigns the process to group "Enabled" on WebRTCExperimentFoo trial
//    and to group "Enabled100kbps" on WebRTCExperimentBar.
//
//  E.g. invalid config:
//    "WebRTC-experiment1/Enabled"  (note missing / separator at the end).
bool FieldTrialsStringIsValidInternal(const absl::string_view trials) {
  if (trials.empty())
    return true;

  size_t next_item = 0;
  std::map<absl::string_view, absl::string_view> field_trials;
  while (next_item < trials.length()) {
    size_t name_end = trials.find(kPersistentStringSeparator, next_item);
    if (name_end == trials.npos || next_item == name_end)
      return false;
    size_t group_name_end =
        trials.find(kPersistentStringSeparator, name_end + 1);
    if (group_name_end == trials.npos || name_end + 1 == group_name_end)
      return false;
    absl::string_view name = trials.substr(next_item, name_end - next_item);
    absl::string_view group_name =
        trials.substr(name_end + 1, group_name_end - name_end - 1);

    next_item = group_name_end + 1;

    // Fail if duplicate with different group name.
    if (field_trials.find(name) != field_trials.end() &&
        field_trials.find(name)->second != group_name) {
      return false;
    }

    field_trials[name] = group_name;
  }

  return true;
}
}  // namespace

bool FieldTrialsStringIsValid(const char* trials_string) {
  return FieldTrialsStringIsValidInternal(trials_string);
}

void InsertOrReplaceFieldTrialStringsInMap(
    std::map<std::string, std::string>& fieldtrial_map,
    const absl::string_view trials_string,
    bool dcheck_on_invalid_fieldtrial) {
  if (FieldTrialsStringIsValidInternal(trials_string)) {
    std::vector<std::string> tokens;
    rtc::split(string(trials_string), '/', &tokens);
    // Skip last token which is empty due to trailing '/'.
    for (size_t idx = 0; idx < tokens.size() - 1; idx += 2) {
      fieldtrial_map[tokens[idx]] = tokens[idx + 1];
    }
  } else {
    if (dcheck_on_invalid_fieldtrial) {
      RTC_DCHECK(false) << "Invalid field trials string:" << trials_string;
    }
  }
}

std::string MergeFieldTrialStrings(const char* first,
                                   const char* second,
                                   bool dcheck_on_invalid_fieldtrial) {
  std::map<std::string, std::string> fieldtrial_map;
  InsertOrReplaceFieldTrialStringsInMap(fieldtrial_map, first,
                                        dcheck_on_invalid_fieldtrial);
  InsertOrReplaceFieldTrialStringsInMap(fieldtrial_map, second,
                                        dcheck_on_invalid_fieldtrial);

  // Merge into fieldtrial string.
  std::string merged = "";
  for (auto const& fieldtrial : fieldtrial_map) {
    merged += fieldtrial.first + '/' + fieldtrial.second + '/';
  }
  return merged;
}

std::string FindFullName(const std::string& name) {
  if (trials_init_string == NULL)
    return std::string();

  std::string trials_string(trials_init_string);
  if (trials_string.empty())
    return std::string();

  size_t next_item = 0;
  while (next_item < trials_string.length()) {
    // Find next name/value pair in field trial configuration string.
    size_t field_name_end =
        trials_string.find(kPersistentStringSeparator, next_item);
    if (field_name_end == trials_string.npos || field_name_end == next_item)
      break;
    size_t field_value_end =
        trials_string.find(kPersistentStringSeparator, field_name_end + 1);
    if (field_value_end == trials_string.npos ||
        field_value_end == field_name_end + 1)
      break;
    std::string field_name(trials_string, next_item,
                           field_name_end - next_item);
    std::string field_value(trials_string, field_name_end + 1,
                            field_value_end - field_name_end - 1);
    next_item = field_value_end + 1;

    if (name == field_name)
      return field_value;
  }
  return std::string();
}
#endif  // WEBRTC_EXCLUDE_FIELD_TRIAL_DEFAULT

// Optionally initialize field trial from a string.
void InitFieldTrialsFromString(const char* trials_string) {
  RTC_LOG(LS_INFO) << "Setting field trial string:" << trials_string;
#ifndef WEBRTC_EXCLUDE_FIELD_TRIAL_DEFAULT
  if (trials_string) {
    RTC_DCHECK(FieldTrialsStringIsValidInternal(trials_string))
        << "Invalid field trials string:" << trials_string;
  };
#endif  // WEBRTC_EXCLUDE_FIELD_TRIAL_DEFAULT
  trials_init_string = trials_string;
}

const char* GetFieldTrialString() {
  return trials_init_string;
}

}  // namespace field_trial
}  // namespace webrtc
