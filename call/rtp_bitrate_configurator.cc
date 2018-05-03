/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "call/rtp_bitrate_configurator.h"

#include <algorithm>

#include "rtc_base/checks.h"

namespace webrtc {
RtpBitrateConfigurator::RtpBitrateConfigurator(
    const BitrateConstraints& bitrate_config)
    : bitrate_config_(bitrate_config), base_bitrate_config_(bitrate_config) {
  RTC_DCHECK_GE(bitrate_config.min_bitrate_bps, 0);
  RTC_DCHECK_GE(bitrate_config.start_bitrate_bps,
                bitrate_config.min_bitrate_bps);
  if (bitrate_config.max_bitrate_bps != -1) {
    RTC_DCHECK_GE(bitrate_config.max_bitrate_bps,
                  bitrate_config.start_bitrate_bps);
  }
}

RtpBitrateConfigurator::~RtpBitrateConfigurator() = default;

BitrateConstraints RtpBitrateConfigurator::GetConfig() const {
  return bitrate_config_;
}

rtc::Optional<BitrateConstraints>
RtpBitrateConfigurator::UpdateWithSdpParameters(
    const BitrateConstraints& bitrate_config) {
  RTC_DCHECK_GE(bitrate_config.min_bitrate_bps, 0);
  RTC_DCHECK_NE(bitrate_config.start_bitrate_bps, 0);
  if (bitrate_config.max_bitrate_bps != -1) {
    RTC_DCHECK_GT(bitrate_config.max_bitrate_bps, 0);
  }

  rtc::Optional<int> new_start;
  // Only update the "start" bitrate if it's set, and different from the old
  // value. In practice, this value comes from the x-google-start-bitrate codec
  // parameter in SDP, and setting the same remote description twice shouldn't
  // restart bandwidth estimation.
  if (bitrate_config.start_bitrate_bps != -1 &&
      bitrate_config.start_bitrate_bps !=
          base_bitrate_config_.start_bitrate_bps) {
    new_start.emplace(bitrate_config.start_bitrate_bps);
  }
  base_bitrate_config_ = bitrate_config;
  return UpdateConstraints(new_start);
}

rtc::Optional<BitrateConstraints>
RtpBitrateConfigurator::UpdateWithClientPreferences(
    const BitrateSettings& bitrate_mask) {
  bitrate_config_mask_ = bitrate_mask;
  return UpdateConstraints(bitrate_mask.start_bitrate_bps);
}

rtc::Optional<BitrateConstraints> RtpBitrateConfigurator::UpdateConstraints(
    const rtc::Optional<int>& new_start) {
  BitrateConstraints updated;
  updated.min_bitrate_bps =
      std::max(bitrate_config_mask_.min_bitrate_bps.value_or(0),
               base_bitrate_config_.min_bitrate_bps);

  updated.max_bitrate_bps =
      MinPositive(bitrate_config_mask_.max_bitrate_bps.value_or(-1),
                  base_bitrate_config_.max_bitrate_bps);

  // If the combined min ends up greater than the combined max, the max takes
  // priority.
  if (updated.max_bitrate_bps != -1 &&
      updated.min_bitrate_bps > updated.max_bitrate_bps) {
    updated.min_bitrate_bps = updated.max_bitrate_bps;
  }

  // If there is nothing to update (min/max unchanged, no new bandwidth
  // estimation start value), return early.
  if (updated.min_bitrate_bps == bitrate_config_.min_bitrate_bps &&
      updated.max_bitrate_bps == bitrate_config_.max_bitrate_bps &&
      !new_start) {
    return rtc::nullopt;
  }

  if (new_start) {
    // Clamp start by min and max.
    updated.start_bitrate_bps = MinPositive(
        std::max(*new_start, updated.min_bitrate_bps), updated.max_bitrate_bps);
  } else {
    updated.start_bitrate_bps = -1;
  }
  BitrateConstraints config_to_return = updated;
  if (!new_start) {
    updated.start_bitrate_bps = bitrate_config_.start_bitrate_bps;
  }
  bitrate_config_ = updated;
  return config_to_return;
}

}  // namespace webrtc
