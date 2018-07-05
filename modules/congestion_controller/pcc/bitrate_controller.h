/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_PCC_BITRATE_CONTROLLER_H_
#define MODULES_CONGESTION_CONTROLLER_PCC_BITRATE_CONTROLLER_H_

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "api/transport/network_control.h"
#include "api/transport/network_types.h"
#include "modules/congestion_controller/pcc/monitor_interval.h"
#include "modules/congestion_controller/pcc/utility_function.h"

namespace webrtc {
namespace pcc {

class ConfidenceAmplifier {
 public:
  ConfidenceAmplifier();
  int64_t operator()(int64_t) const;

 private:
};

class StepSize {
 public:
  StepSize(ConfidenceAmplifier confidende_amplifier,
           double initial_conversion_factor);
  double ComputeStepSize(double utility_gradient);

 private:
  // The value is positive if the adjustments were in the direction of rate
  // increase and negative otherviese.
  int64_t consecutive_rate_adjustments_number_;
  ConfidenceAmplifier confidence_amplifier_;
  double initial_conversion_factor_;
};

class DynamicBoundary {
 public:
  DynamicBoundary(double initial_dynamic_boundary,
                  double dynamic_boundary_increment);
  double Shrink(double rate_chage, double bitrate);

 private:
  int64_t consecutive_rate_adjustments_number_;
  double initial_dynamic_boundary_;
  double dynamic_boundary_increment_;
};

class BitrateController {
 public:
  BitrateController(double initial_conversion_factor,
                    double initial_dynamic_boundary,
                    double dynamic_boundary_increment,
                    ConfidenceAmplifier confidence_amplifier,
                    double rtt_gradient_coefficient,
                    double loss_coefficient,
                    double throughput_coefficient,
                    double rtt_gradient_threshold);

  BitrateController(double initial_conversion_factor,
                    double initial_dynamic_boundary,
                    double dynamic_boundary_increment,
                    ConfidenceAmplifier confidence_amplifier,
                    std::unique_ptr<UtilityFunction> utility_function);

  DataRate ComputeRateUpdate(const MonitorBlock& block,
                             DataRate bandwidth_estimate);

  ~BitrateController();

 private:
  DynamicBoundary dynamic_boundary_;
  std::unique_ptr<UtilityFunction> utility_function_;
  StepSize step_size_;

  // For slow start mode:
  absl::optional<double> previous_function_value;
};

}  // namespace pcc
}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_PCC_BITRATE_CONTROLLER_H_
