/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_PCC_UTILITY_FUNCTION_H_
#define MODULES_CONGESTION_CONTROLLER_PCC_UTILITY_FUNCTION_H_

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "api/transport/network_control.h"
#include "api/transport/network_types.h"
#include "modules/congestion_controller/pcc/monitor_interval.h"

namespace webrtc {
namespace pcc {

class UtilityFunction {
 public:
  virtual double ComputeUtilityFunction(
      const MonitorInterval& monitor_interval) const = 0;
  virtual ~UtilityFunction() = default;
};

class VivaceUtilityFunction : public UtilityFunction {
 public:
  VivaceUtilityFunction(double rtt_gradient_coefficient,
                        double loss_coefficient,
                        double throughput_coefficient,
                        double rtt_gradient_threshold);
  double ComputeUtilityFunction(
      const MonitorInterval& monitor_interval) const override;
  ~VivaceUtilityFunction() override;

 private:
  double ComputeRttGradient(const MonitorInterval& monitor_interval) const;

  double rtt_gradient_coefficient_;
  double loss_coefficient_;
  double throughput_power_;
  double rtt_gradient_threshold_;
};

}  // namespace pcc
}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_PCC_UTILITY_FUNCTION_H_
