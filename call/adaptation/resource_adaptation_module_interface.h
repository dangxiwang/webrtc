/*
 *  Copyright 2019 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef CALL_ADAPTATION_RESOURCE_ADAPTATION_MODULE_INTERFACE_H_
#define CALL_ADAPTATION_RESOURCE_ADAPTATION_MODULE_INTERFACE_H_

#include <limits>
#include <utility>

#include "absl/types/optional.h"

namespace webrtc {

// Describes optional restrictions to the resolution and frame rate of a video
// source.
class VideoSourceRestrictions {
 public:
  VideoSourceRestrictions(absl::optional<size_t> max_pixels_per_frame,
                          absl::optional<double> max_frame_rate);

  const absl::optional<size_t>& max_pixels_per_frame() const;
  const absl::optional<double>& max_frame_rate() const;

 private:
  absl::optional<size_t> max_pixels_per_frame_;
  absl::optional<double> max_frame_rate_;
};

class ResourceAdaptationModuleListener {
 public:
  virtual ~ResourceAdaptationModuleListener();

  virtual void OnVideoSourceRestrictionsUpdated(
      VideoSourceRestrictions restrictions) = 0;
};

// Responsible for reconfiguring encoded streams based on resource consumption,
// such as scaling down resolution or frame rate when CPU is overused. This
// interface is meant to be injectable into VideoStreamEncoder.
//
// [UNDER CONSTRUCTION] This interface is work-in-progress. In the future it
// needs to be able to handle all the necessary input and output for resource
// adaptation decision making.
//
// TODO(https://crbug.com/webrtc/11222): Make this interface feature-complete so
// that a module (such as OveruseFrameDetectorResourceAdaptationModule) is fully
// operational through this abstract interface.
class ResourceAdaptationModuleInterface {
 public:
  virtual ~ResourceAdaptationModuleInterface();

  // TODO(hbos): When input/output of the module is adequetly handled by this
  // interface, these methods need to say which stream to start/stop, enabling
  // multi-stream aware implementations of ResourceAdaptationModuleInterface. We
  // don't want to do this before we have the right interfaces (e.g. if we pass
  // in a VideoStreamEncoder here directly then have a dependency on a different
  // build target). For the multi-stream use case we may consider making
  // ResourceAdaptationModuleInterface reference counted.
  virtual void StartCheckForOveruse(
      ResourceAdaptationModuleListener* adaptation_listener) = 0;
  virtual void StopCheckForOveruse() = 0;
};

}  // namespace webrtc

#endif  // CALL_ADAPTATION_RESOURCE_ADAPTATION_MODULE_INTERFACE_H_
