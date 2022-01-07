/*
 *  Copyright 2022 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_DESKTOP_CAPTURE_LINUX_WAYLAND_MOUSE_CURSOR_MONITOR_PIPEWIRE_H_
#define MODULES_DESKTOP_CAPTURE_LINUX_WAYLAND_MOUSE_CURSOR_MONITOR_PIPEWIRE_H_

#include <memory>

#include "api/scoped_refptr.h"
#include "modules/desktop_capture/desktop_capture_options.h"
#include "modules/desktop_capture/desktop_capture_types.h"
#include "modules/desktop_capture/linux/wayland/shared_screencast_stream.h"
#include "modules/desktop_capture/mouse_cursor.h"
#include "modules/desktop_capture/mouse_cursor_monitor.h"

namespace webrtc {

class MouseCursorMonitorPipeWire : public MouseCursorMonitor {
 public:
  explicit MouseCursorMonitorPipeWire(const DesktopCaptureOptions& options);
  ~MouseCursorMonitorPipeWire() override;

  static MouseCursorMonitor* CreateForWindow(
      const DesktopCaptureOptions& options,
      WindowId window);
  static MouseCursorMonitor* CreateForScreen(
      const DesktopCaptureOptions& options,
      ScreenId screen);
  static std::unique_ptr<MouseCursorMonitor> Create(
      const DesktopCaptureOptions& options);

  void Init(Callback* callback, Mode mode) override;
  void Capture() override;

 private:
  DesktopCaptureOptions options_ = {};
  Callback* callback_ = nullptr;
  Mode mode_;
};

}  // namespace webrtc

#endif  // MODULES_DESKTOP_CAPTURE_LINUX_WAYLAND_MOUSE_CURSOR_MONITOR_PIPEWIRE_H_
