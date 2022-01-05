/*
 *  Copyright (c) 2022 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/desktop_capture/linux/wayland/mouse_cursor_monitor_pipewire.h"

#include <utility>

#include "modules/desktop_capture/desktop_capture_options.h"
#include "modules/desktop_capture/desktop_capturer.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {

MouseCursorMonitorPipeWire::MouseCursorMonitorPipeWire(
    const DesktopCaptureOptions& options)
    : options_(options) {}

MouseCursorMonitorPipeWire::~MouseCursorMonitorPipeWire() {}

void MouseCursorMonitorPipeWire::Init(Callback* callback, Mode mode) {
  RTC_DCHECK(!callback_);
  RTC_DCHECK(callback);

  callback_ = callback;
  mode_ = mode;
}

void MouseCursorMonitorPipeWire::Capture() {
  if (mode_ == SHAPE_AND_POSITION) {
    std::unique_ptr<MouseCursor> mouse_cursor =
        options_.screencast_stream()->CaptureCursor();

    if (mouse_cursor && mouse_cursor->image()->data()) {
      callback_->OnMouseCursor(std::move(mouse_cursor.release()));
    }

    DesktopVector mouse_cursor_position =
        options_.screencast_stream()->CaptureCursorPosition();
    if (mouse_cursor_position.x() >= 0 && mouse_cursor_position.y() >= 0) {
      callback_->OnMouseCursorPosition(mouse_cursor_position);
    }
  }
}

// static
MouseCursorMonitor* MouseCursorMonitorPipeWire::CreateForWindow(
    const DesktopCaptureOptions& options,
    WindowId window) {
  if (!options.screencast_stream())
    return nullptr;

  return new MouseCursorMonitorPipeWire(options);
}

MouseCursorMonitor* MouseCursorMonitorPipeWire::CreateForScreen(
    const DesktopCaptureOptions& options,
    ScreenId screen) {
  if (!options.screencast_stream())
    return nullptr;

  return new MouseCursorMonitorPipeWire(options);
}

std::unique_ptr<MouseCursorMonitor> MouseCursorMonitorPipeWire::Create(
    const DesktopCaptureOptions& options) {
  if (!options.screencast_stream())
    return nullptr;

  return std::make_unique<MouseCursorMonitorPipeWire>(options);
}

}  // namespace webrtc
