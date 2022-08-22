/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_DESKTOP_CAPTURE_DESKTOP_CAPTURER_H_
#define MODULES_DESKTOP_CAPTURE_DESKTOP_CAPTURER_H_

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#if defined(WEBRTC_USE_GIO)
#include "modules/desktop_capture/desktop_capture_metadata.h"
#endif  // defined(WEBRTC_USE_GIO)
#include "modules/desktop_capture/desktop_capture_types.h"
#include "modules/desktop_capture/desktop_frame.h"
#include "modules/desktop_capture/shared_memory.h"
#include "rtc_base/system/rtc_export.h"

namespace webrtc {

class DesktopCaptureOptions;
class DesktopFrame;

// A controller to be implemented and returned by
// GetDelegatedSourceListController in capturers that require showing their own
// source list and managing user selection there. Apart from ensuring the
// visibility of the source list, these capturers should largely be interacted
// with the same as a normal capturer, though there may be some caveats for
// some DesktopCapturer methods. See GetDelegatedSourceListController for more
// information.
class RTC_EXPORT DelegatedSourceListController {
 public:
  // Used to prompt the capturer to show the delegated source list. If the
  // source list is already visible, this will be a no-op. Must be called after
  // starting the DesktopCapturer.
  //
  // Note that any selection from a previous invocation of the source list may
  // be cleared when this method is called.
  virtual void EnsureVisible() = 0;

  // Used to prompt the capturer to hide the delegated source list. If the
  // source list is already hidden, this will be a no-op.
  virtual void EnsureHidden() = 0;

 protected:
  virtual ~DelegatedSourceListController() {}
};

// Abstract interface for screen and window capturers.
class RTC_EXPORT DesktopCapturer {
 public:
  enum class Result {
    // The frame was captured successfully.
    SUCCESS,

    // There was a temporary error. The caller should continue calling
    // CaptureFrame(), in the expectation that it will eventually recover.
    ERROR_TEMPORARY,

    // Capture has failed and will keep failing if the caller tries calling
    // CaptureFrame() again.
    ERROR_PERMANENT,

    MAX_VALUE = ERROR_PERMANENT
  };

  // Interface that must be implemented by the DesktopCapturer consumers.
  class Callback {
   public:
    // Called after a frame has been captured. `frame` is not nullptr if and
    // only if `result` is SUCCESS.
    virtual void OnCaptureResult(Result result,
                                 std::unique_ptr<DesktopFrame> frame) = 0;

   protected:
    virtual ~Callback() {}
  };

#if defined(CHROMEOS)
  typedef int64_t SourceId;
#else
  typedef intptr_t SourceId;
#endif

  static_assert(std::is_same<SourceId, ScreenId>::value,
                "SourceId should be a same type as ScreenId.");

  struct Source {
    // The unique id to represent a Source of current DesktopCapturer.
    SourceId id;

    // Title of the window or screen in UTF-8 encoding, maybe empty. This field
    // should not be used to identify a source.
    std::string title;
  };

  typedef std::vector<Source> SourceList;

  virtual ~DesktopCapturer();

  // Called at the beginning of a capturing session. `callback` must remain
  // valid until capturer is destroyed.
  virtual void Start(Callback* callback) = 0;

  // Returns a valid pointer if the capturer handles (often requires) displaying
  // it's own source list and requires the user to make their selection there.
  // Returns nullptr if the capturer does not provide a UI for the user to make
  // a selection.
  // Callers should not take ownership of the returned pointer, but it is
  // guaranteed to be valid as long as the desktop_capturer is valid.
  // Note that consumers should still use GetSourceList and SelectSource, but
  // their behavior may be modified, if this returns true. See those methods for
  // a more in-depth discussion of those potential modifications.
  virtual DelegatedSourceListController* GetDelegatedSourceListController();

  // Sets SharedMemoryFactory that will be used to create buffers for the
  // captured frames. The factory can be invoked on a thread other than the one
  // where CaptureFrame() is called. It will be destroyed on the same thread.
  // Shared memory is currently supported only by some DesktopCapturer
  // implementations.
  virtual void SetSharedMemoryFactory(
      std::unique_ptr<SharedMemoryFactory> shared_memory_factory);

  // Captures next frame, and involve callback provided by Start() function.
  // Pending capture requests are canceled when DesktopCapturer is deleted.
  virtual void CaptureFrame() = 0;

  // Sets the window to be excluded from the captured image in the future
  // Capture calls. Used to exclude the screenshare notification window for
  // screen capturing.
  virtual void SetExcludedWindow(WindowId window);

  // TODO(zijiehe): Following functions should be pure virtual. The default
  // implementations are for backward compatibility only. Remove default
  // implementations once all DesktopCapturer implementations in Chromium have
  // implemented these functions.

  // Gets a list of sources current capturer supports. Returns false in case of
  // a failure.
  // For DesktopCapturer implementations to capture screens, this function
  // should return monitors.
  // For DesktopCapturer implementations to capture windows, this function
  // should only return root windows owned by applications.
  //
  // Note that capturers who use a delegated source list will return a
  // SourceList with exactly one value, but it is largely a dummy/placeholder
  // value, and may not be viable for capture until the consumer has been
  // notified via Callback::OnDelegatedSourceListSelection (e.g. it may not have
  // a preview or be a valid SourceID that can be passed to another instance of
  // this capturer.
  virtual bool GetSourceList(SourceList* sources);

  // Selects a source to be captured. Returns false in case of a failure (e.g.
  // if there is no source with the specified type and id.)
  //
  // Note that some capturers with delegated source lists may also support
  // selecting a SourceID that is not in the returned source list as a form of
  // restore token.
  virtual bool SelectSource(SourceId id);

  // Brings the selected source to the front and sets the input focus on it.
  // Returns false in case of a failure or no source has been selected or the
  // implementation does not support this functionality.
  virtual bool FocusOnSelectedSource();

  // Returns true if the `pos` on the selected source is covered by other
  // elements on the display, and is not visible to the users.
  // `pos` is in full desktop coordinates, i.e. the top-left monitor always
  // starts from (0, 0).
  // The return value if `pos` is out of the scope of the source is undefined.
  virtual bool IsOccluded(const DesktopVector& pos);

  // Creates a DesktopCapturer instance which targets to capture windows.
  static std::unique_ptr<DesktopCapturer> CreateWindowCapturer(
      const DesktopCaptureOptions& options);

  // Creates a DesktopCapturer instance which targets to capture screens.
  static std::unique_ptr<DesktopCapturer> CreateScreenCapturer(
      const DesktopCaptureOptions& options);

#if defined(WEBRTC_USE_PIPEWIRE) || defined(WEBRTC_USE_X11)
  static bool IsRunningUnderWayland();

  virtual void UpdateResolution(uint32_t width, uint32_t height) {}
#endif  // defined(WEBRTC_USE_PIPEWIRE) || defined(WEBRTC_USE_X11)

#if defined(WEBRTC_USE_GIO)
  // Populates implementation specific metadata into the passed in pointer.
  // Classes can choose to override it or use the default no-op implementation.
  virtual DesktopCaptureMetadata GetMetadata() { return {}; }
#endif  // defined(WEBRTC_USE_GIO)

 protected:
  // CroppingWindowCapturer needs to create raw capturers without wrappers, so
  // the following two functions are protected.

  // Creates a platform specific DesktopCapturer instance which targets to
  // capture windows.
  static std::unique_ptr<DesktopCapturer> CreateRawWindowCapturer(
      const DesktopCaptureOptions& options);

  // Creates a platform specific DesktopCapturer instance which targets to
  // capture screens.
  static std::unique_ptr<DesktopCapturer> CreateRawScreenCapturer(
      const DesktopCaptureOptions& options);
};

}  // namespace webrtc

#endif  // MODULES_DESKTOP_CAPTURE_DESKTOP_CAPTURER_H_
