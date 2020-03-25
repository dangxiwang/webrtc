/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/desktop_capture/win/full_screen_win_application_handler.h"
#include <algorithm>
#include <cwctype>
#include <memory>
#include <string>
#include <vector>
#include "absl/strings/match.h"
#include "modules/desktop_capture/win/screen_capture_utils.h"
#include "modules/desktop_capture/win/window_capture_utils.h"
#include "rtc_base/arraysize.h"
#include "rtc_base/logging.h"  // For RTC_LOG_GLE
#include "rtc_base/string_utils.h"

namespace webrtc {
namespace {

std::string WindowText(HWND window) {
  size_t len = ::GetWindowTextLength(window);
  if (len == 0)
    return std::string();

  std::vector<wchar_t> buffer(len + 1, 0);
  size_t copied = ::GetWindowTextW(window, buffer.data(), buffer.size());
  if (copied == 0)
    return std::string();
  return rtc::ToUtf8(buffer.data(), copied);
}

DWORD WindowProcessId(HWND window) {
  DWORD dwProcessId = 0;
  ::GetWindowThreadProcessId(window, &dwProcessId);
  return dwProcessId;
}

std::wstring FileNameFromPath(const std::wstring& path) {
  auto found = path.rfind(L"\\");
  if (found == std::string::npos)
    return path;
  return path.substr(found + 1);
}

// Returns windows which belong to given process id
// |sources| is a full list of available windows
// |processId| is a process identifier (window owner)
// |window_to_exclude| is a window to be exluded from result
DesktopCapturer::SourceList GetProcessWindows(
    const DesktopCapturer::SourceList& sources,
    DWORD processId,
    HWND window_to_exclude) {
  DesktopCapturer::SourceList result;
  std::copy_if(sources.begin(), sources.end(), std::back_inserter(result),
               [&](DesktopCapturer::Source source) {
                 const HWND source_hwnd = reinterpret_cast<HWND>(source.id);
                 return window_to_exclude != source_hwnd &&
                        WindowProcessId(source_hwnd) == processId;
               });
  return result;
}

class FullScreenPowerPointHandler : public FullScreenApplicationHandler {
 public:
  explicit FullScreenPowerPointHandler(DesktopCapturer::SourceId sourceId)
      : FullScreenApplicationHandler(sourceId) {}

  ~FullScreenPowerPointHandler() override {}

  DesktopCapturer::SourceId FindFullScreenWindow(
      const DesktopCapturer::SourceList& window_list,
      int64_t timestamp) const override {
    if (window_list.empty())
      return 0;

    HWND original_window = reinterpret_cast<HWND>(GetSourceId());
    DWORD process_id = WindowProcessId(original_window);

    DesktopCapturer::SourceList powerpoint_windows =
        GetProcessWindows(window_list, process_id, original_window);

    if (powerpoint_windows.empty())
      return 0;

    if (GetWindowType(original_window) != WindowType::kEditor)
      return 0;

    const auto original_document = GetDocumentFromEditorTitle(original_window);

    for (const auto& source : powerpoint_windows) {
      HWND window = reinterpret_cast<HWND>(source.id);

      // Looking for slide show window for the same document
      if (GetWindowType(window) != WindowType::kSlideShow ||
          GetDocumentFromSlideShowTitle(window) != original_document) {
        continue;
      }

      return source.id;
    }

    return 0;
  }

 private:
  enum class WindowType { kEditor, kSlideShow, kOther };

  WindowType GetWindowType(HWND window) const {
    if (IsEditorWindow(window))
      return WindowType::kEditor;
    else if (IsSlideShowWindow(window))
      return WindowType::kSlideShow;
    else
      return WindowType::kOther;
  }

  constexpr static char kDocumentTitleSeparator[] = " - ";

  std::string GetDocumentFromEditorTitle(HWND window) const {
    std::string title = WindowText(window);
    auto position = title.find(kDocumentTitleSeparator);
    return rtc::string_trim(title.substr(0, position));
  }

  std::string GetDocumentFromSlideShowTitle(HWND window) const {
    std::string title = WindowText(window);
    auto left_pos = title.find(kDocumentTitleSeparator);
    auto right_pos = title.rfind(kDocumentTitleSeparator);
    constexpr size_t kSeparatorLength = arraysize(kDocumentTitleSeparator) - 1;
    if (left_pos == std::string::npos || right_pos == std::string::npos)
      return title;

    if (right_pos > left_pos + kSeparatorLength) {
      auto result_len = right_pos - left_pos - kSeparatorLength;
      auto document = title.substr(left_pos + kSeparatorLength, result_len);
      return rtc::string_trim(document);
    } else {
      auto document =
          title.substr(left_pos + kSeparatorLength, std::wstring::npos);
      return rtc::string_trim(document);
    }
  }

  bool IsEditorWindow(HWND window) const {
    constexpr WCHAR kScreenClassName[] = L"PPTFrameClass";
    constexpr size_t kScreenClassNameLength = arraysize(kScreenClassName) - 1;

    // We need to verify that window class is equal to |kScreenClassName|.
    // To do that we need a buffer large enough to include a null terminated
    // string one code point bigger than |kScreenClassName|. It will help us to
    // check that size of class name string returned by GetClassNameW is equal
    // to |kScreenClassNameLength| not being limited by size of buffer (case
    // when |kScreenClassName| is a prefix for class name string).
    WCHAR buffer[arraysize(kScreenClassName) + 3];
    const int length = ::GetClassNameW(window, buffer, arraysize(buffer));
    if (length != kScreenClassNameLength)
      return false;
    return wcsncmp(buffer, kScreenClassName, kScreenClassNameLength) == 0;
  }

  bool IsSlideShowWindow(HWND window) const {
    const LONG style = ::GetWindowLong(window, GWL_STYLE);
    const bool min_box = WS_MINIMIZEBOX & style;
    const bool max_box = WS_MAXIMIZEBOX & style;
    return !min_box && !max_box;
  }
};

class OpenOfficeApplicationHandler : public FullScreenApplicationHandler {
 public:
  explicit OpenOfficeApplicationHandler(DesktopCapturer::SourceId sourceId)
      : FullScreenApplicationHandler(sourceId) {}

  DesktopCapturer::SourceId FindFullScreenWindow(
      const DesktopCapturer::SourceList& window_list,
      int64_t timestamp) const override {
    if (window_list.empty())
      return 0;

    HWND original_window = reinterpret_cast<HWND>(GetSourceId());
    DWORD process_id = WindowProcessId(original_window);

    DesktopCapturer::SourceList app_windows =
        GetProcessWindows(window_list, process_id, nullptr);

    if (!CheckLastRecentUsedWindowDocument(app_windows))
      return false;

    // Slide Show window is a fullscreen window with empty title.
    auto slide_show_window =
        std::find_if(app_windows.begin(), app_windows.end(), [](const auto& x) {
          if (!x.title.empty())
            return false;

          DesktopRect rect;
          if (!GetWindowRect(reinterpret_cast<HWND>(x.id), &rect))
            return false;

          return EqualsScreenRect(rect);
        });

    if (slide_show_window == app_windows.end())
      return 0;

    return slide_show_window->id;
  }

 private:
  bool CheckLastRecentUsedWindowDocument(
      const DesktopCapturer::SourceList& app_windows) const {
    // We need to check if slide show window belongs to the document window
    // we're sharing. The problem here is the slide show window title is empty
    // and can't be used to do that. As workarround we can go through the list
    // of windows, which is sorted by decreasing z-order (last recent used goes
    // first), and find first window with non empty title. As Presenter Mode in
    // OpenOffice doesn't allow user to switch focus the window we found will be
    // a match to slide show window. The rest of work is to compare titles of
    // the window we found and the window we're sharing

    auto first_titled_window =
        std::find_if(app_windows.begin(), app_windows.end(),
                     [](const auto& x) { return !x.title.empty(); });

    HWND original_window = reinterpret_cast<HWND>(GetSourceId());
    if (first_titled_window == app_windows.end() ||
        first_titled_window->title != WindowText(original_window)) {
      return false;
    }

    return true;
  }
};

std::wstring GetPathByWindowId(HWND window_id) {
  DWORD process_id = WindowProcessId(window_id);
  HANDLE process =
      ::OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, process_id);
  if (process == NULL)
    return L"";
  DWORD path_len = MAX_PATH;
  WCHAR path[MAX_PATH];
  std::wstring result;
  if (::QueryFullProcessImageNameW(process, 0, path, &path_len))
    result = std::wstring(path, path_len);
  else
    RTC_LOG_GLE(LS_ERROR) << "QueryFullProcessImageName failed.";

  ::CloseHandle(process);
  return result;
}

}  // namespace

std::unique_ptr<FullScreenApplicationHandler>
CreateFullScreenWinApplicationHandler(DesktopCapturer::SourceId source_id) {
  std::unique_ptr<FullScreenApplicationHandler> result;
  HWND hwnd = reinterpret_cast<HWND>(source_id);
  std::wstring exe_path = GetPathByWindowId(hwnd);
  std::wstring file_name = FileNameFromPath(exe_path);
  std::transform(file_name.begin(), file_name.end(), file_name.begin(),
                 std::towupper);

  if (file_name == L"POWERPNT.EXE") {
    result = std::make_unique<FullScreenPowerPointHandler>(source_id);
  } else if (file_name == L"SOFFICE.BIN" &&
             absl::EndsWith(WindowText(hwnd), "OpenOffice Impress")) {
    result = std::make_unique<OpenOfficeApplicationHandler>(source_id);
  }

  return result;
}

}  // namespace webrtc
