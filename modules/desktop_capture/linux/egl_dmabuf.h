/*
 *  Copyright 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_DESKTOP_CAPTURE_LINUX_EGL_DMABUF_H_
#define MODULES_DESKTOP_CAPTURE_LINUX_EGL_DMABUF_H_

#include <epoxy/egl.h>
#include <epoxy/gl.h>
#include <gbm.h>

#include <memory>
#include <string>
#include <vector>

#include "modules/desktop_capture/desktop_geometry.h"

namespace webrtc {

class EglDmaBuf {
 public:
  struct EGLStruct {
    std::vector<std::string> extensions;
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLContext context = EGL_NO_CONTEXT;
  };

  EglDmaBuf();
  ~EglDmaBuf();

  std::unique_ptr<uint8_t[]> imageFromDmaBuf(int32_t fd,
                                             const DesktopSize& size,
                                             int32_t stride,
                                             uint32_t format,
                                             uint32_t offset,
                                             uint64_t modifier);
  std::vector<uint64_t> queryDmaBufModifiers(uint32_t format);

  bool isEglInitialized() const { return egl_initialized_; }

 private:
  bool egl_initialized_ = false;
  int32_t drm_fd_ = -1;               // for GBM buffer mmap
  gbm_device* gbm_device_ = nullptr;  // for passed GBM buffer retrieval

  EGLStruct egl_;
};

}  // namespace webrtc

#endif  // MODULES_DESKTOP_CAPTURE_LINUX_EGL_DMABUF_H_
