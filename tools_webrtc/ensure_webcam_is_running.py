#!/usr/bin/env python
# Copyright (c) 2014 The WebRTC project authors. All Rights Reserved.
#
# Use of this source code is governed by a BSD-style license
# that can be found in the LICENSE file in the root of the source
# tree. An additional intellectual property rights grant can be found
# in the file PATENTS.  All contributing project authors may
# be found in the AUTHORS file in the root of the source tree.

"""Checks if a virtual webcam is running and starts it if not.

Returns a non-zero return code if the webcam could not be started.

Prerequisites:
* The Python interpreter must have the psutil package installed.
* Windows: a scheduled task named 'ManyCam' must exist and be configured to
  launch ManyCam preconfigured to auto-play the test clip.
* Mac: ManyCam must be installed in the default location and be preconfigured
  to auto-play the test clip.
* Linux: The v4l2loopback kernel module must be compiled and loaded to the
  kernel already and the v4l2_file_player application must be compiled and put
  in the location specified below.

NOTICE: When running this script as a buildbot step, make sure to set
usePTY=False for the build step when adding it, or the subprocess will die as
soon the step has executed.
"""

import os
# psutil is not installed on non-Linux machines by default.
import psutil  # pylint: disable=F0401
import subprocess
import sys
import time


WEBCAM_WIN = ('schtasks', '/run', '/tn', 'ManyCam')
WEBCAM_MAC = ('open', '/Applications/ManyCam/ManyCam.app')
E = os.path.expandvars
WEBCAM_LINUX = (
    E('$HOME/fake-webcam-driver/linux/v4l2_file_player/v4l2_file_player'),
    E('$HOME/webrtc_video_quality/reference_video.yuv'),
    '640', '480', '/dev/video0',
)


def IsWebCamRunning():
  if sys.platform == 'win32':
    process_name = 'ManyCam.exe'
  elif sys.platform.startswith('darwin'):
    process_name = 'ManyCam'
  elif sys.platform.startswith('linux'):
    process_name = 'v4l2_file_player'
  else:
    raise Exception('Unsupported platform: %s' % sys.platform)
  for p in psutil.process_iter():
    try:
      if process_name == p.name:
        print 'Found a running virtual webcam (%s with PID %s)' % (p.name,
                                                                   p.pid)
        return True
    except psutil.AccessDenied:
      pass  # This is normal if we query sys processes, etc.
  return False


def StartWebCam():
  try:
    if sys.platform == 'win32':
      subprocess.check_call(WEBCAM_WIN)
      print 'Successfully launched virtual webcam.'
    elif sys.platform.startswith('darwin'):
      subprocess.check_call(WEBCAM_MAC)
      print 'Successfully launched virtual webcam.'
    elif sys.platform.startswith('linux'):

      # Must redirect stdout/stderr/stdin to avoid having the subprocess
      # being killed when the parent shell dies (happens on the bots).
      process = subprocess.Popen(WEBCAM_LINUX, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE,
                                 stdin=subprocess.PIPE)
      # If the v4l2loopback module is not loaded or incorrectly configured,
      # the process will still launch but will die immediately.
      # Wait for a second and then check for aliveness to catch such errors.
      time.sleep(1)
      if process.poll() is None:
        print 'Successfully launched virtual webcam with PID %s' % process.pid
      else:
        print 'Failed to launch virtual webcam.'
        return False

  except Exception as e:
    print 'Failed to launch virtual webcam: %s' % e
    return False

  return True


def Main(argv):
  if IsWebCamRunning():
    return 0
  if not StartWebCam():
    return 1

  if argv:
    subprocess.check_call(argv)


if __name__ == '__main__':
  sys.exit(Main(sys.argv[1:]))
