#!/usr/bin/python
#
# Copyright 2015 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Loops Custom Tabs tests and outputs the results into a CSV file."""

import collections
import contextlib
import logging
import optparse
import os
import random
import re
import sys
import time

_SRC_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir, os.pardir, os.pardir))

sys.path.append(os.path.join(_SRC_PATH, 'third_party', 'catapult', 'devil'))
from devil.android import device_errors
from devil.android import device_utils
from devil.android import flag_changer
from devil.android.perf import cache_control
from devil.android.sdk import intent

sys.path.append(os.path.join(_SRC_PATH, 'build', 'android'))
import devil_chromium

sys.path.append(os.path.join(_SRC_PATH, 'tools', 'android', 'loading'))
import chrome_setup


# Local build of Chrome (not Chromium).
_CHROME_PACKAGE = 'com.google.android.apps.chrome'
_COMMAND_LINE_FILE = 'chrome-command-line'
_TEST_APP_PACKAGE_NAME = 'org.chromium.customtabs.test'
_INVALID_VALUE = -1


def RunOnce(device, url, speculated_url, warmup, skip_launcher_activity,
            speculation_mode, delay_to_may_launch_url, delay_to_launch_url,
            cold, chrome_args, reset_chrome_state):
  """Runs a test on a device once.

  Args:
    device: (DeviceUtils) device to run the tests on.
    url: (str) URL to load.
    speculated_url: (str) Speculated URL.
    warmup: (bool) Whether to call warmup.
    skip_launcher_activity: (bool) Whether to skip the launcher activity.
    speculation_mode: (str) Speculation Mode.
    delay_to_may_launch_url: (int) Delay to mayLaunchUrl() in ms.
    delay_to_launch_url: (int) Delay to launchUrl() in ms.
    cold: (bool) Whether the page cache should be dropped.
    chrome_args: ([str]) List of arguments to pass to Chrome.
    reset_chrome_state: (bool) Whether to reset the Chrome local state before
                        the run.

  Returns:
    The output line (str), like this (one line only):
    <warmup>,<prerender_mode>,<delay_to_may_launch_url>,<delay_to_launch>,
      <intent_sent_ms>,<page_load_started_ms>,<page_load_finished_ms>,
      <first_contentful_paint>
    or None on error.
  """
  if not device.HasRoot():
    device.EnableRoot()

  timeout_s = 20
  logcat_timeout = int(timeout_s + delay_to_may_launch_url / 1000.
                       + delay_to_launch_url / 1000.) + 3;

  with flag_changer.CustomCommandLineFlags(
      device, _COMMAND_LINE_FILE, chrome_args):
    launch_intent = intent.Intent(
        action='android.intent.action.MAIN',
        package=_TEST_APP_PACKAGE_NAME,
        activity='org.chromium.customtabs.test.MainActivity',
        extras={'url': str(url),
                'speculated_url': str(speculated_url),
                'warmup': warmup,
                'skip_launcher_activity': skip_launcher_activity,
                'speculation_mode': str(speculation_mode),
                'delay_to_may_launch_url': delay_to_may_launch_url,
                'delay_to_launch_url': delay_to_launch_url,
                'timeout': timeout_s})
    result_line_re = re.compile(r'CUSTOMTABSBENCH.*: (.*)')
    logcat_monitor = device.GetLogcatMonitor(clear=True)
    logcat_monitor.Start()
    device.ForceStop(_CHROME_PACKAGE)
    device.ForceStop(_TEST_APP_PACKAGE_NAME)

    if reset_chrome_state:
      chrome_setup.ResetChromeLocalState(device, _CHROME_PACKAGE)

    if cold:
      cache_control.CacheControl(device).DropRamCaches()

    device.StartActivity(launch_intent, blocking=True)

    match = None
    try:
      match = logcat_monitor.WaitFor(result_line_re, timeout=logcat_timeout)
    except device_errors.CommandTimeoutError as _:
      logging.warning('Timeout waiting for the result line')
    logcat_monitor.Stop()
    logcat_monitor.Close()
    return match.group(1) if match is not None else None


RESULT_FIELDS = ('warmup', 'skip_launcher_activity', 'speculation_mode',
                 'delay_to_may_launch_url', 'delay_to_launch_url', 'commit',
                 'plt', 'first_contentful_paint')
Result = collections.namedtuple('Result', RESULT_FIELDS)


def ParseResult(result_line):
  """Parses a result line, and returns it.

  Args:
    result_line: (str) A result line, as returned by RunOnce().

  Returns:
    An instance of Result.
  """
  tokens = result_line.strip().split(',')
  assert len(tokens) == 9
  intent_sent_timestamp = int(tokens[5])
  return Result(int(tokens[0]), int(tokens[1]), tokens[2], int(tokens[3]),
                int(tokens[4]),
                max(_INVALID_VALUE, int(tokens[6]) - intent_sent_timestamp),
                max(_INVALID_VALUE, int(tokens[7]) - intent_sent_timestamp),
                max(_INVALID_VALUE, int(tokens[8]) - intent_sent_timestamp))


def LoopOnDevice(device, configs, output_filename, once=False,
                 should_stop=None):
  """Loops the tests on a device.

  Args:
    device: (DeviceUtils) device to run the tests on.
    configs: ([dict])
    output_filename: (str) Output filename. '-' for stdout.
    once: (bool) Run only once.
    should_stop: (threading.Event or None) When the event is set, stop looping.
  """
  to_stdout = output_filename == '-'
  out = sys.stdout if to_stdout else open(output_filename, 'a')
  try:
    while should_stop is None or not should_stop.is_set():
      config = configs[random.randint(0, len(configs) - 1)]
      chrome_args = chrome_setup.CHROME_ARGS
      if config['speculation_mode'] == 'no_state_prefetch':
        # NoStatePrefetch is enabled through an experiment.
        chrome_args.extend([
            '--force-fieldtrials=trial/group',
            '--force-fieldtrial-params=trial.group:mode/no_state_prefetch',
            '--enable-features=NoStatePrefetch<trial'])
      elif config['speculation_mode'] == 'speculative_prefetch':
        # Speculative Prefetch is enabled through an experiment.
        chrome_args.extend([
            '--force-fieldtrials=trial/group',
            '--force-fieldtrial-params=trial.group:mode/external-prefetching',
            '--enable-features=SpeculativeResourcePrefetching<trial'])

      result = RunOnce(device, config['url'], config['speculated_url'],
                       config['warmup'], config['skip_launcher_activity'],
                       config['speculation_mode'],
                       config['delay_to_may_launch_url'],
                       config['delay_to_launch_url'], config['cold'],
                       chrome_args, reset_chrome_state=True)
      if result is not None:
        out.write(result + '\n')
        out.flush()
      if once:
        return
      if should_stop is not None:
        should_stop.wait(10.)
      else:
        time.sleep(10)
  finally:
    if not to_stdout:
      out.close()


def ProcessOutput(filename):
  """Reads an output file, and returns a processed numpy array.

  Args:
    filename: (str) file to process.

  Returns:
    A numpy structured array.
  """
  import numpy as np
  entries = []
  with open(filename, 'r') as f:
    lines = f.readlines()
    entries = [ParseResult(line) for line in lines]
  result = np.array(entries,
                    dtype=[('warmup', np.int32),
                           ('skip_launcher_activity', np.int32),
                           ('speculation_mode', str),
                           ('delay_to_may_launch_url', np.int32),
                           ('delay_to_launch_url', np.int32),
                           ('commit', np.int32), ('plt', np.int32),
                           ('first_contentful_paint', np.int32)])
  return result


def _CreateOptionParser():
  parser = optparse.OptionParser(description='Loops Custom Tabs tests on a '
                                 'device, and outputs the navigation timings '
                                 'in a CSV file.')
  parser.add_option('--device', help='Device ID')
  parser.add_option('--speculated_url',
                    help='URL to call mayLaunchUrl() with.',)
  parser.add_option('--url', help='URL to navigate to.',
                    default='https://www.android.com')
  parser.add_option('--warmup', help='Call warmup.', default=False,
                    action='store_true')
  parser.add_option('--skip_launcher_activity',
                    help='Skip ChromeLauncherActivity.', default=False,
                    action='store_true')
  parser.add_option('--speculation_mode', default='prerender',
                    help='The speculation mode (prerender, '
                    'speculative_prefetch or no_state_prefetch).',
                    choices=['disabled', 'prerender', 'hidden_tab'])
  parser.add_option('--delay_to_may_launch_url',
                    help='Delay before calling mayLaunchUrl() in ms.',
                    type='int', default=1000)
  parser.add_option('--delay_to_launch_url',
                    help='Delay before calling launchUrl() in ms.',
                    type='int', default=-1)
  parser.add_option('--cold', help='Purge the page cache before each run.',
                    default=False, action='store_true')
  parser.add_option('--output_file', help='Output file (append). "-" for '
                    'stdout (this is the default)', default='-')
  parser.add_option('--once', help='Run only one iteration.',
                    action='store_true', default=False)

  return parser


def main():
  parser = _CreateOptionParser()
  options, _ = parser.parse_args()
  devil_chromium.Initialize()
  devices = device_utils.DeviceUtils.HealthyDevices()
  device = devices[0]
  if len(devices) != 1 and options.device is None:
    logging.error('Several devices attached, must specify one with --device.')
    sys.exit(0)
  if options.device is not None:
    matching_devices = [d for d in devices if str(d) == options.device]
    if not matching_devices:
      logging.error('Device not found.')
      sys.exit(0)
    device = matching_devices[0]

  config = {
      'url': options.url,
      'skip_launcher_activity': options.skip_launcher_activity,
      'speculated_url': options.speculated_url or options.url,
      'warmup': options.warmup,
      'speculation_mode': options.speculation_mode,
      'delay_to_may_launch_url': options.delay_to_may_launch_url,
      'delay_to_launch_url': options.delay_to_launch_url,
      'cold': options.cold,
  }
  LoopOnDevice(device, [config], options.output_file, once=options.once)


if __name__ == '__main__':
  main()
