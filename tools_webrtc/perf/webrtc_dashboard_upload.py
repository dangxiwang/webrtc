#!/usr/bin/env python
# Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
#
# Use of this source code is governed by a BSD-style license
# that can be found in the LICENSE file in the root of the source
# tree. An additional intellectual property rights grant can be found
# in the file PATENTS.  All contributing project authors may
# be found in the AUTHORS file in the root of the source tree.

"""Converts and uploads results to the Chrome perf dashboard.

This conversion step is needed because test/testsupport/perf_test.cc can't
output histograms natively. There is, unfortunately, no C++ API for histograms.
This script is in python so it can depend on Catapult's python API instead.
See histogram_util.py for how this is done. We should move to the C++ API and
delete the scripts in this dir as soon as there is a C++ API (less conversions =
easier to understand).

This script can't be in recipes, because we can't access the catapult APIs from
there. It needs to be here source-side.

This script is adapted from the downstream variant like this:
  * Follows upstream naming conventions.
  * Downstream-only parameters and concepts go away.
  * oAuth tokens are generated by luci-auth.
"""

import argparse
import httplib2
import json
import sys
import zlib

import histogram_util


def _SendHistogramSetJson(url, histogram_json, oauth_token_file):
  """Make a HTTP POST with the given JSON to the Performance Dashboard.

  Args:
    url: URL of Performance Dashboard instance, e.g.
        "https://chromeperf.appspot.com".
    histogram_json: a JSON object that contains the data to be sent.
    oauth_token_file: Optional path to file with OAuth token string to pass
        to server. If specified, the request will be sent as authorised thus
        not requiring sender IP to be whitelisted.
  """
  headers = {}
  if oauth_token_file:
    with open(oauth_token_file) as oauth_token_fd:
      headers['Authorization'] = 'Bearer %s' % oauth_token_fd.read()

  serialized = json.dumps(histogram_json.AsDicts(), indent=4)
  data = zlib.compress(serialized)

  http = httplib2.Http()
  response, content = http.request(url + '/add_histograms', method='POST',
                                   body=data, headers=headers)
  return response, content


def _LoadHistogramSetJson(options):
  with options.input_results_file as f:
    json_data = json.load(f)

  histograms = histogram_util.LoadHistograms(json_data)
  hs = histogram_util.MakeWebRtcHistogramSet(
      stats=histograms,
      commit_pos=options.commit_position,
      commit_hash=options.webrtc_git_hash,
      master=options.perf_dashboard_machine_group,
      bot=options.bot,
      test_suite=options.test_suite,
      build_url=options.build_page_url)

  return hs


def _CreateParser():
  parser = argparse.ArgumentParser()
  parser.add_argument('--oauth-token-file',
                      help='File with oauth token string to pass to server')
  parser.add_argument('--perf-dashboard-machine-group', required=True,
                      help='The "master" the bots are grouped under. This '
                           'string is the group in the the perf dashboard path '
                            'group/bot/perf_id/metric/subtest.')
  parser.add_argument('--bot', required=True,
                      help='The bot running the test (e.g. '
                           'webrtc-win-large-tests).')
  parser.add_argument('--test-suite', required=True,
                      help='The key for the test in the dashboard (i.e. what '
                      'you select in the top-level test suite selector in the '
                      'dashboard')
  parser.add_argument('--webrtc-git-hash', required=True,
                      help='webrtc.googlesource.com commit hash.')
  parser.add_argument('--commit-position', type=int, required=True,
                      help='Commit pos corresponding to the git hash.')
  parser.add_argument('--build-page-url', required=True,
                      help='URL to the build page for this build.')
  parser.add_argument('--dashboard-url', required=True,
                      help='Which dashboard to use.')
  parser.add_argument('--input-results-file', type=argparse.FileType(),
                      required=True,
                      help='A JSON file with output from WebRTC tests.')
  parser.add_argument('--output-json-file', type=argparse.FileType('w'),
                      help='Where to write the output (for debugging).')
  return parser


def main(args):
  parser = _CreateParser()
  options = parser.parse_args(args)

  histogram_json = _LoadHistogramSetJson(options)

  if options.output_json_file:
    with options.output_json_file as output_file:
      json.dump(histogram_json.AsDicts(), output_file, indent=4)

  response, content = _SendHistogramSetJson(
      options.dashboard_url, histogram_json, options.oauth_token_file)

  if response.status == 200:
    return 0
  else:
    print("Upload failed with %d: %s\n\n%s" % (response.status, response.reason,
                                               content))
    return 1


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
