# Copyright 2014 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""binary_size presubmit script

See http://dev.chromium.org/developers/how-tos/depottools/presubmit-scripts
for more details about the presubmit API built into depot_tools.
"""

def CommonChecks(input_api, output_api):
  output = []
  output.extend(input_api.canned_checks.RunPylint(input_api, output_api))
  py_tests = input_api.canned_checks.GetUnitTestsRecursively(
      input_api, output_api, input_api.PresubmitLocalPath(),
      whitelist=[r'.+_test\.py$'], blacklist=[])

  output.extend(input_api.RunTests(py_tests, False))

  if input_api.is_committing:
    output.extend(input_api.canned_checks.PanProjectChecks(input_api,
                                                           output_api,
                                                           owners_check=False))
  return output


def CheckChangeOnUpload(input_api, output_api):
  return CommonChecks(input_api, output_api)


def CheckChangeOnCommit(input_api, output_api):
  return CommonChecks(input_api, output_api)
