#!/usr/bin/env python
# Copyright 2017 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.


import argparse
import os
import string
import sys


SCRIPT_TEMPLATE = string.Template("""\
#!/usr/bin/env python
#
# This file was generated by build/android/gyp/create_apk_operations_script.py

import os
import sys

def main():
  script_directory = os.path.dirname(__file__)
  resolve = lambda p: p if p is None else os.path.abspath(os.path.join(
      script_directory, p))
  sys.path.append(resolve(${APK_OPERATIONS_DIR}))
  import apk_operations
  output_dir = resolve(${OUTPUT_DIR})
  try:
    apk_operations.Run(
        output_dir,
        resolve(${APK_PATH}),
        resolve(${INC_JSON_PATH}),
        ${FLAGS_FILE},
        ${TARGET_CPU},
        resolve(${MAPPING_PATH}))
  except TypeError:
    rel_output_dir = os.path.relpath(output_dir)
    rel_script_path = os.path.relpath(sys.argv[0], output_dir)
    sys.stderr.write('Script out-of-date. Rebuild via:\\n')
    sys.stderr.write('  ninja -C %s %s\\n' % (rel_output_dir, rel_script_path))
    return 1


if __name__ == '__main__':
  sys.exit(main())
""")


def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('--script-output-path',
                      help='Output path for executable script.')
  parser.add_argument('--apk-path')
  parser.add_argument('--incremental-install-json-path')
  parser.add_argument('--command-line-flags-file')
  parser.add_argument('--target-cpu')
  parser.add_argument('--proguard-mapping-path')
  args = parser.parse_args(args)

  def relativize(path):
    """Returns the path relative to the output script directory."""
    if path is None:
      return path
    return os.path.relpath(path, os.path.dirname(args.script_output_path))
  apk_operations_dir = os.path.join(os.path.dirname(__file__), os.path.pardir)
  apk_operations_dir = relativize(apk_operations_dir)

  with open(args.script_output_path, 'w') as script:
    script_dict = {
        'APK_OPERATIONS_DIR': repr(apk_operations_dir),
        'OUTPUT_DIR': repr(relativize('.')),
        'APK_PATH': repr(relativize(args.apk_path)),
        'INC_JSON_PATH': repr(relativize(args.incremental_install_json_path)),
        'MAPPING_PATH': repr(relativize(args.proguard_mapping_path)),
        'FLAGS_FILE': repr(args.command_line_flags_file),
        'TARGET_CPU': repr(args.target_cpu),
    }
    script.write(SCRIPT_TEMPLATE.substitute(script_dict))
  os.chmod(args.script_output_path, 0750)
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
