#!/usr/bin/env vpython3

# Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
#
# Use of this source code is governed by a BSD-style license
# that can be found in the LICENSE file in the root of the source
# tree. An additional intellectual property rights grant can be found
# in the file PATENTS.  All contributing project authors may
# be found in the AUTHORS file in the root of the source tree.
"""Script to generate libwebrtc.aar for distribution.

The script has to be run from the root src folder.
./tools_webrtc/android/build_aar.py

.aar-file is just a zip-archive containing the files of the library. The file
structure generated by this script looks like this:
 - AndroidManifest.xml
 - classes.jar
 - libs/
   - armeabi-v7a/
     - libjingle_peerconnection_so.so
   - x86/
     - libjingle_peerconnection_so.so
"""

import argparse
import logging
import os
import shutil
import subprocess
import sys
import tempfile
import zipfile

SCRIPT_DIR = os.path.dirname(os.path.realpath(sys.argv[0]))
SRC_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, os.pardir, os.pardir))
DEFAULT_ARCHS = ['armeabi-v7a', 'arm64-v8a', 'x86', 'x86_64']
NEEDED_SO_FILES = ['libjingle_peerconnection_so.so']
JAR_FILE = 'lib.java/sdk/android/libwebrtc.jar'
MANIFEST_FILE = 'sdk/android/AndroidManifest.xml'
TARGETS = [
    'sdk/android:libwebrtc',
    'sdk/android:libjingle_peerconnection_so',
]

sys.path.append(os.path.join(SCRIPT_DIR, '..', 'libs'))
from generate_licenses import LicenseBuilder

sys.path.append(os.path.join(SRC_DIR, 'build'))
import find_depot_tools


def _ParseArgs():
  parser = argparse.ArgumentParser(description='libwebrtc.aar generator.')
  parser.add_argument(
      '--build-dir',
      type=os.path.abspath,
      help='Build dir. By default will create and use temporary dir.')
  parser.add_argument('--output',
                      default='libwebrtc.aar',
                      type=os.path.abspath,
                      help='Output file of the script.')
  parser.add_argument('--arch',
                      default=DEFAULT_ARCHS,
                      nargs='*',
                      help='Architectures to build. Defaults to %(default)s.')
  parser.add_argument('--use-goma',
                      action='store_true',
                      default=False,
                      help='Use goma.')
  parser.add_argument('--use-rbe',
                      action='store_true',
                      default=False,
                      help='Use RBE.')
  parser.add_argument('--use-unstripped-libs',
                      action='store_true',
                      default=False,
                      help='Use unstripped .so files within libwebrtc.aar')
  parser.add_argument('--verbose',
                      action='store_true',
                      default=False,
                      help='Debug logging.')
  parser.add_argument(
      '--extra-gn-args',
      default=[],
      nargs='*',
      help="""Additional GN arguments to be used during Ninja generation.
              These are passed to gn inside `--args` switch and
              applied after any other arguments and will
              override any values defined by the script.
              Example of building debug aar file:
              build_aar.py --extra-gn-args='is_debug=true'""")
  parser.add_argument(
      '--extra-ninja-switches',
      default=[],
      nargs='*',
      help="""Additional Ninja switches to be used during compilation.
              These are applied after any other Ninja switches.
              Example of enabling verbose Ninja output:
              build_aar.py --extra-ninja-switches='-v'""")
  parser.add_argument(
      '--extra-gn-switches',
      default=[],
      nargs='*',
      help="""Additional GN switches to be used during compilation.
              These are applied after any other GN switches.
              Example of enabling verbose GN output:
              build_aar.py --extra-gn-switches='-v'""")
  return parser.parse_args()


def _RunGN(args):
  cmd = [
      sys.executable,
      os.path.join(find_depot_tools.DEPOT_TOOLS_PATH, 'gn.py')
  ]
  cmd.extend(args)
  logging.debug('Running: %r', cmd)
  subprocess.check_call(cmd)


def _RunNinja(output_directory, args):
  cmd = [
      os.path.join(find_depot_tools.DEPOT_TOOLS_PATH, 'ninja'), '-C',
      output_directory
  ]
  cmd.extend(args)
  logging.debug('Running: %r', cmd)
  subprocess.check_call(cmd)


def _EncodeForGN(value):
  """Encodes value as a GN literal."""
  if isinstance(value, str):
    return '"' + value + '"'
  if isinstance(value, bool):
    return repr(value).lower()
  return repr(value)


def _GetOutputDirectory(build_dir, arch):
  """Returns the GN output directory for the target architecture."""
  return os.path.join(build_dir, arch)


def _GetTargetCpu(arch):
  """Returns target_cpu for the GN build with the given architecture."""
  if arch in ['armeabi', 'armeabi-v7a']:
    return 'arm'
  if arch == 'arm64-v8a':
    return 'arm64'
  if arch == 'x86':
    return 'x86'
  if arch == 'x86_64':
    return 'x64'
  raise Exception('Unknown arch: ' + arch)


def _GetArmVersion(arch):
  """Returns arm_version for the GN build with the given architecture."""
  if arch == 'armeabi':
    return 6
  if arch == 'armeabi-v7a':
    return 7
  if arch in ['arm64-v8a', 'x86', 'x86_64']:
    return None
  raise Exception('Unknown arch: ' + arch)


def Build(build_dir, arch, use_goma, use_rbe, extra_gn_args, extra_gn_switches,
          extra_ninja_switches):
  """Generates target architecture using GN and builds it using ninja."""
  logging.info('Building: %s', arch)
  output_directory = _GetOutputDirectory(build_dir, arch)
  gn_args = {
      'target_os': 'android',
      'is_debug': False,
      'is_component_build': False,
      'rtc_include_tests': False,
      'target_cpu': _GetTargetCpu(arch),
      'use_goma': use_goma,
      'use_remoteexec': use_rbe,
  }
  arm_version = _GetArmVersion(arch)
  if arm_version:
    gn_args['arm_version'] = arm_version
  gn_args_str = '--args=' + ' '.join(
      [k + '=' + _EncodeForGN(v) for k, v in gn_args.items()] + extra_gn_args)

  gn_args_list = ['gen', output_directory, gn_args_str]
  gn_args_list.extend(extra_gn_switches)
  _RunGN(gn_args_list)

  ninja_args = TARGETS[:]
  if use_goma or use_rbe:
    ninja_args.extend(['-j', '200'])
  ninja_args.extend(extra_ninja_switches)
  _RunNinja(output_directory, ninja_args)


def CollectCommon(aar_file, build_dir, arch):
  """Collects architecture independent files into the .aar-archive."""
  logging.info('Collecting common files.')
  output_directory = _GetOutputDirectory(build_dir, arch)
  aar_file.write(MANIFEST_FILE, 'AndroidManifest.xml')
  aar_file.write(os.path.join(output_directory, JAR_FILE), 'classes.jar')


def Collect(aar_file, build_dir, arch, unstripped):
  """Collects architecture specific files into the .aar-archive."""
  logging.info('Collecting: %s', arch)
  output_directory = _GetOutputDirectory(build_dir, arch)

  abi_dir = os.path.join('jni', arch)
  for so_file in NEEDED_SO_FILES:
    source_so_file = os.path.join("lib.unstripped",
                                  so_file) if unstripped else so_file
    aar_file.write(os.path.join(output_directory, source_so_file),
                   os.path.join(abi_dir, so_file))


def GenerateLicenses(output_dir, build_dir, archs):
  builder = LicenseBuilder(
      [_GetOutputDirectory(build_dir, arch) for arch in archs], TARGETS)
  builder.GenerateLicenseText(output_dir)


def BuildAar(archs,
             output_file,
             use_goma=False,
             use_rbe=False,
             extra_gn_args=None,
             ext_build_dir=None,
             extra_gn_switches=None,
             extra_ninja_switches=None,
             unstripped=False):
  extra_gn_args = extra_gn_args or []
  extra_gn_switches = extra_gn_switches or []
  extra_ninja_switches = extra_ninja_switches or []
  build_dir = ext_build_dir if ext_build_dir else tempfile.mkdtemp()

  for arch in archs:
    Build(build_dir, arch, use_goma, use_rbe, extra_gn_args, extra_gn_switches,
          extra_ninja_switches)

  with zipfile.ZipFile(output_file, 'w') as aar_file:
    # Architecture doesn't matter here, arbitrarily using the first one.
    CollectCommon(aar_file, build_dir, archs[0])
    for arch in archs:
      Collect(aar_file, build_dir, arch, unstripped)

  license_dir = os.path.dirname(os.path.realpath(output_file))
  GenerateLicenses(license_dir, build_dir, archs)

  if not ext_build_dir:
    shutil.rmtree(build_dir, True)


def main():
  args = _ParseArgs()
  logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

  BuildAar(args.arch, args.output, args.use_goma, args.use_rbe,
           args.extra_gn_args, args.build_dir, args.extra_gn_switches,
           args.extra_ninja_switches, args.use_unstripped_libs)


if __name__ == '__main__':
  sys.exit(main())
