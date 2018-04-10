#!/usr/bin/env python
#
# Copyright 2014 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Pack relocations in a library (or copy unchanged).

If --enable-packing and --configuration-name=='Release', invoke the
relocation_packer tool to pack the .rel.dyn or .rela.dyn section in the given
library files.  This step is inserted after the libraries are stripped.

If --enable-packing is zero, the script copies files verbatim, with no
attempt to pack relocations.
"""

import ast
import optparse
import os
import shutil
import sys
import tempfile

from util import build_utils

def PackLibraryRelocations(android_pack_relocations, library_path, output_path):
  shutil.copy(library_path, output_path)
  pack_command = [android_pack_relocations, output_path]
  build_utils.CheckOutput(pack_command)


def CopyLibraryUnchanged(library_path, output_path):
  shutil.copy(library_path, output_path)


def main(args):
  args = build_utils.ExpandFileArgs(args)
  parser = optparse.OptionParser()
  build_utils.AddDepfileOption(parser)
  parser.add_option('--clear-dir', action='store_true',
                    help='If set, the destination directory will be deleted '
                    'before copying files to it. This is highly recommended to '
                    'ensure that no stale files are left in the directory.')

  parser.add_option('--configuration-name',
      default='Release',
      help='Gyp configuration name (i.e. Debug, Release)')
  parser.add_option('--enable-packing',
      choices=['0', '1'],
      help=('Pack relocations if 1 and configuration name is \'Release\','
            ' otherwise plain file copy'))
  parser.add_option('--android-pack-relocations',
      help='Path to the relocations packer binary')
  parser.add_option('--stripped-libraries-dir',
      help='Directory for stripped libraries')
  parser.add_option('--packed-libraries-dir',
      help='Directory for packed libraries')
  parser.add_option('--libraries', action='append',
      help='List of libraries in Python dictionary format')
  parser.add_option('--stamp', help='Path to touch on success')
  parser.add_option('--filelistjson',
                    help='Output path of filelist.json to write')

  options, _ = parser.parse_args(args)
  enable_packing = (options.enable_packing == '1' and
                    options.configuration_name == 'Release')

  libraries = []
  for libs_arg in options.libraries:
    libraries += ast.literal_eval(libs_arg)

  if options.clear_dir:
    build_utils.DeleteDirectory(options.packed_libraries_dir)

  build_utils.MakeDirectory(options.packed_libraries_dir)

  output_paths = []
  for library in libraries:
    library_path = os.path.join(options.stripped_libraries_dir, library)
    output_path = os.path.join(
        options.packed_libraries_dir, os.path.basename(library))
    output_paths.append(output_path)

    if enable_packing:
      PackLibraryRelocations(options.android_pack_relocations,
                             library_path,
                             output_path)
    else:
      CopyLibraryUnchanged(library_path, output_path)

  if options.filelistjson:
    build_utils.WriteJson({ 'files': output_paths }, options.filelistjson)
    output_paths.append(options.filelistjson)

  if options.depfile:
    build_utils.WriteDepfile(options.depfile, output_paths[-1], libraries)

  if options.stamp:
    build_utils.Touch(options.stamp)

  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
