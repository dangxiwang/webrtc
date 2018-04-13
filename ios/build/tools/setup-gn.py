#!/usr/bin/python
# Copyright 2016 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import convert_gn_xcodeproj
import errno
import os
import re
import shutil
import subprocess
import sys
import tempfile
import ConfigParser

try:
  import cStringIO as StringIO
except ImportError:
  import StringIO


SUPPORTED_TARGETS = ('iphoneos', 'iphonesimulator')
SUPPORTED_CONFIGS = ('Debug', 'Release', 'Profile', 'Official', 'Coverage')


class ConfigParserWithStringInterpolation(ConfigParser.SafeConfigParser):

  '''A .ini file parser that supports strings and environment variables.'''

  ENV_VAR_PATTERN = re.compile('\$([A-Za-z0-9_]+)')

  def values(self, section):
    return map(
        lambda (k, v): self._UnquoteString(self._ExpandEnvVar(v)),
        ConfigParser.SafeConfigParser.items(self, section))

  def getstring(self, section, option):
    return self._UnquoteString(self._ExpandEnvVar(self.get(section, option)))

  def _UnquoteString(self, string):
    if not string or string[0] != '"' or string[-1] != '"':
      return string
    return string[1:-1]

  def _ExpandEnvVar(self, value):
    match = self.ENV_VAR_PATTERN.search(value)
    if not match:
      return value
    name, (begin, end) = match.group(1), match.span(0)
    prefix, suffix = value[:begin], self._ExpandEnvVar(value[end:])
    return prefix + os.environ.get(name, '') + suffix

class GnGenerator(object):

  '''Holds configuration for a build and method to generate gn default files.'''

  FAT_BUILD_DEFAULT_ARCH = '64-bit'

  TARGET_CPU_VALUES = {
    'iphoneos': {
      '32-bit': '"arm"',
      '64-bit': '"arm64"',
    },
    'iphonesimulator': {
      '32-bit': '"x86"',
      '64-bit': '"x64"',
    }
  }

  def __init__(self, settings, config, target):
    assert target in SUPPORTED_TARGETS
    assert config in SUPPORTED_CONFIGS
    self._settings = settings
    self._config = config
    self._target = target

  def _GetGnArgs(self):
    """Build the list of arguments to pass to gn.

    Returns:
      A list of tuple containing gn variable names and variable values (it
      is not a dictionary as the order needs to be preserved).
    """
    args = []

    if self._settings.getboolean('goma', 'enabled'):
      args.append(('use_goma', True))
      goma_dir = self._settings.getstring('goma', 'install')
      if goma_dir:
        args.append(('goma_dir', '"%s"' % os.path.expanduser(goma_dir)))

    args.append(('is_debug', self._config in ('Debug', 'Coverage')))
    args.append(('enable_dsyms', self._config in ('Profile', 'Official')))
    args.append(('enable_stripping', 'enable_dsyms'))
    args.append(('is_official_build', self._config == 'Official'))
    args.append(('is_chrome_branded', 'is_official_build'))
    args.append(('ios_enable_coverage', self._config == 'Coverage'))

    # TODO(crbug.com/75794): the version of llvm-cov used for code coverage is
    # tied to the version of clang used. As no copy of llvm-cov is shipped with
    # the Chrome's clang, the version shipped with Xcode is used, thus code
    # needs to be compiled with Xcode's clang when enabling code coverage.
    # Remove this once llvm-cov is shipped with Chrome's clang.
    args.append(('use_xcode_clang', 'is_official_build || ios_enable_coverage'))

    if os.environ.get('FORCE_MAC_TOOLCHAIN', '0') == '1':
      args.append(('use_system_xcode', False))

    cpu_values = self.TARGET_CPU_VALUES[self._target]
    build_arch = self._settings.getstring('build', 'arch')
    if build_arch == 'fat':
      target_cpu = cpu_values[self.FAT_BUILD_DEFAULT_ARCH]
      args.append(('target_cpu', target_cpu))
      args.append(('additional_target_cpus',
          [cpu for cpu in cpu_values.itervalues() if cpu != target_cpu]))
    else:
      args.append(('target_cpu', cpu_values[build_arch]))

    # Add user overrides after the other configurations so that they can
    # refer to them and override them.
    args.extend(self._settings.items('gn_args'))
    return args


  def Generate(self, gn_path, root_path, out_path):
    buf = StringIO.StringIO()
    self.WriteArgsGn(buf)
    WriteToFileIfChanged(
        os.path.join(out_path, 'args.gn'),
        buf.getvalue(),
        overwrite=True)

    subprocess.check_call(
        self.GetGnCommand(gn_path, root_path, out_path, True))

  def CreateGnRules(self, gn_path, root_path, out_path):
    buf = StringIO.StringIO()
    self.WriteArgsGn(buf)
    WriteToFileIfChanged(
        os.path.join(out_path, 'args.gn'),
        buf.getvalue(),
        overwrite=True)

    buf = StringIO.StringIO()
    gn_command = self.GetGnCommand(gn_path, root_path, out_path, False)
    self.WriteBuildNinja(buf, gn_command)
    WriteToFileIfChanged(
        os.path.join(out_path, 'build.ninja'),
        buf.getvalue(),
        overwrite=False)

    buf = StringIO.StringIO()
    self.WriteBuildNinjaDeps(buf)
    WriteToFileIfChanged(
        os.path.join(out_path, 'build.ninja.d'),
        buf.getvalue(),
        overwrite=False)

  def WriteArgsGn(self, stream):
    stream.write('# This file was generated by setup-gn.py. Do not edit\n')
    stream.write('# but instead use ~/.setup-gn or $repo/.setup-gn files\n')
    stream.write('# to configure settings.\n')
    stream.write('\n')

    if self._settings.has_section('$imports$'):
      for import_rule in self._settings.values('$imports$'):
        stream.write('import("%s")\n' % import_rule)
      stream.write('\n')

    gn_args = self._GetGnArgs()
    for name, value in gn_args:
      if isinstance(value, bool):
        stream.write('%s = %s\n' % (name, str(value).lower()))
      elif isinstance(value, list):
        stream.write('%s = [%s' % (name, '\n' if len(value) > 1 else ''))
        if len(value) == 1:
          prefix = ' '
          suffix = ' '
        else:
          prefix = '  '
          suffix = ',\n'
        for item in value:
          if isinstance(item, bool):
            stream.write('%s%s%s' % (prefix, str(item).lower(), suffix))
          else:
            stream.write('%s%s%s' % (prefix, item, suffix))
        stream.write(']\n')
      else:
        stream.write('%s = %s\n' % (name, value))

  def WriteBuildNinja(self, stream, gn_command):
    stream.write('rule gn\n')
    stream.write('  command = %s\n' % NinjaEscapeCommand(gn_command))
    stream.write('  description = Regenerating ninja files\n')
    stream.write('\n')
    stream.write('build build.ninja: gn\n')
    stream.write('  generator = 1\n')
    stream.write('  depfile = build.ninja.d\n')

  def WriteBuildNinjaDeps(self, stream):
    stream.write('build.ninja: nonexistant_file.gn\n')

  def GetGnCommand(self, gn_path, src_path, out_path, generate_xcode_project):
    gn_command = [ gn_path, '--root=%s' % os.path.realpath(src_path), '-q' ]
    if generate_xcode_project:
      gn_command.append('--ide=xcode')
      gn_command.append('--root-target=gn_all')
      if self._settings.getboolean('goma', 'enabled'):
        ninja_jobs = self._settings.getint('xcode', 'jobs') or 200
        gn_command.append('--ninja-extra-args=-j%s' % ninja_jobs)
      if self._settings.has_section('filters'):
        target_filters = self._settings.values('filters')
        if target_filters:
          gn_command.append('--filters=%s' % ';'.join(target_filters))
    else:
      gn_command.append('--check')
    gn_command.append('gen')
    gn_command.append('//%s' %
        os.path.relpath(os.path.abspath(out_path), os.path.abspath(src_path)))
    return gn_command


def WriteToFileIfChanged(filename, content, overwrite):
  '''Write |content| to |filename| if different. If |overwrite| is False
  and the file already exists it is left untouched.'''
  if os.path.exists(filename):
    if not overwrite:
      return
    with open(filename) as file:
      if file.read() == content:
        return
  if not os.path.isdir(os.path.dirname(filename)):
    os.makedirs(os.path.dirname(filename))
  with open(filename, 'w') as file:
    file.write(content)


def NinjaNeedEscape(arg):
  '''Returns True if |arg| needs to be escaped when written to .ninja file.'''
  return ':' in arg or '*' in arg or ';' in arg


def NinjaEscapeCommand(command):
  '''Escapes |command| in order to write it to .ninja file.'''
  result = []
  for arg in command:
    if NinjaNeedEscape(arg):
      arg = arg.replace(':', '$:')
      arg = arg.replace(';', '\\;')
      arg = arg.replace('*', '\\*')
    else:
      result.append(arg)
  return ' '.join(result)


def FindGn():
  '''Returns absolute path to gn binary looking at the PATH env variable.'''
  for path in os.environ['PATH'].split(os.path.pathsep):
    gn_path = os.path.join(path, 'gn')
    if os.path.isfile(gn_path) and os.access(gn_path, os.X_OK):
      return gn_path
  return None


def GenerateXcodeProject(gn_path, root_dir, out_dir, settings):
  '''Convert GN generated Xcode project into multi-configuration Xcode
  project.'''

  temp_path = tempfile.mkdtemp(prefix=os.path.abspath(
      os.path.join(out_dir, '_temp')))
  try:
    generator = GnGenerator(settings, 'Debug', 'iphonesimulator')
    generator.Generate(gn_path, root_dir, temp_path)
    convert_gn_xcodeproj.ConvertGnXcodeProject(
        os.path.join(temp_path),
        os.path.join(out_dir, 'build'),
        SUPPORTED_CONFIGS)
  finally:
    if os.path.exists(temp_path):
      shutil.rmtree(temp_path)


def GenerateGnBuildRules(gn_path, root_dir, out_dir, settings):
  '''Generates all template configurations for gn.'''
  for config in SUPPORTED_CONFIGS:
    for target in SUPPORTED_TARGETS:
      build_dir = os.path.join(out_dir, '%s-%s' % (config, target))
      generator = GnGenerator(settings, config, target)
      generator.CreateGnRules(gn_path, root_dir, build_dir)


def Main(args):
  default_root = os.path.normpath(os.path.join(
      os.path.dirname(__file__), os.pardir, os.pardir, os.pardir))

  parser = argparse.ArgumentParser(
      description='Generate build directories for use with gn.')
  parser.add_argument(
      'root', default=default_root, nargs='?',
      help='root directory where to generate multiple out configurations')
  parser.add_argument(
      '--import', action='append', dest='import_rules', default=[],
      help='path to file defining default gn variables')
  args = parser.parse_args(args)

  # Load configuration (first global and then any user overrides).
  settings = ConfigParserWithStringInterpolation()
  settings.read([
      os.path.splitext(__file__)[0] + '.config',
      os.path.expanduser('~/.setup-gn'),
  ])

  # Add private sections corresponding to --import argument.
  if args.import_rules:
    settings.add_section('$imports$')
    for i, import_rule in enumerate(args.import_rules):
      if not import_rule.startswith('//'):
        import_rule = '//%s' % os.path.relpath(
            os.path.abspath(import_rule), os.path.abspath(args.root))
      settings.set('$imports$', '$rule%d$' % i, import_rule)

  # Validate settings.
  if settings.getstring('build', 'arch') not in ('64-bit', '32-bit', 'fat'):
    sys.stderr.write('ERROR: invalid value for build.arch: %s\n' %
        settings.getstring('build', 'arch'))
    sys.exit(1)

  if settings.getboolean('goma', 'enabled'):
    if settings.getint('xcode', 'jobs') < 0:
      sys.stderr.write('ERROR: invalid value for xcode.jobs: %s\n' %
          settings.get('xcode', 'jobs'))
      sys.exit(1)
    goma_install = os.path.expanduser(settings.getstring('goma', 'install'))
    if not os.path.isdir(goma_install):
      sys.stderr.write('WARNING: goma.install directory not found: %s\n' %
          settings.get('goma', 'install'))
      sys.stderr.write('WARNING: disabling goma\n')
      settings.set('goma', 'enabled', 'false')

  # Find gn binary in PATH.
  gn_path = FindGn()
  if gn_path is None:
    sys.stderr.write('ERROR: cannot find gn in PATH\n')
    sys.exit(1)

  out_dir = os.path.join(args.root, 'out')
  if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

  GenerateXcodeProject(gn_path, args.root, out_dir, settings)
  GenerateGnBuildRules(gn_path, args.root, out_dir, settings)


if __name__ == '__main__':
  sys.exit(Main(sys.argv[1:]))
