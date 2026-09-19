"""Build a native runner against this checkout, with an isolated precision build."""
import argparse
import os
import pathlib
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[2]


def build(precision, jobs):
  out = ROOT / ('build-linkage-' + precision)
  flags = '-DmjUSESINGLE' if precision == 'single' else ''
  configure = ['cmake', '-S', str(ROOT), '-B', str(out), '-G', 'Ninja',
               '-DCMAKE_BUILD_TYPE=Release', '-DMUJOCO_BUILD_EXAMPLES=OFF',
               '-DMUJOCO_BUILD_SIMULATE=OFF', '-DMUJOCO_BUILD_TESTS=OFF',
               '-DMUJOCO_BUILD_STUDIO=OFF', '-DMUJOCO_BUILD_MACOS_FRAMEWORKS=OFF',
               '-DCMAKE_C_FLAGS=' + flags, '-DCMAKE_CXX_FLAGS=' + flags]
  # Reuse fetched sources, but never build products, across these two task builds.
  deps = ROOT / 'build-linkage-double/_deps'
  if precision == 'single' and deps.exists():
    configure += [f'-DFETCHCONTENT_SOURCE_DIR_{p.name[:-4].upper()}={p}'
                  for p in deps.glob('*-src')]
  subprocess.run(configure, check=True)
  subprocess.run(['cmake', '--build', str(out), '--target', 'mujoco', '-j', str(jobs)], check=True)
  command = [os.environ.get('CXX', 'c++'), '-O2', '-std=c++17', '-I' + str(ROOT/'include'),
             str(ROOT/'experiments/linkage_lab/runner.cc'), '-L' + str(out/'lib'),
             '-lmujoco', '-Wl,-rpath,' + str(out/'lib'), '-o', str(out/'linkage_runner')]
  if flags:
    command.insert(1, flags)
  subprocess.run(command, check=True)
  print(out/'linkage_runner')


if __name__ == '__main__':
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('precision', choices=['double', 'single'])
  p.add_argument('--jobs', type=int, default=6)
  a = p.parse_args()
  build(a.precision, a.jobs)
