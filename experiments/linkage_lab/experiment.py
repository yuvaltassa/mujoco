"""Reproduce selected groups or the complete first-pass experiment."""
import argparse
import json
import pathlib

from audit import audit
from generate import generate
from sweep import run

BASE = pathlib.Path(__file__).resolve().parent
ROOT = BASE.parents[1]
# group: model variant, precision, timesteps, accelerations, controls, axes
GROUPS = {
    'baseline': ('baseline', 'double', [.002], [0, 10, 50], [128], [0, 1, 2]),
    'scale1': ('scale1', 'double', [.002], [0, 10, 50], [128], [0, 2]),
    'noarmature': ('noarmature', 'double', [.002, .001, .0005], [0, 50], [128], [0, 2]),
    'single': ('scale1', 'single', [.002, .001, .0005], [0, 50], [128], [0, 2]),
    'stiff': ('stiff', 'double', [.002, .001, .0005], [0, 50], [128], [0, 2]),
    'convergence': ('scale1', 'double', [.002, .001, .0005], [0, 50], [0, 128, 255], [0, 2]),
    'aligned-single': ('aligned', 'single', [.002, .001, .0005], [0, 50], [128], [1]),
    'aligned-double': ('aligned', 'double', [.002], [0, 50], [128], [1]),
    'stiff-single': ('stiff', 'single', [.002, .0005], [0, 50], [128], [0]),
}


def experiment(source, selected):
  for variant in {GROUPS[g][0] for g in selected}:
    generate(source, BASE/'generated'/variant,
             torquescale=.03 if variant == 'baseline' else 1,
             passive_armature=0 if variant == 'noarmature' else None,
             weld_impedance=.999 if variant == 'stiff' else None,
             aligned_base=variant == 'aligned')
  (BASE/'raw').mkdir(exist_ok=True)
  for precision in {GROUPS[g][1] for g in selected}:
    variant = next(GROUPS[g][0] for g in selected if GROUPS[g][1] == precision)
    result = audit(ROOT/('build-linkage-'+precision)/'linkage_runner', BASE/'generated'/variant)
    (BASE/'raw'/('audit-'+precision+'.json')).write_text(json.dumps(result, indent=2)+'\n')
  for group in selected:
    variant, precision, dts, peaks, controls, axes = GROUPS[group]
    run(ROOT/('build-linkage-'+precision)/'linkage_runner', BASE/'generated'/variant,
        BASE/'raw'/group, precision, dts, peaks, controls, axes, ['implicitfast'], 0)


if __name__ == '__main__':
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('source', type=pathlib.Path, help='pinned Menagerie robotiq_2f85 directory')
  p.add_argument('--group', nargs='+', choices=list(GROUPS), default=list(GROUPS))
  a = p.parse_args()
  experiment(a.source.resolve(), a.group)
