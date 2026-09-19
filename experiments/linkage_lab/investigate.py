"""Check exact diagonals independently and test scaling and near-null-row controls."""
import json
import os
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

BASE = pathlib.Path(__file__).resolve().parent
ROOT = BASE.parents[1]
RAW = BASE/'raw/investigation'
OUT = BASE/'results/investigation'


def execute(model, precision, label, mode='raw', floor=1e-6, exact=1):
  runner = ROOT/('build-linkage-'+precision)/'linkage_runner'
  env = dict(os.environ, LINKAGE_REGULARIZATION=mode, LINKAGE_FLOOR=str(floor))
  path = RAW/(label+'.csv')
  command = [str(runner), str(model), str(path), '0', '0', '5', '.002', '128',
             str(exact), 'implicitfast', '6', '0']
  proc = subprocess.run(command, env=env, capture_output=True, text=True)
  if proc.returncode not in (0, 2):
    raise RuntimeError(proc.stderr + proc.stdout)
  status = json.loads(proc.stdout.strip().splitlines()[-1])
  assert status['num_bytes'] == (8 if precision == 'double' else 4)
  data = np.genfromtxt(path, delimiter=',', names=True)
  settled = data[data['time'] >= 1.8]
  result = dict(label=label, precision=precision, mode=mode, floor=floor, exact=exact,
                failed=status['failed'], end_time=float(data['time'][-1]),
                peak_mm=float(np.max(data['closure_m']))*1000,
                settled_mm=float(np.mean(settled['closure_m']))*1000 if len(settled) else None)
  print(label, result['failed'], result['settled_mm'], flush=True)
  return result


def main():
  RAW.mkdir(parents=True, exist_ok=True)
  OUT.mkdir(parents=True, exist_ok=True)
  checks, controls, scaling = [], [], []
  for precision in ['double', 'single']:
    runner = ROOT/('build-linkage-'+precision)/'linkage_runner'
    for model in ['connect', 'shadow']:
      path = BASE/'repro'/(model+'_exact.xml')
      for mode in ['raw', 'floor', 'block']:
        env = dict(os.environ, LINKAGE_REGULARIZATION=mode, LINKAGE_FLOOR='1e-6')
        snapshot = json.loads(subprocess.check_output([str(runner), str(path), '--diagnose'], env=env, text=True))
        n, ne = snapshot['nv'], snapshot['ne']
        j = np.array(snapshot['equality_jacobian']).reshape(ne,n)
        m = np.array(snapshot['mass_matrix']).reshape(n,n)
        reference = np.einsum('ij,ji->i',j,np.linalg.solve(m,j.T))
        native = np.array(snapshot['exact_diagonal_raw'])
        active = reference > 0
        error = float(np.max(abs(native[active]-reference[active])/reference[active]))
        assert error < (2e-6 if precision == 'single' else 1e-12), error
        generalized = j.T @ np.array(snapshot['equality_force'])
        checks.append(dict(precision=precision,model=model,mode=mode,
                           diagonal_relative_error=error,
                           max_qacc=float(max(abs(x) for x in snapshot['qacc'])),
                           max_generalized_force=float(max(abs(generalized)))))
        folder = OUT if mode == 'raw' else RAW
        (folder/f'{precision}-{model}-{mode}.json').write_text(json.dumps(snapshot,indent=2)+'\n')
        controls.append(dict(model=model,**execute(path,precision,f'{precision}-{model}-{mode}',mode)))
      if precision == 'single':
        for floor in [1e-12,1e-9,1e-3]:
          controls.append(dict(model=model,**execute(path,precision,f'{precision}-{model}-floor{floor}', 'floor', floor)))
  # With constant impedance, exact row scaling should cancel in J, r and R.
  for scale in [.01,.03,.1,.3,1.,3.]:
    for impedance in ['variable','.95','.99','.999']:
      root = ET.parse(BASE/'repro/shadow_exact.xml').getroot()
      for weld in root.findall('./equality/weld'):
        weld.set('torquescale',str(scale))
        if impedance != 'variable':
          weld.set('solimp',f'{impedance} {impedance} .001')
      model = RAW/f'scale{scale}-imp{impedance}.xml'
      ET.ElementTree(root).write(model,encoding='unicode')
      for exact in [0,1]:
        result = execute(model,'double',f'scale{scale}-imp{impedance}-exact{exact}',exact=exact)
        scaling.append(dict(scale=scale,impedance=impedance,**result))
  for model in ['connect', 'shadow']:
    for mode in ['floor', 'block']:
      a = next(r for r in controls if r['label'] == f'double-{model}-{mode}')
      b = next(r for r in controls if r['label'] == f'single-{model}-{mode}')
      assert not b['failed'] and abs(a['settled_mm']-b['settled_mm']) < .0005
  solver_controls = []
  for name in ['connect', 'shadow']:
    for solver, tol, iterations in [('Newton', '1e-6', 100), ('Newton', '0', 1000), ('CG', '1e-10', 1000)]:
      root = ET.parse(BASE/'repro'/(name+'_exact.xml')).getroot()
      root.find('option').set('solver', solver)
      root.find('option').set('tolerance', tol)
      root.find('option').set('iterations', str(iterations))
      label = f'{name}-{solver}-tol{tol}-it{iterations}'
      path = RAW/(label+'.xml')
      ET.ElementTree(root).write(path, encoding='unicode')
      result = execute(path, 'single', label)
      solver_controls.append(dict(model=name, solver=solver, tolerance=tol, iterations=iterations, **result))
  (OUT/'solver-controls.json').write_text(json.dumps(solver_controls, indent=2)+'\n')
  (OUT/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
  (OUT/'controls.json').write_text(json.dumps(controls,indent=2)+'\n')
  (OUT/'scaling.json').write_text(json.dumps(scaling,indent=2)+'\n')


if __name__ == '__main__':
  main()
