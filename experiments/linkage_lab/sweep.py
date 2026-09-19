"""Run the four-way Robotiq comparison and summarize full-rate measurements."""
import argparse
import csv
import itertools
import json
import hashlib
import pathlib
import subprocess

import numpy as np


def run(runner, models, output, precision, dts, peaks, controls, axes, integrators, contacts):
  output.mkdir(parents=True, exist_ok=True)
  rows = []
  binaries = [runner, *sorted((runner.parent/'lib').glob('*mujoco*'))]
  binary_digests = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in binaries if p.is_file()}
  for model, exact, axis, peak, ctrl, dt, integrator in itertools.product(
      ['connect', 'shadow'], [0, 1], axes, peaks, controls, dts, integrators):
    # Zero-acceleration case needs only one axis.
    if peak == 0 and axis != axes[0]:
      continue
    name = f'{precision}_{model}_e{exact}_a{axis}_p{peak}_c{ctrl}_h{dt}_{integrator}_contact{contacts}'
    path = output / (name+'.csv')
    meta_path = output / (name+'.json')
    command = [str(runner), str(models/(model+'.xml')), str(path), str(axis),
               str(peak), '5', str(dt), str(ctrl), str(exact), integrator, '6', str(contacts)]
    model_digest = hashlib.sha256((models/(model+'.xml')).read_bytes()).hexdigest()
    cached = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    if not (path.exists() and cached.get('model_sha256') == model_digest
            and cached.get('command') == command
            and cached.get('binary_sha256') == binary_digests):
      result = subprocess.run(command, text=True, capture_output=True)
      if result.returncode not in (0, 2):
        raise RuntimeError(result.stderr + result.stdout)
      metadata = json.loads(result.stdout.strip().splitlines()[-1])
      metadata['command'] = command
      metadata['binary_sha256'] = binary_digests
      metadata['model_sha256'] = model_digest
      metadata['provenance'] = json.loads((models/'provenance.json').read_text()) if (models/'provenance.json').exists() else None
      metadata['stderr'] = result.stderr
      meta_path.write_text(json.dumps(metadata, indent=2)+'\n')
    meta = json.loads(meta_path.read_text())
    assert meta['num_bytes'] == (8 if precision == 'double' else 4)
    data = np.genfromtxt(path, delimiter=',', names=True)
    active = data[(data['time'] >= 3) & (data['time'] <= 7)]
    settled = data[(data['time'] >= 1.8) & (data['time'] < 2)]
    recovery = data[data['time'] >= 8.8]
    # Report a failed run without presenting its truncated trace as success.
    valid = not meta['failed'] and len(active) and len(settled) and len(recovery)
    row = dict(precision=precision, model=model, exact=exact, axis=axis, peak=peak,
               control=ctrl, dt=dt, integrator=integrator, contacts=contacts,
               failed=not bool(valid), trace=path.name,
               step_us=1e6*meta['step_seconds']/max(1, meta['samples']-1))
    if valid:
      initial = np.mean(settled['aperture_m'])
      pad = np.column_stack([active[f'{s}_{c}'] for s in ['right', 'left'] for c in 'xyz'])
      pad0 = np.array([np.mean(settled[f'{s}_{c}']) for s in ['right', 'left'] for c in 'xyz'])
      row.update(closure_max_mm=1e3*np.max(active['closure_m']),
                 closure_rms_mm=1e3*np.sqrt(np.mean(active['closure_m']**2)),
                 settled_closure_mm=1e3*np.mean(settled['closure_m']),
                 aperture_peak_mm=1e3*np.max(abs(active['aperture_m']-initial)),
                 aperture_pp_mm=1e3*np.ptp(active['aperture_m']),
                 aperture_recovery_mm=1e3*(np.mean(recovery['aperture_m'])-initial),
                 pad_peak_mm=1e3*np.max(np.linalg.norm((pad-pad0).reshape(-1, 2, 3), axis=2)),
                 weld_angle_max_deg=np.rad2deg(np.max(active['weld_rad'])),
                 equality_work_J=np.trapezoid(active['eq_power_W'], active['time']),
                 equality_abs_work_J=np.trapezoid(abs(active['eq_power_W']), active['time']),
                 iterations_mean=np.mean(active['iterations']),
                 contact_count_max=int(np.max(active['ncon'])))
    rows.append(row)
    print(name, 'FAILED' if not valid else f"closure {row['closure_max_mm']:.4g} mm", flush=True)
  (output/'summary.json').write_text(json.dumps(rows, indent=2)+'\n')
  columns = list(dict.fromkeys(k for row in rows for k in row))
  with (output/'summary.csv').open('w') as f:
    writer = csv.DictWriter(f, fieldnames=columns, lineterminator='\n')
    writer.writeheader()
    writer.writerows(rows)


if __name__ == '__main__':
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('runner', type=pathlib.Path)
  p.add_argument('models', type=pathlib.Path)
  p.add_argument('output', type=pathlib.Path)
  p.add_argument('--precision', choices=['double', 'single'], default='double')
  p.add_argument('--dt', nargs='+', type=float, default=[.002])
  p.add_argument('--peak', nargs='+', type=float, default=[0, 10, 50])
  p.add_argument('--control', nargs='+', type=float, default=[128])
  p.add_argument('--axis', nargs='+', type=int, default=[0, 1, 2])
  p.add_argument('--integrator', nargs='+', default=['implicitfast'])
  p.add_argument('--contacts', type=int, choices=[0, 1], default=0)
  a = p.parse_args()
  run(a.runner.resolve(), a.models.resolve(), a.output.resolve(), a.precision,
      a.dt, a.peak, a.control, a.axis, a.integrator, a.contacts)
