"""Verify compiled shadow conversion: geometry, mass, mobility, kinetic energy."""
import argparse
import json
import pathlib
import subprocess

import numpy as np


def audit(runner, models):
  snapshots = {}
  for name in ['connect', 'shadow']:
    snapshots[name] = json.loads(subprocess.check_output(
        [str(runner), str(models / (name+'.xml')), '--audit'], text=True))
  a, b = snapshots['connect'], snapshots['shadow']
  tolerance = 2e-6 if a['num_bytes'] == 4 else 1e-10

  def composite(snapshot):
    mass, first, inertia = 0., np.zeros(3), np.zeros((3, 3))
    for body in snapshot['bodies']:
      m = body['mass']
      p = np.array(body['com'])
      r = np.array(body['rotation']).reshape(3, 3)
      mass += m
      first += m*p
      inertia += r @ np.diag(body['inertia']) @ r.T + m*((p@p)*np.eye(3)-np.outer(p, p))
    return mass, first, inertia

  errors = [float(np.max(np.abs(x-y))) for x, y in zip(composite(a), composite(b))]
  assert max(errors) < tolerance, errors
  assert a['colliding_geoms'] == b['colliding_geoms']
  assert max(abs(x) for s in snapshots.values() for x in s['equality_residual']) < tolerance
  ja = np.array(a['equality_jacobian']).reshape(a['ne'], a['nv'])
  jb = np.array(b['equality_jacobian']).reshape(b['ne'], b['nv'])
  ua, sa, va = np.linalg.svd(ja, full_matrices=True)
  rank_a = np.count_nonzero(sa > tolerance)
  rank_b = np.linalg.matrix_rank(jb, tol=tolerance)
  assert a['nv']-rank_a == b['nv']-rank_b == 3
  # Lift any original admissible velocity into the shadow coordinates.
  null = va[rank_a:].T
  common = [b['joints'].index(j) for j in a['joints']]
  extra = [i for i in range(b['nv']) if i not in common]
  lift = np.zeros((b['nv'], null.shape[1]))
  lift[common] = null
  lift[extra] = np.linalg.lstsq(jb[:, extra], -jb[:, common]@null, rcond=None)[0]
  velocity_error = float(np.max(np.abs(jb@lift)))
  ma = np.array(a['mass_matrix']).reshape(a['nv'], a['nv'])
  mb = np.array(b['mass_matrix']).reshape(b['nv'], b['nv'])
  energy_error = float(np.max(np.abs(null.T@ma@null-lift.T@mb@lift)))
  assert velocity_error < tolerance, velocity_error
  assert energy_error < tolerance, energy_error
  return {'precision_bytes': a['num_bytes'], 'mass_com_inertia_errors': errors,
          'connect_nv': a['nv'], 'shadow_nv': b['nv'],
          'connect_equality_rank': int(rank_a), 'shadow_equality_rank': int(rank_b),
          'mobility': 3, 'velocity_lift_error': velocity_error,
          'kinetic_energy_matrix_error': energy_error,
          'colliding_geoms': a['colliding_geoms'],
          'scope': 'Compiled initial configuration; not a trajectory-equivalence proof.'}


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('runner', type=pathlib.Path)
  parser.add_argument('models', type=pathlib.Path)
  args = parser.parse_args()
  print(json.dumps(audit(args.runner.resolve(), args.models.resolve()), indent=2))
