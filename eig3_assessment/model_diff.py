"""Field-by-field comparison of two model_dump outputs (streaming).

  model_diff.py <base.bin> <new.bin> [--top N]

Reports, per mjModel field, how many models differ and by how much, plus two
physically meaningful comparisons that are insensitive to the arbitrary choice
of principal axes: the body inertia tensor in the body frame, and mesh-geom
vertices in the body frame.
"""

import struct
import sys
from collections import defaultdict

import numpy as np


def read_string(f):
  raw = f.read(4)
  if len(raw) < 4:
    return None
  (n,) = struct.unpack('<I', raw)
  return f.read(n).decode('utf-8', 'replace')


def read_models(path):
  """Yields (model_path, error, {field: array}) one model at a time."""
  with open(path, 'rb') as f:
    while True:
      model = read_string(f)
      if model is None:
        return
      error = read_string(f)
      fields = {}
      if not error:
        while True:
          name = read_string(f)
          (count,) = struct.unpack('<Q', f.read(8))
          if name == 'END':
            break
          fields[name] = np.frombuffer(f.read(8 * count), dtype=np.float64)
      yield model, error, fields


def quat2mat(q):
  w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
  return np.stack([
      np.stack([w*w + x*x - y*y - z*z, 2*(x*y - w*z), 2*(x*z + w*y)], -1),
      np.stack([2*(x*y + w*z), w*w - x*x + y*y - z*z, 2*(y*z - w*x)], -1),
      np.stack([2*(x*z - w*y), 2*(y*z + w*x), w*w - x*x - y*y + z*z], -1),
  ], -2)


def body_tensors(fields):
  quat = fields['body_iquat'].reshape(-1, 4)
  inertia = fields['body_inertia'].reshape(-1, 3)
  R = quat2mat(quat)
  return np.einsum('bik,bk,bjk->bij', R, inertia, R), inertia


def mesh_geom_vertices(fields):
  """Mesh vertices in the body frame, per mesh geom."""
  geom_type = fields['geom_type'].astype(int)
  dataid = fields['geom_dataid'].astype(int)
  vertadr = fields['mesh_vertadr'].astype(int)
  vertnum = fields['mesh_vertnum'].astype(int)
  vert = fields['mesh_vert'].reshape(-1, 3)
  pos = fields['geom_pos'].reshape(-1, 3)
  R = quat2mat(fields['geom_quat'].reshape(-1, 4))
  out = []
  for g in np.nonzero((geom_type == 7) & (dataid >= 0))[0]:
    m = dataid[g]
    v = vert[vertadr[m]:vertadr[m] + vertnum[m]]
    out.append(v @ R[g].T + pos[g])
  return out


SKIP = {'tex_data', 'hfield_data', 'text_data', 'names', 'paths'}


def main():
  base_path, new_path = sys.argv[1], sys.argv[2]
  top = int(sys.argv[sys.argv.index('--top') + 1]) if '--top' in sys.argv else 8

  nmodel = nload = nidentical = 0
  load_mismatch = []
  size_mismatch = []
  field_stats = defaultdict(lambda: dict(models=0, maxabs=0.0, maxrel=0.0, worst=''))
  tensor_rows = []   # (rel diff, model, body, scale)
  mesh_rows = []     # (rel diff, model, geom index)
  models_over = defaultdict(list)  # threshold -> models with a raw diff above it

  for (pa, ea, fa), (pb, eb, fb) in zip(read_models(base_path), read_models(new_path)):
    assert pa == pb, (pa, pb)
    nmodel += 1
    if ea or eb:
      if ea != eb:
        load_mismatch.append((pa, ea, eb))
      continue
    nload += 1
    # shorten: path inside the mujoco tree, else the last four components
    for root in ('/model/', '/test/'):
      if root in pa:
        short = root[1:] + pa.split(root, 1)[1]
        break
    else:
      short = '/'.join(pa.split('/')[-4:])

    identical = True
    worst_raw = 0.0
    for name, a in fa.items():
      b = fb.get(name)
      if name in SKIP:
        continue
      if b is None or a.shape != b.shape:
        size_mismatch.append((short, name))
        identical = False
        continue
      if a.size == 0 or np.array_equal(a, b):
        continue
      identical = False
      diff = np.abs(a - b)
      maxabs = float(diff.max())
      scale = float(max(np.abs(a).max(), np.abs(b).max()))
      maxrel = maxabs / scale if scale > 0 else 0.0
      s = field_stats[name]
      s['models'] += 1
      if maxrel > s['maxrel']:
        s['maxrel'] = maxrel
        s['worst'] = short
      s['maxabs'] = max(s['maxabs'], maxabs)
      if name not in ('SIZES',):
        worst_raw = max(worst_raw, maxrel)
    nidentical += identical
    for thr in (1e-3, 1e-6, 1e-9):
      if worst_raw > thr:
        models_over[thr].append((worst_raw, short))

    # inertia tensor in the body frame
    Ta, ia = body_tensors(fa)
    Tb, ib = body_tensors(fb)
    if Ta.shape == Tb.shape and Ta.size:
      scale = np.maximum(np.abs(ia).max(axis=1), np.abs(ib).max(axis=1))
      d = np.abs(Ta - Tb).reshape(len(Ta), -1).max(axis=1)
      rel = np.where(scale > 0, d / np.where(scale > 0, scale, 1), 0)
      k = int(rel.argmax())
      if rel[k] > 0:
        tensor_rows.append((float(rel[k]), short, k, float(scale[k])))

    # mesh geoms in the body frame
    if 'mesh_vert' in fa and fa['mesh_vert'].size:
      va, vb = mesh_geom_vertices(fa), mesh_geom_vertices(fb)
      for g, (x, y) in enumerate(zip(va, vb)):
        if x.shape != y.shape or not x.size:
          continue
        size = float(np.abs(x).max()) or 1.0
        rel = float(np.abs(x - y).max()) / size
        if rel > 0:
          mesh_rows.append((rel, short, g))

  print(f'models listed {nmodel}, loaded by both {nload}, bit-identical {nidentical}')
  print(f'load status differs: {len(load_mismatch)}')
  for row in load_mismatch:
    print('   ', row)
  print(f'array size differs: {len(size_mismatch)}')
  for row in size_mismatch[:20]:
    print('   ', row)

  for thr in (1e-3, 1e-6, 1e-9):
    rows = sorted(models_over[thr], reverse=True)
    print(f'\nmodels with some raw field differing by more than {thr:g} '
          f'(relative to the field\'s largest element): {len(rows)}')
    for rel, m in rows[:top]:
      print(f'    {rel:9.2e}  {m}')

  print('\nraw field differences (relative to the largest element of the field)')
  print(f'{"field":24s} {"models":>6s} {"max rel":>10s} {"max abs":>10s}  worst model')
  for name, s in sorted(field_stats.items(), key=lambda kv: -kv[1]['maxrel']):
    print(f'{name:24s} {s["models"]:6d} {s["maxrel"]:10.2e} {s["maxabs"]:10.2e}  {s["worst"]}')

  print('\nbody inertia tensor R diag(I) R^T in the body frame, '
        'max change relative to the body\'s largest moment')
  tensor_rows.sort(reverse=True)
  print(f'  models with any change: {len(tensor_rows)}')
  for thr in (1e-3, 1e-5, 1e-6, 1e-7, 1e-9, 1e-12):
    print(f'  models above {thr:g}: {sum(r[0] > thr for r in tensor_rows)}')
  for rel, m, k, scale in tensor_rows[:top]:
    print(f'    {rel:9.2e}  body {k:4d}  moment {scale:9.2e}  {m}')

  print('\nmesh geom vertices in the body frame, max change relative to mesh size')
  mesh_rows.sort(reverse=True)
  print(f'  mesh geoms with any change: {len(mesh_rows)}')
  for thr in (1e-3, 1e-6, 1e-9, 1e-12):
    print(f'  above {thr:g}: {sum(r[0] > thr for r in mesh_rows)}')
  for rel, m, g in mesh_rows[:top]:
    print(f'    {rel:9.2e}  mesh geom #{g:3d}  {m}')


if __name__ == '__main__':
  main()
