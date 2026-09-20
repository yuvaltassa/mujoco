"""Per-body look at principal-axis changes: angle between old and new inertial
frames vs. the relative eigenvalue gap and the inertia scale."""
import sys, numpy as np
sys.path.insert(0, '.')
from model_diff import read_models, quat2mat

rows = []
for (pa, ea, fa), (pb, eb, fb) in zip(read_models(sys.argv[1]), read_models(sys.argv[2])):
  if ea or eb: continue
  qa = fa['body_iquat'].reshape(-1, 4); qb = fb['body_iquat'].reshape(-1, 4)
  ia = fa['body_inertia'].reshape(-1, 3)
  for k in range(1, len(qa)):
    if np.array_equal(qa[k], qb[k]): continue
    # rotation angle between frames, modulo axis sign/permutation symmetry is ignored
    d = abs(float(np.dot(qa[k], qb[k])))
    ang = 2 * np.arccos(min(1.0, d))
    I = np.sort(ia[k])[::-1]
    gap = min(I[0] - I[1], I[1] - I[2]) / I[0] if I[0] > 0 else 0
    rows.append((ang, gap, I[0], pa.split('/')[-1], k))
rows.sort(reverse=True)
print(f'bodies with changed iquat: {len(rows)}')
for thr in (1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6):
  print(f'  frame rotated by more than {thr:g} rad: {sum(r[0] > thr for r in rows)}')
print(f'{"angle[rad]":>11s} {"min rel gap":>12s} {"max moment":>11s}  model:body')
for ang, gap, I0, m, k in rows[:25]:
  print(f'{ang:11.2e} {gap:12.2e} {I0:11.2e}  {m}:{k}')
# angle vs predicted d/gap from the absolute 1e-12 threshold
print('\nlargest angle among bodies with well-separated moments (rel gap > 1e-2):')
sep = [r for r in rows if r[1] > 1e-2]
for ang, gap, I0, m, k in sep[:8]:
  print(f'{ang:11.2e} {gap:12.2e} {I0:11.2e}  {m}:{k}')
