# Cone-fold calibration bench (local tooling, not for upstream)

The measurement rig behind the cone-fold flop-count gate in
`engine_solver.c` (see `cone_fold.md` at the repo root). Reusable for other
solver perf investigations: it replays identical states through two engine
variants and produces per-state and per-call cost data.

## Pieces

- `clump.xml` / `clump_free.xml`: Newton/elliptic/sparse conversions of
  test/engine/testdata/island/2humanoid100.xml (walled box, 100 free objects,
  +-2 humanoids). Sweep axes are applied at runtime by cone_bench flags.
- `../src/experimental/studio/cone_bench.cc` (built by the local stanza in
  that directory's CMakeLists): replay benchmark. Usage:
  `cone_bench model.xml [nsteps] [modeB] [reps] [dump.csv]`
  `[-tilt T] [-condim N] [-noisland] [-nowarmstart] [-settle N] [-rollout]`
  Per step it restores the pre-step state and times mj_forward under variant A
  vs variant B, checks qacc equivalence, and bins by max per-island ncone
  (`d->efc_island`). `-rollout` just steps (for in-engine logging).
- `collect.sh` / `validate.sh`: the campaign scripts (absolute paths, adjust).
- `fit.py` / `fit2.py`: cost-model fits and policy-regret evaluation over the
  per-call CSVs (numpy; ~/venvs/mujoco/bin/python).

## Re-instrumenting the engine

The committed engine has no toggles. To re-run a campaign, temporarily
re-add to `HessianCone` in engine_solver.c:

1. force override: read MJ_CONE_FORCE (-1 stock, 1 fold, else predictor)
   before the gate -- enables cone_bench A/B comparisons;
2. per-call logger: run and time both paths, print features
   (U, F, S, pathcost via the reverse-etree pass) to MJ_CONE_LOG as CSV --
   enables fit.py.

Both existed in this worktree's session history (branch cone-fold-bench).
mjcb_time must be installed by the driver (cone_bench does).
