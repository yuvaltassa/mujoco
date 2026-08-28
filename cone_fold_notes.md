# Cone-fold: independent validation, findings, and follow-ups

**STATUS 2026-08-28: COMPLETE.** All follow-ups resolved; committed on this
branch as `cc967df0c` (Kevin's fold, his authorship preserved) + `936ae9fba`
(flop-count gate replacing the constant, cached per state-change; test;
changelog; cone_fold.md extended with the measurement campaign). Resolution of
the TODO list below:
- Predictor: DONE -- `10U > 3(F+2S)` from ~300k instrumented per-call timings;
  0.8% regret vs per-call oracle (best constant: 3.4%); end-to-end >= 0.99x
  everywhere, wins kept (see cone_fold.md). The cheap path-cost feature proved
  EXACTLY equal to the true update closure (corr 1.0000).
- Unit test: DONE -- SolverTest.ConeFoldEquivalent (sliding walled clump,
  condim 3/4/6, sparse-folding vs dense-rank-1), green in double and single.
- nupdate: NON-ISSUE -- stock cone path also counts += dim; Kevin's counting is
  consistent, and FactorizeHessian overwrites with nefc afterwards.
- mjUSESINGLE: DONE -- full solver suite green under single; direct fold-vs-
  stock qacc gap ~1e-4 relative = eps*conditioning (both factorizations valid).
- H-reuse (fold only the cone delta): REJECTED by measurement -- S_quad is
  3-13% of folded-path time; not worth maintaining H beside the updated L.
- Shared-sparsity assert: covered by comment; guaranteed by constraint
  assembly, zero-cost convention kept.
- Recalibration procedure: rebuild with a temporary force override in
  HessianCone (MJ_CONE_FORCE env: -1 stock / 1 fold, ~10 lines, see git log of
  this file's session) + `cone_bench -rollout` with MJ_CONE_LOG to re-collect
  per-call CSVs, then scratchpad fit.py.


Working notes on Kevin's `Fold cone contributions into the Hessian when many
contacts slide` (branch `cone-fold`), cherry-picked onto our tree and benchmarked
with the local `cone_bench` replay tool (`src/experimental/studio/cone_bench.cc`).

## Validation (our numbers, Apple M-series, double precision)

`cone_bench` replays each pre-step state and times `mj_forward` (constraint timer,
min-of-3, warmstart restored) with folding **off** (`MJ_CONE_FOLD` huge) vs **on**,
then compares `qacc`.

- **Correctness:** `max|qacc_folded - qacc_stock| = 1.3e-9` on house of cards over
  1500 steps. The fold is numerically equivalent to the stock rank-1 path, as the
  math predicts (same Hessian, different factor construction). Confirmed by hand
  against stock `HessianCone`/`FactorizeHessian`.
- **Speedup, house of cards** (nv=156, 26 free bodies): mean 1.29x, worst step
  **3.81x** (22.5ms -> 5.9ms). Gain concentrates in the many-sliding-contact
  transients, exactly as Kevin reports.

## The calibration finding: a constant threshold is wrong

Forcing fold on every state (`threshold=0`) and binning by **`ncone`** (cone-state
rows, counted from `d->efc_state`, NOT `d->ncon`) gives a clean, monotonic crossover
-- but it sits in a completely different place per model:

| model            | struct       | solve cost | crossover `ncone` |
|------------------|--------------|-----------|-------------------|
| house of cards   | free bodies  | expensive (~5ms) | **~12** |
| hyperbolic arch  | free bodies  | cheap (~0.2ms)   | **~100** |

Both are free-body models, yet the crossover differs **~8x**. Cards have an
expensive factorization (dense contact graph -> lots of fill), so one refactor beats
few updates early; the arch's solves are cheap (chain/ring contact graph, banded L),
so it takes ~100 sliding rows before a refactor pays off. `ncon` (total contacts) is
the *wrong* axis entirely -- it's non-monotonic (a standing pile has many *stuck*
contacts that are cheap).

**Conclusion:** no single `mjCONE_FOLD` constant is right. 24 is safe (conservative)
for cards but leaves the 12-23 band on the table; it is roughly right for the arch by
accident. The real fix is the **runtime cost predictor** Kevin already sketches:
fold when `estimated update cost (~ sum(dim) * nnz-along-update-path)` exceeds
`estimated factorization cost (~ Cholesky flop count, known from the symbolic phase)`.
That is structure-aware and needs no magic number.

## Follow-up TODO list

Correctness / upstreamability
- [ ] **Unit test**: force both paths on the same many-cone state (drop the
      threshold) and assert `Lcone_folded ~= Lcone_stock` (or identical solve to
      tol). The main gap for landing; our replay validation lives outside the repo.
- [ ] **float32 (`mjUSESINGLE`) check**: rank-1-update vs from-scratch accumulate
      roundoff differently; confirm equivalence still holds. (cone_bench is
      double-only right now.)
- [ ] **`ncone += dim` accounting**: the folded path bumps `ctx->nupdate` by
      `sum(dim)` though it does one factorization and no updates; the stock cone
      path doesn't count cone updates at all. It's a printed diagnostic only
      (engine_solver.c ~L2469), but inconsistent -- drop it or fix both paths.
- [ ] **Assert the shared-sparsity invariant**: the fold adds contact rows
      element-wise assuming all `dim` rows share one colind pattern. Stock relies on
      the same, but a defensive `assert` (debug) documents the load-bearing
      assumption.

Performance / design
- [ ] **Replace the constant with the cost-model predictor** (the real fix; see
      above). Compare `sum(dim) * c1 * update_work` to `c2 * factorize_flops`; both
      are cheaply computable (factorization flop count from the symbolic L structure;
      update work from `nnz(L)` and the update-vector sparsity). Calibrate c1,c2 on
      the benchmark set.
- [ ] **Reuse `ctx->H` for the quadratic part**: the fold recomputes the full
      `Jmod'DmodJmod` including quadratic rows, though `ctx->H` already holds that;
      only the cone rows changed. Could add just the cone delta to `ctx->H`.
- [ ] **Reuse `ctx->JT` structure**: `JTmod`'s sparsity equals `ctx->JT`'s (Jmod
      shares J's pattern); recompute values only, not the transpose structure.

Coordination
- [ ] **Model path divergence**: Kevin's commit adds the model at
      `model/house_of_cards/`; ours is `model/cards/house_of_cards.xml`. Pick one
      canonical location before both land.

## Clump-family results (bench/clump.xml, bench/clump_free.xml)

Derived from test/engine/testdata/island/2humanoid100.xml (walled box, 100 free
objects, 2 humanoids, sideways gravity for clumping), converted to Newton +
elliptic + sparse. Sweep axes are runtime flags in cone_bench (`-tilt`,
`-condim`, `-noisland`, `-nowarmstart`, `-settle`). All runs benched in the
settled clumped regime (settle=800, i.e. t=4s), threshold=0 (always fold),
min-of-3, binned by max **per-island** ncone (the gate's actual axis):

| config                      | island-ncone seen | fold vs stock |
|-----------------------------|-------------------|---------------|
| clump tilt=1 (islands)      | 0..49             | **loses 0.75x everywhere** |
| clump tilt=5 (islands)      | 175..299          | wins 1.3-1.8x, all states |
| free  tilt=5 (islands)      | 125..249          | crossover at ~175 (0.97x below, 1.2-1.6x above) |
| clump tilt=5 -noisland      | 125..249          | wins 1.5-2.0x, all states |
| free  tilt=5 -condim 6      | 300..474          | wins 2.1-2.5x, all states |
| clump tilt=5 -nowarmstart   | (final-state ncone meaningless) | wins 1.5x, all states |

Headline findings:
1. **Threshold 24 is actively harmful on a standard scene**: clump tilt=1 sits at
   island-ncone 25..49 where Kevin's constant engages the fold and it *loses 25%*.
   Not just conservative -- wrong-signed. Empirical crossovers across models now
   span **~12 (cards) .. ~50-175 (clump variants) .. ~100 (arch) .. ~175 (free
   clump)**: >10x spread. Constant untenable; the cost-model predictor is the fix.
2. **Islands matter**: with islands off (one nv=654 solve) the fold wins bigger at
   equal ncone (1.7-2.0x vs 1.5-1.6x). The gate correctly operates per island; the
   benchmark must bin by per-island ncone (global ncone is misleading), via
   d->efc_island.
3. **condim is a strong axis**: condim=6 doubles sum(dim) per contact and gives the
   largest wins (2.1-2.5x at ncone up to 474). condim 3/4/6 all belong in the set.
4. **Warmstart measurement subtlety**: the fold gate is evaluated *per Newton
   iteration* with the current efc_state; with warmstart off the solver passes
   through many cone states mid-solve even when the converged state has ~none, so
   end-of-solve ncone under-reports. Any predictor calibration must use the
   per-iteration ncone (visible inside HessianCone), not the final state.
5. **Articulation**: at tilt=5 the humanoid-bearing clump wins at ncone 175-199
   (1.28x) where the free-body version is only borderline (1.18x), and the
   humanoids drag the whole distribution to higher solve costs. Consistent with
   tree fill making factorization relatively cheaper vs updates. Free bodies alone
   are not representative.

Existing infra worth reusing (test/benchmark/): `chol_benchmark_test.cc` benches
exactly this machinery (sparse J'DJ + Cholesky) as google/benchmark fixtures, and
`testdata/2humanoid100_chol.xml` is this same clump scene with the settled state
**baked as a keyframe (t=3.59)** -- upstream's pattern for "bench the interesting
regime". The upstream vehicle for cone-fold benchmarking is a fixture in that
style (keyframed clumped state, elliptic/Newton variant of the _chol testdata);
`run_ablation.py` shows how ablations are automated.

## What a proper benchmark set looks like

The threshold/predictor is governed by the ratio `factorize_cost / update_cost`,
which is set by the Hessian's fill-in structure -- so the set must span the axes that
move that ratio, not just contact count:

1. **Kinematic structure (the axis that matters most, and answers "is free-body
   enough?" -- NO):**
   - *Free-body piles*, and within them a range of contact-graph density: dense
     (house of cards tent, box stack, granular box) vs sparse/banded (arch ring,
     brick course, chain of blocks). We already see 8x crossover spread here alone.
   - *Articulated systems*: the kinematic tree adds base fill to M independent of
     contacts, changing `factorize_cost` in a way free bodies never sample --
     ragdoll piles, a hand/gripper closing on clutter, a quadruped/humanoid in a
     heap, a rope/chain with self-contact. Essential: a predictor tuned only on
     free-body piles will misfire on manipulation and ragdoll workloads.
   - *Mixed*: articulated manipulator + many free objects (the real manipulation
     case, and where many-sliding-contact spikes actually hurt real-time).
2. **Problem size `nv`**: dozens to thousands of DOF in one island; factorization
   cost scales super-linearly, updates ~ `nnz(L)`, so the crossover drifts with size.
3. **Controllable `ncone`**: each family should scale a parameter (N objects / N
   links) to sweep simultaneous sliding contacts through the crossover, so we see
   both sides of it per model.
4. **`condim` / cone dim**: dim in {3,4,6} changes `sum(dim)` per contact (the
   update count), shifting the crossover; include a spread.
5. **Island structure**: fold is per-island (`ncone` is per-island). Include big
   single-island pile-ups (fold helps) and many-small-island scenes (must correctly
   stay on the rank-1 path).

The set's job is **not** to find a better constant but to **validate that the
cost-model predictor picks the faster path across all these regimes**, and to fit its
two constants. A good deliverable is a table like the one above, one row per
model/scale, with measured `factorize_cost`, `update_cost`, empirical crossover, and
predictor's chosen crossover -- the predictor is good when its column tracks the
empirical one everywhere.
