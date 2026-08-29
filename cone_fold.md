# Cone fold

**Problem.** In the elliptic Newton solver, `HessianCone` rebuilds `Lcone`
every iteration by copying `L` and applying `dim` sparse rank-1 updates per
cone-state contact. Profiling a 26-card house of cards being knocked over
(~90 contacts, nv=156, one island, 8.6 iterations/step) put
`mju_cholUpdateSparse` at 63% of total step time: on this problem one sparse
rank-1 update costs about a third of a full numeric factorization, so with
many sliding contacts the updates dwarf the factorization they avoid.

**Fix.** When folding is predicted cheaper (below), build `Lcone` with one
factorization. Writing `Hc = Lc Lc'`, the cone contribution `Jc' Hc Jc`
equals `(Lc' Jc)'(Lc' Jc)`, so replacing each cone contact's Jacobian rows
with `Lc' Jc` (weight 1 in D) lets the existing `J'DJ` + factorize pipeline
produce the cone-inclusive Hessian. Rows of one contact share a sparsity
pattern, so J, H, and L structures are unchanged; no symbolic work. Two new
static functions in `engine_solver.c`, sparse path only.

**Gate.** Instrumenting `HessianCone` to run and time both paths on every
call (~300k calls, 10 scenes: card house stable/collapsing, stone arch, and a
walled clump of 100 boxes + 2 humanoids swept over gravity tilt, condim
3/4/6, islands on/off, warmstart on/off) shows the update/folded crossover
moving from ~12 cone rows (card house) to ~100 (arch) to ~175 (box clump):
the >10x spread tracks the factor's fill-in, not the cone count, so the gate
compares estimated flop counts. All quantities depend only on the
fixed-per-solve sparsity patterns and on `efc_state`:

- update path: a rank-1 update starting at column c walks the reverse-etree
  path of c (parent(r) = largest off-diagonal column of L row r), paying the
  L row nonzeros along it. `pathcost[]` is one O(nv) ascending pass. A
  contact's dim updates all start from its last dof, whose path its lower
  dofs join immediately, so `U = sum over cone contacts of dim *
  pathcost[last dof]`. Measured against the exact update closure: identical
  (correlation 1.0000).
- folded path: factorization flops `Lflops/2` where `Lflops = sum of squared
  L row nonzeros`, plus `S = sum of squared J row nonzeros over the D != 0
  rows` for `J'DJ`.

Per-call regressions on the instrumented data give R2 = 0.98 (update) and
1.00 (folded), with one twist: an update flop costs ~2x a factorization flop
(scattered Givens rotations vs a dense-accumulator factorization sweep),
which the gate constant absorbs: **fold when `10 U > 3 (Lflops + 2 S)`**
(i.e. alpha=0.6 in `U > alpha (Lflops/2 + S)`; regret against the per-call
oracle is flat within 0.5-0.8% for alpha in [0.5, 0.65], so the constant is
robust to machine variation). `pathcost` and `Lflops` are computed once per
solve alongside the symbolic factorization, whose pattern they describe; the
remaining O(nefc) decision is cached and recomputed only when a constraint
changes state (measured worth 0.5-2% end-to-end on cheap-solve scenes, where
many small islands make the per-iteration scan visible).

**Results.** Replaying recorded states identically per variant (single
`mj_forward`, warmstart restored) removes trajectory chaos; iteration counts
are identical, so this is pure cost. Choosing per call, the gate is within
0.8% of the per-call oracle that always picks the faster path, uniformly at
most 2.8% per scene, against 6.7% for always folding. End-to-end
(full-`mj_forward` replays, min-of-3, Apple M-series): every scene at 0.99x
or better vs never folding, keeping the wins -- card house collapse 1.85x
mean / 3.1x worst step, clumped pile 1.67x / 1.9x, condim-6 pile 2.1x / 2.5x
-- while the fold-hostile mild-tilt clump, where always folding costs 24%,
stays at 0.99x. The gain concentrates in the many-sliding-contact steps that
spike during pile-ups.

**Not done, deliberately.** Reusing `ctx->H` for the quadratic rows (folding
only the cone delta) was measured at 3-13% of folded-path time: not worth
maintaining H alongside the incrementally-updated L. `mjUSESINGLE` agreement
between the paths is ~1e-4 relative (eps * conditioning, both factorizations
valid); the equivalence test passes under both precisions.
