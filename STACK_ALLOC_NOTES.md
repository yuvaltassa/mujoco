# Stack allocation under threaded island solving

Measurements and a recommendation on the mjData stack-accumulation model used by threaded
island dispatch. No engine changes are proposed for immediate landing; see Recommendation.

## Current design (verified)

`mju_dispatch` (engine_thread.cc) wraps the whole dispatch in a single `mj_markStack` /
`mj_freeStack` pair and sets `d->threadlock`. While `threadlock` is set:

- `mj_stackAlloc` reserves `size + alignment - 1` bytes with one lock-free
  `mj_atomic_add_size_t` on `d->pstack`; there is no per-thread state.
- `mj_markStack` / `mj_freeStack` are no-ops. Nothing is reclaimed until *all* tasks of the
  dispatch complete — accumulation is per dispatch, not per task.
- `maxuse_stack` / `maxuse_arena` are updated from the final `pstack` at dispatch end, so
  the existing diagnostics capture threaded high-water exactly.
- With `ntask < 2` the dispatch runs inline without threadlock: a single-island step
  reclaims normally and never accumulates.

Three dispatch sites exist: island solving (engine_forward.c), narrowphase collision
(engine_collision_driver.c), tactile sensors (engine_sensor.c). Collision and tactile
pre-carve their workspace before dispatching (per-thread EPA slices, task-args arrays);
their tasks allocate nothing under threadlock — measured accumulation is exactly 0.
The constraint solver is the only accumulator. Noslip runs on the main thread, outside
threadlock.

This design (Kyle, May 2026, b935d4153) replaced per-thread stack shards: equal static
division giving each thread `narena / (2*(nthread+1))` bytes, working per-thread mark/free,
a mutex on arena allocation, and a hard failure when any one task outgrew its shard.

## Allocation map of the island solver path

Once per task (never reclaimed under threadlock, by design — the task needs them live):

- `PrimalAllocate`: two carved blocks, ~`(10-13)*nefc + (10-20)*nv + 2*nJ + 2*nC` numbers
  plus int structure — includes island-local copies of M, J, JT.
- Newton only, `MakeHessian`: H block (`nH` num + `2*nH` int), L block (`3*nL` int +
  `nL` num, `2*nL` elliptic), plus one-shot kernel scratch: `mju_cholFactorSymbolic` ×2
  (2n/4n ints), `mju_sqrMatTDSparseSymbolic` ×2 (~3n ints), `mju_sqrMatTDSparseNumeric`
  (`8*nv` num), `mju_cholFactorNumeric` (`nv` num).
- PGS: `ARinv` (nefc) + `oldstate` (2*nefc int) (+2*nefc Nesterov).

Per iteration (pure leak under threadlock — would be reclaimed by mark/free inline):

- `mju_cholUpdateSparse` allocates a dense accumulator (`start+1 <= nv` numbers) per
  rank-1 update. Called by `HessianIncremental` once per constraint-state change, and by
  `HessianCone` once per cone dimension of every cone-state contact — and `HessianCone`
  re-adds *all* cones on every iteration (plus the initial factorization). Measured:
  ~85 updates per pass (~340 per task at 3 iterations) on a 600-dof clump, with the dense
  buffer averaging ~160 entries — well under nv, since island-local dof compaction keeps
  `start` small.
- Rank-drop recovery repeats the full `FactorizeHessian` scratch.

CG and PGS iterations allocate nothing (measured 0 kernel bytes). The dense (non-sparse)
Newton path also leaks nothing (`mju_cholUpdate` / `mju_cholFactor` are allocation-free);
only sparse Newton has per-iteration leaks, elliptic much more than pyramidal.

## Measurements

Apple M-series, Release, double precision. Instrumented build (`-DMJ_STACKPROBE`, local
uncommitted patch in this worktree) logs per-dispatch accumulation and per-task bytes with
per-site attribution; cross-check: task sums equal dispatch accumulation to the byte.
Results are bitwise-identical threaded vs. unthreaded and independent of worker count
(3/7/15 workers give identical trajectories and identical accumulation).

Peak stack (`maxuse_stack`), 2000 steps unless noted:

| model | config | unthreaded | threaded | ratio |
|---|---|---|---|---|
| 2humanoid100 (654 dof, ~85 islands) | CG pyramidal (shipped) | 132 KB | 488 KB | 3.7× |
| | CG elliptic | 128 KB | 368 KB | 2.9× |
| | Newton pyramidal | 411 KB | 1.06 MB | 2.6× |
| | Newton elliptic | 201 KB | 1.23 MB | 6.1× |
| two 100-box clumps (2×600 dof islands) | Newton elliptic | 1.29 MB | 3.00 MB | 2.3× |
| | Newton pyramidal | 1.92 MB | 4.07 MB | 2.1× |
| 50 small tower islands | Newton elliptic | 118 KB | 1.12 MB | 9.5× |

The ratio is ~(number of comparably-sized islands) plus the iteration leak: unthreaded
solves reuse one context sequentially, threadlock holds the sum. One 100-box clump alone
(single island) shows ~no threaded penalty — it takes the inline path.

Worst single task (600 dof, 1656 efc, elliptic, 3 iterations): **1.50 MB in 359
allocations** — 65% one-time blocks, 29% `cholUpdateSparse` dense buffers (440 KB), ~6%
one-shot kernel scratch. Each additional Newton iteration costs ~110–150 KB here (one
full `HessianCone` re-add), so the prompt-level estimate of "several MB per 100-object
clump task" is reached around 8–10 iterations. On 2humanoid100 elliptic, `cholUpdateSparse`
is 14% of all task bytes, and the worst humanoid-island task (78 dof, 7 iterations) is 72%
leak — high iteration counts flip the balance from blocks to leak.

Minimal working `memory=` (bisected to 16 KB; failure = contact/constraint warning or
stack overflow):

| model | unthreaded min | threaded min | vs. monolithic (islands off) |
|---|---|---|---|
| two clumps, Newton elliptic | 3.07 MB | 4.58 MB (1.49×) | 3.66 MB → 1.25× |
| 2humanoid100, shipped CG | 0.78 MB | 1.15 MB (1.47×) | 0.94 MB → 1.22× |

Two useful identities fell out:

- **Minimal `narena` equals `maxuse_arena` exactly** (both bisections landed within
  resolution of the diagnostic). Run once with headroom, read `maxuse_arena`, done. Worth
  documenting as the sizing recipe.
- Threaded island solving needs ~1.2–1.7× the *monolithic* arena requirement of the same
  model: Σ island blocks ≈ 0.96–1.25× the monolithic context (per-island M/J copies vs.
  one big H), plus the iteration leak on top (elliptic Newton).

## Answers

1. **Allocation sites**: mapped above. Only sparse-Newton islands leak per iteration;
   `mju_cholUpdateSparse` dominates, `mju_cholFactorNumeric`/`mju_sqrMatTDSparse*` are
   once-per-solve and minor. Collision and tactile dispatches accumulate zero.
2. **Is accumulation a real constraint?** Real but bounded and modest: ~1.5× on minimal
   `memory=` in deliberately bad cases, a few MB absolute on 600–1200-dof contact-rich
   scenes. `2humanoid100.xml`'s `memory="100M"` is ~85× its threaded requirement —
   generosity, not necessity. Nothing observed forces oversized `memory=`; failure is
   loud, attributable (`(threadlock)`-tagged overflow message), and sizable via
   `maxuse_arena`.
3. **Hoisting**: one `nv`-sized scratch in `mjPrimalContext`, threaded through
   `mju_cholUpdateSparse` and `mju_cholFactorNumeric` (their internal uses are serial
   within a task, so one buffer serves both), removes the entire per-iteration leak —
   the only term that grows with iterations — for ~5 KB of context per island. Threaded
   sizing then becomes iteration-independent: Σ blocks ≈ monolithic. Churn is small:
   both functions are internal-header MJAPI, called from engine_solver.c,
   engine_setconst.c and tests only; a trailing `scratch` argument with NULL →
   self-allocate (precedent: `mju_cholFactorSymbolic`'s existing `d ? stack : malloc`
   dual mode) keeps every other caller working unchanged. Hoisting the once-per-solve
   scratch (`sqrMatTD*`, `cholFactorSymbolic`) buys another ~6% — optional.
4. **Per-thread arena**: not warranted. The deleted shard model *was* one, and its static
   split is strictly worse on these workloads: it would need
   `narena >= 2*(nthread+1)*max-task-live` ≈ 18 MB for the two-clump model at 8 threads,
   vs. 4.6 MB today — plus an allocation mutex and a hard per-thread capacity cliff
   insensitive to idle memory elsewhere. A dynamic variant (thread-local chunks carved
   from the shared bump, mark/free restored within a task) only wins in the
   many-small-islands regime, where the absolute numbers are trivially small (1.12 MB
   worst observed), and the hoists already remove the growing term. Complexity without a
   constituency.

## Recommendation

Keep the threadlock design — it is simpler, mutex-free, deterministic, exactly
diagnosable, and more memory-efficient than the shard model it replaced. Do the targeted
hoist of item 3 (one context scratch through the two leaking Cholesky kernels) as a small
self-contained change; it also unifies the solver's two idioms (carved context vs.
per-call alloc) in the direction the collision/tactile dispatches already took. Skip the
arena redesign. Separately, document `maxuse_arena` as the exact `memory=` sizing recipe.

## Reproduction

- Probe: standalone C program (load model, optionally `mju_threadpool(d, nworker)`,
  override `narena`/solver/cone, step, report running-max `maxuse_*`, per-island max
  nv/nefc, warnings; exit nonzero on any warning). Link against `build/lib`.
- Instrumentation: rebuild with `-DMJ_STACKPROBE` (uncommitted patch in this worktree:
  engine_memory.c thread-local byte/site counters, engine_thread.cc per-dispatch line,
  engine_forward.c per-task line, per-site deltas in engine_util_solve.c /
  engine_util_sparse.c).
- Models: `test/engine/testdata/island/2humanoid100.xml` as shipped; "clump" models are
  100 free boxes (0.11 cube, spacing 0.115, 4 layers) in a walled 0.66-wide pit with
  gravity (-1,-0.5,-10) so they pack into a single 600-dof island; the two-clump model is
  two such pits 3 m apart. Towers variant: 2 layers, spacing wide enough not to jam.
