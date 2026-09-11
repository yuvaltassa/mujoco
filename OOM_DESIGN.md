# Recoverable out-of-memory in the simulation pipeline

*Prototype, 2026-09-12. Branch `mjok-oom`, one commit on top of `mjok-returns`. Explores whether
running out of `mjData` workspace can be reported through the status returns of the `onwarn`
design, so that a host keeps its process, without the boundary machinery that made the general
`noexit` design expensive. This document records the mechanism, what it cost, where it was
awkward, and an assessment.*

## Summary

A scratch allocation that overflows the `mjData` arena no longer raises an error. The engine
function that owns the allocation frees its frame and returns `mjSTATUS_OOM` (-1); its callers
propagate the status as they propagate a warning, and the call ends at the first stage boundary
whatever the `onwarn` setting. `mjData` is left partially updated, for inspection or a reset,
and a data of sufficient size completes the call: the recovery loop of a host is to catch the
status, remake the data with a larger arena, and retry.

Three rules of the status design carry over unchanged: a status is a local composed by the
function that produced it, callers stage it with `mjSTAGE`, and public entries mirror it into
`mjData.status`. Two are new: an error is kept over any warning (`mji_join` prefers the negative
value; among errors the first), and an error stops the call under every policy (`mji_stop` is
true for any negative status).

Out of scope, as asked: `mju_malloc` failures, the model compiler, and code outside the pipeline.

## Mechanism

**Checked frames.** A stack frame is marked with `mj_markStackChecked` instead of
`mj_markStack`. An engine allocation (`mjSTACKALLOC`, `mj_stackAllocInfo`) that overflows on a
checked frame returns `NULL` and counts the failure in the frame record; on a plain frame it
raises an error as before. Frame records grow the `mjStackFrame` struct by two words, `checked`
and `failed`. The owner checks after allocating, before any use, with `mjSTACKCHECK(d)`: free
the frame, record `mjSTATUS_OOM` in the data, return it. `mjSTACKCHECK_(d, cleanup, retval)`
is the general form, for frames owned by a caller (`(void)0`) and for helpers that return counts
(`-1`).

Checking the frame rather than each pointer lets a batch of allocations be checked once, and
keeps zero-size allocations, which have always returned `NULL`, out of the failure path.

**Reserve for frame records.** An allocation on a checked frame leaves the last 2 KB of the stack
(`mjSTACKRESERVE`) to frame records. Entering a function from a checked frame therefore always
succeeds and it is the allocations on the frame that fail, so a function never has to handle a
failure of its own `mj_markStackChecked`. Fifty nested frames fit in the reserve; the pipeline
nests fewer than twenty. Allocations on plain frames are unaffected, so the model compiler, whose
frames are plain, still builds a model with a 1 KB arena.

**Threads.** `mju_dispatch` marks its shared frame as checked, so a worker's allocation that
overflows fails instead of raising on the worker thread. Workers run their normal checks
(`mj_stackFailed` reads the dispatcher's frame under `threadlock`) and report through their
arguments: the island tasks write a per-island status array, the collision task leaves a
negative contact count for its chunk. The dispatch returns a status of its own, composed with
the tasks' after the join, and releases the frame only then: workers are finished before the
workspace is released. The overflow is reported once, by the dispatcher, after the join.

**Arena allocations without a fallback.** Two arena allocations raised errors on failure: the
candidate pair buffer of the collision driver and the effective-metric arrays of the discrete
integrator. Both now return `mjSTATUS_OOM`. The arena allocations with fallbacks, contacts and
constraint rows dropped with a warning, are unchanged, as asked.

**Composition.** Every function that can run out of memory returns `mjtStatus`, or a negative
count for the helpers that return counts, and its callers stage the call. The must-use attribute
of `mjNODISCARD` found every dropped status in the engine's C sources during the conversion;
the count helpers have no such guard and were checked by hand.

## Footprint

Measured against `mjok-returns`, the narrowed status design this is built on:

| | `mjok-returns` | `mjok-oom` |
|---|---|---|
| engine functions returning `mjtStatus` | 48 | 145 |
| public pipeline functions returning `mjtStatus` | 26 | 44 |
| stack frames | 102 plain | 84 checked, 18 plain |
| frame checks | 0 | 157 |
| `mjSTAGE` sites | ~90 | 276 |
| diff in `src/engine` and `include` | | +1304 / -588 lines |

The 18 public signatures that change from `void` to `mjtStatus`: `mj_fwdKinematics`,
`mj_fwdVelocity`, `mj_invVelocity`, `mj_invConstraint`, `mj_sensorPos`, `mj_sensorVel`,
`mj_sensorAcc`, `mj_energyVel`, `mj_flex`, `mj_tendon`, `mj_transmission`, `mj_passive`,
`mj_subtreeVel`, `mj_rne`, `mj_referenceConstraint`, `mj_jacPointAxis`, `mj_applyFT`,
`mj_multiRay`. The narrowing of the status design, 26 reporting functions out of 52, does not
survive: almost every pipeline function allocates scratch, so almost every one can fail. The
functions left `void` are those that allocate nothing: `mj_kinematics`, `mj_comPos`,
`mj_camlight`, `mj_crb`, `mj_makeM`, `mj_comVel`, `mj_rnePostConstraint`, `mj_energyPos`,
`mj_solveM`, `mj_solveM2`, `mj_constraintUpdate`.

The 18 plain frames left are outside the pipeline: `mj_setConst` and the length-range
computation, `mj_resetData`, the visualizer, `mj_ray`, the dense derivative helpers, and the
public wrappers described below.

## The awkward cases

**Helpers that return numbers.** Twenty-two functions on the paths from a public entry to an
allocation return a count, a distance or a pointer. Counts became `-1` on failure
(`mj_contactJacobian`, `mj_jacSum`, `mj_ne`, `mj_nc`, `mj_broadphase`, `mj_SAP`, the collision
kernels through `mjc_penetration`, `mju_cholFactorSymbolic`, `mju_sqrMatTDSparseSymbolic`,
`mjd_flexStiff_assemble`, ...); `cell_pos_and_jac` returns `NULL`; `PrimalSearch` gained a status
out-parameter. `mj_SAP` had used `-1` for an input-check failure, which became an error inside
it. Two public numeric utilities had their scratch moved to the caller, the option suggested:
`mj_tendonBias` allocates once for all tendons and calls an internal `tendonDot`; the sensor
pipeline allocates a CCD buffer and hands it to `mj_geomDistance` through the buffer the
collision workers already use. The public `mj_tendonDot` and `mj_geomDistance` keep their
signatures, allocate on a plain frame, and raise on overflow: outside the pipeline the stack
allocator is treated as engine-internal, and nothing was done for callers outside the engine.

**Nested frames in one function.** `mj_narrowphase`, `mj_collision`, `mj_instantiateEquality`
and `mj_flex` open a frame inside a frame. A check on the inner frame has to release both, so
`mjSTACKCHECK` does not fit and the check is written out (eight sites). The sweep found the first
of these as a leaked frame; the pattern is easy to miss by eye and hard to check mechanically.

**Use between allocation and check.** A check must precede every use of the pointer, including
a `mju_zero` on the line after the allocation. The sweep found four of these as null
dereferences after the conversion looked complete (`mj_island`, `mj_computeSensorAcc`,
`cell_pos_and_jac`, `mj_instantiateEquality`). The discipline is simple but nothing enforces it;
this is the largest maintenance cost of the design.

**The state after a failure.** The contract is the stop contract of `onwarn`: no rollback, the
data partially updated, and a reset before the data is used again, as after an intercepted error
today. The sweep resets between calls for that reason. (It also turned up an unrelated,
pre-existing failure: `mjd_transitionFD` on a model with sleeping trees raises `found sleeping
tree in island`, on `mjok-returns` as well; the probe skips those models.)

**Function pointers.** Task functions and the collision kernel table are `void` and `int`
respectively and stay so; they report through their arguments and their counts. This keeps the
dispatcher generic and the kernel table unchanged, at the price of two conventions to remember.

**Public utilities that allocate.** `mj_applyFT`, `mj_jacPointAxis` and `mj_multiRay` are
utilities users call directly, and they now return a status because the pipeline calls them.
`mj_geomDistance` and `mj_tendonDot` kept their signatures only because their scratch could be
hoisted.

## Exercise

`engine_oom_test.cc` sweeps the arena of four models (humanoid, islands, flex, tendon wrapping)
from its default size down to just above the reserve, halving each time, and runs the step, the
inverse, the finite-difference derivatives and a reset-and-forward on each: every call returns
`mjSTATUS_OK`, a warning, or `mjSTATUS_OOM`, mirrors it in the data, and leaves the stack empty.
The island model is swept again under each solver with four worker threads. A further test shows
the failure stops the step before the state advances under every `onwarn` setting, is repeatable,
and that a data with the default arena completes the step.

The same sweep, run as a probe over the 220 models of the repository (flex, adhesion, tactile and
arch models included), completed with no crash and no leaked frame: 2048 calls ran out of memory
and 12457 completed in the debug build, 2117 and 12418 under the address sanitizer, whose build
instruments the stack frames and red-zones every allocation. The OOM and onwarn test binaries pass
under the sanitizer as well. Memory Sanitizer is not available on macOS; that run is left for a
Linux box.

## Assessment

What explicit propagation buys: a host that runs out of `mjData` workspace gets a status instead
of `exit()` or a `longjmp` through C frames, on every thread, with no setjmp at entries, no
thread-local boundary chain and no callback-arming, which were the costs of the general design.
The threaded case fell out of the frame mechanism and two conventions for task reporting.

What it costs: the conversion touched 145 functions and added 157 checks and 180 stage sites,
and the maintenance burden is a discipline, not a structure: every new allocation needs a check
before the first use of its pointer, every new frame must be checked, every nested frame must
release its parent on the way out, and the count-returning helpers need a sentinel and a
caller that tests it. The compiler enforces the propagation of statuses (`mjNODISCARD`); nothing
enforces the checks. The reachability test of the status design will flag a public function that
can reach an allocation and does not report, which catches the signature but not the check.

A middle ground worth considering, if the prototype is not adopted as is: keep the mechanism
(checked frames, the reserve, the dispatcher's status) but convert only the frames that hold the
large allocations, the constraint assembly, the solvers, the collision buffers and the derivative
matrices, leaving the many small `nv`-sized scratch frames plain. Those are the allocations that
fail in practice, since a stack that cannot fit a Jacobian row fails long before it cannot fit a
frame's worth of small vectors; the plain frames would keep raising, and the signature change
would stay close to the 26 functions of the status design. The prototype converted everything on
the pipeline's paths so that the sweep could exercise every failure mode; the numbers above are
the cost of that completeness.
