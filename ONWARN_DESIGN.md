# `onwarn`: unified handling of simulation warnings

*Design, 2026-09-09. Branch `mjok`, second commit. Supersedes the 2026-08 proposal on branch
`onwarn`, whose status-return part was withdrawn; `DIAGNOSTICS_SURFACE_OPTIONS.md` (branch
`diagnostics-design`) records how the field design was chosen. A separate branch,
`mjok-noexit`, explores recovering errors into the negative range of the same status; nothing
here depends on it, and this design is complete without it.*

## Summary

The seven `mjtWarning` events are near-fatal: the engine recovers and continues, but the
physics is impaired — from dropped contacts to a fully reset state. Handling used to be an
ad-hoc trio: per-`mjData` counters with first-occurrence printing, hardcoded recovery actions,
and one disable flag (`mjDSBL_AUTORESET`) gating only the state resets. There was no way to
*stop* on a warning, and no per-call signal: callers polled counters or watched for time going
backwards. Studio's step loop had a private `kDiverged`/`kAutoReset` status enum downstream
(`step_control.cc`) — evidence that this belonged in the engine API.

Three changes:

1. **An option attribute** selecting what the engine does after a warning:

   ```xml
   <option onwarn="auto"/>      <!-- auto | continue | stop -->
   ```

2. **A status field.** `mjData.status` reports what the most recent pipeline call ran into.
   The public functions keep their `void` signatures. In all modes the field reports what
   occurred; the mode only controls what the engine does about it.

3. **`mjDSBL_AUTORESET` is removed**, subsumed by the attribute: flag enabled (default) ≡
   `auto`, flag disabled ≡ `continue`.

There is deliberately no "fatal" mode. `stop` gives the caller a clean point to escalate
(`mj_step(m, d); if (!mjOK(d)) mju_error(...)`); routing warnings into `mju_error` inside the
engine would poison multithreaded rollouts. Bindings can layer raise-on-status sugar on top.

**Why a field rather than a return value.** The first draft changed ~28 public functions from
`void` to `mjtStatus`. That is a permanent widening of the API surface, and the field is
needed regardless by the consumers that never see a return value: callbacks and post-step
hooks receiving only `(m, d)`, and trajectory recorders snapshotting `mjData`. `void → non-void`
is also the non-breaking direction, so shipping the
field first keeps returns available as later sugar, whereas the reverse order would break
`if (mj_step(m, d))` callers one release after teaching it.

## Saves vs. resets

The mode semantics rest on a distinction already present in the code:

- **Saves** keep the current step well-defined and are constitutive of continuing at all:
  zeroing a bad ctrl before use, dropping contacts/constraints that don't fit in the arena,
  clamping a singular pivot during factorization. These have always been unconditional —
  `mjDSBL_AUTORESET` never gated them — and remain so in `auto` and `continue`.
- **Resets** (`mj_resetData` on bad qpos/qvel/qacc) rewrite the trajectory: time goes
  backwards. These are what the old flag gated, and what `onwarn` now gates.

The default mode is named `auto` — "the engine applies the per-event recovery" — rather than
`reset`, which would misdescribe the four warnings that never reset anything. The third mode
is named `stop` rather than `return`: every mode records a status, so "return" names nothing
unique; what is unique is that the step stops early.

| event | `auto` (default) | `continue` | `stop` |
|---|---|---|---|
| BADQPOS / BADQVEL | warn, reset | warn, proceed with bad state | warn, return (nothing touched) |
| BADQACC | warn, reset, re-run forward | warn, proceed | warn, return before integration |
| BADCTRL | warn, zero ctrl copy, proceed | same as `auto` | warn, return before actuation |
| CONTACTFULL / CNSTRFULL | warn, drop, proceed degraded | same as `auto` | warn, return at stage boundary |
| INERTIA | warn, clamp pivot, proceed | same as `auto` | warn, return after factorization |

`auto` and `continue` differ only for the three state checks; elsewhere there is only one
way to continue, and the modes coincide.

**Why `continue` exists** (and is not merely legacy): an automatic mid-episode reset produces
data that *looks* valid — a silent teleport corrupting a training batch. NaN-poisoned
trajectories are detectable and maskable; batch RL users legitimately prefer them. It is also
today's MJX semantics (NaNs propagate freely), and the debugging mode for watching a
divergence evolve.

**Why `stop` exists**: it is the missing "stop at the scene" — the state preserved for
autopsy, the failure reported in-band. The old nearest approximation (disable autoreset, poll
counters, pause) keeps integrating garbage until the caller notices.

## Nomenclature

Three different things share the words "warning" and "error":

1. **Log messages** — the transport layer: `mjfLogHandler`, `mjLogMessage`, severity levels
   (`mjLOG_DEBUG/INFO/WARNING/ERROR`), topics, `MUJOCO_LOG.TXT`. Stateless communication to
   humans and handlers. Here "warning" is a *severity label*.
2. **Simulation warnings** — the seven `mjtWarning` events: detected runtime conditions with
   per-`mjData` statistics, a documented recovery (save or reset), and — this design — a status
   and a policy option. Here "warning" names an *event category*. Each event also emits one
   warning-level log message on first occurrence: the log is how it is *reported*, not what it
   *is*.
3. **Errors** — fatal by contract: the handler must not return, and the default one exits.
   Unchanged here, and named only to keep the three apart.

The collision is "warning" as severity vs. "warning" as event. **Decision:** resolve at the
documentation level. The C names stay; docs consistently say *simulation warning* for the
events and reserve bare "warning" for the log level; the `siError` section introduces the
two domains explicitly. (A rename of the events to *faults* — the control-systems term for
exactly this semantic — was considered and rejected: the blast radius of renaming a public
enum and an `mjData` field is too large for the gain.)

## `mjtStatus`

```c
typedef enum mjtStatus {            // status of a pipeline call, stored in mjData.status
  mjSTATUS_OK          = 0,         // nothing to report

  // simulation warnings, mjtWarning + 1
  mjSTATUS_INERTIA     = 1,
  ...
  mjSTATUS_BADCTRL
} mjtStatus;
```

A signed code, not a bitmask. A call reports its *first* warning: `mj_warning` writes
`d->status` only while it is still `0`, so the field never needs `|=` plumbing and a caller
can `switch` on it in every mode. The counters in `d->warning` remain the full record of a
cascade — status is per-call, counters are history — and per-event detail (which DOF, which
actuator, how many bytes) stays in `d->warning[w].lastinfo`.

A bitmask was the first draft's choice, for the natural composition `status |= child()` and to
keep every event of a cascade. It was dropped with the return values: once the field is the
only surface, keeping first is simpler, and a signed code leaves the negative range free for
whatever a later design wants of it, which a bitmask does not. A by-value struct was rejected
outright: it freezes its ABI at first release, the same trap that forced the INERTIA enum reuse in
`mjd_effSolve`.

One wart disappears: the manual counter re-bump in the check functions existed to survive
`mj_resetData` wiping the stats; it moves inside the reset branch, fixing the double-count
when not resetting. The reset path restores `d->status` and `d->nested` along with the
counters.

**`mjOK(d)`** in `mjmacro.h` is `((d)->status == mjSTATUS_OK)`, following the `mjDISABLED`
pattern. A companion `mjCHECK(d)`, raising `mju_error` with the warning text, was considered
and dropped: escalation is two lines at the call site, and the macro would have been the one
piece of the design that reintroduces `exit()`.

## Control flow

**Leaves report.** `mj_warning` is the engine's own mechanism and is deprecated as an interface
for user code: called from a callback it injects a warning into whatever call is running, which
is not something a caller should be able to do. That settles what was an open question here —
plugins report through log messages, not simulation warnings. Warning sites call `mj_warning`,
which counts, prints on first occurrence,
and writes `d->status` if it is still `mjSTATUS_OK`. It is the single writer of the field and
returns `void`; no status travels along the internal chains.

**The status belongs to the outermost pipeline call.** Each of the 53 pipeline functions — the
public functions of the *Main simulation*, *Components*, *Sub components* and *Derivatives*
sections of `mujoco.h` that take a non-const `mjData` — begins with `mjENTER(d)` and ends every
return path with `mjLEAVE(d)` (`engine_core_util.h`). The outermost call on a `d` clears
`d->status` and sets `d->nested` for its duration; nested calls — the stages a driver runs, or
pipeline calls made from callbacks on the same `d` — leave both alone and report into the
enclosing call. A call on a *different* `mjData` from a callback is outermost for that data.
`d->nested` is a plain field, zeroed by `mj_makeData`, `mj_copyData` and `mj_resetData`. It is
kept out of the X macros, alongside `threadlock`: it is set only while a call is in progress,
so it is always false in an `mjData` a caller can observe, and a binding that let it be written
would corrupt the call it describes. The pair is the same balanced discipline as
`mj_markStack`/`mj_freeStack`, and
a scanner test checks the pairing.

A log handler that intercepts an error and transfers control out of the engine — which is how
the bindings raise `FatalError`, and how a Python exception in a callback leaves — skips the
`mjLEAVE` of every call it unwinds, so `d->nested` stays set. That is not a new hazard: the
same jump skips the `mj_freeStack` of every stage it unwinds, so the `mjData` of an abandoned
call already had to be reset before reuse. `mj_resetData` clears the flag along with the stack,
and the contract is stated where the field is documented rather than worked around.

Recovering the flag from the thread instead was tried and dropped. Nothing the thread knows can
tell an abandoned call from a live one — a count of calls in progress says how many are
running, not which of them the jump crossed, so it either leaves an abandoned data stuck or
clears the status of an enclosing call that is still running. A stack of the calls, exact
enough to answer, is thread state that no public call can clean up: it outlives a failed
`mj_compile`, and it accumulates across error-and-reset cycles in an ordinary native host. The
unwind information that would settle it is a boundary at each entry, which is the subject of a
separate branch and not of this one.

**Drivers branch once.** The places that already contained bail-or-degrade control flow
(arena-full macros in constraint and island assembly, contact insertion, the three state
checks, ctrl zeroing) consult `mji_stop(m, d)`: in `auto` and `continue` they proceed as
before; in `stop` they unwind. Between stages a driver reads as its list of stages, each
wrapped in `mjSTAGE(mj_fwdVelocity(m, d))` — call the stage, leave the call if it stopped —
with `mjSTAGE_(call, cleanup)` for a driver that owns a stack frame across its stages. Policy
never smears below the drivers, and unwinding is ordinary return plumbing.

**Entries just return.** There is no escalation layer, since there is no fatal mode.

**Threading.** All warning sites execute on the calling thread. The engine's `mju_dispatch`
sites (`collisionTask`, `solveIslandTask`, `tactileTask`) contain none: collision workers write
into pre-sized thread-local buffers, and the arena allocation and the CONTACTFULL merge happen
after the join. No cross-thread status plumbing is needed.

**`stop`-mode data contract.** The engine returns at the first driver-level boundary after
detection. `mjData` is valid through the last completed stage; later-stage arrays are stale or
cleared. Saves intrinsic to detection have already been applied (the contact was dropped, the
pivot was clamped); saves that merely enable continuation have not (ctrl is not zeroed —
`d->ctrl` is pristine for inspection, courtesy of the local-copy design). No reset is
performed. Supported operations afterwards: inspect, `mj_resetData`, restore a state. Calling
step again without fixing anything re-detects and stops again.

**Re-entry composes.** `mj_RungeKutta` calls `mj_forwardSkip` per stage; `mj_checkAcc` in
`auto` mode re-runs `mj_forward` after resetting. Both are nested calls that report into the
enclosing status.

## Signatures

The public pipeline functions stay `void`. Two decisions from the withdrawn return-value draft
survive it:

**`mj_addContact`** keeps its documented `0/1` return. Redefining it as a status code was
proposed while returns were on the table; with the field carrying the status there is nothing
to gain from changing a working contract.

**`mj_factorI`** is MJAPI but internal-header-only, and as a pure kernel holds no `mjData` — it
cannot warn. The division of labor: kernels report facts (the first clamped DOF),
`mjData`-holding callers convert facts to a warning. The INERTIA clamp-and-warn path had been
silently dead since the Feb 2025 CSR switch; its restoration lands first, as a standalone
commit with no `onwarn` dependency, and extends the warning to the qH factorizations in
`mj_EulerSkip` and `mj_implicitSkip`, which under the old code raised a fatal error or nothing
at all. Note the user-visible effect of the restoration alone: since the guard was lost, a
singular pivot yielded huge or negative `invD` and surfaced downstream as a mystery BADQACC
autoreset; with the clamp restored these are warned, finite steps again.

## Removing `mjDSBL_AUTORESET`

- `mjDSBL_AUTORESET = 1<<16` deleted from `mjtDisableBit`; the three later bits
  (`NATIVECCD`, `ISLAND`, `MULTICCD`) renumber and `mjNDISABLE` drops to 19 (precedent:
  earlier flag removals renumbered). Everything indexed by flag order is swept: the
  `mjcf.schema` keyword table, simulate's flag-name UI arrays, the USD decoder, Studio's
  `step_control.cc`. Checked-in generated files that carry the enum (the Python introspect
  tables, the WASM bindings) are regenerated; the Unity C# bindings are generated by tooling
  that does not live in this repository and are regenerated at import.
- `mjOption` gains `int onwarn` (`mjtOnWarn`: `mjONWARN_AUTO = 0` default,
  `mjONWARN_CONTINUE`, `mjONWARN_STOP`).
- XML: `<option onwarn="auto|continue|stop"/>` keyword attribute (precedent: `integrator`,
  `cone`). Writer emits only when non-default.
- Migration: hard removal — `<flag autoreset="disable"/>` becomes a schema error; the
  changelog carries the one-line replacement (`onwarn="continue"`). A grace-period mapping was
  considered and rejected: the flag is rarely used, the shim would need a second removal
  commit, would keep a zombie attribute in the published schema, and had a precedence wart
  (the mapped flag stomping an explicit `onwarn`). The USD schema is the exception: its
  `mjc:flag:autoreset` attribute is published separately, so the decoder maps a disabled flag
  to `onwarn=continue` rather than dropping it. Replacing that attribute with an onwarn token
  belongs to the USD schema owners.
- Studio: `step_control.cc` drops the counter-polling and time-went-backwards inference.
  Studio and simulate both pause on a negative status, and under `stop` on any warning that
  stopped the step.

## Bindings and downstream

- Python: `data.status` reports the status; `mj_step(m, d, nstep)` reports the first warning of
  the `nstep` steps, as one call would, and stops early on an error. Optional sugar (a
  raise-on-status helper or context) needs no engine involvement.
- dm_control: nothing to do — the signatures did not change.
- MJX and MJWarp: out of scope. Their `Data` carries neither `warning` nor `status`, so there
  is nothing to mirror; a diagnostics story there would be new surface in an execution model
  with no exceptions and no in-band failure. Their current NaN-propagation semantics are
  exactly `continue`, which the MJWarp option documentation now says.

## Documentation

The documentation was the primary deliverable; the code change was comparatively easy. Where
it landed: `programming/simulation.rst` carries the canonical treatment, its `siError`
introduction separating the event categories from the logging transport and `siSimWarning`
owning the events — moved out of `siDiagnostics`, which keeps only the counter statistics and a
back-link. `XMLreference.rst` documents the attribute, `APItypes.rst` the two new enums,
`APIglobals.rst` the `mjOK` macro. `computation/index.rst`, `overview.rst`,
`modeling.rst` and `python.rst` are updated where they described autoreset semantics as
hardcoded, and the MJWarp option list records that `onwarn` is not available there.

## Open questions

1. Future absorption of stateless step-path `mju_warning`s (EPA out-of-memory, mesh_support,
   flex bending-damping, mixed solref) into `mjtWarning`/`mjtStatus` — blocked today by
   `d->warning[mjNWARNING]` sizing (an `mjData` ABI break), the same constraint that forced
   `mjd_effSolve` to reuse INERTIA.

## Appendix: pre-change audit (@ `fca913b4`)

*The state of the code this design was written against; line numbers are stale, the shape is
not. Kept as the record of which sites the policy has to cover.*

### Leaf warning sites — 18 live + 1 dead

| site | warning | enclosing function |
|---|---|---|
| engine_forward.c:59 / 80 / 101 | BADQPOS/QVEL/QACC | `mj_checkPos/Vel/Acc` (public void) |
| engine_forward.c:396 | BADCTRL | `mj_fwdActuation` (public void) |
| engine_collision_driver.c:2029 | CONTACTFULL | `mj_narrowphase` (static void, post-join) |
| engine_collision_driver.c:2480 / 2588 / 2680 | CONTACTFULL | `mj_collideGeomElem/Elems/ElemVert` (internal void) |
| engine_core_constraint.c:402 | CONTACTFULL | `mj_addContact` (public int) |
| engine_core_constraint.c:148 | CNSTRFULL | `arenaAllocEfc` (static int) |
| engine_core_constraint.c:2936 / 2953 / 2986 | CNSTRFULL | `mj_makeY` (static void) |
| engine_core_constraint.c:3032 / 3049 / 3081 | CNSTRFULL | `mj_makeAR` (static void) |
| engine_island.c:67 | CNSTRFULL | `arenaAllocIsland` (static int) |
| engine_derivative.c:3368 | INERTIA (enum reuse) | `mjd_effSolve` (MJAPI-internal void) |
| engine_core_smooth.c `mj_factorI` | INERTIA (dead since Feb 2025) | via `mj_factorM`, `mj_EulerSkip`, `mj_implicitSkip` |

### Chains to the public surface

- `mj_collideGeomElem/Elems/ElemVert`, `mj_narrowphase`, `mj_collideTree`,
  `mj_collideFlexInternal/SAP` — all under `mj_collision`.
- `mj_addContact` ← collision driver (6 sites) + user code.
- `arenaAllocEfc` ← `mj_makeConstraint`; `mj_makeY`/`mj_makeAR` ← `mj_projectConstraint`;
  `arenaAllocIsland` ← `mj_island`.
- `mjd_effSolve` ← `mj_fwdAcceleration`.
- `mj_factorI` ← `mj_factorM`, `mj_EulerSkip`, `mj_implicitSkip`.
- `mj_fwdPosition` aggregates five sources: `mj_factorM`, `mj_collision` (×2 call sites),
  `mj_makeConstraint`, `mj_island`, `mj_projectConstraint`.
- `mj_invPosition`: `mj_factorM`, `mj_collision`, `mj_makeConstraint`, `mj_projectConstraint`.
- Entry composition: `mj_step` = checks + forward + integrator; `mj_step1` = checks +
  position + velocity; `mj_step2` = actuation + acceleration + constraint + checkAcc +
  integrate; `mjd_transitionFD`/`mjd_inverseFD` → `mj_forwardSkip`/`mj_inverseSkip`.

### Threading

`mju_dispatch` sites: `collisionTask` (engine_collision_driver.c:2011), `solveIslandTask`
(engine_forward.c:1196, 1210), `tactileTask` (engine_sensor.c:1242). None contains a warning
site; all seven warnings fire on the calling thread.
