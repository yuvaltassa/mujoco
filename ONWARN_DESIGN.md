# `onwarn`: unified handling of simulation warnings

*Design, 2026-09-11. Branch `mjok-returns`, one commit. The status return of the 2026-08 proposal
on branch `onwarn`, narrowed to the functions that can report one. Branch `mjok` holds the
field-only alternative it was compared against (see *Why a return value* below). A separate
branch, `mjok-noexit`, built on the field design, explores recovering errors into the negative
range of the same status; nothing here depends on it.*

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

2. **A status return.** The pipeline functions that can raise a simulation warning return an
   `mjtStatus` reporting what the call ran into, and record it in `mjData.status` for code that
   sees only the data. In all modes the status reports what occurred; the mode only controls
   what the engine does about it.

3. **`mjDSBL_AUTORESET` is removed**, subsumed by the attribute: flag enabled (default) ≡
   `auto`, flag disabled ≡ `continue`.

There is deliberately no "fatal" mode. `stop` gives the caller a clean point to escalate
(`if (mj_step(m, d)) mju_error(...)`); routing warnings into `mju_error` inside the
engine would poison multithreaded rollouts. Bindings can layer raise-on-status sugar on top.

**Why a return value.** A field-only status was built first, on branch `mjok`, keeping the
signatures `void`. It needed the engine to tell a fresh public call, which clears the status, from
a nested one, which must preserve it: a flag in every `mjData`, an entry and exit pair on every
pipeline function, and a contract for log handlers that transfer control out of a call and leave
the flag set. With returns each function owns its status as a local, keeps the first thing its
stages report, and returns it. A reset cannot erase it, a nested call cannot overwrite it, and
there is no ownership state to repair. The field survives as a mirror of the result, assigned just
before a call returns; the outermost call returns last, so the field ends up holding its value.
The price is a signature change on the functions that report, and the plumbing that carries a
status from each warning site to a public entry.

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

A signed code, not a bitmask. A call reports its *first* warning: each function keeps the first
status its stages return, so the value names the earliest problem, usually the cause of the rest,
and a caller can `switch` on it in every mode; `simulate` and Studio index it as a single warning.
What this gives up is the set of warnings a call raised, which a bitmask would record. The
counters in `d->warning` do not replace that set: they are history, not a per-call record, so
recovering the set means snapshotting them around the call, and an auto reset clears them
mid-call. They do keep the per-event detail (which DOF, which actuator, how many bytes) in
`d->warning[w].lastinfo`.

A bitmask was the first draft's choice, for the natural composition `status |= child()` and to
keep every event of a cascade. Keeping the first composes as simply (`mji_join`), and a signed
code leaves the negative range free for whatever a later design wants of it, which a bitmask does
not. The choice is representational, confined to `mji_join`, the enum values and the two
consumers, and has to be settled before the API is published. A by-value struct was rejected
outright: it freezes its ABI at first release, the same trap that forced the INERTIA enum reuse in
`mjd_effSolve`.

One wart disappears: the manual counter re-bump in the check functions existed to survive
`mj_resetData` wiping the stats; it moves inside the reset branch, fixing the double-count when
not resetting. The reset path restores the counters; the status is a local, so the reset cannot
touch it.

**`mjOK(d)`** in `mjmacro.h` is `((d)->status == mjSTATUS_OK)`, following the `mjDISABLED`
pattern. A companion `mjCHECK(d)`, raising `mju_error` with the warning text, was considered
and dropped: escalation is two lines at the call site, and the macro would have been the one
piece of the design that reintroduces `exit()`.

## Control flow

**Leaves report.** Warning sites call `mj_warning`, which counts, prints on first occurrence, and
returns the status of the warning; the function it was raised in keeps it and returns it, and so
does every frame between that site and a public entry. `mj_warning` is the engine's own mechanism
and is deprecated as an interface for user code: called from a callback it counts and prints, but
cannot reach the call in progress, whose status is a local. That settles what was an open question
here — plugins report through log messages, not simulation warnings — and means that no driver
needs to check a status after calling a callback.

**Which functions report.** A public function returns a status if it can reach a warning, directly
or through a callee: 26 of the pipeline functions. The first draft converted whole API sections
for uniformity; converting only what can report keeps the signature change where it means
something. The rule is then a property of the call graph rather than of the section a function
sits in, so a test recomputes the graph from the sources and fails when a function that can reach
a warning does not return a status: a new warning site in a void function changes that function's
signature, as it should. The converse holds at introduction, but is not a rule for later: a
published signature outlives the warning sites behind it, so a reporting function whose last one
is refactored away keeps returning `mjtStatus`, always OK. The test lists such functions
explicitly and flags any other status that cannot be raised, which also keeps its recomputation of
the graph honest. Callbacks and plugins cannot raise a simulation warning, `mj_warning` being
deprecated for user code, so a function pointer does not hide a source.

**A dropped status is silent.** A caller that forgets to keep what a callee returned loses the
warning, and the code still compiles. The engine's own declarations of every function that returns
a status, the public ones included, carry `mjNODISCARD`, so clang rejects a dropped status
anywhere in the engine's C sources, and a deliberate discard is written as a `(void)` cast: the
compiler, a reset, and a status-free derivative helper. Users and the C++ tests see the plain
declarations and may ignore the result. Keeping a status and honoring `stop` are separate
obligations: the attribute checks only the first, and the second, that no stage runs after the one
that warned, is covered by the stop tests, one per kind of driver.

**Handlers that transfer control.** A log handler that jumps out of a call, as the bindings do to
raise `FatalError`, abandons locals that were never shared, so nothing is left to repair: the next
call on the data records its own status. The abandoned call's stack frame is leaked as it always
was, and `mj_resetData` clears it.

**Drivers branch once.** The places that already contained bail-or-degrade control flow
(arena-full macros in constraint and island assembly, contact insertion, the three state checks,
ctrl zeroing) consult `mji_stop(m, status)` on their own accumulated status: in `auto` and
`continue` they proceed as before; in `stop` they return. A driver reads as its list of stages,
each reporting stage wrapped in `mjSTAGE(mj_fwdPosition(m, d))`, which keeps the stage's status
and returns if the call stopped, with `mjSTAGE_(call, cleanup)` for a driver that owns a stack
frame across its stages; stages that cannot warn are plain calls. The finite-difference drivers
run independent evaluations, composed by `stepSkip` and `inverseSkip`: each keeps the first
warning, runs nothing once the driver has stopped, and ends at the stage that warned, so a warning
in a perturbed evaluation is reported and honored like one in the nominal step. Policy never
smears below the drivers.

**Entries just return.** There is no escalation layer, since there is no fatal mode.

**Threading.** All warning sites execute on the calling thread. The engine's `mju_dispatch`
sites (`collisionTask`, `solveIslandTask`, `tactileTask`) contain none: collision workers write
into pre-sized thread-local buffers, and the arena allocation and the CONTACTFULL merge happen
after the join. No cross-thread status plumbing is needed.

**`stop`-mode data contract.** The engine returns at the first stage boundary after detection,
without rolling back: what ran before the warning is in `mjData`, what would have run after it is
not, and the stage that warned may have run partway (the integrator advances the activations
before the sleep re-forward that can warn; RK4 restores the state and the time but leaves the
position-dependent arrays of its last evaluation). Saves intrinsic to detection have been applied
(the contact was dropped, the pivot was clamped); saves that merely enable continuation have not
(ctrl is not zeroed — `d->ctrl` is pristine for inspection, courtesy of the local-copy design). No
reset is performed. The data is partially updated, for inspection, and the outputs of the call,
derivative matrices included, are not valid. Supported operations afterwards: inspect,
`mj_resetData`, restore a state. Calling step again without fixing anything re-detects and stops
again. One compile-time consumer simulates: `mj_setLengthRange` steps the model until a total time
to find an actuator's length range, and a step that stops cannot advance it, so under `stop` it
returns an error naming the warning rather than wait forever; with the model's own policy in
force, a warning during that simulation is a compile error.

**Re-entry composes.** `mj_RungeKutta` keeps the status `mj_forwardSkip` returns for each stage;
`mj_checkAcc` in `auto` mode re-runs `mj_forward` after resetting and keeps what it returns; the
re-forward in `mj_advance` after islands fall asleep is a stage of the step like any other.

## Signatures

The reporting functions return `mjtStatus`; the rest of the pipeline stays `void`. Calls that
ignore the result are unaffected, and code that stores one of these functions in a pointer to a
`void` function must change the pointer type.

**`mj_addContact`** keeps its documented `0/1` return, and its callers translate a full buffer
into `mjSTATUS_CONTACTFULL`. Returning the status itself would have been truthy-compatible but
breaks callers that compare against one.

**`mj_factorI`** is MJAPI but internal-header-only, and as a pure kernel holds no `mjData` — it
cannot warn. The division of labor: kernels report facts (the first clamped DOF),
`mjData`-holding callers convert facts to a warning. The INERTIA clamp-and-warn path had been
silently dead since the Feb 2025 CSR switch; its restoration landed first, as a standalone
commit with no `onwarn` dependency, and extended the warning to the qH factorizations in
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
- Studio and simulate pause on the status of a step that stopped under `stop`; under `continue`
  and `auto` they keep their divergence checks, the counters and time going backwards, now keyed
  on the option instead of the flag.

## Bindings and downstream

- Python: the reporting functions return the status, and `data.status` mirrors it;
  `mj_step(m, d, nstep)` returns the first warning of the `nstep` steps, as one call would, and
  under `stop` takes no step after it. Optional sugar (a raise-on-status helper or context) needs
  no engine involvement.
- dm_control: nothing to do — its calls ignore the result.
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
