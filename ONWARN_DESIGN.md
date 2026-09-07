# `onwarn`: unified handling of engine warnings

*Design proposal, 2026-08-12. Branch `onwarn`. Line references pinned to `fca913b4`.*

## Summary

The seven `mjtWarning` events are near-fatal: the engine recovers and continues, but the
physics is impaired — from dropped contacts to a fully reset state. Current handling is an
ad-hoc trio: per-`mjData` counters with first-occurrence printing, hardcoded recovery actions,
and one disable flag (`mjDSBL_AUTORESET`) gating only the state resets. There is no way to
*stop* on a warning, and no per-call signal: callers must poll counters or watch for time
going backwards. Studio's step loop implements a private `kDiverged`/`kAutoReset` status enum
downstream (`step_control.cc`) — evidence that this belongs in the engine API.

Proposal, in three parts:

1. **A new option attribute** selecting what the engine does after a warning:

   ```xml
   <option onwarn="auto"/>      <!-- auto | continue | stop -->
   ```

2. **Status returns.** Public pipeline functions change `void → mjtStatus`, a bitmask with
   `0` = nothing happened. In *all* modes the return value reports what occurred; the mode
   only controls what the engine does about it.

3. **`mjDSBL_AUTORESET` is removed**, subsumed by the attribute: flag enabled (default) ≡
   `auto`, flag disabled ≡ `continue`.

There is deliberately no "fatal" mode. `stop` gives the caller a clean point to escalate
(`if (mj_step(m, d)) mju_error(...)`); routing warnings into `mju_error` inside the engine
would reintroduce `exit()` into library code and poison multithreaded rollouts. Bindings can
layer raise-on-status sugar on top.

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
is named `stop` rather than `return`: every mode returns a status, so "return" names nothing
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
autopsy, the failure reported in-band. Today's nearest approximation (disable autoreset, poll
counters, pause) keeps integrating garbage until the caller notices.

## Nomenclature

Three different things currently share the words "warning" and "error":

1. **Log messages** — the transport layer: `mjfLogHandler`, `mjLogMessage`, severity levels
   (`mjLOG_DEBUG/INFO/WARNING/ERROR`), topics, `MUJOCO_LOG.TXT`. Stateless communication to
   humans and handlers. Here "warning" is a *severity label*.
2. **Simulation warnings** — the seven `mjtWarning` events: detected runtime conditions with
   per-`mjData` statistics, a documented recovery (save or reset), and — this design — a
   status return and a policy option. Here "warning" names an *event category*. Each event
   also emits one warning-level log message per reset: the log is how it is *reported*, not
   what it *is*.
3. **Errors** — fatal by contract; handlers must not return.

The collision is "warning" as severity vs. "warning" as event. **Decision:** resolve at the
documentation level. The C names stay; docs consistently say *simulation warning* for the
events and reserve bare "warning" for the log level; the `siError` section introduces the
two domains explicitly. (A rename of the events to *faults* — the control-systems term for
exactly this semantic — was considered and rejected: the blast radius of renaming a public
enum and an `mjData` field is too large for the gain.)

## `mjtStatus`

```c
typedef enum mjtStatus_ {          // status bitmask returned by pipeline functions
  mjSTATUS_OK          = 0,        // nothing to report
  mjSTATUS_INERTIA     = 1 << mjWARN_INERTIA,
  mjSTATUS_CONTACTFULL = 1 << mjWARN_CONTACTFULL,
  mjSTATUS_CNSTRFULL   = 1 << mjWARN_CNSTRFULL,
  mjSTATUS_BADQPOS     = 1 << mjWARN_BADQPOS,
  mjSTATUS_BADQVEL     = 1 << mjWARN_BADQVEL,
  mjSTATUS_BADQACC     = 1 << mjWARN_BADQACC,
  mjSTATUS_BADCTRL     = 1 << mjWARN_BADCTRL,
} mjtStatus;
```

- In `stop` mode exactly one bit is set (the first event unwinds), so callers can `switch`.
- In `auto`/`continue` several bits may accumulate in one call (correlated cascades: arena
  pressure hits contacts, then constraints, then islands).
- Per-event detail (which DOF, which actuator, how many bytes) stays in
  `d->warning[w].lastinfo` — the return value says *which* entries are fresh.

Considered alternatives: a plain enum (`mjSTATUS_OK = 0`, first event wins) loses the natural
composition `status |= child()` and drops later events in a cascade; a by-value struct
freezes its ABI at first release (adding a field later breaks every compiled caller — the
same trap that forced the INERTIA enum reuse in `mjd_effSolve`), whereas an int gains bits
freely. `if (mj_step(m, d))` stays terse in C and maps trivially into every binding. The
unused bit range is reserved (candidates: today's stateless step-path `mju_warning`s, future
hard errors).

`d->warning` is unchanged: cumulative counters since reset, first-occurrence printing,
`lastinfo`. Status is per-call; counters are history. One wart disappears: the manual
counter re-bump in the check functions ([engine_forward.c:63](src/engine/engine_forward.c))
exists to survive `mj_resetData` wiping the stats; it moves inside the reset branch, fixing
the double-count when not resetting.

**`mjData.status`.** The plumbed returns make an accumulator unnecessary as a *mechanism*,
but a passive mirror is cheap and serves consumers that never see return values: post-step
hooks and callbacks receiving only `(m, d)`, trajectory recorders snapshotting `mjData`, and
MJX/Warp, which need a field regardless (there are no return values in JAX). Semantics,
chosen to require no machinery: every public entry point assigns its return value to
`d->status` immediately before returning — "the status of the most recently completed
top-level call"; `mj_resetData` zeroes it; the engine never reads it. Mid-pipeline callbacks
(e.g. `mjcb_control`) therefore see the *previous* call's value — documented; live per-event
data remains `d->warning`. Cost: one int and one store per entry point; severable without
consequence if unwanted.

## Control flow: three layers

**Leaves report.** Warning sites keep calling `mj_warning` (counters + first-print are
diagnostics, unaffected by mode); it returns its bit so sites read
`status |= mj_warning(d, mjWARN_CONTACTFULL, ncon)`.

**Drivers branch once.** The ~dozen places that already contain bail-or-degrade control flow
(arena-full macros in constraint/island assembly, contact insertion, the three state checks,
ctrl zeroing) gain one branch: in `auto`/`continue` proceed as today; in `stop` unwind with
the status. Policy never smears below the drivers. Unwinding is ordinary return plumbing —
the paths already exist (constraint assembly already bails on full, the checks already return
early); we are carrying a value through them.

**Entries just return.** No escalation layer, since there is no fatal mode.

**Threading.** All 18 current warning sites execute on the calling thread (audit below). The
engine's three `mju_dispatch` sites (`collisionTask`, `solveIslandTask`, `tactileTask`)
contain no warning sites: collision workers write into pre-sized thread-local buffers and the
arena allocation + CONTACTFULL merge happens after the join on the calling thread
([engine_collision_driver.c:2026](src/engine/engine_collision_driver.c)). No cross-thread
status plumbing is needed. (Pre-existing and out of scope: a worker-side `mjERROR` at
[engine_collision_driver.c:1946](src/engine/engine_collision_driver.c) and stateless EPA/GJK
`mju_warning`s.)

**`stop`-mode data contract.** The engine returns at the first driver-level boundary after
detection. `mjData` is valid through the last completed stage; later-stage arrays are stale
or cleared. Saves intrinsic to detection have already been applied (the contact was dropped,
the pivot was clamped); saves that merely enable continuation have not (ctrl is not zeroed —
`d->ctrl` is pristine for inspection, courtesy of the local-copy design). No reset is
performed. Supported operations after a nonzero return: inspect, `mj_resetData`, restore a
state. Calling step again without fixing anything re-detects and returns again.

**Re-entry composes.** `mj_RungeKutta` calls `mj_forwardSkip` per stage; `mj_checkAcc` in
`auto` mode re-runs `mj_forward` after resetting; both OR the inner status into their own.

## API change

**Public functions, `void → mjtStatus`** (~28, all currently `void`, from `mujoco.h`):

`mj_step`, `mj_step1`, `mj_step2`, `mj_forward`, `mj_forwardSkip`, `mj_inverse`,
`mj_inverseSkip`, `mj_fwdPosition`, `mj_fwdVelocity`, `mj_fwdActuation`,
`mj_fwdAcceleration`, `mj_fwdConstraint`, `mj_invPosition`, `mj_invVelocity`,
`mj_invConstraint`, `mj_Euler`, `mj_implicit`, `mj_RungeKutta`, `mj_checkPos`, `mj_checkVel`,
`mj_checkAcc`, `mj_factorM`, `mj_collision`, `mj_makeConstraint`, `mj_island`,
`mj_projectConstraint`, `mj_referenceConstraint`.

A few have no site today (`mj_fwdVelocity`, `mj_invConstraint`, `mj_referenceConstraint`) but
sit between sources and entries or share the API section; converting the whole section keeps
signatures uniform and future-proof. The FD derivative wrappers `mjd_transitionFD` and
`mjd_inverseFD` deliberately keep `void`: they run perturbed evaluations that they restore,
so a partially-plumbed status would mislead; callers can consult `mjData.status` and
`mjData.warning`.

**Internal functions**: exported-internal `mj_EulerSkip`, `mj_implicitSkip`, `mjd_effSolve`;
file-local `mj_narrowphase`, `mj_collideTree`, `mj_collideFlexInternal`, `mj_collideFlexSAP`,
`mj_makeY`, `mj_makeAR`; internal-header `mj_collideGeomElem`, `mj_collideElems`,
`mj_collideElemVert`. The statics `arenaAllocEfc`/`arenaAllocIsland` already return 0/1;
their callers translate.

**Compatibility.** `void → mjtStatus` is source-compatible (a discarded scalar return is
legal C/C++) and ABI-compatible in practice (the return travels in a register a `void` caller
never reads). The one diagnostic: code storing `&mj_step` in a void-returning raw function
pointer stops compiling (`std::function<void(...)>` is unaffected — it discards returns).
Changelog line, not a design constraint.

**`mj_addContact`** is the one public non-void function on the chains, documented
"return 0 if success; 1 if buffer full" — already a per-event status in different clothing.
**Decision:** redefine the return as `mjtStatus` (`mjSTATUS_CONTACTFULL` on full):
truthiness-compatible with every `if (mj_addContact(...))` caller; only exact `== 1`
comparisons would notice.

**`mj_factorI`** is MJAPI but internal-header-only, and as a pure kernel holds no `mjData` —
it cannot warn. Division of labor: kernels report facts (first clamped DOF), `mjData`-holding
callers (`mj_factorM`, `mj_EulerSkip`, `mj_implicitSkip`) convert facts to warning + status
bit. **Dependency**: the INERTIA clamp-and-warn path has been silently dead since the
Feb 2025 CSR switch (see `mjWARN_INERTIA_findings.md`, `inertia-clamp` worktree); its
restoration should land first, as a standalone fix with no `onwarn` dependency: the kernel
clamps unconditionally for every caller (the numeric save, including the qH sites) and
returns the first clamped DOF; only `mj_factorM` warns — the exact historical behavior.
Extending the warning to the qH factorizations in `mj_EulerSkip`/`mj_implicitSkip` is
deferred to this design, where they naturally become INERTIA status sources. Note the
user-visible effect of the restoration alone: since the guard was lost, a singular pivot
yields huge/negative `invD` and surfaces downstream as a mystery BADQACC autoreset (or
sign-flipped garbage); with the clamp restored these become warned, finite steps again.

## Removing `mjDSBL_AUTORESET`

- `mjDSBL_AUTORESET = 1<<16` deleted from `mjtDisableBit`; the three later bits
  (`NATIVECCD`, `ISLAND`, `MULTICCD`) renumber and `mjNDISABLE` drops to 19 (precedent:
  earlier flag removals renumbered). Sweep everything indexed by flag order: the
  `mjcf.schema` keyword table, simulate's flag-name UI arrays, USD tokens
  (`mujoco_to_usd.cc`), Studio `step_control.cc`.
- `mjOption` gains `int onwarn` (`mjtOnWarn`: `mjONWARN_AUTO = 0` default,
  `mjONWARN_CONTINUE`, `mjONWARN_STOP`). Struct layout change → routine model-format
  version bump.
- XML: `<option onwarn="auto|continue|stop"/>` keyword attribute (precedent: `integrator`,
  `cone`). Writer emits only when non-default.
- Migration: hard removal — `<flag autoreset="disable"/>` becomes a schema error; the
  changelog carries the one-line replacement (`onwarn="continue"`). A grace-period mapping
  was considered and rejected: the flag is rarely used, the shim would need a second removal
  commit, would keep a zombie attribute in the published schema, and had a precedence wart
  (the mapped flag stomping an explicit `onwarn`).
- Studio: `step_control.cc` drops the counter-polling and time-went-backwards inference —
  `Status::kDiverged` ≈ `stop` mode + nonzero status, `Status::kAutoReset` ≈ `auto` mode +
  nonzero status; expose `onwarn` in the option UI.

## Bindings and downstream

- Python: regenerate; `mj_step` et al. return an `IntFlag`-wrapped status. Optional sugar
  (e.g. a `raise_on_status` helper or context) — no engine involvement.
- dm_control: follow-up sweep for wrappers that assume `None` returns.
- MJX: mirror as a status field in `mjx.Data` (a return value has no JAX analogue); MJX's
  current NaN-propagation semantics are exactly `continue`.

## Documentation audit

The documentation is the primary deliverable; the code change is comparatively easy. Every
existing section that discusses errors, warnings, or logging, with its verdict:

| document | section | work |
|---|---|---|
| `programming/simulation.rst` | `siError` "Errors, warnings, logging" | Rewrite the intro to separate the two domains: the logging transport (levels, handlers, topics — already well documented here) vs. simulation warnings (events). The current tier sentence — "Warnings indicate problematic but non-fatal conditions" — is exactly the conflation the Nomenclature section resolves. |
| `programming/simulation.rst` | `siDiagnostics` "Diagnostics" | The canonical `mjData.warning` paragraph. Full rewrite: events with saves/resets, status returns, `onwarn`, the stop-mode data contract; keep the counters/`lastinfo` description. This becomes the canonical "simulation warnings" section. |
| `computation/index.rst` | pipeline steps 1 and 24 | Both describe the divergence checks with hardcoded autoreset semantics ("If divergence is detected, the state is reset..."). Update for the three modes; cross-link the canonical section. |
| `overview.rst` | line ~750 | "Divergence is endemic to all physics simulation..." paragraph names `mjWARN_BADQACC` and autoreset behavior; update and cross-link. |
| `XMLreference.rst` | `option` / `option-flag` | New `onwarn` keyword attribute entry; deprecation note under `flag/autoreset`. The attribute is added in `src/xml/mjcf.schema` (single source of truth) — parser tables and `XMLschema.rst` regenerate. |
| `APIreference/APItypes.rst` | enums/structs | Add `mjtStatus`, `mjtOnWarn`; reframe `mjtWarning`/`mjWarningStat` entries as the event category; cross-link the log types (`mjtLogLevel`, `mjLogMessage`). |
| `APIreference/functions.rst` | generated | The ~28 signature changes and return-value docs ride on header-comment updates + introspect regen. |
| `APIreference/functions_override.rst` | "Main simulation" intro | Manual prose for `mj_step`/`mj_step1/2`/`mj_forward` gains the status-return description — this is where most readers meet the API. |
| `APIreference/APIglobals.rst` | legacy handlers | `mju_user_error`/`mju_user_warning` docs: unchanged behavior, pointer to the new scheme. |
| `python.rst` | "Error handling" (§443) | Add status returns (`mj_step` returns an `IntFlag`; optional raise-on-status helper); exception/interception story unchanged. |
| `programming/extension.rst` | plugin guidance | Plugins are told to use `mju_warning`-style messages; clarify: plugins emit *log messages*; whether plugins may raise *simulation warnings* via `mj_warning` is an open question below. |
| `modeling.rst` | line ~1816 | "unexplained solver divergence (``badqacc`` warnings)" — cross-link the canonical section. |
| `mjx.rst` / `mjwarp` | — | Note that current NaN-propagation semantics ≡ `continue`; status-field parity is a follow-up. |
| `changelog.rst` | — | Attribute, signature change, flag removal, function-pointer caveat, `mjData.status`. |
| signature sweep | `overview.rst:105`, `programming/simulation.rst:172,209,224,455` | Docs that quote signatures — the overview's `void mj_step(...)` and the pseudocode listings of `mj_step`/`mj_step1`/`mj_step2`/`mj_forwardSkip` — become `mjtStatus mj_...`; the entire top-level engine signature is changing and quoted prose must not show `void`. Re-grep `void mj_` at the end. |

## Open questions

1. Whether plugins may raise simulation warnings via `mj_warning` (today the enum has no
   plugin-appropriate value), or are restricted to log messages.
2. Future absorption of stateless step-path `mju_warning`s (EPA out-of-memory, mesh_support,
   flex bending-damping, mixed solref) into `mjtWarning`/`mjtStatus` — blocked today by
   `d->warning[mjNWARNING]` sizing (an `mjData` ABI break), the same constraint that forced
   `mjd_effSolve` to reuse INERTIA.

## Appendix: audit (@ `fca913b4`)

### Leaf warning sites — 18 live + 1 to restore

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
| engine_core_smooth.c `mj_factorI` | INERTIA (to restore) | via `mj_factorM`, `mj_EulerSkip`, `mj_implicitSkip` |

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
