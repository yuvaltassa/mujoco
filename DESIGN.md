# A retained mjvScene

*Branch-scoped design doc for the `vis-policy` branch. Rides the branch for
review and does not land. It sits at the bottom of the branch and describes
everything above it, including measured results; it is not modified by the
commits it describes.*

## Summary

mjvScene gains an opt-in **retained mode**. A retained scene is laid out
as fixed per-model-element slots plus an arena for decor, and is updated
in place by **`mjv_syncScene`** — the retained counterpart of
`mjv_updateScene` — which compares as it fills and records what changed
in the scene's change list, with **`mjtSyncBit`** naming the four kinds
of change: mesh, pose, material, visibility. A retained renderer's frame
loop is then two steps — sync, apply the listed changes — and it keeps
only the code that talks to its own graphics API: no scene walk, no
diffing, no identity bookkeeping, because the producer that computes the
state also knows what changed. The semantics live in one place; syncScene
runs exactly updateScene's per-element policy.

Compatibility: immediate mode is bit-for-bit unchanged, mjvGeom's ABI is
untouched (bookkeeping lives in parallel scene arrays), and
`mjv_addGeoms`/ghost recipes work verbatim on retained scenes. Measured
here: against the bridge at head, with everything moving, bridge work
drops 9x at 3641 geoms (3.45 to 0.38 ms) — and to zero when nothing
moves — with 38 of 40 compared frames byte-identical (the other two are
frame zero, rendered with the correct camera).

## Problem

`mjv_updateScene` is immediate mode: the producer regenerates the scene
every frame and forgets it, so a stateful renderer must recover the
frame-to-frame delta on its own. The tree contains both known strategies
and each gives up half of what we want:

- **Rebuild everything.** SceneBridge at head destroys and recreates every
  renderable every frame ("Remove all drawables from previous render") —
  full mjv semantics, model elements, decor, ghosts, labels, plugins, at
  the cost of unbounded churn against a renderer that is designed to
  retain.
- **Re-derive per element.** A model-indexed renderer that reads
  mjModel/mjData directly is fast — no scene walk, no churn — but the
  visual semantics (category and group rules, island and sleep coloring,
  transparency, hull switching, plane handling) must be reimplemented
  renderer-side, forking the single source of visual truth and drifting
  from it.

The observation this branch is built on: the producer *computes* the
state, so the producer is the one place the delta is available for free.
Make the scene retainable, update it in place, and emit what changed. A
retained renderer then does no bookkeeping at all — its frame loop is
"sync, apply the change list" — as fast as re-derivation (it touches only
what changed) and as powerful as the scene (it *is* updateScene's
semantics).

## Design

### Retained mode

mjvScene remains the single scene type, with a mode chosen at
`mjv_makeScene` time: set `scn->retained = 1` between `mjv_defaultScene`
and `mjv_makeScene` (makeScene latches the request across its internal
reset). Immediate mode is bit-for-bit today's behavior — classic, python,
and every existing consumer see nothing. Retained mode lays the geom
buffer out as:

- **slots** `[0, nslot)`: one per model element (geoms, sites, flexes,
  skins, in model order per type), fixed at makeScene, self-describing
  from birth (objtype/objid set, `segid` = slot index) and invisible until
  the first sync;
- **arena** `[nslot, maxgeom)`: decor families and appended geoms,
  regenerated each sync.

Bookkeeping lives in parallel scene arrays — **mjvGeom is untouched**:

```c
int      nslot;        // number of model-element slots
mjtByte* visible;      // per-slot visibility (replaces omission-from-list)
int      nchanged;     // number of entries in the change list
int*     changed;      // indices of changed entries
int*     changebits;   // what changed: mjtSyncBit (mesh/pose/material/visible)
```

### mjv_syncScene

```c
void mjv_syncScene(const mjModel* m, mjData* d, const mjvOption* opt,
                   const mjvPerturb* pert, mjvCamera* cam,
                   int catmask, mjvScene* scn);
```

The retained counterpart of `mjv_updateScene`. Each slot is filled by the
same per-element policy as the immediate path and compared
field-group-wise against what the slot already holds; differences set
change bits and append to the change list. The comparison happens during
the fill the engine is already doing, against memory it is already
touching — as cheap as diffing can be, with no second copy of the state.
Filtered-out elements (group toggles, alpha) become `visible = 0` — a
visibility change, not a disappearance — and a slot that reappears is
marked for full re-application. Unlike updateScene, the camera is updated
*before* the slots, so infinite-plane re-centering sees the current
frame's camera rather than the previous one (a latent staleness in the
immediate path, visible in its first rendered frame).

The consumer's frame loop:

```c
mjv_syncScene(m, d, opt, pert, cam, mjCAT_ALL, scn);
for (int k=0; k < scn->nchanged; k++) {
  apply(scn, scn->changed[k], scn->changebits[k]);   // renderer-specific
}
```

Two testable contracts, both tested on this branch:

- **Equivalence**: the drawn content of a synced scene equals the content
  of `mjv_updateScene` on the same state, field for field, slots and
  arena.
- **Incrementality**: an element whose state did not change does not
  appear in the change list (guaranteed for slots).

The change list is consume-once: cleared at each sync, valid until the
next. One consumer per synced scene; per-slot version counters are the
known upgrade if multiple independent consumers ever matter.

### Policy functions are the substrate, not the API

`makeGeomGeom`, `makeSiteGeom`, `makeFlexGeom` and `makeSkinGeom` are
**internal statics** shared by syncScene and the immediate-mode
generators — which is what makes the equivalence contract structural
rather than aspirational. Public exposure of a per-element policy call is
deliberately deferred: the retained scene is the low-level interface
consumers actually need, and `mjv_isCatenary` sets the precedent of
exposing policy case-by-case when a concrete external need appears.

### Decor and the arena

Decor has no cross-frame identity, so the arena is regenerated each sync
(plugin geoms, tendons and slider-cranks, then the decor families, in the
immediate path's order) and diffed **positionally** against a snapshot of
the previous sync's arena: stable content produces no change entries,
moved decor produces pose bits, and new positions are marked for full
application. An insertion (a new contact) shifts everything after it and
over-fires change bits for the shifted tail — a bounded perf blip, never
a correctness issue (the results below attach a number to it). Confining
the shift to per-family ranges is a compatible refinement — the snapshot
already holds what it needs — deferred until profiling asks for it, since
it requires plumbing family identity through every generator.

`mjv_addGeoms` and the manual `mjv_initGeom(scn->geoms + scn->ngeom++)`
idiom keep their exact semantics, landing in the arena in both modes, so
ghost and trajectory recipes run verbatim; entries appended between syncs
get no automatic change entries (the appender applies them itself),
vanish at the next sync like all arena content, and are invisible to its
diff — arena shrinkage is observed via `ngeom`.

### The consumer, demonstrated

The last commit on the branch makes SceneBridge consume the change list:
`Update` branches on `scn->nslot`, and the retained path is ~90 lines —
walk `changed`/`changebits`, apply what the bits say, hide the arena tail
on shrink, draw labels if a text callback is set. No diffing, no keying,
no pooling, no shadow copies: the fused create-only path in
scene_geom_util is split into callable applicators (`ApplyGeomMesh`,
`ApplyGeomPose`, `ApplyGeomMaterial`), and per-entry application is
driven entirely by the producer's bits. Two renderable-layer
accommodations ride along: multi-part builtin renderables are
single-assignment, so entries recreate on re-typing and a repeat mesh bit
demotes to pose+material; and `Renderable::RemoveFromScene` now rebinds
the default material instance, since a renderable outside a scene is not
re-prepared and must not keep referencing an instance the MaterialManager
may destroy — required for any consumer that hides rather than destroys.

### What retained mode does not change

Deformable vertex streams (skins, flex tessellation) refresh per sync as
in updateScene, gated on catmask; their slots always report a mesh change
since the streams are per-sync. Moving that work to the GPU (bone poses,
raw vertices, fixed topology) is a producer capability-flag design that
composes with retained mode and is out of scope here. Likewise semantic
fields for multiview (island id, sleep state, filled by the policy
functions instead of baked colors) are a separate, co-designed mjvGeom
addition.

### The sleep dividend (future)

Because filling and diffing are producer-side, syncScene can eventually
skip slots belonging to sleeping trees entirely — sync cost scaling with
the awake set. No consumer-side scheme can do this without reimplementing
sleep semantics; it is the clearest expression of why the delta belongs
to the producer.

## Commits on this branch

1. This doc.
2. **Extract per-element policy functions** from the generators (geoms,
   sites, flexes, skins), plus flex/skin coverage for the scene test.
3. **Retained mode layout** in mjvScene.
4. **mjv_syncScene**: slots, arena, and the contract tests.
5. **SceneBridge consumes the change list**, with the applicator split,
   the renderable accommodations, and the benchmark.

(The sequence is fine-grained for review; commits can be squashed at
landing. Python bindings regeneration for the new fields and function is
deferred to landing polish.)

## Results

`scene_benchmark` (built with `MUJOCO_USE_FILAMENT_MJR_COMPAT=ON`)
renders a simulated trajectory offscreen: 100 frames at 512x512, contact
decor enabled, filament OpenGL backend, Apple M5. "bridge" is
`mjr_render`, i.e. the SceneBridge update; immediate mode is the bridge
at head (rebuild everything), retained is `--retained`. The harness steps
physics every frame, so nearly everything moves — the retained path's
*worst* case, as the changed/sync column shows:

| model         | ngeom | changed/sync | immediate sync+bridge | retained sync+bridge |
|---------------|-------|--------------|-----------------------|----------------------|
| humanoid      |    36 |           35 | 0.002 + 0.048         | 0.002 + 0.005        |
| humanoid100   |   327 |          200 | 0.006 + 0.174         | 0.007 + 0.050        |
| flag (flex)   |     2 |            1 | 0.004 + 0.027         | 0.005 + 0.025        |
| 100_humanoids |  3641 |         3536 | 0.097 + 3.449         | 0.107 + 0.376        |

Readings:

- Bridge work drops about 10x at 36 geoms, 3.5x at 327 and 9x at 3641 —
  with everything moving. The immediate bridge pays for what *exists*;
  the retained bridge pays for what *changed*. In the regimes this
  harness cannot show — resting bodies, camera-only motion, decor-rate
  updates — the change list is empty and the bridge does nothing at all,
  which is unit-tested producer-side.
- Emitting the delta is essentially free at production time: sync costs
  what updateScene costs (0.107 vs 0.097 ms at 3641 geoms).
- The arena's positional over-fire is visible in changed/sync (contact
  churn keeps the counts near ngeom in this regime), and the retained
  bridge still wins 9x; per-family ranges wait for a profile that needs
  them.

Parity: 38 of 40 dumped frame pairs are byte-identical. The two that
differ are frame 0 of two models, where syncScene's current-camera plane
re-centering renders the first frame more correctly than updateScene's
previous-frame behavior; from the next dumped frame on, output is
identical.

Findings for upstream, made while building the demonstration:

- `mjrf_setMeshData` errors when the index count differs from the config,
  and the flex path sizes the config from the first frame's face count —
  flexes with varying `flexfaceused` (3D layer switching) will
  hard-error. Pre-existing; independent of this branch.
- Multi-part builtin renderables (`SetGeomMesh`) are single-assignment:
  re-applying appends duplicate parts. The bridge works around it;
  hardening `SetGeomMesh` for reassignment would remove the workaround.
- A renderable removed from a SceneView must be deregistered before the
  view is destroyed; the bridge destructor does so.

## Non-goals

- **Public per-element policy** (`mjv_objectGeom`-style): deferred;
  expose on concrete demand, per the isCatenary precedent.
- **Tendons and slider-cranks as slots**: variable cardinality; they are
  arena families, diffed positionally like other decor.
- **Deformable stream capability flags** and **multiview semantic
  fields**: separate, co-designed follow-ups as above.
- **Multi-consumer change tracking**: version counters, future.
- **Per-family arena ranges**: deferred until the churn number in the
  results justifies the plumbing.
