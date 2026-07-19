# Scene reconciler: retained rendering from mjvScene

*Branch-scoped design doc for the `vis-reconciler` commit sequence. Rides the
branch for review; does not land.*

## Problem

The filament stack currently has three consumers of simulation state:

- **SceneBridge** is correct and complete (everything mjv produces, including
  all decor), but does the only thing mjvScene's contract allows a stateful
  renderer to do: destroy every renderable and recreate it, every frame
  (`scene_bridge.cc`, "Remove all drawables from previous render").
- **RenderableManager** is fast — persistent renderables, model-indexed — but
  gets there by re-implementing mjv's visual semantics in C++: body category,
  island/sleep coloring, tendon limit-impedance coloring, transparency
  scaling, convex-hull switching, slider-crank kinematics, plane tiling
  (`renderable_manager.cc`: "duplicated from engine_vis_visualize.c to ensure
  UV scaling matches"). Two implementations of every visual rule drift; some
  already have (the `mjVIS_TRANSPARENT` toggle restores alpha by multiplying
  by `1/alpha`; no labels; every new mjv feature — e.g. the recent
  surface-velocity arrows — must be ported by hand or silently missing).
- **SceneDecorator** concedes that decor needs mjv, so it keeps a private
  mjvScene — but must run the *full* `mjv_updateScene` (model geoms and
  flex/skin CPU fill included), discard everything that isn't `mjCAT_DECOR`
  ("Ideally, we would only update the mjvGeoms in the scene, but the API is
  not capable of that"), and still churns every decoration every frame.

All three paths also rebuild flex/skin meshes from scratch each frame
(`CreateFlexMesh` / `CreateSkinMesh` / `CreateSkinFlexMesh` → `mjrf_createMesh`).

The claim of this branch: the churn is a property of mjvScene's *contract*
("the list is reborn each frame, identity is position in the list"), not of
abstract visualization. Lights are the existing counterexample: `mjvLight`
carries an `id`, `LightManager` keys on it, and lights update in place with
zero churn — through mjvScene. The geoms fail only because their identity is
implicit.

## Architecture

Keep the producer stateless: mjv regenerates the scene description every
frame, and remains the single source of visual truth for every renderer
(classic, filament, whatever comes next). Make the consumer stateful: a
**reconciler** diffs each frame's mjvScene against its own shadow state and
applies deltas to filament. Same shape as virtual-DOM reconciliation; the
statefulness renderers want lives entirely renderer-side.

Two facts make this work against today's mjvScene, unmodified:

1. **Model elements already carry exact identity.** `acquireGeom` stamps
   `(objtype, objid)` on every geom, site, skin, and flex. A map
   `(objtype, objid) → renderable` gives exact — not heuristic — matching
   today. The slot refactor (phase 2) merely makes this positional and
   official.
2. **Elements with ambiguous ids don't need identity.** Decor is ephemeral by
   nature (a frame decor is 3 cylinders sharing one objid; a contact emits
   several geoms; tendon segments split and merge). These are served from
   pools keyed by `(geomtype, dataid)`, assigned in generation order. mjv
   generation order is deterministic, so stable content ⇒ stable assignment ⇒
   zero steady-state churn. This also covers `mjv_addGeoms` ghosts: a ghost
   humanoid appends the same (mesh, dataid) sequence every frame and gets
   slot-grade reuse from the pool without any identity contract; the colab
   ghost/trajectory idiom runs verbatim.

Correctness never depends on matching quality: the reconciler re-applies
every field of the matched geom every frame (memcmp against a shadow copy,
write on change). A mismatch costs a mesh rebind, never a wrong picture.
Identity is purely a performance knob — which is what makes the pooled,
identity-free arena safe.

Per frame:

1. Classify each `scn->geoms[i]`: model element → keyed lookup; everything
   else → next free renderable in `pool[(type, dataid)]`.
2. memcmp against shadow copy; on change set transform/size/material; on
   `dataid` change rebind mesh (e.g. convex-hull toggle).
3. Mark-and-sweep: unclaimed renderables leave the filament scene but stay in
   their pool. No destruction in steady state.
4. Flex/skin: persistent meshes sized to makeScene capacities
   (`flexfacenum` is capacity, `flexfaceused` is per-frame count; skin vertex
   counts are constant), per-frame vertex re-upload + draw range.
5. Lights: unchanged (already keyed).

## What the reconciler cannot fix

Producer-side residuals. Each needs an mjv change (phase 2), and each is
justified independently by profiling phase 1:

- **CPU skinning/tessellation always runs**, even though filament has native
  4-weight skinning → consumer *capability flags* on mjvScene, default off:
  skip-skinning (mjv then provides per-frame bone poses only),
  skip-flex-tessellation (raw `flexvert` + fixed topology).
- **Triangle-soup output** (9 floats/face position + normal + uv, unindexed):
  ~3x upload bandwidth, topology re-expanded per frame → indexed streams with
  topology fixed at makeScene (per-layer index ranges for 3D flex).
- **Infinite-plane re-centering** is a fixed-function-GL workaround the
  renderer must mirror (`GetPlaneTileSize`) → native-infinite-plane
  capability flag.
- **Decor-only updates still pay the deformable fill** (gated on opt flags,
  not catmask) → gate on catmask; makes decor-rate ≠ physics-rate real.
- **segid = list position**: frame-unstable, and already diverges from
  filament's model-based `GetSegmentationId` → canonical stable ids.
- **Identity is provenance, not contract** → slotted model-element section
  (slot = model element, `visible` flag instead of omission) + append arena
  for everything variable. `mjv_addGeoms` semantics are untouched: it
  appends (to the arena); only `mjv_updateScene` owns slots. The manual
  `scn->geoms[scn->ngeom]`/`ngeom++` idiom keeps working because the arena
  starts where slots end.

## Commit sequence

Phase 1 — no public API changes; `scene_bridge` internals plus one mjrf
addition:

1. This doc.
2. **mjrf: updatable mesh vertex data.** Filament has dynamic vertex buffers;
   mjrf only exposes create/destroy today.
3. **Reconciler core:** keyed model elements with shadow-copy diffing and
   mark-and-sweep.
4. **Decor pools:** shape/dataid pools for decor, tendons, ghosts.
5. **Persistent deformables:** flex/skin meshes updated in place (uses 2).
6. **Verification:** rendering parity with the current bridge; frame times on
   humanoid / flex-heavy / contact-heavy models in the commit message.

Phase 2 — mjv changes, additive first, breaking last:

7. Gate flex/skin fill on catmask.
8. mjvScene capability flags (skip-skinning + bone poses in scene,
   skip-flex-tessellation, native-infinite-plane); adopt in reconciler.
9. Indexed flex/skin streams.
10. Slotted layout + `visible` flag + arena; segid = slot id; reconciler
    swaps its hash map for array indexing.

Endgame: one consumer path. The reconciler *is* RenderableManager's
architecture — persistent, model-indexed renderables — fed by mjv's output
instead of re-deriving its semantics; SceneDecorator becomes unnecessary
because decor arrives through the same scene, pooled. `mjr_compat` and studio
sit on top unchanged. A future renderer costs one reconciler, not a second
implementation of `engine_vis_visualize.c`.

## Non-goals, for now

- **Instance blocks** (`(instance, objtype, objid)`) for first-class
  ghosts/trajectories. The arena+pool already gives ghosts full performance;
  instance blocks are a compatible later extension — and the only clean path
  to per-ghost skin deformation, which is broken today (`scn->skinvert` is a
  singleton, so a ghost skin renders the primary's vertices) and stays out of
  scope here.
- **Multi-model assets.** Cross-model `addGeoms` keeps today's rule:
  primitives from anywhere; `dataid`/`matid` must resolve in the uploaded
  model.
