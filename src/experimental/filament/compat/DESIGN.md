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

Phase 2 — mjv changes, additive first, breaking last. Not started; specified
here in enough detail to be picked up independently of the phase 1 authors.
Roughly one commit each, and each justified separately by the phase 1
profile:

7. **Gate deformable fill on catmask.** `mjv_updateScene` runs the flex and
   skin fill whenever the vis options ask for it, regardless of catmask, so
   a decor-only update pays the full deformable cost. Gate it on
   `catmask & mjCAT_DYNAMIC`. This makes `mjv_updateScene(..., mjCAT_DECOR)`
   genuinely cheap and unlocks decor-at-display-rate vs physics-at-sim-rate
   consumers — the split SceneDecorator wanted and had to fake.

8. **Consumer capability flags on mjvScene.** New fields set by the consumer
   between makeScene and updateScene; zero (default) = exact classic
   behavior:
   - *skip skinning*: mjv does not compute skinned vertices; it publishes
     per-frame bone poses instead (new fixed-size scene arrays; exact layout
     to be co-designed against filament's native 4-weight skinning), with
     bind-pose vertices and weights uploaded once from the model.
   - *skip flex tessellation*: mjv skips smooth-normal computation and soup
     emission; it publishes raw per-frame flex vertices with topology fixed
     at makeScene.
   - *native infinite plane*: mjv emits the plane without camera-following
     re-centering; the renderer draws a true infinite plane and the
     `GetPlaneTileSize` duplication is deleted.
   The reconciler adopts each flag; classic leaves them all off.

9. **Indexed deformable streams.** Replace the unindexed nine-floats-per-face
   soup with indexed vertices and an index buffer fixed at makeScene; 3D
   flex layers become per-layer index ranges, so a layer switch is a draw
   range change rather than a re-tessellation. Cuts upload bandwidth about
   3x and feeds mjrf_updateMeshVertexData without conversion.

10. **Slotted layout.** The geom list becomes: slots `[0, nslot)`, one per
    model element (geoms, sites, flexes, skins), fixed at makeScene, with a
    `visible` flag replacing omission-from-the-list; then an arena
    `[nslot, ngeom)` regenerated each frame (decor, tendon segments,
    appended geoms). `mjv_addGeoms` keeps its exact semantics — it appends,
    to the arena; only updateScene owns slots — and the manual
    `mjv_initGeom(scn->geoms + scn->ngeom++)` idiom keeps working because
    the arena starts where slots end, so the colab ghost/trajectory recipes
    run verbatim. segid becomes the slot id: stable across frames and
    aligned with model-indexed renderer schemes. The reconciler swaps its
    hash map for direct indexing. This is the one breaking change for
    third-party consumers that iterate `scn->geoms` densely; migration is a
    visibility check.

Open questions, deliberately left unresolved for co-design:

- Bone-pose layout for GPU skinning: driven by what filament (and future
  renderers) want to consume.
- Whether capability flags are new `scn->flags` entries or discrete fields.
- segid canonicalization changes segmentation images for consumers that
  relied on list position; needs a changelog note and possibly a
  transition flag.
- Instance blocks (see non-goals) if ghosts/trajectories deserve
  first-class slots — also the only clean path to per-ghost skin
  deformation.

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

## Appendix: phase 1 as landed

The commits following this doc, in order (titles as committed):

- **Add mjrf_updateMeshVertexData for in-place mesh updates.**[^bugs] The
  one mjrf addition: re-upload vertex data into an existing mesh, with the
  attribute layout fixed at creation, vertex count up to the created
  capacity, and bounds refreshed. Filament's vertex buffers were always
  dynamic; mjrf just didn't expose it.
- **Reconcile model elements against persistent renderables in
  SceneBridge.** One persistent renderable per model element, keyed on the
  `(objtype, objid)` that `acquireGeom` already stamps. `DiffGeom`
  classifies each frame's delta into mesh/pose/material bits against a
  shadow copy of the last-applied geom; unclaimed elements are swept out of
  the scene but kept. The fused create-only path in `scene_geom_util` is
  split into `ApplyGeomMesh` / `ApplyGeomPose` / `ApplyGeomMaterial`.
- **Pool per-frame renderables in SceneBridge by shape.** Everything
  without identity (decor, tendon segments, appended/ghost geoms) claims
  renderables from `(type, dataid)` pools in generation order; the flex
  vertex/edge swarm gets its own sphere and cylinder pools. Steady state
  creates and destroys nothing, anywhere.
- **Update flex and skin meshes in place instead of recreating them.**
  `SceneObjects` uploads into persistent meshes; recreation happens only
  when a frame outgrows the buffers, and the active face count is a draw
  range.
- **Detach material instances when renderables leave the scene.**[^bugs] A
  one-line core fix required for any retained consumer.
- **Add reconciler benchmark and record phase 1 results.** Offscreen
  trajectory rendering through the mjr compat layer
  (`MUJOCO_USE_FILAMENT_MJR_COMPAT=ON`): per-stage timings plus frame
  dumps for cross-build pixel comparison.

Steady-state ms/frame over 100 simulated frames at 512x512, contact decor
enabled, filament OpenGL backend, Apple M5. "bridge" is `mjr_render`, i.e.
the SceneBridge update; "old" is the destroy-everything bridge at this
doc's commit, "new" is the reconciler. The first frame, which creates all
persistent state, costs about what one old-bridge frame did.

| model                  | ngeom | updateScene | bridge old | bridge new | render+read |
|------------------------|-------|-------------|------------|------------|-------------|
| humanoid               |    36 |       0.002 |      0.053 |      0.005 |         1.4 |
| humanoid100            |   312 |       0.006 |      0.190 |      0.028 |         2.2 |
| flag (flex)            |     2 |       0.005 |      0.026 |      0.022 |         1.6 |
| 100_humanoids          |  3641 |       0.084 |      3.514 |      0.439 |        21.2 |

Every compared frame (4 models x 10 frames each) is byte-identical between
the old and new bridge. Three readings of the table:

- Bridge cost drops 7-10x and now scales with what *changes* rather than
  what *exists*; render+read (GPU render, sync, readback) is unchanged, as
  expected -- the churn was CPU/submission-side.
- `mjv_updateScene` regenerates all 3641 geoms in 0.084 ms, 0.4% of the
  frame: the stateless producer is measurably not the bottleneck. The
  contract was.
- Flex moves least because its remaining cost is producer-side
  tessellation and the upload itself -- exactly the phase 2 items
  (capability flags, indexed streams).

[^bugs]: Verification surfaced two latent bugs, both invisible to a bridge
    that rebuilds everything every frame. (a) `Renderable::RemoveFromScene`
    kept the entity's material-instance binding, so `MaterialManager`'s
    per-frame GC could destroy an instance filament still saw attached --
    a precondition panic in `FEngine::destroy`. Unreachable when every
    renderable is destroyed each frame; guaranteed eventually for any
    retained consumer. (b) `Mesh::HasVertexAttribute` scanned the full
    fixed-size attribute array, whose entries beyond the declared count
    are uninitialized -- and the interleaved path never set the count --
    so UV presence, which selects the material variant, could read
    garbage. Both are fixed in the commits marked with this footnote.
    Related, not a bug fixed here: windowless contexts panic when the
    default backend resolves to Vulkan on macOS, so the benchmark
    requests OpenGL explicitly.
