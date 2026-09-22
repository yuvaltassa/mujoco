# Separating the spec from the compilation process

Status: draft for design review, 2026-09-22. Nothing here is decided; the
directions in section 5 are alternatives, not a plan. Written against upstream
main 1d6332316 plus the `frames-roundtrip` stack (c69dff3b1). Line numbers
refer to that tree and are there so that a reviewer can check the claims.

## 1. The question

Should compiling an `mjSpec` be a pure function of the spec, and if so, how do
the things that legitimately need to change the spec (binding names to ids,
fusing static bodies, discarding visual geoms, naming assets after their files,
resolving keyframes, copying edits of an `mjModel` back) get expressed?

The code already contains the mechanism that a pure compile needs: every
`mjCXxx` holds an authored `spec` and a separate compiled working copy, and
every `Compile()` begins by regenerating the copy with `CopyFromSpec()`. This
document is about why that design does not hold today, what it would take to
make it hold, and which parts of the answer are open.

## 2. How the two representations work today

Each element class is laid out the same way, e.g. `mjCGeom`:

- `spec`, an `mjsGeom` value member. This is what the XML reader writes, what
  the `mjs_*` API hands to users (`mjs_addGeom` returns `&geom->spec`), and what
  Python wraps. Its pointer-typed fields alias storage in the C++ object
  (`spec.name = &name`, `spec.userdata = &spec_userdata_`, ...), set up by
  `PointToLocal()`.
- the working copy: a private `mjsGeom` base plus the `_`-suffixed members
  (`userdata_`, `meshname_`, `mesh`, `inertia`, `aabb`, `mass_`, ...). It is
  what `Compile()` transforms and what is copied into `mjModel`.
- `CopyFromSpec()` (`src/user/user_objects.cc:3357` for geoms) assigns the
  spec value onto the base and copies the shadowed strings and vectors. Every
  `Compile()` starts with it (`mjCBody::Compile`, `user_objects.cc:2688`), so a
  second compilation re-derives the working copy from the authored values.

The intent is visible: authored state is never touched by compilation, and the
compiled state is transient. Read, edit and `mjs_*` operate on `spec`; compile
reads `spec` and writes the working copy and `mjModel`.

## 3. Where the design leaks

Four classes, in order of depth, plus the writes that are intended. Each has
produced real bugs in the last weeks; the evidence is listed so the
classification can be argued with.

### 3.1 The writer serialises the working copy

`mjXWriter::Write` refuses a model that has not been compiled
(`src/xml/xml_native_writer.cc:972`) and reads the compiled base everywhere:
`geom->pos`, `joint->axis`, `body->pos`, `actuator->gear`. The whole writer
contains four reads of `->spec.`, all in the flex section. The defaults path is
no exception: `Default()` calls `OneGeom(elem, &def->Geom(), parent)`
(writer.cc:1216), which emits the compiled base of the default's own object;
`def->Geom().spec` appears only as the comparison value that decides whether an
attribute is written at all.

The writer predates `mjSpec`. When `mjCModel` was the only representation, the
compiled object graph was the model, and saving it was the natural thing. When
the authored `spec` was added next to it, the writer was not moved over.

Consequences:

- Every compile-time transformation needs an inverse in the writer, or the
  saved file does not reproduce the model: recovering the pre-mesh-transform
  pose of mesh geoms and sites; recognising inferred `limited`, `actlimited`,
  `ctrllimited`; `FrameLocal` (frames-roundtrip stack), which strips the frame
  transformation from every framed element; and the still-missing inverse of
  free-joint alignment, whose absence is why `<joint align>` is silently baked
  on save.
- Edits of an `mjModel` reach the saved file through the working copy.
  `mjXWriter::SetModel` calls `mj_copyBack` when given a model
  (writer.cc:965), and `mjCModel::CopyBack` (`user_model.cc:5943`) writes
  working-copy fields: `pb->pos`, `pb->mass`, `pb->inertia`, `pa->lengthrange`,
  key state, and so on. Nothing reaches `spec`, so the next `CopyFromSpec`
  discards the edits; they survive only as far as the next save. This is what
  `mj_saveLastXML` is built on ("edit the mjModel, then save"), and
  `DecompilerTest.SaveAndReadXml` tests exactly that path.
- The writer also normalises. `Compiler()` writes `angle="radian"`
  unconditionally (writer.cc:1022) and omits settings whose effect it has
  already baked: `eulerseq`, `inertiafromgeom`, `inertiagrouprange`,
  `settotalmass`. Poses authored as `fromto` or through an orientation
  alternative are written as the resolved `pos` and `quat`; `fullinertia` as a
  diagonal with `iquat`. This is consistent today because everything written
  is compiled, but it means "read authored values instead" is not a local
  change (section 5.1).

What the writer is not responsible for: the places where the compiler writes
into `spec`. `IndexAssets` writes the inferred inertial into `body->spec` when
`discardvisual` removes the geoms that defined it (`user_model.cc:2056`). The
comment there says "for XML writer", but the write is needed regardless of
whether anything is ever saved: once the geoms are gone, the next compilation
has no other source for that body's mass. The cause is destructive lowering on
the authored graph (3.2), and the writer merely benefits. The same write also
flips the global inertia policy from `true` to `auto` (`user_model.cc:2053`),
which changes the meaning of every other body's authored inertial in the model
(5.1).

### 3.2 Structure is not part of the working copy

`CopyFromSpec` covers value fields and, through the `spec_xxx_` shadows,
strings and vectors. It does not cover the object graph: the child lists of a
body, `parent`, the `frame` pointer of an element and the frames themselves,
keyframes, plugin instances, ids. There is exactly one tree, shared by the
authored and the compiled view. Anything that edits structure during
compilation therefore edits the authored model:

- `FuseStatic` (`user_model.cc:4729`, called from `TryCompile` at line 5459,
  after assets and bodies have been compiled) reparents the children of a
  static body and deletes the body. Before the fix on the parked branch it
  left children pointing at deleted frames (read from freed memory on the next
  compile and dropped by the writer), and recompiling restored the children's
  authored poses relative to a body that no longer existed.
- `discardvisual` deletes geoms, and then the materials, textures and meshes
  nothing references any more (`IndexAssets`, line 5491, `DeleteAll`).
- `mjCBase::frame` (`user_objects.cc:1558`) is set by the reader and by
  `mjs_setFrame`, i.e. it is authored, and reassigned by `FuseStatic`, i.e. it
  is compiled. Nobody owns it, which is where the frames bugs of the last week
  lived.
- keyframes: a body deletion moves every key to a pending state and the next
  compile re-creates them as new elements (`ResolveKeyframes`,
  `user_model.cc:5289`); `key->spec.time` is written at compile.
- pairs and excludes are `stable_sort`ed in place at compile and their ids
  reassigned (`user_model.cc:5485`), so compiling reorders the spec's own
  lists.

Both structural transformations consume compiled information. Fusing needs
the mass and inertia of the fused body, which for geom-inferred inertia comes
from compiled geoms and, for meshes, from processed assets; that is why
`FuseStatic` runs where it does. Discarding needs the inertia the discarded
geoms implied. Wherever the transformation ends up living, the resolved
inertia has to be available to it, and after the source bodies or geoms are
gone it has to live somewhere the next compilation can find it: in the
surviving authored body if lowering is in place, or nowhere special if
lowering runs on a copy and the authored geoms survive (5.2).

### 3.3 Compile-only caches that survive between compilations

Working-copy members that `CopyFromSpec` does not reset and a later
`Compile()` reads before writing:

- `mjCFrame::compiled`, set by the first compile and never cleared, so frame
  pose edits were ignored on recompile (fixed in the frames stack, efb7c2421).
- the body BVH: `ComputeBVH` returned early for a body without geoms, keeping
  the previous tree, and `AllocateBoundingVolumes` never cleared the pointers
  to leaf ids, so deleting a body's geoms and recompiling read freed memory
  (fixed, 8260505d4).

This class is small and mechanical. The invariant is easy to state: a
`Compile()` may not read a working-copy member it has not written in the same
call. Nobody has audited for it, and there are certainly more. Note that both
examples only show when the spec is edited between two compilations; compiling
the same spec twice reuses the stale cache correctly (section 6).

### 3.4 Fields with no spec copy

`name`, `classname` and `info` are single strings in `mjCBase`; `spec.name`
points at them (`user_objects.cc:8591`). There is no authored/compiled pair.
So:

- meshes, hfields and textures name themselves after their file when the name
  is empty (`mjCMesh::CopyFromSpec`, `user_mesh.cc:320`; also in `NameSpace`
  and the hfield and texture equivalents, `user_objects.cc:4678, 4947`). This
  writes the authored name during compilation. It is also sticky: change the
  file afterwards and the old stem stays.
- the reference check in `FuseStatic` erases and reinserts names in the
  name-to-id maps while iterating, which is where the stale-id bugs on the
  parked branch came from.

### 3.5 The sanctioned writes

Some compile-to-spec writes are the point: `id` on every element, the model
`signature`, dof and address offsets used by `mj_recompile` to carry state
across. These are binding, the correspondence between a spec and the model
compiled from it. They are not the problem; the problem is that they are not
distinguished from the accidental writes above, so "compile mutates the spec"
covers both. Binding is not a free-standing choice either: if automatic
transformations ever run on a copy or a view of the spec (5.2), the
correspondence between source elements and compiled ids is exactly what
binding has to provide, including for source elements that have no compiled
counterpart (5.5).

What has not been inventoried yet: whether `mj_setLengthRange` results, user
data lengths (`SetNuser`, `def->Compile` "to enforce userdata length for
writer"), and implicit plugin instances created at compile flow back into the
spec. A mechanical inventory is proposed in section 6.

## 4. What "solved" would mean

Candidate invariants, to be argued about:

1. Purity: compiling does not change the authored graph (spec fields,
   structure, names), except for a well-defined binding record. A compilation
   that fails leaves the authored graph as it was.
2. Idempotence: compiling twice yields the same model, bitwise.
3. Round trip: save, load, compile yields the same model as compile, for every
   model that compiles.
4. Explicitness: every transformation that does change the spec (fusing,
   discarding, naming, baking) is an operation with a name, callable on its
   own, requested by the caller, and not interleaved with compilation.

Two distinctions matter for how these are read.

A destructive operation the user asks for (`mjs_bodyToFrame`, `mjs_delete`, a
future `mjs_fuseStatic`) is compatible with invariant 1: the user changed the
spec. An automatic transformation performed because a `<compiler>` flag is
set is not, if it runs on the caller's elements: it deletes and reparents
authored objects the caller may hold pointers to, and a compilation that fails
halfway has consumed source structure. Moving such a step out of `TryCompile`
into a pre-pass changes nothing about that. Under invariant 1, automatic
lowering has to operate on something other than the authored graph.

Resolved values have three distinct uses, and conflating them is what makes
purity look incompatible with the workflows people have:

- Inspection: read a resolved result without touching the source. The stated
  use case for inertia is exactly this: see what the geometry implies while
  keeping inference live. `mjModel` already exposes `body_mass`, `body_ipos`,
  `body_iquat` and `body_inertia` for every surviving body, so a basic
  inspection API is a lookup through binding. What it cannot answer today is a
  per-source question for a body that lowering removed, or a question about a
  spec that has been edited since it was compiled. So an inspection API has to
  say which of two things it is: a view of a particular compilation (cheap,
  bound by ids, stale after an edit) or a recomputation from the current spec
  (a pure function, costs a resolution pass). Both may be wanted.
- Adoption: take selected resolved results and make them authored values.
  This is the only one that mutates the source, it is requested, and it is a
  bake in the sense of 5.1.
- Export: write a file containing resolved or baked values without changing
  the live source. URDF import saving the fused model, `saveinertial`,
  `mj_saveLastXML` after editing an `mjModel`, keyframes resolved against
  qpos0, asset names that were never written: all of these are exports. An
  export can be an adoption applied to a copy, or a serialisation view that
  applies bakes on the way out. Either way the source is untouched, so there
  is no conflict between invariant 1 and saving processed values. Whether the
  default `mj_saveXML` is an export of resolved values (as today) or a save of
  the authored spec is a decision, not a consequence (section 7).

## 5. Directions

These are independent to a degree, but they interact; section 5.6 says how.
None is prescribed.

### 5.1 The writer reads the spec; resolved values are inspected, adopted or exported

Move the writer from the compiled base to `spec`. The inverses in 3.1 are then
deleted rather than maintained, alignment needs no un-bake, and `FrameLocal`
goes away. Saving an uncompiled spec becomes possible.

This is not a field-by-field substitution, for three reasons:

- Authored encodings. `spec.pos` and `spec.quat` are not the pose when the
  pose was authored as `fromto` or through `spec.alt`; `fullinertia` has its
  own representation. Either the writer preserves the encodings (write
  `fromto`, write the alternative that was authored) or it normalises through
  a serialisation view: a pure function of the spec that resolves
  alternatives, `fromto` and units, and is then written. The second keeps the
  output canonical; the first keeps the file closer to what the author wrote.
- Compiler settings. Today's `angle="radian"` and the omitted `eulerseq`,
  `inertiafromgeom`, `inertiagrouprange` and `settotalmass` are correct only
  because the values written are resolved. Writing authored values under
  today's compiler section changes what a reload means (degrees read as
  radians, a non-default Euler sequence dropped). So the choice above has to be
  made for settings as well: write the authored settings, or normalise the
  values. Round-trip tests for degrees, non-default `eulerseq`, `fromto`,
  `fullinertia` and the inertia settings are needed either way.
- `mj_copyBack`. A writer that reads `spec` no longer sees the working-copy
  fields that `CopyBack` writes, so `mj_saveLastXML` after editing an `mjModel`
  would silently lose the edits. `CopyBack` cannot be kept as it is; it becomes
  an adoption, with the contract below, or `mj_saveLastXML` becomes an export
  that never touches the source.

The adoption contract. One contract should cover `mj_copyBack`, inferred
inertia, resolved keyframes, computed length ranges and autonames; the
requirements known so far:

- Values: convert resolved model values back to authored semantics (units,
  frames, each element's own coordinate conventions) and clear the competing
  inputs (`fromto`, orientation alternatives, `fullinertia`) so the authored
  spec is not left contradictory.
- Policy: adopting a value is only meaningful if it then wins. For inertia it
  does not automatically: under `inertiafromgeom="true"`, `mjCBody::Compile`
  runs `InertiaFromGeom` for every body whatever the authored inertial says
  (`user_objects.cc:2757, 2766`), so an adopted `mass`/`inertia` would be
  overridden on the next compile. The discardvisual write-back handles this by
  switching the whole model to `auto` (`user_model.cc:2053`), which silently
  changes every other body whose authored inertial was being overridden. The
  contract has to say what the scope of an adoption is (the whole model, or
  selected bodies), how an adopted value takes precedence (a per-body
  override, or adopting all bodies together with the policy change), and how
  bodies that were not adopted keep their behaviour. The test is: adopt, edit
  geometry, recompile; adopted inertias stay fixed, the others are
  re-inferred.
- Which value: the working copy's mass is not the model's mass when
  `settotalmass` is set (`mj_setTotalmass` runs on the finished model,
  `user_model.cc:5627`). The contract has to name which one is adopted, the
  raw inferred value or the final one, and the same question applies to any
  later global scaling.
- Merged results: a fused body's mass and inertia, or anything else that
  aggregates several source elements, cannot be distributed back over the
  authored elements uniquely. Adoption from an `mjModel` into such a source
  has to be rejected, or take an explicit distribution policy, or be
  redirected to an export of a processed copy. An id table alone cannot
  implement that inverse (5.5).

Open: how many models in the sweep change their canonical output, and whether
Studio or the Python `to_xml` path rely on compiled values in saved XML.
Keyframes need their own look, because their spec side is thin.

### 5.2 Structural transformations leave `TryCompile`

Two different things are bundled under `fusestatic` and `discardvisual` today:
a transformation of the model graph, and the fact that it happens
automatically inside compilation because a flag says so. They can be separated.

The transformations as operators. Fusing decomposes into `mjs_bodyToFrame`
(the frames stack fixed it for this purpose) plus a policy for which bodies
may be fused (referenced by name, gravcomp, plugins, fluid, flexes; the parked
branch has the tests for each). Discarding decomposes into `mjs_delete` plus
an unreferenced-asset sweep. Neither is equivalent to today's code yet,
because of inertia: `FuseStatic` accumulates compiled inertias, whereas
`ToFrame` merges only an explicitly authored child inertial
(`user_objects.cc:2160`). An explicit parent with a child whose mass comes
from its geoms loses that mass on conversion, since the moved geoms do not
contribute to an explicit inertial; the reverse mixture, an inferred parent
with an explicit child, stops counting the parent's own geoms. Mesh inertia
needs processed assets on top. So the operators need a resolution stage they
can call, inertia as a query over resolved geoms and meshes, per source body
and before anything is merged, followed by the structural change and an
adoption of the resolved inertia into the surviving body under the contract of
5.1. Mixed explicit and inferred inertia, with and without meshes, is the
test. The same per-source query is what inspection (section 4) needs for
bodies that lowering removes.

Automatic lowering. If the flags stay, `mj_compile` has to apply the
operators somewhere. Applying them in place on the caller's spec keeps the
current destructive behaviour, now explicit and idempotent, but it does not
satisfy invariant 1 (section 4) and it leaves the pointer question as it is:
the converted body is deleted, only survivors keep their handles. The adoption
of the resolved inertia into the surviving body is then required and
legitimate; what is questionable is only that it happens automatically on the
caller's graph. Applying the operators to a private graph, a copied spec as
the first implementation, satisfies invariant 1 and keeps the authored geoms
alive so recompilation re-derives inertia without any write, at the cost of a
correspondence back to source elements for ids and for `mj_recompile`. That
correspondence is the binding design of 5.5, so the two choices are one
choice. A third option is to drop the flags and offer only the operators;
URDF import would then call them explicitly and save the result as an export.

Other choices inside this direction: where the policy lives (in the operator,
as arguments, or both with the flag using the conservative default), and
whether a separate compiled tree is needed at all once nothing structural
happens inside compile. If lowering runs on a copy, the copy is that tree.

### 5.3 Autonaming

Options: keep writing the name at load (status quo, but at load rather than at
compile, so it is authored); treat the file stem as an implicit alias resolved
at lookup time and never write `name`, with the saved file reproducing it from
the file attribute; or stop autonaming and require names. The third breaks
every MJCF that writes `<mesh file="x.stl"/>` and `mesh="x"`, which is most of
them. The alias option also fixes the stickiness, at the cost of a collision
rule between explicit names and implicit stems.

### 5.4 Working-copy hygiene

Regardless of the rest: state the invariant of 3.3, audit for it, and add the
tests in section 6 so that it stays true. A stronger form is to make the
working copy a separate object created per compilation rather than members
that persist; that removes the class of bug outright but touches every element
class and the writer (which, until 5.1, reads those members).

### 5.5 Binding

Keep `id` and `signature` on the elements as the one documented compile-to-spec
write, or move them into a per-model table keyed by element, so that a spec
compiled twice into two models has two bindings. The table form is also what
lowering on a copy needs (5.2): the compiled element is not the authored one,
and the id has to be reported against the authored one.

Whichever form, binding needs more than "source element to compiled id":

- Eliminated elements. With private lowering, an authored body removed by
  fusion or a geom removed by discard stays a valid source object with no
  `mjModel` row. `mjs_getId` on it must report that (an invalid id, or an
  explicit "absent" state), not the id of the surviving parent: reporting the
  parent would make an inertia inspection return the aggregate where the
  child's own contribution was asked for.
- Merged correspondence. Which source bodies were merged into which compiled
  body is useful information (for inspection, for state carry-over in
  `mj_recompile`) but it is a separate relation from the id, and it is
  many-to-one, so it does not support the reverse direction: an edited fused
  body cannot be distributed back over its sources without a policy (5.1).
- The Python bindings hold references to elements (upstream e38573c64) and
  read ids after compilation; whichever form is chosen has to keep that
  working, including for eliminated elements.

`mjs_getId` can remain as "as of the last compile" in either form.

### 5.6 How the directions interact

5.4 is independent and can come first. 5.1 and 5.2 are independent of each
other: the writer migration removes the inverses in 3.1, and the lowering
choice in 5.2 decides whether adoption into the surviving body is needed at
all (in place: yes; private copy: no). Neither is a prerequisite for the
other, and isolating compiler mutations does not wait on the writer. What 5.2
does depend on is the resolution stage for inertia and, if lowering runs on a
copy, on 5.5, which is why those two are one decision. The adoption contract
of 5.1 is shared by 5.2's in-place variant, `mj_copyBack` and `saveinertial`,
so it should be designed once even if the writer is migrated later. 5.3 is
"move the mutation out of compile" like 5.2 and can be done in either order
relative to it.

The alternative to all of the above is to keep patching: every new transform
gets its inverse in the writer, and every destructive step gets its own ad hoc
write into the authored graph, with its own policy side effects. The frames
work of the last two weeks is a measure of what the first costs; the fusestatic
branch shows the second: each fix there needed another write and another rule
about when it applies.

## 6. Verification

The existing gates are `WriteReadCompare` and `RecompileCompare` over the
model sweep. The second already compiles repeatedly and compares copies
against the original at zero tolerance, so idempotence on an unchanged spec is
covered. What is not covered is what actually failed in 3.3: a stale cache is
only wrong once its inputs have changed. The properties to add:

- edit between compiles: for every model in the sweep, and for a set of
  targeted edits (frame pose, geom deletion, asset file change, keyframe
  attachment and deletion, body-to-frame), compile, edit, compile again, and
  compare with a freshly constructed spec that has the edit authored in. This
  is the test that would have caught both of yesterday's bugs.
- authored state is untouched: snapshot the authored graph (an `mj_copySpec`
  plus a structural walk, or a saved XML once 5.1 lands) and compare after
  every compile, successful or not, modulo the binding record.
- failure and retry: make a compilation fail after the point where lowering or
  resolution would have run, repair the spec, compile again, and check that no
  authored structure was consumed. This is the test for the lowering choice in
  5.2.
- adoption stays adopted: adopt an inferred inertia, edit the geometry,
  recompile; the adopted body keeps its values and the others are re-inferred,
  under each inertia policy.
- the two sweeps, whose canonical outputs will change under 5.1; the changes
  have to be reviewed model by model, not re-baselined.

Under ASAN, since the failure mode of 3.2 and 3.3 is reading freed memory.

## 7. Open questions for review

1. Is purity (invariant 1) the goal, or only explicitness (invariant 4)? The
   second is achievable without a private graph and without changing what
   `mj_saveXML` writes.
2. Should `mj_saveXML` write the authored spec by default (5.1), with export
   for resolved values, or keep writing compiled values with the spec-writing
   form as a new function? Who depends on the current behaviour, including
   `mj_saveLastXML`?
3. If the writer reads the spec: preserve authored encodings and settings, or
   normalise through a serialisation view?
4. Inspection: a view of the last compilation, a recomputation from the
   current spec, or both? Does it need per-source results for bodies that
   lowering removes?
5. The adoption contract: scope (model or selected elements), precedence over
   the inference policy, raw versus final values, and what to do with merged
   results. Is one contract enough for `mj_copyBack`, inferred inertia,
   keyframes, length ranges and autonames?
6. fusestatic and discardvisual are "a bit clunky" and may be worth rethinking
   rather than porting. Explicit operators only, automatic lowering in place,
   or automatic lowering on a private copy? Is the body-to-frame
   representation of a fused body (a frame carrying the body's pose, contents
   in authored coordinates) the right one?
7. Where does resolved inertia come from and where does it go: a per-source
   query over resolved geoms, an adoption into the surviving body, or the
   private copy keeping the source geoms?
8. Autonaming: alias, load-time write, or drop?
9. How far should the working copy go: reset invariant only (5.4 light), or a
   per-compilation object?
10. Binding on elements or per model, what an eliminated element reports, and
    how the Python bindings (upstream e38573c64) keep working?

## 8. Related state

- `frames-roundtrip` on the fork (c69dff3b1, 4 commits): fixes to frame pose
  caching, frame preservation on save, framed inertials, `mjs_delete` on
  frames and `mjs_bodyToFrame`. Compatible with every direction here; its
  writer additions are deleted by 5.1; its `ToFrame` inertial merge is the
  spec-only half of what 5.2 needs.
- `frames-roundtrip-fusestatic` on the fork (fc239cd4f, 5 commits, parked):
  makes fusestatic coherent under recompile and save by replacing the fused
  body with a frame and adopting the fused inertia into the parent's spec.
  That adoption is what in-place fusing requires; what this document questions
  is that it runs automatically, inside compilation, on the caller's graph
  (question 6). The tests, the policy cases and the body-to-frame idea survive
  under every direction.
- `delete-validation` and `delete-dependents` on the fork: built on the
  `detached_` list that upstream e38573c64 removed; stale.
- The scaling design (`SCALE_DESIGN.md` on `claude/mjoco-scaling-design-33b126`)
  is a spec operator in the sense of 5.2 and is the origin of this question.
