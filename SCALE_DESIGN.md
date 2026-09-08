# Scaling models in MJCF and mjSpec

*Design proposal, 2026-09-08. Branch `claude/mjoco-scaling-design-33b126`. Early draft: the
user-facing shape is the thing to settle first; mechanism follows.*

## Summary

MuJoCo does not fix a system of units ([Units are unspecified](doc/overview.rst)): a 1 m,
1 kg spaceship with a 1 N thruster and a 1 cm, 1 g spaceship with a 1 dyn thruster have the
same dynamics. The corollary is that multiplying every quantity by $s_L^a s_M^b s_T^c$,
where $[L^a M^b T^c]$ is its dimension, is an exact symmetry of the simulator. Applied to a
*part* of a model the symmetry is broken by everything that is not scaled, gravity, the
timestep, the neighbours, and the transform becomes a physical resize.

One transform therefore covers every scaling use case; what differs is the scope and the
three factors:

| use case | scope | $(s_L, s_M, s_T)$ |
|---|---|---|
| "make this robot 2× bigger" at constant density | subtree | $(s, s^3, 1)$ |
| same, keeping the motors and masses | subtree | $(s, 1, 1)$ |
| dynamically similar creature (Froude scaling) | subtree | $(s, s^3, \sqrt{s})$ |
| unit conversion (CAD in mm, CGS to MKS) | whole spec | anything; the model is re-expressed, not changed |

The proposal has three parts:

1. **The primitive is a `scale` on `<frame>`.** A frame already wraps a subtree and applies
   a transform to it at compile time; scale makes it a similarity transform. Because it is
   applied at compile time it is a *parameter*, not an operation: set it, compile, set it
   again, compile. Attach returns a frame, so attaching at a scale is attaching onto a
   scaled frame. The scale is a vector in the frame's own axes, so anisotropic scaling of
   rigid assets has a well-defined home from day one, even if v1 accepts only uniform values.
2. **Per-field dimensions live in `mjcf.schema`** as a `dim` facet, generating the units
   column of the XML reference, the scaling table, and a coverage test.
3. **Unit conversion at load** is a `<compiler>` attribute that converts the numbers the
   file states into MKS, paired with a documentation change: the simulator stays
   unit-agnostic, the defaults are declared MKS.

There is a prerequisite (section 3.0): frames do not round-trip today, their pose is baked
into their children on save and only the name survives. A scale that is a parameter needs
the writer to emit authored frames and frame-relative contents, and the same contract
covers free-joint alignment, the other transform the compiler currently bakes.

## 1. What the user gets

Items marked **decision** are open.

### 1.1 `<frame scale="...">`

```xml
<frame scale="2">
  <attach model="hand" prefix="h_"/>
</frame>

<frame pos="1 0 0" scale="0.5">
  <body name="tree"> ... </body>
</frame>
```

Everything under the frame is scaled geometrically by the factor: positions, sizes, contact
lengths, slide-joint ranges, tendon lengths, keyframe columns, camera intrinsics, light
attenuation, everything with a length in its dimension. Masses follow density: inferred
inertials scale by construction, and explicit `<inertial>` clauses get $s^3$ on mass and
$s^5$ on inertia so the two kinds of body agree. Time-dimensioned quantities are untouched,
so `solref` time constants, actuator dynamics and the timestep stay as they are.

Properties:

- **A parameter, not an operation.** The spec is untouched. Saved XML contains the frame
  with its `scale` and the original numbers (this needs the writer change in section 3.0).
  Domain randomization is `frame.scale = s; compile()` in a loop, with no copy and no risk
  of scaling twice.
- **Composes.** Nested frames multiply. A body's effective scale is the product of the
  frame scales above it, all the way to the world.
- **Attach.** `<attach>` places the child under a frame, so "attach at scale 2" is the
  first example above and needs no attribute of its own. Three copies at three scales are
  three frames. Since attach copies the child spec and prefixes its assets, an attached
  model never shares anything with the rest of the model and never triggers the
  shared-asset refusal in section 3.
- **Vector-valued.** `scale="sx sy sz"`, in the frame's axes. A single value means uniform.
  Section 1.4 sets out what anisotropic values require.

Decisions:

- **decision:** name. `scale` is short and matches `mesh scale`; `lengthscale` leaves room
  for the mass and time siblings below and reads unambiguously next to them.
- **decision:** whether frames also carry mass and time factors (`massscale`, `timescale`)
  for the general gauge and for Froude-similar subtrees, or whether those stay API-only.
  The physically meaningful defaults ($s^3$, $1$) are what the frame does without them.
  Leaning to API-only in v1, with `scale` on frames meaning geometric similarity at
  constant density.

### 1.2 Unit conversion at load, and "defaults are MKS"

A file authored in millimetres and grams should be loadable without a preprocessing script:

```xml
<compiler lengthscale="0.001" massscale="0.001"/>
```

This is deliberately *not* a scale on the root. Take a file in mm that does not mention
gravity. Scaling everything would turn the default 9.81 into 0.00981, which nobody wants.
The default was never in the file's units: it is an MKS number, and it was already wrong for
a self-consistent mm model. The coherent rule is that defaults are MKS and explicit values
are in the file's units, and the conversion maps the latter into the former. A specified
`density="0.001"` in g/mm³ becomes 1000 kg/m³; an unspecified density stays at the default
1000 kg/m³. Both are water.

Two consequences:

- **Documentation.** The overview keeps "the simulator is unit-agnostic" and adds "the
  defaults assume MKS", which the current text half-admits already (gravity, density).
  With that sentence in place the attribute is well-defined.
- **Mechanism.** Explicit-versus-default is only known in the reader, so this is a
  reader-time conversion driven by the schema's `dim` facets, not a compile-time transform.
  Binary assets are in file units too, so mesh, skin and flex vertex data are converted
  through mesh `scale` and friends even when those are unspecified.

- **decision:** naming and form. `lengthscale`/`massscale`/`timescale`, or a keyword
  `units="mm g"` form with a fixed table, or both.
- **decision:** v1 or follow-up. It is the largest reader change in this proposal and is
  independent of frame scaling.

### 1.3 API

Frame scale is a field on `mjsFrame`, and therefore on the Python `MjsFrame`:

```python
hand = spec.worldbody.add_frame(scale=[2, 2, 2])   # or scale=2 in Python
hand.attach_body(child.body('palm'), 'h_')

for s in rng.uniform(0.8, 1.2, size=100):           # domain randomization
  spec.frame('fork').scale = s
  model = spec.compile()
```

The destructive form, one C function explicit about all three factors, applies the same
transform to the spec itself:

```c
// Scale a subtree (body or frame) or the whole spec (pass mjs_getSpec(...)->element).
MJAPI void mjs_scale(mjsElement* element, double length, double mass, double time);
```

```python
body.scale(2)                       # bake in: length 2, mass 8, time 1
body.scale(2, time=2**0.5)          # Froude-similar
spec.scale(length=1e-3, mass=1e-3)  # whole spec, the exact symmetry: mm, g -> m, kg
```

It exists for baking a scale into saved numbers, for the whole-spec gauge (the world body
has no frame), and for factors the frame does not carry. It is the same traversal as the
compile-time one run against the spec structs instead of the compiled copies, so it is
cheap once frames work, and it is v2.

- **decision:** the Python mass default for `body.scale`. $s_L^3$ matches the frame and the
  compiler; it is a trap for `spec.scale(length=1e-3)` used as unit conversion, where
  masses would silently change by $10^{-9}$. Leaning to one rule and a docstring, since
  `<compiler>` is the primary unit-conversion route.

### 1.4 Anisotropic scaling: designed in, gated

Cartesian anisotropic scaling $S = \mathrm{diag}(s_x, s_y, s_z)$ does not commute with
rotation, so it cannot be applied to an arbitrary articulated subtree; but it is exactly
what domain randomization of rigid objects wants (a fork stretched along its handle), and
the frame is the right place for it because the stretch axes are the frame's own. The
rules, so that v1 can accept only uniform values while leaving the door open:

- **Propagation.** Into a child body with orientation $R$ the scale becomes $R^T S R$,
  which must remain diagonal: $R$ must be a signed axis permutation, or $S$ must be
  uniform in the plane $R$ mixes. Otherwise the child is sheared and compilation fails.
- **Primitives.** Boxes, ellipsoids, sites and meshes accept any diagonal scale (a mesh
  absorbs it into its `scale`). Capsules and cylinders need the two transverse factors
  equal; spheres need all three equal. `fromto` endpoints are just points and always scale.
- **Joints.** Stretching along $x$ turns a rotation about $z$ into a non-rigid motion.
  A hinge needs the scale uniform in the plane perpendicular to its axis; a ball or free
  joint needs it fully uniform; slides are unconditionally fine. Being axis-aligned is
  necessary but not sufficient, which is the check one would naively write.
- **Inertia.** Explicit `<inertial>` clauses have no simple transform under anisotropy and
  are refused; inferred inertials are computed from the stretched geoms by the compiler.

In practice anisotropy is for rigid leaf assets with at most slide joints, and that is the
important case.

- **decision:** v1 accepts uniform only, or anisotropic wherever the rules above allow. The
  rules are needed either way to produce the errors; the difference is testing surface.

## 2. Where the dimensions live: a `dim` facet in `mjcf.schema`

The catalogue of which fields carry length is long and has traps: torsional and rolling
friction coefficients are lengths, `solimp[2]` is a length for contacts and tendon limits
but a joint coordinate for joint limits, `solref` is $[T, 1]$ when positive and
$[T^{-2}, T^{-1}]$ when negative (stiffness and damping per unit inertia), light
attenuation is $[L^{-1}, L^{-2}]$, sensor `noise` and `cutoff` take the dimension of the
sensor. Maintaining that catalogue by hand, in a scaling function, is how fields get missed.

Proposal: the dimension is a facet on the attribute in `src/xml/mjcf.schema`, next to
`field=` and `min=`:

```
  pos        : double[3] = {0, 0, 0} (dim=L)
  friction   : double[1..3] = {1, 0.005, 0.0001} (dim="1 L L")
  solref     : double[1..mjNREF] = {0.02, 1} (dim=solref)
  stiffness  : double = 0 (dim="M L^2 T^-2 Q^-2")
```

This buys three things at once:

1. **Documentation.** The XML reference tables gain a units column generated from the
   schema, answering the perennial "what units is torsional friction in".
2. **The scaling code.** Both the compile-time frame scaling and `mjs_scale` are driven by
   (or tested against) a generated table of `(struct, field, exponents)`, the same way the
   reader is driven by `mjcf_read_table.inc` today. Every mjSpec field that is an XML
   attribute is covered.
3. **Coverage.** A `doc_test` check that every numeric attribute carries a `dim` (including
   `dim=1` and `dim=custom`), so a new attribute cannot be added without stating its
   units. That closes the "hidden length" class of bugs permanently, rather than by
   catalogue.

**Generalized coordinates.** Joint-dimensioned quantities depend on the joint type: a slide
joint's stiffness is $[M T^{-2}]$, a hinge's is $[M L^2 T^{-2}]$. Introduce a symbol $Q$,
"one unit of the coordinate", resolved when scaling to $L$ for slide joints and
translational free-joint components and to $1$ for angles. Then each such attribute has one
declaration: energy is $\tfrac12 k q^2$, so `stiffness` is $M L^2 T^{-2} Q^{-2}$, `damping`
is $M L^2 T^{-1} Q^{-2}$, `armature` is $M L^2 Q^{-2}$, `range` and `springref` are $Q$,
`gear` on a joint transmission is $L\,Q^{-1}$ (a moment arm on a hinge, dimensionless on a
slide), and a motor's `ctrlrange` is a force $[M L T^{-2}]$ in both cases. The tables of
"slide vs hinge" collapse.

**Custom dimensions.** A minority of attributes have dimensions that depend on an enum on the
same element: `sensor noise`/`cutoff` on the sensor type, `gainprm`/`biasprm`/`ctrlrange`
on the actuator's gain and bias type, `key qpos`/`qvel`/`ctrl` per column. These are
declared `dim=custom` and handled in code, exactly as `reading=custom` attributes are read
by hand today. The coverage check still applies; custom is an annotation, not an omission.

- **decision:** facet grammar details: per-component lists (`"1 L L"`), named special
  cases (`solref`). Exponents are spelled `L^2`.
- **decision:** generated table consumed at runtime, versus hand-written scaling code with
  a test that diffs it against the schema. The former cannot drift; the latter keeps the
  user layer readable and matches how the rest of it is written.

## 3. Mechanism and scope

### 3.0 Prerequisite: frames must round-trip

Frames today are half transient. The XML reference says that in a saved model "the frame
elements have disappeared" with their transformation accumulated into the children. The
writer no longer does that: `OneFrame` emits `<frame name childclass>` with no `pos` or
`quat` (a frame with neither is dropped), while the children are written from their compiled
poses, which had the frame folded in. A frame survives saving as a named husk with an
identity transform; after reload `spec.frame('x').pos` is zero and the children have moved.
The compiled model is identical, the structure is not, and the name survives only because
`<attach frame="...">` needs a target.

A frame scale that survives saving requires resolving this, and the resolution is a
contract for the writer: **emit authored values and authored structure; the compiler
re-derives everything it derives.** Concretely:

- Frames are written with `pos`, `quat` and `scale`, and their contents are written in
  frame-relative coordinates, sourced from the spec structs rather than the compiled
  copies. The writer already does this for one case, recovering a mesh geom's pose before
  the mesh re-centering transform, because the authored value is the one that round-trips.
  Orientations authored as `euler` or `axisangle` are resolved to a quaternion without the
  frame folded in; `fromto` geoms come out as `pos`/`quat` as they do today. So "authored"
  means authored, canonicalized, frame-relative.
- Every authored frame persists, named or not. Making persistence depend on having a name
  (named frames as parameters, anonymous ones baked, which is one step from the current
  writer rule) was considered: it keeps the flat output of the anonymous-`add_frame`-then-
  attach idiom and ties "survives the file" to "can be found in the file", but it doesn't
  shrink the writer change, it adds a second path plus the nested cases, and it makes the
  saved file depend on whether a debugging name was added. Nothing else in MJCF changes
  meaning by being named. If churn to existing saved files turns out to matter, this is the
  fallback, since the machinery is a superset either way.
- The XML reference paragraph is updated to match.

**Alignment is a transient frame.** Free-joint alignment (`joint align`, `compiler
alignfree`) is the other baked transform: in `mjCBody::Compile`, right after the enclosing
frame's pose is accumulated, a leaf body with a lone free joint has its pose composed with
its inertial pose, `ipos`/`iquat` zeroed, and its geoms, sites, cameras and lights
counter-transformed by the inverse, in two phases because sites compile later than geoms.
The writer emits the aligned numbers and does not write the joint's `align` attribute back;
reload is stable only because aligning an aligned body is the identity. Under the contract
above the writer emits the unaligned `pos` and `ipos` and writes `align`, and the compiler
re-aligns on load. Structurally, alignment is a frame the compiler inserts between the body
and its contents, with the inverse inertial pose as its transform, plus the matching shift
of the body. Once the accumulated-transform pass for scale exists (below), alignment can be
expressed as a compiler-generated transient frame in that pass rather than as the two-phase
special case. Generated frames are transient by an internal flag, never written, which gives
two kinds of frame, authored and generated, without a user-facing rule. This refactor is a
consequence, not a prerequisite. One ordering constraint either way: scale is accumulated
before alignment, since `ipos` is a length and must be scaled before it re-bases the body.

- **decision:** the writer change as a standalone commit ahead of everything else, or
  together with frame scale. Standalone is reviewable on its own and fixes a documented
  behaviour that the code already contradicts.

### 3.1 Scaling at compile time

**Where it runs.** Compile already copies each element's `spec` struct into the internal
object (`CopyFromSpec`) and only then applies the enclosing frame's pose. Scale rides the
same path with one difference: a pose affects only the frame's immediate children, since
grandchildren are posed relative to their parent body, but scale must propagate down the
whole subtree. So each body gets an accumulated scale, the product of the frame scales on
its chain to the world, computed in a pass before element compilation, and every element
multiplies its dimensioned fields by its body's accumulated scale during compile. Mass and
inertia use $s_L^3$ and $s_L^5$ in the uniform case. `mjs_scale` is the same traversal
applied to the spec structs.

**In the tree.** The frame's bodies and everything under them: bodies (`pos`, `ipos`,
`mass`, `inertia`), geoms, sites, joints, cameras, lights, and flexes whose bodies are all
inside.

**Outside the tree, referencing in.** Tendons, actuators, equality constraints, contact
pairs, sensors, and keyframes are not tree elements but carry dimensioned quantities tied
to things that are. The rule is uniform: an element is scaled if everything it references
has the same accumulated scale, and compilation fails with a clear message otherwise. A
spatial tendon routed through bodies at two scales has no consistent `springlength`; a
`connect` between a scaled subtree and the floor has no consistent anchor. Keyframes are
scaled per column, since each `qpos`/`qvel`/`act`/`ctrl` column belongs to a joint or
actuator whose scale is known. Attached models never hit the failure, since a child spec
contains all of its own references.

- **decision:** contact pairs with geoms at two scales. Their `margin`, `gap`, torsional
  and rolling friction are lengths of the *contact*, and there is a defensible
  geometric-mean answer. Leaning to the uniform rule (error) for v1.

**Shared assets.** A mesh or heightfield holds its own `scale`, and one asset can be
referenced by geoms at different accumulated scales. Rather than cloning the asset, this is
refused: a mesh may be referenced at one accumulated scale. The user duplicates the asset,
or attaches, which duplicates it for them. This is the one place the frame design costs
something relative to a destructive per-geom mutation, and it is the right trade.

**Defaults.** Nothing to do. An mjSpec element holds resolved values (its constructor copies
the default class in), so scaling touches elements, never `<default>`.

**Plugins.** Plugin configuration is opaque strings; it is not scaled, and this is
documented. A plugin callback for scaling is a possible later addition.

**Whole-spec.** The world body has no frame, so the exact symmetry, everything including
`option`, `visual`, `statistic` and assets, is `mjs_scale` on the spec, and it cannot fail
because there are no outside references.

## 4. Things this is not

- Not a physics-preserving resize. No choice of $(s_L, s_M, s_T)$ preserves both the
  strength-to-weight ratio and the natural frequencies of a subtree in a fixed world; that
  is the square-cube law, not a bug. The tools expose the factors and stop there.
- Not allometry. Muscle strength scaling with cross-section ($s^2$) is a modelling choice
  layered on top, one line of Python on the caller's side.
- Not per-segment morphing ("longer legs"). Scaling along each segment's own axis is a
  different feature with its own design.

## 5. Suggested order of work

0. Frames round-trip (section 3.0): the writer emits authored frames with `pos`/`quat` and
   frame-relative contents, writes `align` back, and the XML reference is corrected. A
   round-trip test: load, save, reload, compare specs. Independent of everything below.
1. `dim` facets in `mjcf.schema`, the units column in the XML reference, and the coverage
   test. Self-contained, reviewable on its own, and immediately useful as documentation.
2. `scale` on `mjsFrame` and `<frame>`, uniform, with the accumulated-scale pass, the
   reference rule, the shared-asset refusal, and tests that compile a scaled model and
   check `mjModel` field by field against the exponent rule.
3. Optionally, alignment re-expressed as a generated transient frame in the same pass.
4. Anisotropic values with the section 1.4 checks.
5. `<compiler>` unit conversion in the reader, with the overview change.
6. `mjs_scale` and the Python `scale` methods.
7. Later, if wanted: mass and time factors on frames, plugin scaling callbacks.
