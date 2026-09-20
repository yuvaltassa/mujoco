# Scaling models in MJCF and mjSpec

*Design proposal, 2026-09-08, revised 2026-09-20 after review. Branch
`claude/mjoco-scaling-design-33b126`. Early draft: the user-facing shape and the physical
policy are the things to settle first; mechanism follows.*

## Summary

The proposal is a non-destructive **`scale` attribute on `<frame>`**: everything under the
frame is resized at compile time, the spec is untouched, and the parameter survives
save/load. Three things need settling around it:

1. **What resizing means physically.** Geometry and constant-density inertia are not
   controversial. What happens to everything force-like (actuator gains, springs, dampers,
   force limits) is a *policy*, and the doc names two (section 2).
2. **Where per-field dimensions come from.** A `dim` facet in `mjcf.schema`, generating the
   units column of the XML reference, the scaling table, and a coverage test (section 4).
3. **What happens at the boundary** between scaled and unscaled parts of a model, which is
   the normal case, not an error (section 5).

Section 3 works three examples through to the numbers; they are the quickest way to see what
is being proposed. Unit conversion at load is a separate proposal (appendix A).

Status: the prerequisite, frames round-tripping through save, is implemented (fork branch
`frames-roundtrip`, 0572fd554). Free-joint alignment is still baked on save (section 6.1).

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

- **A parameter, not an operation.** The spec is untouched; the scale is applied during
  compilation. Saved XML contains the frame with its `scale` and the original numbers.
  Domain randomization is `frame.scale = s; compile()` in a loop, with no copy and no risk
  of scaling twice.
- **Scales contents, not itself.** A frame's `pos` and `quat` are in its parent's
  coordinates and are not affected by its own scale. The frame defines a similarity
  transform about its own origin.
- **Composes along element ancestry.** An element's effective scale is the product of the
  scales of every frame on its ancestry path: its own enclosing frames, its body, that
  body's enclosing frames, the parent body, and so on to the world. Scale therefore lives on
  elements, not bodies: a frame may wrap two of a body's five geoms.
- **Attach.** `<attach>` places the child under a frame, so "attach at scale 2" is the first
  example above and needs no attribute of its own. Three copies at three scales are three
  frames.
- **Vector-valued.** `scale="sx sy sz"` in the frame's axes; a single value means uniform.
  v1 accepts uniform values only (section 1.3).

In Python, `scale` is a field on `MjsFrame`:

```python
for s in rng.uniform(0.8, 1.2, size=100):
  spec.frame('fork').scale = s
  model = spec.compile()
```

- **decision:** name. `scale` is short and matches `mesh scale`.
- **decision:** how the physical policy of section 2 is selected: a second frame attribute
  with two keywords, and which is the default.

### 1.2 API: `mjs_scale`, later

A destructive counterpart that applies the same transform to the spec itself, for baking a
scale into saved numbers and for whole-spec rescaling (the world has no frame):

```c
MJAPI void mjs_scale(mjsElement* element, double length, double mass, double time);
```

On the whole spec with all three factors this is the exact unit-system symmetry of the
simulator and cannot fail. It shares the traversal with frame scaling and is v2.

### 1.3 Anisotropic scaling: uniform in v1, attribute shaped for later

The use case is reshaping rigid objects for domain randomization: one fork, stretched along
its handle. The semantics, when it comes:

- The **reference configuration** of the scaled subtree is the affine image of the original
  under $S = \mathrm{diag}(s_x, s_y, s_z)$ in the frame's axes. After that, bodies move
  rigidly as always. Nothing requires *every* configuration to be an affine image, so joints
  impose no restriction: a stretched fork on a free joint is fine.
- Each body's contents are reshaped in the body's own frame by $A = R_0^T S R_0$, where
  $R_0$ is the body's reference orientation relative to the scaling frame. The only
  restriction is **whether each shape can represent its reshaped self**: meshes always can
  (variants are derived by an arbitrary linear map, section 5.3); boxes, ellipsoids and
  heightfields need $A$ diagonal in the geom's frame; capsules and cylinders additionally
  need the two transverse factors equal; spheres need $A$ uniform. Anything else is a
  compile error naming the geom.
- Explicit inertia has an exact transform. With second-moment tensor
  $\Sigma = \tfrac12 \mathrm{tr}(I)\,\mathbb{1} - I$, the map is $m' = \det(A)\, m$,
  $\Sigma' = \det(A)\, A \Sigma A^T$, $I' = \mathrm{tr}(\Sigma')\,\mathbb{1} - \Sigma'$.
  Off-diagonal terms go through the existing `fullinertia` principal-axis path.

v1 is uniform-only because it needs none of the representability checks. The restrictions
above are recorded as the expected shape, not as a commitment.

## 2. Physical policy: what "bigger" means

Dimensional analysis tells us how each field transforms under a change of units
$(s_L, s_M, s_T)$: a field of dimension $[L^a M^b T^c]$ is multiplied by
$s_L^a s_M^b s_T^c$. Applied to a whole model this is an exact symmetry
([Units are unspecified](doc/overview.rst)). Applied to a subtree it is a physical change,
and *which* factors to use is a modelling decision that dimensional analysis does not make.
Two policies are worth naming:

| | **similar** | **geometry** |
|---|---|---|
| definition | the unit-change $(s, s^3, 1)$ applied to the subtree | lengths and inertials only |
| lengths (positions, sizes, contact lengths, slide ranges, tendon lengths) | $\times s$ | $\times s$ |
| mass, inertia (explicit; inferred follow from geometry) | $\times s^3$, $\times s^5$ | $\times s^3$, $\times s^5$ |
| forces, slide stiffness and damping | $\times s^4$, $\times s^3$ | unchanged |
| torques, hinge stiffness, damping and armature | $\times s^5$ | unchanged |
| time-like (`solref` timeconst, actuator dynamics) | unchanged | unchanged |
| behaviour | same motion in the time domain: servo bandwidth, damping ratios and spring frequencies are preserved; only gravity (and other unscaled surroundings) breaks the similarity | the same motors, springs and gains on a body with $s^5$ the inertia: slower and weaker, as in "same actuators, longer links" |

Neither is "correct". *similar* is the policy under which a resized robot still works
without retuning, which is what scene assembly and size randomization want; *geometry* is
the policy for evaluating fixed hardware on different morphology. Anything in between
(muscle force with cross-section, $s^2$) is *geometry* plus a line of user code on the
actuators. The rule for *geometry* is mechanical given the schema dimensions: fields with
no mass dimension scale by their length exponent, inertials scale at constant density,
every other mass-bearing field is left alone.

Not available under either policy: preserving both strength-to-weight and natural
frequencies in a fixed gravity field. That is the square-cube law.

- **decision:** the default. Leaning *similar*.
- **decision:** whether a third, general form (independent mass and time factors on the
  frame, e.g. Froude scaling $(s, s^3, \sqrt{s})$) is wanted in MJCF or only in `mjs_scale`.

### 2.1 Actuator semantics

The dimensions assigned to actuator fields must describe MuJoCo's established semantics and
must keep control inputs meaningful:

- **`gear` is dimensionless** for every transmission. Actuator length carries the
  transmission's coordinate dimension ($Q$, section 4: an angle for a hinge, a length for a
  slide or tendon), and actuator force is the matching generalized force.
- **`ctrl` keeps the dimension of what it competes with** in
  `force = gain*ctrl + bias0 + bias1*length + bias2*velocity`: $Q$ if the bias has a length
  term (position servo), $Q/T$ if it has only a velocity term (velocity servo),
  dimensionless otherwise (motor). `gainprm` carries the rest. So a position target on a
  slide scales with the robot, a target angle does not, and a normalized motor command in
  $[-1, 1]$ keeps its meaning while the gain carries the force scaling.
- `forcerange` is a generalized force; `ctrlrange` has the dimension of `ctrl`;
  `lengthrange` is $Q$. Muscles: only the peak `force` is dimensioned, the rest is in units
  of $L_0$.

For hinges all of this is invisible: $Q = 1$, so `ctrl`, `ctrlrange` and `gear` never change
and only gains and force limits do.

## 3. Worked examples

### 3.1 A resized actuated pendulum

```xml
<worldbody>
  <frame scale="2">
    <body name="arm" pos="0 0 1">
      <joint name="hinge" axis="0 1 0" damping="0.1" armature="0.01"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05"/>
    </body>
  </frame>
</worldbody>
<actuator>
  <position joint="hinge" kp="10" kv="1" ctrlrange="-1 1" forcerange="-5 5"/>
</actuator>
```

The actuator is outside the tree; it references a joint at scale 2, so it is scaled with it.

| compiled quantity | unscaled | *similar*, $s=2$ | *geometry*, $s=2$ |
|---|---|---|---|
| body `pos` | 0 0 1 | 0 0 2 | 0 0 2 |
| capsule half-length, radius | 0.25, 0.05 | 0.5, 0.1 | 0.5, 0.1 |
| mass, inertia (inferred) | $m$, $I$ | $8m$, $32I$ | $8m$, $32I$ |
| `damping`, `armature` | 0.1, 0.01 | 3.2, 0.32 | 0.1, 0.01 |
| `kp`, `kv` | 10, 1 | 320, 32 | 10, 1 |
| `forcerange` | ±5 | ±160 | ±5 |
| `ctrlrange`, `gear` | ±1, 1 | ±1, 1 | ±1, 1 |
| servo frequency $\sqrt{k_p/I}$ | $\omega$ | $\omega$ | $\omega/\sqrt{32}$ |
| gravity sag $m g l / k_p$ | $\theta$ | $\theta/2$ | $16\,\theta$ |
| pendulum frequency $\sqrt{g/l}$ | $\omega_g$ | $\omega_g/\sqrt2$ | $\omega_g/\sqrt2$ |

`ctrl = 0.5` means "go to 0.5 rad" in all three columns.

The slide-joint variant, where $Q = L$: with `range="0 0.2"`, a position servo with
`kp="100" ctrlrange="0 0.2"` becomes, under *similar*, `range` 0 0.4, `ctrlrange` 0 0.4,
`kp` 800 ($[M T^{-2}]$, $s^3$), and a force limit scales by 16. A motor on the same joint
with `gear="50" ctrlrange="-1 1"` keeps both; its gain goes from 1 to 16.

### 3.2 Three sizes of one mesh object

```xml
<asset>
  <mesh name="fork" file="fork.stl"/>
</asset>
<worldbody>
  <frame pos="0 0 0">            <body><freejoint/><geom type="mesh" mesh="fork"/></body></frame>
  <frame pos=".3 0 0" scale="1.5"><body><freejoint/><geom type="mesh" mesh="fork"/></body></frame>
  <frame pos=".6 0 0" scale="0.5"><body><freejoint/><geom type="mesh" mesh="fork"/></body></frame>
</worldbody>
```

One authored asset, three geoms. The frames sit at x = 0, 0.3, 0.6 regardless of their
scales. The compiled model has three meshes: `fork` and two compiler-generated variants
(section 5.3), with volumes, masses and inertias in ratio $1 : 1.5^3 : 0.5^3$ and
$1 : 1.5^5 : 0.5^5$. The saved XML is the text above. With a future anisotropic
`scale="1 1 1.5"` the third fork is stretched along the frame's z axis and still falls and
tumbles as a rigid body.

### 3.3 A resized robot connected to an unchanged environment

```xml
<worldbody>
  <geom name="floor" type="plane" size="5 5 .1"/>
  <site name="hook" pos="0 0 3"/>
  <frame scale="2">
    <body name="arm" pos="0 0 1">
      <joint name="hinge" axis="0 1 0"/>
      <geom name="hand" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05"/>
      <site name="tip" pos="0 0 -0.5"/>
    </body>
  </frame>
</worldbody>
<contact>
  <pair geom1="hand" geom2="floor" margin="0.01" friction="1 1 0.005 0.0001 0.0001"/>
</contact>
<equality>
  <connect body1="arm" body2="world" anchor="0 0 0.2"/>
  <connect site1="tip" site2="hook"/>
</equality>
<tendon>
  <spatial name="bungee" stiffness="100">
    <site site="tip"/> <site site="hook"/>
  </spatial>
</tendon>
<sensor>
  <framepos objtype="site" objname="tip" reftype="site" refname="hook" cutoff="4"/>
  <touch site="tip" cutoff="10"/>
</sensor>
<keyframe>
  <key qpos="0.3"/>
</keyframe>
```

| element | what happens | why |
|---|---|---|
| `connect` with `anchor` | anchor becomes 0 0 0.4 | the anchor is expressed in `body1` coordinates; it takes `body1`'s scale |
| `connect` with sites | nothing to scale | both attachment points are elements with their own scales |
| `pair` hand/floor | `margin` 0.02, torsional 0.01, rolling 0.0002 | the floor is unscaled and abstains; contact lengths follow the scaled geom (section 5.2) |
| `bungee` | path and auto `springlength` are recomputed from the scaled sites; `stiffness` unchanged | the tendon belongs to neither side; an *explicit* `springlength` here would be a compile error, since neither $\times1$ nor $\times2$ is right |
| `framepos` tip relative to hook | `cutoff` unchanged | object and reference are at different scales; the output is in the reference's units |
| `touch` on tip | `cutoff` 160 under *similar*, 10 under *geometry* | a force on a scaled element |
| keyframe `qpos` | unchanged | hinge angle; a slide would scale, a free joint maps through the frame (section 5.4) |

## 4. Where the dimensions live: a `dim` facet in `mjcf.schema`

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
  gear       : double[1..6] = {1, 0, 0, 0, 0, 0} (dim=1)
```

This buys three things at once:

1. **Documentation.** The XML reference tables gain a units column generated from the
   schema, answering the perennial "what units is torsional friction in".
2. **The scaling code.** Frame scaling and `mjs_scale` are driven by (or tested against) a
   generated table of `(struct, field, exponents)`, the same way the reader is driven by
   `mjcf_read_table.inc` today. Both policies of section 2 are functions of the exponents.
3. **Coverage.** A `doc_test` check that every numeric attribute carries a `dim` (including
   `dim=1` and `dim=custom`), so a new attribute cannot be added without stating its units.

**Generalized coordinates.** Joint-dimensioned quantities depend on the joint type: a slide
joint's stiffness is $[M T^{-2}]$, a hinge's is $[M L^2 T^{-2}]$. Introduce a symbol $Q$,
"one unit of the coordinate", resolved when scaling to $L$ for slide joints, tendons and
translational free-joint components and to $1$ for angles. Energy is $\tfrac12 k q^2$, so
`stiffness` is $M L^2 T^{-2} Q^{-2}$, `damping` is $M L^2 T^{-1} Q^{-2}$, `armature` is
$M L^2 Q^{-2}$, `frictionloss` and actuator forces are $M L^2 T^{-2} Q^{-1}$, `range` and
`springref` are $Q$. The tables of "slide vs hinge" collapse to one declaration each.

**Custom dimensions.** A minority of attributes have dimensions that depend on other
attributes of the same element: sensor `noise`/`cutoff` on the sensor type,
`gainprm`/`biasprm`/`ctrlrange`/`actrange` on the actuator's gain, bias and dynamics types
(section 2.1), keyframe columns on the joint or actuator they belong to. These are declared
`dim=custom` and handled in code, as `reading=custom` attributes are read by hand today.
The coverage check still applies; custom is an annotation, not an omission.

- **decision:** facet grammar details: per-component lists (`"1 L L"`), named special
  cases (`solref`). Exponents are spelled `L^2`.
- **decision:** generated table consumed at runtime, versus hand-written scaling code with
  a test that diffs it against the schema.

## 5. The boundary between scaled and unscaled

A scaled subtree in an unscaled world is the normal case. Elements outside the kinematic
tree (tendons, actuators, equalities, pairs, sensors, keyframes) and shared assets are
handled by a rule per relationship, with compile errors reserved for cases that are
genuinely undefined.

### 5.1 The owner rule

Most dimensioned fields of a non-tree element are expressed relative to one specific
referenced element, their *owner*, and take the owner's effective scale:

- An actuator takes the scale of its transmission target (joint, tendon, site, body).
- A joint or tendon sensor, a site-attached sensor, a frame sensor without a reference:
  the scale of its object. A frame sensor whose object and reference are at different
  scales keeps its `noise` and `cutoff` unscaled.
- `connect anchor` and `weld relpose` are in `body1` coordinates and take `body1`'s scale.
  Site-based equalities have nothing to scale. Joint and tendon equalities (`polycoef`) are
  polynomial in two coordinates and scale term by term from their dimensions when both
  sides share a scale; otherwise they are an error.
- A keyframe column belongs to a joint or actuator (section 5.4).

### 5.2 Relationships with no owner

- **Contact pairs.** `margin`, `gap`, the `solimp` width and torsional and rolling friction
  are lengths of the contact. Geoms with effective scale 1 abstain; the pair takes the
  geometric mean of the scales of the others. A robot on an unscaled floor takes the
  robot's scale, consistent with what the geom-level parameters of the scaled geom would
  have produced.
- **Tendons spanning scales.** The path is defined by sites and geoms that each have their
  own scale, so geometry is always meaningful and `springlength="-1"` is recomputed.
  Explicit length fields (`springlength`, `range`, `margin`) on a tendon whose path elements
  are not all at one scale are a compile error: no single factor is right. Its force-like
  fields are left unscaled. A tendon entirely inside one scale is an ordinary owned element.
- **decision:** whether spanning tendons with explicit lengths are an error or are left
  unscaled with a warning.

### 5.3 Shared meshes: compiler-generated variants

A mesh asset referenced at several effective scales compiles to several meshes in
`mjModel`: the authored one and one variant per distinct scale. The spec and the saved XML
keep the single shared asset. Variants are cheap because everything the mesh compiler
produces is covariant under a linear map $A$: $\mathrm{hull}(AV) = A\,\mathrm{hull}(V)$, so
the convex hull's graph and polygon structure are reused; vertices map by $A$, normals by
$A^{-T}$, volume by $\det A$, inertia by the rule in section 1.3, and a negative determinant
flips the winding. Heightfields and skins follow the same scheme. v1 may ship with a compile
error for multi-scale references and add variants immediately after; the error is
forward-compatible.

- **decision:** naming of variants in `mjModel` (unnamed, or a reserved suffix).

**Alternative: scale meshes at runtime through `geom_size`.** Today a mesh geom's
`geom_size` is not an input: the compiler overwrites it with the mesh's AABB half-extents.
It could instead be a per-geom scale, default 1 1 1, applied wherever mesh data is consumed.
Mesh data would then be shared across scales with no variants, and mesh geoms would reach
parity with primitives, whose `geom_size` can already be edited in `mjModel` (with the same
caveat that mass, inertia, `geom_rbound` and `geom_aabb` are compile-time and do not
follow). The arithmetic cost is small: the support function of a scaled convex set is
$S\,\mathrm{support}_K(S d)$, rays are transformed into the unscaled frame, BVH boxes are
scaled. The cost is blast radius: every reader of `mesh_vert` has to apply the geom's scale
(convex, SDF and continuous collision, rays, sensors, IPC, the classic and Filament
renderers with inverse-transpose normals, MJX, MuJoCo Warp), and third-party consumers of
`mjModel` would be silently wrong until they did. It also needs a new home for the AABB
half-extents currently stored in `geom_size`. Since the scale is known at compile time,
variants deliver the same authoring experience with no change to `mjModel` semantics, at the
price of memory. Recommendation: not a prerequisite. If runtime mesh scaling lands later on
its own merits (runtime randomization without recompiling), variants collapse into it with
no user-visible change.

### 5.4 Keyframes and mocap

Slide-joint `qpos`/`qvel` and other $Q$-dimensioned columns scale by the joint's scale. A
free joint's position is a point in its parent body's coordinates, and the scaling frame
acts about its own origin, not the parent's: for a frame at pose $(p, R)$ the keyframe
position maps as $x' = p + R\,S\,R^T (x - p)$, composed over nested frames. Multiplying by
$s$ is only right when the frame sits at the parent's origin. `mpos` of a mocap body under
a scaled frame maps the same way.

### 5.5 Other scope notes

- **Defaults.** Nothing to do: an mjSpec element holds resolved values, so scaling touches
  elements, never `<default>`.
- **Plugins.** Plugin configuration is opaque strings and is not scaled. Documented.

## 6. Mechanism

### 6.1 Frames round-trip (done), alignment (not yet)

Frames used to be half transient on save: written with only their name, their transform
baked into the children. The writer now emits every authored frame with its pose and its
contents in frame-relative coordinates (fork branch `frames-roundtrip`), under the contract
**emit authored values and authored structure; the compiler re-derives what it derives.**
Every authored frame persists, named or not; making persistence depend on a name was
considered and rejected (it adds a second writer path and makes saved output depend on a
debugging name).

Free-joint alignment (`joint align`, `compiler alignfree`) is the other transform baked on
save: the writer emits aligned poses and drops the joint's `align`. Under the contract it
should write the unaligned values and `align`. Structurally alignment is a frame the
compiler inserts between a body and its contents, so it can later be expressed as a
compiler-generated transient frame in the pass below. Ordering constraint: scale before
alignment, since `ipos` is a length.

### 6.2 Scaling at compile time

Compile copies each element's `spec` struct into the internal object (`CopyFromSpec`) and
then applies the enclosing frame's pose. Scale rides the same path. A pass before element
compilation computes each element's effective scale along its ancestry (section 1.1); each
element then multiplies its dimensioned fields according to the policy and the schema
exponents, non-tree elements resolve their owner's scale (section 5), and mesh references
resolve to variants. A frame's pose is applied after scaling the child's position.

## 7. Things this is not

- Not a physics-preserving resize; see the end of section 2.
- Not per-segment morphing ("longer legs"): scaling along each segment's own axis is a
  different feature, though nested anisotropic frames get part of the way there.
- Not unit conversion (appendix A).

## 8. Suggested order of work

0. Frames round-trip. **Done**, pending import. Alignment un-baking is a follow-up commit.
1. `dim` facets in `mjcf.schema`, the units column in the XML reference, the coverage test.
   Self-contained and immediately useful as documentation. Settles section 2.1 in writing.
2. `scale` on `mjsFrame` and `<frame>`, uniform, both policies, the ancestry pass, owner and
   boundary rules, multi-scale mesh references as an error. Tests: the three examples of
   section 3, checked field by field in `mjModel`.
3. Mesh variants.
4. Anisotropic values with the representability checks.
5. `mjs_scale` and the Python methods.
6. Separately: unit conversion at load; runtime mesh scaling; plugin scaling callbacks.

## Appendix A. Deferred: unit conversion at load

Loading a file authored in millimetres and grams (`<compiler lengthscale="0.001"
massscale="0.001"/>` or a `units` keyword) is a different feature with different questions
and gets its own proposal. The one conclusion worth recording here: it cannot be a scale on
the root. A file in mm that does not mention gravity must not end up with gravity 0.00981;
the default was never in the file's units. The coherent rule is that **defaults are MKS and
explicit values are in the file's units**, which makes it a reader-time conversion of
explicitly present attributes (and of binary assets), driven by the same `dim` facets, and
which wants the overview's "units are unspecified" section to add that the defaults assume
MKS.
