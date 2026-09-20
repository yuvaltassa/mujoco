# Scaling models in MJCF and mjSpec

*Design proposal, 2026-09-08; third revision 2026-09-20. Branch
`claude/mjoco-scaling-design-33b126`. The rules in sections 3 to 8 are implemented in a
Python reference prototype on this branch (`scale_prototype.py`, checks in
`scale_prototype_test.py`) and verified against simulation; numbers quoted below come from it.*

## 1. Summary

A non-destructive **`scale` attribute on `<frame>`**, with a **`scaling`** attribute that
selects the physical policy. Everything under the frame is resized at compile time, the spec
is untouched, and both attributes survive save and load.

The design rests on one verified fact. Multiplying every quantity of dimension
$[L^a M^b T^c]$ by $s^{a+3b}$ is the change of units $(s, s^3, 1)$, an exact symmetry of the
simulator ([Units are unspecified](doc/overview.rst)) *provided gravity is scaled too*.
Applied to a subtree with gravity left alone it gives this exact statement, which is the
definition of the default policy and its acceptance test:

> A model scaled by $s$ under `scaling="similar"`, in gravity $g$, moves exactly like the
> original in gravity $g/s$: lengths multiplied by $s$, angles and times unchanged.

The prototype confirms this to roundoff ($10^{-15}$ relative over the first 50 steps) on the
humanoid, a muscle-driven arm, a slider-crank, a tendon-driven car, the fluid-force balloons,
a Cartesian servo arm, and a feature model exercising every classic actuator shortcut on
hinges and slides, site and free-joint transmissions, fixed and spatial tendons, equalities
and explicit contact pairs. The flex hammock with an attached humanoid agrees to roundoff
with a converged solver and to $10^{-8}$ with its own 30-iteration CG settings; the mesh-based
3x3x3 cube agrees to $10^{-10}$, the precision of `float` mesh vertices.

Status: frames round-trip through save (fork branch `frames-roundtrip`, 0572fd554). One more
prerequisite surfaced, a frame recompile bug (section 11).

## 2. User-facing scope of v1

```xml
<frame scale="2">                       <!-- scaling="similar" is the default -->
  <attach model="hand" prefix="h_"/>
</frame>
<frame pos="1 0 0" scale="0.5" scaling="geometry">
  <body name="arm"> ... </body>
</frame>
```

**`scale`**: `real(1 or 3)`, default 1. One value $s$ means $(s, s, s)$; the spec field is
`double scale[3]`. Every component must be finite and positive, else a compile error. v1
additionally requires the three components to be equal, else a compile error naming the
frame (anisotropy, section 12). The writer emits one value when the three are equal and
omits the attribute at 1.

**`scaling`**: `[similar, geometry]`, default `similar` (`mjtScaling` in the spec). It
qualifies this frame's own `scale` and nothing else: it is not inherited by nested frames
and has no effect on a frame without a `scale`. An attached model's frames therefore mean
the same thing wherever the model is attached.

**What a frame scales.** Its contents, not itself: a frame's `pos` and `quat` are in its
parent's coordinates. The frame acts as a similarity transform about its own origin.

**Attach** needs no attribute: `<attach>` places the child under a frame.

**Python**: `frame.scale` (accepts a scalar or three values) and `frame.scaling`. Domain
randomization is `frame.scale = s; spec.compile()` in a loop; the spec is never modified.

**In v1**: uniform scale, both policies, composition, the actuator table of section 6, the
boundary rules of section 7, and shared-asset variants (section 8), so that every example
in section 5 compiles. **Not in v1**: anisotropic values, a third policy, `mjs_scale`, unit
conversion (section 12).

## 3. Physical policy

Dimensional analysis says how a field transforms under a change of units; it does not say
which factors to apply to a subtree. That is a modelling decision, exposed as `scaling`.
Fields fall in two classes:

- **shape class**: every field with no mass dimension (positions, sizes, contact lengths,
  slide ranges, tendon lengths, position-like control ranges), plus the inertial fields
  (`mass`, `inertia`, geom `mass`, `density`). Scaled under both policies, at constant
  density.
- **force class**: every other mass-bearing field: stiffness, damping, armature,
  frictionloss, actuator gains and force limits, force-sensor cutoffs, flex moduli.

| | `similar` (default) | `geometry` |
|---|---|---|
| shape class | $s^{a+3b}$ | $s^{a+3b}$ |
| force class | $s^{a+3b}$: forces $s^4$, torques $s^5$, slide stiffness $s^3$, pressure $s^2$ | unchanged |
| time-like fields (`solref` timeconst, actuator time constants, damping ratios) | unchanged | unchanged |
| meaning | the statement of section 1: same motion in time, in gravity $g/s$ | the same springs, motors and gains on a body with $s^5$ the inertia |

What `similar` does **not** promise, and the documentation must say so: tuning is retained
only relative to the model's own forces. Gravity and every other unscaled influence is
relatively weaker by $1/s$ (a controller tuned against gravity, a walking gait, does not
transfer); contacts with unscaled geoms go through the engine's usual parameter mixing; and
global options with dimensions are not touched by a frame (`ccd_tolerance` is a length, and
was the last thing standing between the balloons model and exactness).

Known inexactness of the engine itself, independent of this feature: a joint equality that
couples a slide to a hinge breaks the units symmetry at the $10^{-3}$ level, because the
constraint's diagonal approximation adds the two joints' inverse weights, a $1/\text{mass}$
and a $1/\text{inertia}$, without the polynomial's Jacobian. Same-kind couplings are exact.
Mesh vertices are `float`, so scaled meshes agree to $10^{-7}$ relative, not to roundoff.

A third policy is already expressible in the machinery of section 4 and is left for later:
*material* (constant stress: forces $s^2$, torques $s^3$, Young's modulus unchanged), which is
the muscle and structural-strength law.

## 4. Composition

Each element carries an accumulated **gauge** of three positive numbers
$(\sigma, \lambda, \mu)$: the shape length factor, and the length and mass factors of the
force class.

- A frame with `scale` $s$ contributes $(s, s, s^3)$ under `similar` and $(s, 1, 1)$ under
  `geometry`. (`material` would be $(s, s, s)$.)
- Gauges compose by componentwise product along the element's ancestry: the frames that
  enclose the element, its body, the frames that enclose that body, the parent body, and so
  on. A frame does not enclose itself. Scale lives on elements, not bodies: a frame may wrap
  two of a body's five geoms.
- A field of dimension $L^a M^b$ is multiplied by $\sigma^{a+3b}$ if it is shape class and by
  $\lambda^a \mu^b$ if it is force class.

Mixed policies therefore compose rather than override. Outer `scale="2"` `similar`, inner
`scale="0.5"` `geometry` gives $(1, 2, 8)$: original geometry and inertia, hinge stiffness
times 32, which is what applying the two operations in sequence produces.

**Partial-body scaling.** A frame inside a body scales the geoms it wraps. Inferred body
inertia follows, since it is computed from the scaled geoms. A geom with an explicit `mass`
takes $\sigma^3$ of its own gauge. An explicit `<inertial>` belongs to the body and takes the
body's gauge only; it is not decomposed, and frames nested inside the body do not apply to
it (today they do not apply their pose to it either, section 11).

## 5. Worked examples

### 5.1 A resized actuated pendulum

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
  <position joint="hinge" kp="100" kv="10" ctrlrange="-1 1" forcerange="-50 50"/>
</actuator>
```

The actuator is outside the tree; its owner is the joint (section 7).

| compiled quantity | unscaled | `similar` | `geometry` |
|---|---|---|---|
| body `pos` | 0 0 1 | 0 0 2 | 0 0 2 |
| mass | 4.451 | 35.60 (×8) | 35.60 (×8) |
| inertia about the hinge, with armature | 0.4106 | 13.14 (×32) | 12.83 (×31.2) |
| `armature`, `damping` | 0.01, 0.1 | 0.32, 3.2 | 0.01, 0.1 |
| `kp`, `kv` | 100, 10 | 3200, 320 | 100, 10 |
| `forcerange` | ±50 | ±1600 | ±50 |
| `ctrlrange`, `gear` | ±1, 1 | ±1, 1 | ±1, 1 |
| servo frequency $\sqrt{k_p/I}/2\pi$ | 2.48 Hz | 2.48 Hz | 0.444 Hz (×0.179) |
| gravity sag $m g l_c / k_p$ | 0.109 | 0.0546 (×½) | 1.75 (×16) |
| pendulum frequency | 0.821 Hz | 0.580 Hz (×0.707) | 0.587 Hz (×0.716) |

Under `geometry` the armature is hardware and stays, so the effective inertia grows by 31.2
rather than 32 and the frequency ratios are not exactly $1/\sqrt{32}$ and $1/\sqrt{2}$.
`ctrl = 0.5` means "go to 0.5 rad" in every column. Driven by the same control signal, the
`similar` arm in gravity $g$ and the original in $g/2$ produce identical joint angles
(difference exactly 0 over 2000 steps).

Slide variant ($Q = L$): with `range="0 0.2"`, a position servo `kp="300"
ctrlrange="0 0.2" forcerange="-40 40"` becomes `range` 0 0.4, `ctrlrange` 0 0.4, `kp` 2400
($s^3$), `forcerange` ±640 ($s^4$). A motor on the same joint with `gear="50"
ctrlrange="-1 1"` keeps both; its gain goes from 1 to 16.

### 5.2 Three sizes of one mesh object

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

One authored asset, three geoms, frames at x = 0, 0.3, 0.6 whatever their scales. `mjModel`
has three meshes, `fork`, `fork@1`, `fork@2` (section 8), with masses in ratio
$1 : 1.5^3 : 0.5^3$. The saved XML is the text above.

### 5.3 A resized robot in an unchanged environment

The arm of 5.1 under `<frame scale="2">`, plus a floor, a world site `hook` at 0 0 3, a site
`tip` and a site `tip2` on the arm, and:

| element | authored | compiled | rule |
|---|---|---|---|
| `connect body1="arm" body2="world" anchor="0 0 0.2"` | | anchor 0 0 0.4 | the anchor is in `body1` coordinates: owner `body1` |
| `connect site1="tip" site2="hook"` | | unchanged | both points are elements with their own gauges |
| `pair geom1="hand" geom2="floor"` | `margin` 0.01, torsional 0.005 | 0.01414, 0.00707 | two owners: geometric mean of their gauges, $\sqrt{2\cdot1}$ |
| spatial tendon `tip`–`tip2` | `springlength` 0.3, `stiffness` 100 | 0.6, 800 | one gauge throughout: $\sigma$, and $\mu$ for force per length |
| spatial tendon `tip`–`hook` | `springlength` 0.8, `stiffness` 100 | 1.131, 282.8 | path elements at two gauges: their mean. With the default `springlength="-1"` the rest length is recomputed from the scaled sites and no rule is involved |
| `framepos` of `tip` relative to `hook` | `cutoff` 4 | 8 | owner is the sensor's object; the reference only chooses coordinates |
| `touch` on `tip` | `cutoff` 10 | 160 (`similar`), 10 (`geometry`) | a force, on a scaled element |
| keyframe `qpos` | 0.3 | 0.3 | a hinge angle; a slide scales, a free joint maps through the frame (section 7) |

## 6. Dimensions and the actuator table

### 6.1 The `dim` facet

Each numeric attribute in `src/xml/mjcf.schema` declares its dimension:

```
  pos        : double[3] = {0, 0, 0} (dim=L)
  friction   : double[1..3] = {1, 0.005, 0.0001} (dim="1, L, L")
  solref     : double[1..mjNREF] = {0.02, 1} (dim=solref)
  stiffness  : double[1..3] = {0, 0, 0} (dim="M L^2 T^-2 Q^-2, M L^2 T^-2 Q^-3, M L^2 T^-2 Q^-4")
  mass       : double (dim=M, inertial)
```

The class is derived: no mass dimension or an `inertial` facet means shape, otherwise force.
From the facet we generate the units column of the XML reference, the table that drives the
scaling and its inverse, and a coverage test that every numeric attribute is annotated
(`dim=1` and `dim=custom` included).

**$Q$**, one unit of a generalized coordinate, resolves to $L$ for a length-kind coordinate
and to 1 for an angle. Energy fixes the rest: stiffness $M L^2 T^{-2} Q^{-2}$ (polynomial
coefficients one more $Q^{-1}$ each), damping the same with $T$, armature $M L^2 Q^{-2}$,
`frictionloss`, `actuatorfrcrange` and every actuator force $M L^2 T^{-2} Q^{-1}$.

A coordinate's **kind** comes from authored structure, never from numerical values:

| coordinate | kind |
|---|---|
| hinge, ball | angle |
| slide; spatial tendon; slider-crank | length |
| fixed tendon | angle if all its joints are hinges; otherwise length, and the coefficients of its hinge joints are moment arms (shape class, $L$) |
| site or free-joint transmission (6-vector `gear`) | length if `gear[0:3]` is nonzero, and then `gear[3:6]` are moment arms (shape class, $L$); angle if the linear part is zero |

This is the convention the documentation already recommends ("nonzeros in only the first 3
or the last 3 elements of gear, so the actuator length will be in either length units or
radians"), extended to the one case it leaves open: a coordinate that mixes translation and
rotation is a length, and the rotational coefficients are lengths. It is physically right
for the documented use: a propeller's torque-to-thrust ratio is a length that grows with
the propeller. `gear` is otherwise dimensionless, consistent with actuator `armature` and
`damping` being reflected through `gear` squared.

Traps the prototype caught, all `dim=custom` or easy to miss: an equality's `solimp` width
has the dimension of its residual ($L$ for connect and weld, $Q$ of the first joint or tendon
for couplings); joint and tendon `actuatorfrcrange` are generalized forces; geom `density`
is $M L^{-2}$ when `shellinertia` is true; positive `biasprm[2]` on a position servo is a
damping ratio, negative is $-k_v$; `solref` is $[T, 1]$ when positive and
$[T^{-2}, T^{-1}]$ when negative; flex `young` is a pressure ($s^2$ under `similar`), flex
`damping` a time.

### 6.2 Actuators

Shortcuts are compiled to `general` at parse time, so the rules are stated on the general
parametrization. $G$ is the generalized force of the actuator's coordinate kind
($M L T^{-2}$ or $M L^2 T^{-2}$), $Q$ its coordinate. The control's dimension $[u]$ is not
inferred from the presence of bias terms; it follows from the algebraic form:

> With a fixed gain and an affine bias, if `biasprm[1] == -gainprm[0]` the force depends on
> `ctrl` and length only through `ctrl - length`, so `ctrl` is commensurate with length:
> $[u] = Q$. If `biasprm[1] == 0` and `biasprm[2] == -gainprm[0]`, likewise
> $[u] = Q/T$. Otherwise `ctrl` is a dimensionless command and the gain carries the force.

These identities hold exactly for `position`, `intvelocity` and `velocity` and fail for a
normalized command with a parallel spring (`gainprm="30" biasprm="0 -100 -3"`), which keeps
`ctrlrange="-1 1"`. Under `similar` the choice never affects the physics, only which
numbers the user sends; under `geometry` it decides only whether a length-kind `ctrlrange`
follows the joint range.

| gain / bias family | `ctrl` | `gainprm` | `biasprm` | notes |
|---|---|---|---|---|
| fixed or affine gain, no or affine bias: `motor`, `position`, `velocity`, `intvelocity`, `damper`, `cylinder`, `adhesion`, `general` | $Q$, $Q/T$ or 1 by the rule above | `[0]`: $G/[u]$; affine `[1]`, `[2]`: $G/([u]\,Q)$ | `[0]`: $G$; `[1]`: $G/Q$; `[2]`: $G/Q$ if negative, unchanged if positive (damping ratio) | verified on hinge, slide, tendon, site, free joint and slider-crank |
| `muscle` | 1 | `[2]` peak force if positive: $G$; everything else unchanged (lengths are in units of $L_0$, `scale` is left alone) | same | `lengthrange`: $Q$, or recomputed when not given |
| `pid` | declared by `input`: `pos` $Q$, `vel` $Q/T$, `ff` $G$ | $k_i$: $G/Q$ | $k_p$, $k_v$: $G/Q$ | `imax`, `slewmax`: $Q$; `posrange`, `velrange`: $Q$; `ffrange`: $G$ |
| `orientation` (`so3`) | angle or quaternion: unchanged | $k_p$: $G$ (angle) | $k_v$: $G$ | `forcerange`: $G$ |
| `dcmotor` | | | | datasheet parameters in electrical and thermal units: **compile error** under a gauge with $(\lambda, \mu) \ne (1, 1)$; compiles under `geometry` |
| user gain, bias or dynamics; plugin actuators | | | | opaque parameters: **compile error** under $(\lambda, \mu) \ne (1, 1)$; untouched otherwise |

Common to all: `forcerange` $G$; `ctrlrange` $[u]$; `actrange` and keyframe `act` take
$[u]$ for integrator and filter dynamics and are unchanged for muscles; keyframe `ctrl` takes
$[u]$; `lengthrange` $Q$; `cranklength` $L$; actuator `damping[n]` $G/Q^{n+1}$ and `armature`
$G/Q$; `dynprm` time constants unchanged.

## 7. The boundary between gauges

A scaled subtree in an unscaled world is the normal case. Elements outside the tree take
their gauge from the elements they are expressed relative to, their **owners**. One owner:
its gauge. Several owners: the componentwise geometric mean of their gauges. There is no
abstention and no test for equal or unequal scales, so every factor varies continuously
with every `scale`, and reduces to the single-owner case when the owners agree.

| element | owners | fields |
|---|---|---|
| actuator | its transmission target (joint, tendon, body; the site, not the `refsite`; both sites of a slider-crank) | section 6.2 |
| tendon | every element on its path: sites, wrapping geoms, joints | `springlength` (when not −1), `range`, `margin`, `solimplimit` width: $Q$; `stiffness`, `damping`, `frictionloss`, `armature`, `actuatorfrcrange`: force class; `width`: $L$ |
| `connect` with `anchor` | `body1` | anchor $L$; `solimp` width $L$ |
| `weld` | `relpose`: `body1`; `anchor`: `body2`; `torquescale` and `solimp` width: both | $L$ |
| `connect`, `weld` with sites | | nothing but the `solimp` width (both sites) |
| joint or tendon equality | each coordinate separately | `polycoef[k]`: $Q_1 / Q_2^k$, with $Q_1$, $Q_2$ resolved by each joint's own gauge; `solimp` width: $Q_1$ |
| flex equality, flex | the flex's bodies | `radius`, `thickness`, vertex and node positions, `margin`, `gap`, `solimp` width, torsional and rolling friction: $L$; `young`: pressure; `edgestiffness`, `edgedamping`: force per length |
| contact `pair` | its two geoms | `margin`, `gap`, `solimp` width, `friction[2:5]`: $L$ |
| sensor | its object (never the reference) | `cutoff`, `noise`: the dimension of the output, force class if mass-bearing; user and plugin sensors untouched |
| keyframe | per column: the joint or actuator | slide `qpos`/`qvel`: $Q$; `ctrl`, `act`: section 6.2; free-joint position and `mpos`: below |
| `exclude`, tuples, text, numeric | | nothing |

**Free joints and mocap bodies.** A keyframe position of a free joint is authored in the
parent body's coordinates, which already include the frame's pose. The scaling frame acts
about its own origin, so for a frame at $(p, R)$ with shape factor $\sigma$ the position maps
as $x' = p + \sigma\,(x - p)$, composed over nested frames, and linear velocities scale by
$\sigma$. Multiplying by $\sigma$ is right only when the frame sits at the parent's origin.

**Deliberately unsupported, as compile errors**: a skin whose bones carry different gauges;
any element with a plugin under a non-identity gauge (a plugin scaling callback can lift this
later); the actuator cases of section 6.2.

## 8. Shared assets

A mesh or heightfield referenced at several shape factors compiles to several instances in
`mjModel`. The spec and the saved XML keep the single authored asset.

- **Identity.** An instance is keyed by (asset, $\sigma$), $\sigma$ compared bitwise. An asset
  referenced at one $\sigma$ compiles to one instance under its own name, scaled. With
  several, instances are created in order of first reference by geom id; the first keeps the
  asset's name and the others are named `name@1`, `name@2`, ... A collision with an authored
  name is a compile error.
- **Ownership.** Instances belong to the compile, not the spec: they are not visible to
  `mjs_*` iteration or to the writer, and are rebuilt by every compile.
- **Contract.** An instance is numerically equivalent to compiling the asset with its
  `scale` multiplied by $\sigma$. That is also the simplest implementation and the acceptance
  test. Deriving instances from the processed base mesh is an optimization: under a *uniform*
  factor every product of the mesh compiler is either invariant (faces, normals, texture
  coordinates, hull graph and polygon structure, principal-axis orientation) or a power of
  $\sigma$ (vertices, centre, BVH boxes, inertia box: 1; area: 2; volume: 3; inertia: 5). The
  general linear case is *not* a plain product, because meshes are stored in their
  centre-of-mass principal frame, which a stretch changes; there only the hull topology is
  reusable. Skins are scaled with their bones (one gauge, section 7); heightfields scale
  through `size`.

**Alternative considered: scale meshes at runtime through `geom_size`.** Today a mesh geom's
`geom_size` is not an input: the compiler overwrites it with the mesh's AABB half-extents. It
could instead be a per-geom scale, default 1 1 1, applied wherever mesh data is consumed.
The arithmetic is cheap (the support function of a scaled convex set is
$S\,\mathrm{support}_K(S d)$; rays and BVH boxes likewise) and it would let mesh sizes be
randomized without recompiling, with the caveat primitives already have, that mass,
inertia, `geom_rbound` and `geom_aabb` do not follow. The cost is blast radius: every reader
of `mesh_vert` must apply it (convex, SDF and continuous collision, rays, sensors, IPC, both
renderers with inverse-transpose normals, MJX, MuJoCo Warp), third-party consumers of
`mjModel` would be silently wrong, and the AABB needs a new home. Since the factor is known
at compile time, instances give the same authoring experience with no change to `mjModel`
semantics, at the price of memory. Not a prerequisite; if it lands later on its own merits,
instances collapse into it invisibly.

## 9. Compiler lifecycle and ownership

1. **Authored values** live in the `spec` structs, including `mjsFrame.scale` and
   `scaling`. Scaling never writes to them. Defaults need no handling: an element's
   constructor has already copied its class in.
2. **Working copies.** Each compile begins by copying `spec` into the element's working
   fields (`CopyFromSpec`, as today). Gauges are computed top-down over bodies and frames at
   the start of the compile. Each element applies its gauge to its working copy immediately
   after the copy and *before* any derivation. Recompiling starts from `spec` again, so
   nothing accumulates: compiling at 2, then 0.5, then 2 reproduces the first model bit for
   bit.
3. **Order, all by construction.** Asset instances are collected from geom and site
   references before meshes compile. Everything downstream consumes scaled working copies
   unchanged: `fromto` resolution, primitive fitting to meshes, frame pose accumulation (a
   child's position is scaled by its own gauge, then posed by the enclosing frame, which
   reproduces the nested similarity with no change to the accumulation code), inertia
   inference, free-joint alignment, bounding volumes. Non-tree elements resolve their
   references, take their owners' gauge, scale, then run their existing derivations
   (`inheritrange`, DC-motor parameters). Quantities computed on `mjModel` afterwards follow
   automatically: damping ratios, `springdamper`, auto `springlength`, `lengthrange`, `acc0`,
   muscle `force="-1"`.
4. **Saving.** The writer's input is already compiled-space data: `CopyBack` copies
   `mjModel` into the working copies so that runtime edits are saved. The writer therefore
   applies the inverse transform to a temporary copy of each element before writing: the
   inverse frame pose (in place on `frames-roundtrip`) and then the inverse gauge, which is
   the same generated table and the same custom functions evaluated at the reciprocal
   factors. External actuators, tendons, equalities, pairs, sensors and keyframes are
   covered by the same path. A runtime edit of `geom_size` under a `scale="2"` frame is saved
   as half its value and reloads as edited. Values pass through $x \cdot s / s$, so a save at
   full precision can differ from the authored text in the last digit, as quaternions
   already do.

## 10. Acceptance tests

Table-independent, so that a wrong exponent cannot certify itself:

1. **Symmetry.** For a corpus of models, wrap the world's contents in one `similar` frame;
   the result in gravity $g$ must match the original in gravity $g/s$ over a rollout
   (lengths over $s$, angles equal; sensors by their output dimension), with the documented
   exceptions of section 3. This is the prototype's main check and the test of the whole
   exponent table, including every `dim=custom` function.
2. **Identity.** `scale="1"` compiles to a bit-identical `mjModel`.
3. **No accumulation.** Compile at 2, 0.5, 2: first and third models are bit-identical.
4. **Round trip.** Save and reload preserve the model, the frames with `scale` and
   `scaling`, and the ability to change the scale: reload-then-rescale equals
   rescale-then-save.
5. **Composition.** Nested frames with mixed policies produce the product gauge (the
   $(1, 2, 8)$ example); partial-body frames scale only what they wrap.
6. **Nested translated frames.** Free-joint keyframes and mocap positions land where the
   composed similarity puts them (`xpos` after `mj_resetDataKeyframe`).
7. **Boundary.** The numbers of section 5.3, and continuity: a pair at scales $(2, 1)$ and
   $(2, 1 + 10^{-9})$ differ by $O(10^{-9})$.
8. **Assets.** Example 5.2; an instance equals the asset compiled with `scale` multiplied
   by $\sigma$; instances do not appear in the spec or the saved XML.
9. **Examples.** The tables of section 5 field by field; under `geometry` only the shape
   class changes.
10. **Errors.** Non-positive, non-finite and (in v1) anisotropic `scale`; `dcmotor`, user
    and plugin actuators under a force gauge; a skin across gauges.

## 11. Prerequisites and findings

- **Frames round-trip through save.** Done (`frames-roundtrip`). Contract: the writer emits
  authored values and authored structure; the compiler re-derives what it derives. Every
  authored frame persists, named or not.
- **Frames must recompile.** Found with the prototype: `mjCFrame::compiled` is set in the
  constructor and never reset, so editing a frame's `pos` or `quat` after a first compile is
  silently ignored by later compiles. This defeats "a parameter, not an operation" for the
  attributes frames already have, and must be fixed first.
- **Free-joint alignment** is the other transform baked on save: the writer emits aligned
  poses and drops the joint's `align`. Under the writer contract it should write the
  unaligned values and `align`. Structurally alignment is a compiler-generated frame, so it
  can later move into the same pass; scale must precede it (`ipos` is a length), which the
  lifecycle above guarantees.
- **`<inertial>` nested in a frame ignores the frame's pose** today. Separate bug; for this
  design it fixes the rule that the inertial takes the body's gauge.
- **Engine**: the mixed-units sum in the joint-equality diagonal approximation (section 3).

## 12. Later

- **Anisotropy.** The attribute is already a vector in the frame's axes. Semantics when it
  comes: the *reference configuration* of the subtree is the affine image under
  $S = \mathrm{diag}(s_x, s_y, s_z)$, after which bodies move rigidly, so joints impose no
  restriction and a stretched fork on a free joint is fine. Each body's contents are reshaped
  in its own frame by $A = R_0^T S R_0$; the only restriction is whether each shape can
  represent its reshaped self (meshes always; boxes, ellipsoids and heightfields need $A$
  diagonal in the geom's frame; capsules and cylinders also need equal transverse factors;
  spheres need $A$ uniform). Explicit inertia transforms exactly through the second-moment
  tensor $\Sigma = \tfrac12 \mathrm{tr}(I)\mathbb{1} - I$: $m' = \det(A)\, m$,
  $\Sigma' = \det(A)\, A \Sigma A^T$. Scalar lengths without a direction and the force gauge
  use $\bar{s} = (\det S)^{1/3}$, and the exactness statement of section 1 is promised for
  uniform scale only. Recorded as the expected shape, not a commitment.
- **`material` policy**, $(s, s, s)$.
- **`mjs_scale(element, length, mass, time)`**: the destructive counterpart, for baking a
  scale into saved numbers and for whole-spec rescaling including `option`, where it is the
  exact symmetry. Same tables.
- **Unit conversion at load** is a separate proposal. The one conclusion worth keeping: it
  cannot be a scale on the root, because a file in millimetres that does not mention gravity
  must not end up with gravity 0.00981. Defaults are MKS and explicit values are in the
  file's units, which makes it a reader-time conversion of explicitly present attributes,
  driven by the same `dim` facets.
- **Runtime mesh scaling** (section 8), plugin scaling callbacks.

## 13. Order of work

0. Frames as parameters: round-trip (done), the recompile fix, optionally alignment
   un-baking.
1. `dim` facets in `mjcf.schema`, the units column, the coverage test. Self-contained and
   useful on its own; fixes the actuator conventions of section 6 in the documentation.
2. `scale` and `scaling` on frames: gauges, the generated table and the custom functions
   with their inverses in the writer, the boundary rules, asset instances compiled the
   simple way. Tests of section 10, with the prototype as the reference for expected values.
3. Asset instance derivation as an optimization; anisotropy; `material`; `mjs_scale`.
