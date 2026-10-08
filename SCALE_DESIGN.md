# Scaling models in MJCF and mjSpec

*Design proposal, 2026-09-08; seventh revision 2026-10-08. The reference scripts of
section 10.1 sit beside this document (fork branch `scale-design`). Everything this
design needs from the rest of MuJoCo is upstream, and so are its two preparatory steps
(section 13); the feature itself, `scale` and `scaling` on frames, is not implemented. The
dimensional rules of sections 3 and 6 are checked against simulation by the scripts; the
composition, boundary, asset and save rules of sections 4, 7, 8 and 9 are design. Section 10.1
says exactly what has been checked. Numbers in section 5 come from the Python prototype.*

## 1. Summary

A non-destructive **`scale` attribute on `<frame>`**, with a **`scaling`** attribute that
selects the physical policy. Everything under the frame is resized at compile time and the
spec is untouched. A model saved as written keeps both attributes; saved compiled, the
default, it keeps the scaled values instead (section 9).

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

Status, upstream: frames are saved and recompiled as parameters (`e312ce82d`, 3.14.0), an
`<inertial>` inside a frame follows the frame (`a28137941`, 3.14.0), and `mjs_delete` and
`mjs_bodyToFrame` handle frames (`b1ccf6796`). After 3.15.0, compilation stopped writing into
the spec (`dc8210572`), `mj_copyBack` writes model edits to the spec (`592c54ad3`), a spec can
be saved as it was written (`savecompiled`, `savecanonical`, `92b8f4b27`), and actuators
record and keep their shortcut element (`ab7f005c9`, `d4595d020`). The two preparatory steps
of this design followed: the physical dimension of every real MJCF attribute (`1b7d9e83b`,
section 6.1) and the declared meaning of actuator controls (`fcb83a173`, section 6.2).

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
frame (anisotropy, section 12). Saved as written, the attribute is one value when the three
are equal and is left out at 1.

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
- **force class**: every other mass-bearing field (stiffness, damping, armature,
  frictionloss, force limits, force-sensor cutoffs, flex moduli), and every actuator gain and
  bias parameter whatever its dimension.

The class is policy, not dimension. A cylinder's gain is its bore area, $L^2$, but `geometry`
promises the same actuators, so actuator gains are force class by rule; for every other
actuator the rule changes nothing, since their gains carry mass.

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

**Derived quantities.** Scaling transforms authored values; the compiler derives the rest
afterwards (section 9). A quantity the author specified in the time domain keeps that
specification, so under `geometry` the forces derived from it follow the inertia, while
forces the author specified explicitly stay. Under `similar` the two agree. With $s = 2$:

| authored | under `geometry` |
|---|---|
| slide joint, `springdamper="0.1 1"` | stiffness and damping $\times 8$ with the body's mass: time constant 0.1 and damping ratio 1 kept |
| slide joint, explicit `stiffness` and `damping` | unchanged: time constant $\times\sqrt 8$, damping ratio $\times 1/\sqrt 8$ |
| servo on a hinge, `dampratio="1"` | `kv` $\times\sqrt{32}$ with the reflected inertia (no armature): damping ratio 1 kept |
| servo on a hinge, explicit `kv` | unchanged: damping ratio $\times 1/\sqrt{32}$ |
| muscle, `force="-1"` | peak force `scale / acc0` follows the body: $\times 32$ on a hinge joint, $\times 16$ through a tendon whose moment arm doubles |
| muscle, explicit `force` | unchanged |
| contact `solref` time constant and damping ratio | unchanged, so contact stiffness follows the inertia, as in every MuJoCo model |

The rule: `geometry` keeps behaviour where the author specified behaviour, and forces where
the author specified forces.

Known inexactness of the engine itself, independent of this feature: a joint equality that
couples a slide to a hinge breaks the units symmetry at the $10^{-3}$ level, because the
constraint's diagonal approximation adds the two joints' inverse weights, a $1/\text{mass}$
and a $1/\text{inertia}$, without the polynomial's Jacobian. Same-kind couplings are exact.
Mesh vertices are `float`, so scaled meshes agree to $10^{-7}$ relative, not to roundoff.
The solver's early termination is not unit-invariant either: it compares the cost improvement
and the gradient norm, normalized by `meaninertia`, with one `tolerance`, quantities of
dimension $T^{-4}$ and $T^{-2}$ for angular coordinates. Exact similarity needs
`tolerance="0"` (a fixed iteration count) or a tolerance small enough not to matter. And a
model that starts exactly at a contact (a humanoid standing on the floor at distance 0) can
detect the contact in one copy and not in the other: tests start off such knife edges.

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
takes $\sigma^3$ of its own gauge. An explicit `<inertial>` takes the gauge of its own frame
ancestry, like any other element: since `a28137941` an inertial inside a frame is posed by
that frame, and it is scaled by it too. A body has one inertial, so there is nothing to
decompose.

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
$1 : 1.5^3 : 0.5^3$. Saved as written, the XML is the text above.

### 5.3 A resized robot in an unchanged environment

The arm of 5.1 under `<frame scale="2">`, plus a floor, a world site `hook` at 0 0 3, a site
`tip` and a site `tip2` on the arm, and:

| element | authored | compiled | rule |
|---|---|---|---|
| `connect body1="arm" body2="world" anchor="0 0 0.2"` | | anchor 0 0 0.4 | the anchor is in `body1` coordinates: owner `body1` |
| `connect site1="tip" site2="hook"` | | unchanged | both points are elements with their own gauges |
| `pair geom1="hand" geom2="floor"` | `margin` 0.01, torsional 0.005 | 0.01414, 0.00707 | two owners: geometric mean of their gauges, $\sqrt{2\cdot1}$ |
| spatial tendon `tip`–`tip2` | `springlength` 0.3, `stiffness` 100 | 0.6, 800 | one gauge throughout: $\sigma$, and $\mu$ for force per length |
| spatial tendon `tip`–`hook` | `springlength` 0.8, `stiffness` 100 | 0.8, 100 | the lowest common ancestor of `tip` and `hook` is the world: the bungee belongs to the environment. A second site on the arm changes nothing. With the default `springlength="-1"` the rest length is recomputed from the scaled sites |
| `framepos` of `tip` relative to `hook` | `cutoff` 4 | 8 | owner is the sensor's object; the reference only chooses coordinates |
| `touch` on `tip` | `cutoff` 10 | 160 (`similar`), 10 (`geometry`) | a force, on a scaled element |
| keyframe `qpos` | 0.3 | 0.3 | a hinge angle; a slide scales, a free joint maps through the frame (section 7) |

## 6. Dimensions and the actuator table

### 6.1 The `dim` facet (upstream, `1b7d9e83b`)

Every `double` and `float` attribute in `src/xml/mjcf.schema` declares its dimension; `int`
attributes are counts, indices and flags and carry none:

```
  pos        : double[3] = {0, 0, 0} (dim=L)
  friction   : double[1..3] = {1, 0.005, 0.0001} (dim="1, L, L")
  solref     : double[1..mjNREF] = {0.02, 1} (dim=solref)
  stiffness  : double[1..3] (dim="F Q^-1, F Q^-2, F Q^-3")
  damping    : double[1..3] (dim="F T Q^-1, F T^2 Q^-2, F T^3 Q^-3")
  ctrlrange  : double[2] (dim=U)
  element position : mjsActuator (control=Q) { ... }
  element touch : mjsSensor (output="M L T^-2") { ... }
```

Symbols: $L$, $M$, $T$; $A$, a plane angle, dimensionless but recorded; $Q$, the coordinate
of the joint, tendon or transmission; $F = M L^2 T^{-2} Q^{-1}$, the generalized force
conjugate to $Q$; $U$ and $Y$, an actuator's control and a sensor's output, declared once per
element by the `control` and `output` element facets (the parser requires them wherever an
attribute uses $U$ or $Y$). Keywords: `solref`; `custom`, depending on other attributes of
the element (`gainprm`, `polycoef`, geom `density` under `shellinertia`, sensor outputs
selected by `data`, camera `fovy`, `gear`); `opaque`, not expressible in these symbols (user
data, `dcmotor` electrical and thermal parameters, light `intensity`, `magnetic`). A
generated table in a new "Attribute dimensions" section of the XML reference lists all 767
attribute rows with $U$ and $Y$ resolved, and `doc_test` fails if a real attribute lacks a
dimension.

Raw asset data is in the asset's own units: mesh `vertex` and `refpos`, flexcomp `point` and
`origin` are dimensionless, and `scale`, which converts them to lengths, is $L$. A change of
units touches only `scale`, which is the only rule that also covers data loaded from files,
and it is the rule asset instances (section 8) follow. (Flexcomp applies `scale` before its
pose, contrary to what the reference said; the commit corrects the text.)

The shape/force class of section 3 is not in the facet: `armature` of a hinge and the
inertia of a body share the dimension $M L^2$, so the implementation marks the inertial
fields (mass, inertia, density, `boundmass`, ...) where it consumes them.

**Validation.** A script beside this document, `units_check.py` (run by
`units_check_run.py`), applies a change of units $(s_L, s_M, s_T)$ with independent factors
to every field of a
model, each factor computed from the facets, and compares trajectories and sensor values step
by step. This is stronger than the `similar` check of `scale_prototype.py`, which cannot tell
$M L$ from $L^4$. It reproduces to roundoff the humanoid models, `arm26` (muscles), the
slider-crank, the car (fixed tendons), `refsite`, the orientation servo, adhesion, and the
flex hammock, trilinear, flag and gripper models. A model with every
classic actuator shortcut, tendons, equalities, pairs and 90 sensor values of 39 sensor types
matches to the solver's early-termination test, and to roundoff with the tolerance set to 0
(section 3). It caught three errors in this document's actuator table, fixed in 6.2. It
reads the facets from any checkout at or after `1b7d9e83b` and runs on a released wheel:
`MUJOCO_SRC=<checkout> python units_check_run.py`. Rerun on the landed schema with the
3.15.0 wheel (2026-10-08), it gives the same results. The elasticity cable, which agreed to
roundoff on the 3.13.1 wheel, now differs at $10^{-2}$: its moduli are plugin configuration,
which the facets do not describe and the script leaves alone, and its plugin forces on this
model are ten times larger in 3.15.0. Plugins stay outside the facets (section 7).

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

Shortcuts are lowered to `general` parameters at parse time. The shortcut element is recorded
(`mjsActuator.type`, `ab7f005c9`) and kept when saving as written (`d4595d020`), but a
`general` actuator, and every actuator saved compiled or in the canonical notation, carries
only parameters, so what a control means is declared, not inferred. $G$ is the
generalized force of the actuator's coordinate ($M L T^{-2}$ or $M L^2 T^{-2}$), $Q$ the
coordinate.

An earlier draft inferred the control's meaning from coefficients: fixed gain and affine bias
with `biasprm[1] == -gainprm[0]` was a position setpoint. That is discontinuous. Under
`geometry` at $s = 2$, a slide actuator with gain 30 and position bias −30 had its
`ctrlrange` doubled, and one with bias −30.000001 did not.

**Declaration** (upstream, `fcb83a173`). A control is a
position setpoint (`pos`, dimension $Q$), a velocity setpoint (`vel`, $Q/T$), a pressure
(`pressure`, $F/L^2$, the gain an area), or a command (dimensionless, the gain carrying $G$).
The declaration is `general/input`, recorded in `mjsActuator.ctrlspec` and
`mjModel.actuator_ctrlspec`, which already hold the input signature of `pid`, `dcmotor` and
`orientation`. `position` declares `pos`; `velocity` and `intvelocity` declare `vel`;
`cylinder` declares `pressure`; the other shortcuts, and `general` without `input`, declare a
command, and `mj_actuatorInputName` reports the declaration. A `general` inherits an input
signature only from a default with the same gaintype (otherwise a velocity class would turn
an `orientation` actuator's chart into `quat`), a shortcut only from a default written with
the same shortcut or with a `general` of its gain model (`ab7f005c9`), and `input=""` clears an
inherited one. Files saved before
the change load as commands. The gain
multiplies the activation if the actuator has dynamics, of dimension $[ctrl]\,T$ for an
integrator (an `intvelocity`'s activation is its position setpoint) and $[ctrl]$ for a
filter, and the control otherwise; call that dimension $[in]$.

Under `similar` the declaration never changes the motion, only which numbers the user sends.
Commands keep their numbers under both policies: under `similar` the gain absorbs the force
scaling, so a controller transfers unchanged; under `geometry` the gain is force class and
unchanged, so the actuator stays the same actuator. Setpoints follow their coordinate. A
pressure changes with the units under `similar` ($s^2$, as does the area) and not at all under
`geometry`.

**The cylinder** keeps its physical meaning: the control is a pressure and `area` an area,
as documented and as the `schema-dims` facets already say. An earlier draft made it a command
with `area` a spelling of the gain, so that `geometry` would not enlarge the bore; that
changed a dimension to obtain a policy. The rule of section 3 states the policy directly:
under `geometry` the cylinder keeps its bore and the same pressure gives the same force, and
under `similar` pressure and area each scale by $s^2$, a change of units.

| gain / bias family | `ctrl` | `gainprm` | `biasprm` | notes |
|---|---|---|---|---|
| fixed or affine gain, no or affine bias: `motor`, `position`, `velocity`, `intvelocity`, `damper`, `cylinder`, `adhesion`, `general` | declared: `pos` $Q$, `vel` $Q/T$, `pressure` $F/L^2$, command 1 | `[0]`: $G/[in]$; affine `[1]`: $G/([in]\,Q)$, `[2]`: $G\,T/([in]\,Q)$ | `[0]`: $G$; `[1]`: $G/Q$; `[2]`: $G\,T/Q$ if negative, unchanged if positive (damping ratio) | verified on hinge, slide, tendon, site, free joint and slider-crank under independent $L$, $M$, $T$ |
| `muscle` | 1 | `[2]` peak force if positive: $G$; `[3]` `scale`: $T^{-2}$ for angular joint spaces (`force = scale / acc0`); `[6]` `vmax`: $T^{-1}$; the rest are in units of $L_0$ | same | `dynprm` time constants $T$; `lengthrange` $Q$, or recomputed by a simulation whose `compiler/lengthrange` times scale with $T$ |
| `pid` | declared by `input`: `pos` $Q$, `vel` $Q/T$, `ff` $G$ | $k_i$: $G/Q$ | $k_p$, $k_v$: $G/Q$ | `imax`, `slewmax`: $Q$; `posrange`, `velrange`: $Q$; `ffrange`: $G$ |
| `orientation` (`so3`) | angle or quaternion: unchanged | $k_p$: $G$ (angle) | $k_v$: $G\,T$ | `forcerange`: $G$; verified |
| `dcmotor` | | | | datasheet parameters in electrical and thermal units: **compile error** under a gauge with $(\lambda, \mu) \ne (1, 1)$; compiles under `geometry` |
| user gain, bias or dynamics; plugin actuators | | | | opaque parameters: **compile error** under $(\lambda, \mu) \ne (1, 1)$; untouched otherwise |

Common to all: `forcerange` $G$; `ctrlrange` and keyframe `ctrl` the dimension of `ctrl`;
`actrange` and keyframe `act` $[in]$, unchanged for muscles; `lengthrange` $Q$; `cranklength`
$L$; actuator `damping[n]` $G\,T^{n+1}/Q^{n+1}$ and `armature` $G\,T^2/Q$; `dynprm` time
constants $T$.

## 7. The boundary between gauges

A scaled subtree in an unscaled world is the normal case. Elements outside the tree take
their gauge from the elements they are expressed relative to, their **owners**, by a rule per
relationship. Every rule is structural: none tests scale values for equality, so every factor
varies continuously with every `scale`, and none depends on how a path is described.

- **One owner**: its gauge.
- **Two participants**, a contact `pair` or a `weld`: the componentwise geometric mean of the
  two gauges.
- **A path**, a tendon or a flex: the gauge of the lowest frame or body containing all of its
  path elements (sites, wrapping geoms, joints) or bodies. A redundant routing point does not
  change it. An earlier draft took the geometric mean over path elements, and a second site
  at the same position on the scaled arm of example 5.3 changed the bungee's scaled stiffness
  from 282.8 to 400 with the path unchanged. A fixed tendon's coefficients absorb its joints'
  own gauges, $c_i' = c_i\,Q_t / Q_i$, so its length scales exactly by the common gauge. A
  spatial tendon's length is whatever the scaled geometry gives: an automatic `springlength`
  is exact, an explicit one takes the common gauge.

  That gauge is for the path's shared properties. Its points are attachment coordinates and
  stay with their bodies: a site is an element of its body already, and a flex vertex
  position, given in its body's coordinates, takes that body's gauge, so a flex spanning a
  scaled body and the world moves its attachment on the scaled body exactly as a site there
  would move. When the lowest container is a frame, its gauge is the one in force inside it,
  the frame's own factor included.

| element | owners | fields |
|---|---|---|
| actuator | its transmission target (joint, tendon, body; the site, not the `refsite`; both sites of a slider-crank) | section 6.2 |
| tendon | the lowest frame or body containing its path; fixed tendon coefficients per joint | `springlength` (when not −1), `range`, `margin`, `solimplimit` width: $Q$; `stiffness`, `damping`, `frictionloss`, `armature`, `actuatorfrcrange`: force class; `width`: $L$; `coef`: $Q_t/Q_i$ |
| `connect` with `anchor` | `body1` | anchor $L$; `solimp` width $L$ |
| `weld` | `relpose`: `body1`; `anchor`: `body2`; `torquescale` and `solimp` width: both | $L$ |
| `connect`, `weld` with sites | | nothing but the `solimp` width (both sites) |
| joint or tendon equality | each coordinate separately | `polycoef[k]`: $Q_1 / Q_2^k$, with $Q_1$, $Q_2$ resolved by each joint's own gauge; `solimp` width: $Q_1$ |
| flex equality, flex | the lowest frame or body containing the flex's bodies; vertex positions: their own bodies | `radius`, `thickness`, `margin`, `gap`, `solimp` width, torsional and rolling friction: $L$; `young`: pressure; `edgestiffness`, `edgedamping`: force per length; vertex positions: $L$ |
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
  `mjs_*` iteration or in a model saved as written, and are rebuilt by every compile. Saved
  compiled, each generated instance is written as an asset of its own, `name@k` with its
  `scale` multiplied by $\sigma$, which reloads to the same model.
- **Correspondence.** Everywhere else one authored element compiles to one model element,
  which `mj_copyBack` and the spec operations rely on. An asset compiles to a primary
  instance, under its own name, and generated ones. `mjs_getId` and binding return the
  primary; generated instances are reached by name. `mj_copyBack` copies an asset with
  several instances as a family: a change is written only if every instance carries it, and
  otherwise is an error like the others it lists, since one authored asset cannot hold an edit
  which some of its instances lack (an edit to the primary alone would also change the
  untouched instances at the next compile). Of what it copies, only heightfield elevation can
  differ between instances, and elevation data is dimensionless, so a consistent family has
  identical data: a changed mesh frame, and the size of a mesh or heightfield geom, are
  already errors.
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
   `scaling`. Scaling never writes to them, and since `dc8210572` neither does the rest of
   compilation. Defaults need no handling: an element's constructor has already copied its
   class in.
2. **Compiled copies.** Each compile begins by copying `spec` into the element's compiled
   copy (`CopyFromSpec`, as today). Gauges are computed top-down over bodies and frames at
   the start of the compile. Each element applies its gauge to its compiled copy immediately
   after the copy and *before* any derivation. Recompiling starts from `spec` again, so
   nothing accumulates: compiling at 2, then 0.5, then 2 reproduces the first model bit for
   bit.
3. **Order, all by construction.** Asset instances are collected from geom and site
   references before meshes compile. Everything downstream consumes scaled compiled copies
   unchanged: `fromto` resolution, primitive fitting to meshes, frame pose accumulation (a
   child's position is scaled by its own gauge, then posed by the enclosing frame, which
   reproduces the nested similarity with no change to the accumulation code), inertia
   inference, free-joint alignment, bounding volumes. Non-tree elements resolve their
   references, take their owners' gauge, scale, then run their existing derivations
   (`inheritrange`, DC-motor parameters). Quantities computed on `mjModel` afterwards follow
   automatically: damping ratios, `springdamper`, auto `springlength`, `lengthrange`, `acc0`,
   muscle `force="-1"`.
4. **Saving.** Both forms are upstream (`92b8f4b27`). Saved as written
   (`savecompiled="false"`), the writer reads the spec, so it needs only the frame's `scale`
   and `scaling`. Saved compiled (the default), the writer writes what compilation made, so
   the scale is baked: scaled values in frame-local coordinates and no `scale` (an unnamed
   identity frame is then left out, as for any frame), and generated asset instances written
   as assets of their own (section 8). Neither form needs an inverse, with one exception:
   saved as written with `saveinertial="true"`, the writer inserts the inertia which
   compilation inferred (`mjXWriter::WriteSpecInertial`) into otherwise authored XML, inside
   frames which keep their `scale`. It is written in its body's gauge, divided as
   `mj_copyBack` divides, as it is already written from before `settotalmass` scaled it;
   the spec is not modified.
5. **Writing compiled values back.** `mj_copyBack` (`592c54ad3`) writes each changed value as
   what compiles to it. Under a gauge that is the value divided by its factor, from the same
   table at the reciprocal factors, beside the conversions it already makes (poses into the
   enclosing frame, angles into the spec's unit). The spec operations which write what
   compilation computes follow the same rule, in the gauge of the element that receives the
   value: `mjs_adoptInertial`, and the inertia that `mjs_fuseStatic`, `mjs_discardVisual` and
   `mjs_bodyToFrame` merge or adopt. Without it, an inertia adopted under a `scale="2"` frame
   is scaled again by the next compile. A merge needs the conversion on the way in as well:
   `mjCBody::MergeInertial` combines a parent's inertia with a child's, each either given in
   the spec or inferred by compilation, so both are first brought to compiled values in the
   parent's coordinates (a given inertia multiplied by its own gauge), merged there, and the
   sum divided by the receiving body's gauge. An unscaled parent of mass 1 and a child of
   mass 1 under `scale="2"` merge to 9, not 2. `mjs_fuseStatic` replaces a body by a frame
   in the same place, so the elements it moves keep their gauge.

## 10. Acceptance tests

Table-independent, so that a wrong exponent cannot certify itself:

1. **Symmetry.** For a corpus of models, wrap the world's contents in one `similar` frame;
   the result in gravity $g$ must match the original in gravity $g/s$ over a rollout
   (lengths over $s$, angles equal; sensors by their output dimension), with the documented
   exceptions of section 3. This is the prototype's main check and the test of the whole
   exponent table, including every `dim=custom` function.
2. **Identity.** `scale="1"` compiles to a bit-identical `mjModel`.
3. **No accumulation.** Compile at 2, 0.5, 2: first and third models are bit-identical.
4. **Round trip.** Saved as written and reloaded: the same model, the frames with `scale`
   and `scaling`, and the ability to change the scale (reload-then-rescale equals
   rescale-then-save). Saved compiled and reloaded: the same model, scale baked. An edit to
   a scaled element in `mjModel`, copied back and recompiled, reproduces the edit, and so
   does `mjs_adoptInertial` on a body under a scaled frame.
5. **Composition.** Nested frames with mixed policies produce the product gauge (the
   $(1, 2, 8)$ example); partial-body frames scale only what they wrap.
6. **Nested translated frames.** Free-joint keyframes and mocap positions land where the
   composed similarity puts them (`xpos` after `mj_resetDataKeyframe`).
7. **Boundary.** The numbers of section 5.3; continuity: a pair at scales $(2, 1)$ and
   $(2, 1 + 10^{-9})$ differ by $O(10^{-9})$; path independence: a tendon with a redundant
   routing point at an existing point compiles to the same stiffness and rest length.
   Actuator semantics: a slide servo declared `pos` and a `general` with the same
   coefficients and no declaration differ only in `ctrlrange` under `geometry`, and both
   survive save and reload.
8. **Assets.** Example 5.2; an instance equals the asset compiled with `scale` multiplied
   by $\sigma$; instances do not appear in the spec or in XML saved as written. A
   heightfield with two instances: elevation edited in the primary alone is a copy-back
   error, edited identically in both is copied.
9. **Examples.** The tables of section 5 field by field; under `geometry` only the shape
   class changes.
10. **Errors.** Non-positive, non-finite and (in v1) anisotropic `scale`; `dcmotor`, user
    and plugin actuators under a force gauge; a skin across gauges.
11. **Inertia written from compilation.** Saved as written with `saveinertial="true"` under
    a scaled frame and reloaded: the same model. Merges by `mjs_fuseStatic` and
    `mjs_bodyToFrame` for every combination of given and inferred inertia, parent and child,
    scaled and unscaled, compile to the model which the unmerged spec compiles to (the case
    of section 9: 1 and 1 under `scale="2"` give 9).

### 10.1 What has been checked, and what has not

The scripts beside this document are investigation tools: they print discrepancies and do not
fail. The implementation's tests assert.

- **Checked against simulation:** the dimension of every field the corpus exercises, under
  independent changes of length, mass and time units, on whole models (`units_check.py`,
  section 6.1); the `similar` policy on whole models and on subtrees chosen per body, with
  the scaling frame at the parent's origin (`scale_prototype.py`).
- **Not checked, because nothing implements it yet:** gauges along frame ancestry (the
  prototype assigns them per body), frames away from their parent's origin and nested frames,
  partial-body scaling, mixed policies, asset instances, recompilation, saving, and
  copy-back. The boundary numbers of section 5.3 are printed by the prototype, not asserted,
  and it predates two rules: it takes the geometric mean over a tendon's path (the bungee
  prints stiffness 282.8, where section 7 gives 100) and infers control semantics from
  coefficients rather than reading `input`. Section 5.3 and section 6.2 are the reference.
- **Upstream with their own tests:** the facets and their coverage (section 6.1) and the
  declared control semantics (section 6.2).

## 11. Prerequisites and findings

- **Frames round-trip through save.** Upstream, `e312ce82d`. Saved compiled, frames keep
  their poses and their contents are frame-local, except an unnamed identity frame with an
  inherited class, which is left out; saved as written (`92b8f4b27`), every frame is kept as
  it was written.
- **Frames must recompile.** Found with the prototype: `mjCFrame::compiled` was set by the
  first compile and never reset, so editing a frame's `pos` or `quat` afterwards was silently
  ignored by later compiles. Fixed in the same upstream commit. Alignment adjusts the
  compiled pose of a body's frames, which must be recomputed by every compile.
- **Free-joint alignment** is baked when saving compiled (aligned poses, no `align`) and kept
  when saving as written (`92b8f4b27`). Frames inside an aligned body are carried along by
  the alignment. Structurally alignment is a compiler-generated frame; scale must precede it
  (`ipos` is a length), which the lifecycle above guarantees.
- **`<inertial>` nested in a frame** now follows the frame (`a28137941`), so it takes the
  frame's gauge (section 4).
- **Spec and compiled model** are separate upstream: compiling no longer writes into the spec
  (`dc8210572`), `mj_copyBack` writes model edits to it (`592c54ad3`), and fusing static
  bodies, discarding visual elements and adopting an inferred inertia are operations on the
  spec (`mjs_fuseStatic`, `mjs_discardVisual`, `mjs_adoptInertial`). Section 9 builds on all
  three.
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

0. Frames as parameters, and the separation of spec and compiled model. **Upstream**:
   `e312ce82d`, `a28137941`, `b1ccf6796`; `dc8210572`, `592c54ad3`, `92b8f4b27`.
1. `dim` facets in `mjcf.schema`, the units table, the coverage test. **Upstream**:
   `1b7d9e83b`, validated by `units_check.py`.
2. Declared control semantics (section 6.2). **Upstream**: `fcb83a173`.
3. An end-to-end milestone before the full step: `scale` on `<frame>` with one gauge pass
   along frame ancestry, exercised by a scaled frame holding an actuated body and a shared
   mesh, nested and translated frames, recompilation, saving in both forms and reloading, and
   copy-back of an edit under a scale, with asserting tests, including cases 4, 8 and 11
   of section 10. It tests the compiler architecture while the policies are still cheap to
   change.
4. The rest of `scale` and `scaling`: the remaining relationships of section 7, `geometry`,
   keyframes, the generated table and custom functions, and their inverses for `mj_copyBack`
   and the spec operations (section 9). Tests of section 10, with `units_check.py` and the
   prototype as references.
5. Asset instance derivation as an optimization; anisotropy; `material`; `mjs_scale`.
