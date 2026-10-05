# What compiling does to a spec

Status: direction agreed in review. The three stretches of section 11 are
implemented, on the fork branches `spec-compile-fixes`,
`spec-compile-operations` and `spec-compile-saving`, and combined with the
neighbouring fork branches. The series to review and import is the branch
`spec-compile-review`: this document, then 19 commits on upstream main
73b63c92a, one per unit of review, listed at the end of section 11;
`spec-compile-stack` is the same code in the 63 commits it was made of, on
07dfe9165.
Statements are labelled where it matters:

- **Decided**: agreed in review.
- **Preference**: what we want users to get.
- **Strategy**: how we propose to get it. Open to change.
- **To verify**: believed but not yet checked.

"Today" behaviour marked *measured* was observed by running the current code;
*read* means it follows from the source but was not run. Checked against
upstream main e385ec699.

## 1. The direction

1. **A compile that restructures nothing does not change your spec.** Values,
   structure, names and order are as you wrote them. Compilation records
   binding next to the spec (which compiled id and state addresses each
   element got) and diagnostics. The one exception is that keyframes are
   completed after the tree has changed (section 7).
2. **Restructuring is an operation on the spec.** Fusing static bodies and
   discarding visual elements are named, documented operations that change the
   spec in place. `<compiler fusestatic>` and `<compiler discardvisual>` mean
   "apply the operation, then compile". After it, every element of the spec
   has exactly one counterpart in the model, as always.
3. **Inferred values can be inspected, adopted or exported**, and these are
   three different things.
4. **Saving writes what was authored; compiled values are requested.** A
   direction, implemented separately from the rest (section 5).

**Not a goal**: compiling on a hidden copy so that the restructuring options
leave the spec alone. It would give up the one-to-one correspondence between
spec and model, which is what keeps binding, `mj_copyBack`, saving and
`mj_recompile` simple. Anyone who wants to keep the original spec calls
`mj_copySpec` before compiling.

## 2. What changes for users

| You do | Today | Proposed |
|---|---|---|
| Compile, no restructuring options | Compiling reorders your `<contact><pair>` list and writes names onto unnamed file-backed assets (*measured*). Values cached by one compile are sometimes reused by the next (two such bugs fixed last month). | Your spec is untouched, by test. Compile, edit, compile gives the model of a newly written spec with that edit. |
| Compile with `fusestatic` | The static bodies are deleted from your spec and pointers to them are dead (*read*). A second compile of the same spec moves the fused body's geoms: one at `1 0 0.5` lands at `0 0 0.5`. Looking up a geom or site by name on the spec returns a different element when ids have shifted (*measured*). | `mjs_fuseStatic` is applied: the body becomes a frame in its parent, holding its contents in their authored coordinates. Compiling again, copying and saving all give the same model. Lookups are right. Handles to the fused body stay valid and say it is gone. |
| Compile with `discardvisual` | Visual geoms and meshes and all materials and textures are deleted from your spec. Looking up a discarded geom or material by name then throws a C++ exception through the C API. With `inertiafromgeom="true"`, a second compile drops the body's mass from 37.7 to 4.19 (*measured*). | `mjs_discardVisual` is applied. Lookups work. Compiling again gives the same model. Under `inertiafromgeom="true"`, if a body's inertia would change, it is an error raised before anything is deleted. |
| Compilation fails | If it fails after fusing, the bodies are already gone from your spec (*measured*). | A restructuring that succeeded stays applied, the spec is complete and equivalent to the one you wrote, and the error says so. An operation that itself fails changes nothing. |
| Keep the original while compiling a fused model | No documented way. | `mj_copySpec`, then compile the copy. |
| Fuse or discard without compiling for it | Not possible. | Call the operation. |
| Read an inferred inertia | `m->body_mass[mjs_getId(body->element)]`; the body stays geometry-driven (*measured*). | Same, documented as the way to inspect. |
| Make an inferred inertia permanent | No operation. | An explicit adoption operation. |
| Add a mesh by file through the API and refer to it by the file stem | Works: the compiler writes the stem as the name. | Give the mesh a name. In XML the parser still derives it from the file. |
| Edit an `mjModel`, then `mj_copyBack` | The next save has the edit. Your spec does not: the values you read from it are unchanged, and the next compile discards the edit (*measured*). | `mj_copyBack` writes the edit into the spec, as its documentation says. Saving and compiling both keep it. |
| `mj_saveXML` | Writes compiled values: resolved poses, quaternions, radians. | Direction: writes what you authored. Compiled values on request. |
| `mj_recompile` | | Unchanged. |

Compatibility. Three things a working program could notice from rules 1 and 2:
the pair list keeps its authored order; an asset created through the API with
only a file no longer answers to the file stem; and a model combining
`discardvisual` with `inertiafromgeom="true"` whose visual geoms carry mass
fails to compile with an error naming the fix. The saving direction changes
the default output of `mj_saveXML` and is taken separately.

## 3. Walkthrough: inertia

A body with geoms and no `<inertial>`.

```c
mjSpec*  s    = mj_parseXML("arm.xml", NULL, err, sizeof(err));
mjsBody* link = mjs_findBody(s, "link");

mjModel* m = mj_compile(s, NULL);                       // infer
int id = mjs_getId(link->element);
double mass = m->body_mass[id];                         // inspect

mjs_asGeom(mjs_findElement(s, mjOBJ_GEOM, "shell"))->size[0] *= 2;   // edit
mjModel* m2 = mj_compile(s, NULL);                      // infer again
```

**Inspect.** The values are in `mjModel`, reached through binding. Nothing is
written to the spec and inference stays on, so `m2` has the new mass. This
works today and needs only documenting. What you read is what the model has,
after `settotalmass`; to see values before global scaling, compile with it
off. A model describes the spec as it was when compiled: after an edit,
compile again. We do not promise to detect a stale model, because the spec
signature covers structure only.

**Adopt.** An explicit operation makes the calculated inertial part of the
authored body, so later geometry edits no longer change it.

```c
mjs_adoptInertial(link);
```

It calculates the inertia from the spec as it is now, as the operations of
section 4 do (**decided**), so there is no model to be out of date. The value
written is the one before `settotalmass` scaling, which is applied to the
compiled model, so compiling again gives the same model whichever bodies were
adopted. The route from a model is `mj_copyBack`.

Writing the fields is not enough on its own: under `inertiafromgeom="true"`
the compiler infers inertia for every body whatever its authored inertial
says. **Decided**: adoption is supported per body under `auto` and `false`,
and is an error under `true`. No operation changes the policy on the user's
behalf.

**Export.** Write a file with the calculated inertials while the live spec
stays geometry-driven: `<compiler saveinertial="true"/>`, today and under
section 5.

**How the operations treat inference.** Fusing moves the geoms of the fused
body to its parent, so if both bodies were geometry-driven the result still
is. If either had an authored inertial, the parent gets the merged one,
authored. Discarding deletes geoms, so a body that loses geoms carrying mass
has its inertia adopted, under the rule above.

## 4. The operations

`mjs_fuseStatic` and `mjs_discardVisual` (**decided** names). Each is the
single implementation behind both the explicit call and the compiler option.

**Contract**

- In place, and all or nothing. The operation resolves and checks everything
  it needs before it changes anything. If it fails, the spec is as it was. If
  it succeeds, the spec is a valid spec that compiles to the same model as
  before, apart from what the operation is for.
- A later compilation error does not undo it. The error message says that the
  restructuring was applied and that the spec can be fixed and compiled again.
- Idempotent: applying it to its own result does nothing.
- Handles to elements it removes stay valid and report that the element is
  gone, as after `mjs_delete`. Name lookups on the spec are correct afterwards.

**Fuse.** A body is fused when it has no joints, is not a mocap body and is
not referenced by name. It is replaced by a frame in its parent carrying the
body's pose; its geoms, sites, cameras, lights, frames and child bodies are
nested in that frame and keep their authored coordinates. The body's name and
user data are discarded. Bodies whose fusing would change the physics are left
alone: a `gravcomp` different from the parent's, a plugin, a sleep policy, a
non-zero fluid medium, a reference from a flex or a skin.

Opting a body out (**decided**): a body attribute, `fuse="auto"` by default
and `fuse="false"` to keep the body. The policy is then authored, visible and
saved with the model, and the operation needs no arguments. Bodies have no
defaults, so it is set per body; that fits, since keeping a body is the rare
case and `<compiler fusestatic>` remains the switch for the model. It adds a
field to `mjsBody`. There is no `"true"`: forcing a fuse past a reference or a
physics rule would produce a different model.

Binding after a fuse (*measured* on today's code). `bind()` checks that the
spec and model signatures agree and then indexes by the element's id. Both
hold: the signature is updated, and every surviving body, joint, geom, site
and camera reports the id it has in the model, so binding a handle held from
before the compile is correct. What is wrong is the lookup: asking the spec
for a geom or site by name can return a different element, so
`model.bind(spec.geom('a'))` binds the wrong one, and a handle to the fused
body itself is dead (*read*). The contract above removes both.

**Discard.** Deletes geoms that do not collide, are not referenced by a pair,
a tendon path, a sensor or a tuple, and do not use the ellipsoid fluid model;
meshes that no remaining geom and no site uses and nothing refers to; and all
materials and textures, clearing the references to them wherever they are
(geoms, sites, meshes, skins, flexes, tendons, lights, default classes).
Bodies that lose geoms carrying mass have their inertia adopted. Under
`inertiafromgeom="true"` that adoption cannot hold, so the operation reports
the conflict and deletes nothing.

**`mjs_bodyToFrame`** (**decided**) uses resolved inertia. Today it merges
only an inertial the child authored, so an authored parent with a child whose
mass comes from its geoms loses that mass, and the reverse case stops counting
the parent's geoms. With resolution, a mix of authored and inferred inertia
gives the same result as fusing, and never depends on whether the caller
compiled earlier. Resolution is needed only in the mixed case.

**Resolution: from the current spec, not from an earlier compile**
(**preference**). The alternative, requiring the spec to have been compiled,
is the cheapest to build, since compiled masses and processed meshes are
already on the elements, and it costs nothing when the compiler option invokes
the operation. But "has been compiled" does not mean "is current": an edit
after the compile leaves those values stale, and nothing detects it. The
operation would silently use old inertia.

So an operation runs the part of compilation it depends on:

| | Needs resolved |
|---|---|
| `mjs_adoptInertial` | assets and the kinematic tree |
| `mjs_bodyToFrame`, mixed inertia only | the same |
| `mjs_fuseStatic`, if a body is static | the same, and the bone names of skins read from files |
| `mjs_discardVisual`, if inertia is inferred from a discarded geom | the same, without textures |

Implementation (*as built*). The first stage of `TryCompile`, assets and the
kinematic tree, is callable on its own (`mjCModel::Resolve`), so one piece of
code serves compilation and the operations. A second stage turned out not to
be needed: what refers to a geom or a mesh is read from the names in the spec
(pairs, tendon paths, sensors, tuples, sites), and compilation no longer marks
geoms and meshes as visual. A discard therefore compiles nothing unless a body
infers its inertia from a geom that is discarded, and a fuse compiles nothing
unless a body is static.

Cost (*measured*, 351 models, release build, interleaved runs). A compilation
that applies an operation compiles the assets once: the operation does, and
the compilation that follows reuses them and compiles only the kinematic tree
again. Without that, `fusestatic` took 1.39 times as long as before over the
corpus and nearly twice as long on models dominated by meshes or signed
distance fields; with it, 1.03. `discardvisual`, both options together and
plain compilation are within a few percent of what they were. An explicit
operation followed by a separate compile does compile the assets twice, since
the spec may have been edited in between.

## 5. Saving

**Direction (decided), implemented separately.** `mj_saveXML` writes what was
authored. Compiled values are something you ask for.

"What was authored" has three levels, and they need separate answers.

1. **Values**: the fields of the spec. An authored save writes them. A
   compiled-values save writes what compilation made of them instead: aligned
   poses, calculated inertia, fitted sizes.
2. **Representation**: how a value is expressed. An orientation as Euler
   angles or a quaternion, a capsule as `fromto` or as pose and size, angles
   in degrees or radians, `fullinertia` or a diagonal with a frame. The spec
   keeps these, so an authored save can preserve them; a separate control
   normalises them (quaternions, radians), which is what the writer does
   unconditionally today.
3. **Notation**: which XML spelling the author used when several produce the
   same spec fields. The spec does not keep this. Actuator shortcuts are the
   main case: the parser expands `<position>` and its siblings into general
   gain and bias parameters, and the writer emits `<general>` for every
   actuator today. Reading authored fields does not recover the tag. Either
   the shortcut is recorded on the element when the parser or `mjs_setTo*`
   applies it, with a rule for when a later edit invalidates it; or the writer
   recognises the parameter pattern of each shortcut; or `<general>` stays.
   Outside this work (section 8).

**Where authored and compiled differ.** These are the categories a saving
policy has to cover:

| Category | Authored | Compiled | Kind |
|---|---|---|---|
| Orientation | Euler, axis-angle, `xyaxes`, `zaxis`, or quat | quat | representation |
| Angle unit, Euler sequence | degrees or radians, any sequence | radians | representation |
| Capsule-like shapes | `fromto` | pose and size | representation |
| Inertia spelling | `fullinertia` | diagonal and frame | representation |
| Inertia from geoms | no `<inertial>` | calculated mass, frame, inertia | value |
| Free-joint alignment | authored pose, `align` | aligned pose | value |
| Sizes fitted to a mesh | `fitscale`, mesh reference | numbers | value |
| Global mass scaling | `settotalmass` | scaled masses | value |
| Actuator length range | unset | computed range | value |
| Actuator shortcuts, macro elements, includes | `<position>`, `<replicate>`, ... | expanded | notation |

Frames and default classes are preserved already and need no policy.

**Controls** (**decided**). Global, not per element, for three reasons.
Units are per file by nature. The per-element choice already exists where it
matters, because each element records which spelling it was written in; the
control only says whether to honour it. And a saving switch on every element
would be tooling state in the model, with bodies not even having defaults to
set it from.

Three are enough, all on `<compiler>`, where API users can set them before a
save and XML users can author them:

- which values: authored, or compiled. Compiled writes the value column above;
- which representation: as written, or canonical (quaternions, radians, pose
  and size);
- `saveinertial`, as today: add calculated inertials, whatever the other two
  say.

They are instructions for a save, so the writer does not echo them into the
file; `saveinertial` already behaves this way. Names and rollout are in
section 8. Notation is not a control: whatever is done about shortcuts applies
to every save.

**`mj_copyBack` does not do what its documentation says.** The header reads
"copy real-valued arrays from model to spec", and one would expect the spec's
values to change. They do not. It writes the spec's internal compiled fields,
which are separate from the fields a user reads and edits, and which the
writer happens to read. Measured: set a geom size to 0.5 in the model and copy
back; the size read from the spec is still 1; the saved XML says 0.5; compile
the spec again and the model has 1. The function was written for
`mj_saveLastXML` at a time when the compiled copy was all there was.

**Decided**: one function, doing what it says. `mj_copyBack` writes the
authored fields, so that saving and compiling both keep the edit. It goes on
writing the compiled fields as well, so that a compiled-values save, which
reads those, has the edit without compiling again.

Of the roughly hundred fields it touches, most are parameters compilation
never transforms (friction, solver parameters, margins, colours, damping,
armature, ranges, gears, gain and bias parameters, light and material
properties) and copy directly, with units converted back. Poses, fitted sizes,
inertia and keyframes are transformed by compilation: these go through the
inverse transformations the writer already has, clearing the competing inputs,
and inertia follows the rule of section 3. A value with no unique inverse is
refused. It should write only what differs from what the spec compiles to, so
that an untouched body keeps its Euler angles.

The compiled-values option above is today's writer, and keeps today's
requirement that the spec has been compiled. An authored save should not need
a compiled spec at all (to verify for keyframes waiting to be completed,
section 7). Both writer paths stay: copy-back writes only what was edited in
the model, so it does not turn an authored save into a compiled-values one.

## 6. Names derived from files

**Decided.** The XML parser derives the name when it reads a file-backed
asset without one. The name is then authored: visible before compiling, saved,
and stable if the file is changed later. The compiler no longer writes names.

An asset created through the API must be given a name if anything refers to
it by name. Unnamed assets remain legal where nothing does: a skybox texture,
a skin.

Compatibility for API users:

- code that sets only `file` and refers to the stem from a geom, material or
  height-field reference gets a compile error. The message names the unnamed
  asset with that file and says to name it;
- `mjs_findElement` finds an unnamed mesh or texture by its file stem today,
  deliberately, with a test. That lookup goes;
- the compiled model has no name for such an asset.

The migration in every case is passing `name=` when the asset is added.

## 7. Keyframes

A keyframe is flat vectors (`qpos`, `qvel`, `act`, `ctrl`, mocap poses) laid
out by the compiled model. When the tree changes, the layout changes.

Today, deleting a body or attaching a spec stashes every key value on the
joint, actuator or mocap body it belongs to, deletes the key elements, and
leaves a note of their names. The next compile creates new key elements and
reassembles the vectors under the new layout. Handles to keys die on the way.

Why it waits for the compile: reassembly needs a value for every entry the key
never covered. Example: a parent with one hinge and a key `home` with
`qpos="0.5"`; attach a child whose root has a free joint. `home` must now have
eight numbers, and the seven new ones are the child's reference pose, which is
where the free body sits after frames, orientation alternatives and free-joint
alignment have been applied, and alignment depends on inferred inertia. That
is compiled information (`m->qpos0`). Deleting a body needs none of it: every
remaining entry is already known.

Can the reference pose be resolved when attaching? Mostly, not always:

- hinge and slide joints: the authored `ref`. Ball joints: identity. Trivial.
- free joints and mocap bodies: the body's pose through its frames and its
  orientation alternative. Computable from the specs without compiling; the
  code for it exists.
- free joints under alignment (`alignfree`, or `align` on the joint; off by
  default): the body moves to its inertial frame, so the pose needs the body's
  inertia, and if that comes from geoms it needs their meshes processed. Not
  easy, and attaching is something people do many times while building a
  model.

There is also a reason not to do it early even where it is easy. An entry a
key does not specify means "the model's reference pose". If it is filled in
at attach time and the body is moved before compiling, the key is left with a
value that is no longer the reference pose.

So this is the exception to rule 1 (**decided**: an exception when the pose
is not easy to resolve). It can be made small: keep the key elements alive, so
handles survive, and let the next compile only complete their vectors.

## 8. Decisions

**Settled in review**

- Adoption is an error under `inertiafromgeom="true"`; `mjs_discardVisual`
  reports the conflict before deleting anything.
- A restructuring that succeeded stays applied when compilation later fails,
  and the error says so. Operations themselves are all or nothing.
- The names `mjs_fuseStatic` and `mjs_discardVisual`.
- Operations resolve from the current spec.
- `mjs_bodyToFrame` uses resolved inertia.
- A body attribute `fuse="false"` keeps a body, in place of arguments to the
  operation.
- The parser names file-backed assets; API-created assets referenced by name
  must be named.
- Keyframes are completed at compile after a change to the tree: the one
  exception to rule 1.
- `mj_copyBack` writes the authored fields, as documented, with the scope in
  section 5. Under `inertiafromgeom="true"` an edited body mass or inertia is
  an error, for the reason adoption is (section 3): the inferred value would
  win at the next compile. Every other field copies back as usual.
- Saving authored values by default, compiled values on request. The controls
  are two booleans beside `saveinertial`, named in its style: `savecompiled`
  writes compiled values instead of authored ones, and `savecanonical` writes
  quaternions, radians, and pose and size instead of the authored spelling.
  Rollout in three steps, so that no release changes output without a switch
  to get the old one back: add both attributes defaulting to `true`, which is
  today's output; implement the authored and as-written paths behind `false`;
  then change the defaults in one release, with a changelog line saying that
  setting both to `true` restores the previous files.
- The adoption interface: `mjs_adoptInertial(body)` calculates the inertia
  from the spec as it is now, as the operations do, and not from a model
  (section 3). The operations need the same routine and have no model to pass.

**Outside this work, noted so they are not lost**

- **Actuator notation.** Retain, rather than reconstruct. Retaining means the
  parser and `mjs_setTo*` record on the actuator which shortcut was applied,
  and the writer writes that tag back; it is exact. Reconstructing means the
  writer guesses the shortcut from the gain and bias parameters; it is a
  heuristic. Either way it is orthogonal to everything above and can follow
  on its own.
- **Rethinking `inertiafromgeom`.** `auto` is a per-body rule: an authored
  inertial is used, a missing one is inferred. `true` is a global override
  that makes authored inertials meaningless, which is what makes adoption and
  copy-back awkward. What it is used for, ignoring bad inertials from a
  conversion, is a one-off act and would be clearer as an operation that
  removes the authored inertials, after which `auto` infers everything. That
  would leave `auto` as the only rule and let `true` be deprecated.

## 9. To verify

- Whether published models combine `discardvisual` with
  `inertiafromgeom="true"`. None in this repository does.
- That `mj_copyBack` can tell which values differ from what the last
  compilation produced, field by field, for the fields it supports (settled
  as the rule, see the end of section 11; the fields, how each is written and
  which changes are refused are listed in the documentation of the function).
- Python handles to bodies removed by `fusestatic` (*read*: dangling today,
  as after `mjs_delete`).
- Which other compile-time members are read before being written. Stretch B
  found a family of them, now fixed: ids resolved from a reference were only
  written when the reference was set, so removing a material, texture, mesh
  or target body from a compiled spec left it in the next model. The sweep of
  stretch A compares authored content, not compiled state, so it does not look
  for these; the test for them compiles, edits, compiles again and compares
  with a newly written spec.
- An operation which compiles the kinematic tree leaves the spec not
  compiled, also when it then finds nothing to do. Saving compiled values
  requires a compiled spec, so such a call must be followed by a compile
  before them; saving the spec as written (stretch C) does not. One case is
  still refused: a spec whose keyframes wait for a compilation after a change
  to the tree, as in section 7.
- The XML writer does not save `inertiagrouprange`, so a model which uses it
  reloads with other masses (*measured*). Saved as written (stretch C), every
  compiler setting comes from the spec and it is kept; for compiled values
  the fork branch `writer-inertiagrouprange` fixes it.
- Where stale name lookups are fixed. Stretch A fixes them in
  `mjs_findElement`: the maps are a shortcut whose answer is checked, and
  failing that the element is searched for. The fork branch
  `find-element-after-edit` (a8c36bb81) fixes them one level down, in the
  lookup that the compiler itself uses, which also covers internal lookups on
  an edited spec. It costs load time (*measured*): lookups that are expected
  to miss during attachment and replication then search the whole list,
  196,000 searches and 210 million name comparisons over the corpus, and
  `100_humanoids` loads 13% slower. The two commits are alternatives and
  conflict; the operations of stretch B work with either.
- What keyframes should mean under `<replicate>`. Today each replica gets a
  suffixed copy of every keyframe and the authored one is emptied
  (*measured*), and a keyframe written for the whole model fails validation
  while the replicate is expanded (upstream issue 3071). Stretch A leaves
  this as it is.

## 10. Tests that define done

- with no restructuring option, the authored spec is identical before and
  after a compile, successful or failed, apart from binding. This needs a
  utility that compares two specs;
- compile, edit, compile equals a newly written spec with the edit, for frame
  poses, geom deletion, asset file changes and keyframe changes;
- for each operation: applying it and compiling equals compiling with the
  option; applying it twice equals applying it once; compiling again, copying
  and saving reproduce the model; lookups and held handles behave; a failing
  operation leaves the spec identical;
- fused and discarded models match the original in global poses and
  joint-space inertia, including mixed authored and inferred inertia, with and
  without meshes; the same for `mjs_bodyToFrame`;
- `mjs_discardVisual` under `inertiafromgeom="true"`: an error when a body
  would lose mass, success when it would not;
- adopt, edit geometry, recompile: adopted inertia stays, the rest follows the
  geometry;
- after a fuse, every element found by name on the spec is the element with
  that name, and binds to its own row; `fuse="false"` keeps a body;
- edit a model, copy back, recompile: the edit is in the spec and in the new
  model, and an element that was not edited keeps its authored spelling;
- attach and delete with keyframes: handles to keys survive, and entries a key
  does not specify equal the compiled reference pose;
- the write-read and recompile sweeps, under ASAN.

## 11. Skeleton of the commit chain

Three stretches, each ending where the work could stop and still be worth
having. The steps are units of review, each with its tests, documentation and
bindings; at import they squash to about one commit per changelog line. The
work starts from current main and takes the fuse policy and its tests from the
parked branch. Every step runs the full suite; each stopping point also runs
the write-read and recompile sweeps, in double and single precision and under
ASAN.

**A. Fixes that stand alone.** Independent of each other. Implemented on the
fork branch `spec-compile-fixes`; what the audit of step 6 found is listed
after the steps.

1. **A utility that compares two authored specs**, and the tests that already
   pass with it: the spec unchanged by a plain compile, and compile, edit,
   compile against a newly written spec. Each later fix adds its failing case.
   Saved XML cannot serve as the comparison: it holds compiled values today.
2. **The pair and exclude lists keep their order.** Sort a private order.
3. **Names come from the parser.** The reader names file-backed assets, the
   compiler stops writing names, and an API-created asset referenced by name
   must be named, with the error that says so. Tested through attachment and
   its prefixes. Changelog, with the migration.
4. **Keyframes keep their elements** across a change to the tree, completed at
   compile.
5. **Lookups after `fusestatic` and `discardvisual`.** Both fixes are written
   or in progress: the parked branch has the one for the wrong element after a
   fuse, and a separate fix covers the exception after a discard. They need
   not wait for stretch B, which later replaces the code they patch.
6. **Whatever else the utility finds**, one small fix each: keyframe padding
   (section 9), user data lengths, compile-time members read before they are
   written.

Stopping point: rule 1 holds, by test, apart from keyframes. The test is the
recompile sweep, which now compares every model of the corpus before and
after compiling it, and against a copy.

What the audit found, all fixed (*measured* on upstream main e385ec699):

- compiling wrote the pixels of file and builtin textures into
  `mjsTexture.data`. A buffer given by the user was flipped in place under
  `hflip` or `vflip`, so every other compilation gave the unflipped texture,
  and a buffer set after a compilation was discarded by the next, which then
  failed;
- `meshdir` and `texturedir` received a trailing separator, `mjsMesh.needsdf`
  was set for meshes used by SDF geoms, and the configuration of an SDF plugin
  instance received empty values for the attributes that were not set;
- all of these, and the reordering of pairs, also happened when the
  compilation failed;
- `mj_copySpec` and attachment silently dropped a tendon wrapping a cylinder
  geom, with the actuators attached to it, when the spec had not been compiled
  and the default class of the geom had another type;
- after `fusestatic`, sensors, tendons, actuators, pairs, equalities and
  tuples compiled after the fuse were bound to the wrong geom, site, camera or
  light when ids had shifted, and some static bodies were left unfused;
- a reference removed from a compiled spec (a material, texture, mesh, height
  field, target body or reference site) stayed in the models compiled
  afterwards, or made the next compilation fail. Found during stretch B, and
  added to this branch;
- after the tree of a compiled spec had changed once, a keyframe vector set
  for the changed tree was rejected for its size by the next change: the
  keyframes were read in the layout of the last compilation, whose addresses
  are kept for `mj_recompile`. Found in review of stretch B, and added to this
  branch as its last commit rather than folded into the keyframe commit, on
  which two other branches are based.

Two things are treated as recorded by compilation rather than authored:
`mjsWrap.type` of a wrapped geom, which the API documents as set during
compilation, and keyframes added up to `nkey`, which join the keyframe
exception of section 7. The order of keyframes in the model is unchanged:
pending keyframes are kept last in the spec, where compilation used to create
them.

**B. Restructuring as operations.** Implemented on the fork branch
`spec-compile-operations`, on top of A; what it found is listed after the
steps.

7. **Resolution as callable stages** (section 4). A refactor with no change in
   behaviour, checked by the sweeps, and the riskiest step.
8. **Inertia adoption.** One internal routine that writes a resolved inertial
   into the authored body, in authored coordinates, clearing the competing
   inputs, and refuses under `inertiafromgeom="true"`. The public operation of
   section 3 is a thin layer over it, and steps 9 to 11 use it.
9. **`mjs_bodyToFrame` uses resolved inertia**, for the mixed case.
10. **`mjs_fuseStatic`**, built on 8 and 9 with the policy and tests of the
    parked branch. `<compiler fusestatic>` calls it, and the old path is
    removed in the same step. The `fuse` body attribute follows as a small
    commit of its own.
11. **`mjs_discardVisual`**, with the error under `inertiafromgeom="true"`
    raised before anything is deleted. `<compiler discardvisual>` calls it,
    and the old path is removed.

All or nothing means that every error an operation can report is raised before
its first change: check, then change. There is no rollback. Running out of
memory is fatal everywhere in the compiler and stays so.

Stopping point: rules 2 and 3 hold. Saving is as it was. The recompile sweep
checks, for every model, that a second compilation leaves the spec as the
first left it, with or without the options.

How explicit operations reach assets (**decided**): `mjs_fuseStatic`,
`mjs_discardVisual` and `mjs_adoptInertial` take a VFS like `mj_compile`; in
Python they are methods of the spec and use its assets as `compile()` does.
`mjs_bodyToFrame` keeps its signature, and in the mixed case reports an error
that points to `mjs_adoptInertial` when a mesh cannot be read without a VFS.

What the old operations did, found by forcing each option on for the 351
models of the corpus and comparing with the model compiled without it
(*measured* on upstream main e385ec699); all fixed:

- `fusestatic`: 9 models failed to compile, 7 of them because a flex, or a
  skin read from a file, referred to a fused body (the other 2 for a reason
  which stretch A fixed). In 31, a camera or a light of a fused
  body was at another world pose: cameras and lights were compiled again
  after the fuse, which discarded the change of frame, and with `alignfree`
  the alignment of a body with a free joint as well. The count of bounding
  volumes was too large.
- `discardvisual`: 21 models failed to compile, because references to the
  deleted materials and textures were left in flexes and lights; the same
  holds for tendons, skins and meshes with a material, and a default class
  with a material gave a saved file that could not be loaded. A mesh used
  only by a site or named by a tuple was deleted, which gave an invalid
  model. In one model the passive forces changed: geoms which use the
  ellipsoid fluid model were discarded.

With the operations, all 351 compile with either option or both; fusing moves
no geom, site, camera or light, and discarding changes no inertia, kinematics
or passive force. Where the old code compiled and the result did not change
for one of the reasons above, the models agree: bit for bit for the discard,
to roundoff (1e-14) for the fuse. An option gives the same model, bit for bit,
as its operation followed by a compilation without the option.

Policy that changed with the operations, each because the old behaviour
changed the physics: a static body is kept when it has a plugin or a sleep
policy, when it has mass and its `gravcomp` differs from its parent's or the
model is in a fluid, or when a flex or a skin refers to it; a geom which uses
the ellipsoid fluid model is not discarded; and `discardvisual` under
`inertiafromgeom="true"` is an error when a discarded geom carries mass.
Texture coordinates are not discarded, which is what happened before in
effect: the code that meant to delete them ran before the meshes were loaded.

Three rules were sharpened in review. When both bodies of a fuse infer their
inertia, moving the geoms reproduces the sum only if compilation adjusted
neither (`boundmass`, `boundinertia`, `balanceinertia`) and both count the
same geom groups; otherwise the resolved sum is written to the parent, and
under `inertiafromgeom="true"`, where it cannot be, the body is kept. One rule
covers what a discard keeps: nothing that another element refers to by name is
discarded, which includes a material or texture named by a sensor or a custom
tuple (it stays as an element that nothing renders with). And an operation
does not create or complete keyframes: that remains the one thing a full
compilation writes into the spec.

**C. From model to spec, and saving.** Depends on B only through step 8, for
inertia in copy-back; the writer steps can proceed alongside B. Implemented on
the fork branch `spec-compile-saving`, on top of B; what was built and what it
found is listed after the steps.

12. **`mj_copyBack` writes the authored fields** as well as the compiled ones.
    By family: direct copies first, then poses, sizes, inertia and keyframes.
    A value with no unique inverse is refused.
13. **The two saving controls**, defaulting to `true`. Today's writer becomes
    the `true` path, `mj_saveLastXML` included.
14. **The authored writer**, behind `false`, by section: the body tree and
    defaults first, then assets, contacts, equalities, tendons, actuators,
    sensors, custom data, keyframes and compiler settings.
15. **The defaults change**, in one release, after step 12:
    `mj_saveLastXML(file, m)` relies on copy-back to keep edits made to the
    model.

To settle before its step rather than before starting:

- before step 8, how explicit operations reach assets held in a VFS: settled,
  see stretch B;
- before step 12, what copy-back compares against when the spec and the model
  have both been edited since the last compile. **Decided**: the values of
  the last compilation, which the elements hold. A field whose value in the
  model equals them is not touched, so an edit made to the spec since that
  compile stays unless the model changed the same field; a model with another
  structure is refused, as today. Copying every field instead would rewrite
  the spec in compiled form: quaternions for every orientation, an explicit
  inertial on every body, values in place of defaults;
- before step 12, a mass edited under `settotalmass`. **Decided**: the
  attribute is deprecated now and removed later, when the scaling mechanism
  supersedes it. It cannot simply disappear: no model or test in this
  repository uses it, but dm_control's `cheetah.xml` and Gymnasium's
  `half_cheetah.xml` do (`settotalmass="14"`). While it exists it is parsed
  and applied as before, with a warning, and copy-back refuses mass and
  inertia under it, as it does under `inertiafromgeom="true"`. Migration:
  call `mj_setTotalmass` after loading, or give the bodies their masses.

As built, in sixteen review units:

- **Copy-back** (step 12). Compilation now ends by giving every element the
  values which the model was given, so the comparison decided above is one of
  the model with the element. A check pass runs before anything is written:
  a change which the spec cannot express is an error which names the element,
  nothing is copied, and `mj_saveLastXML` saves nothing rather than a file
  without the change. Each changed value is written as what compiles to it:
  a pose in the frame of the element, with an orientation which was not
  changed left in its notation; angles in the unit of the spec; a size and
  pose in place of `fromto` or of a fit to a mesh; an explicit inertial in
  place of an inferred one; the `limited` attribute where the new range would
  be inferred otherwise; stiffness and damping in place of `springdamper`; a
  range in place of `inheritrange`. *Measured*: copying back an unedited model
  leaves all 351 specs of the corpus unchanged, and for every family of model
  fields, perturbing it, copying back and compiling again gives the perturbed
  values, wherever the perturbed model is valid. A copy of a compiled spec is
  refused until it is compiled: `mj_copySpec` builds its elements from what
  the spec gives, so they do not hold what the compilation gave the model, and
  every derived value would count as changed (*measured*: a `dampratio` became
  its damping, a weld gained its relative pose).
- **The controls** (step 13): `savecompiled` and `savecanonical`.
  `savecanonical` only applies when `savecompiled` is "false": compiled values
  have no notation to keep.
- **The authored writer** (step 14). One writer: each value is read from the
  spec struct of the element or from its compiled copy. Numbers are saved
  exactly, as the shortest text which reads back as the same number, and a
  value is left out only if it is its default. Attributes of `compiler`,
  `option` and `visual` which were written are saved also with their default
  values, from the bits which record what was written. In the canonical
  notation the conversions use the expressions of the compiler, so the file
  compiles to the same model. *Measured*: each of the 351 models, saved as
  written and saved canonically, compiles to the same model bit for bit and
  is saved again as the same file; the spec which is read back equals the one
  which was saved, up to bookkeeping (the name of the plugin beside an
  instance, the class of a frame without one, what is recorded as written);
  and a spec is saved the same before and after it is compiled, except where
  compilation completes keyframes or restructures. The same holds for 1019 of
  the 1021 models which the tests write inline and which compile; the two
  exceptions are a model whose positions are NaN and a quadratic flex which
  the compiled values do not reproduce either. Saving compiled values, as
  before, reproduces 1001 of them: it loses `inertiagrouprange` and the
  components of a size which the type of the geom does not use, and
  reproduces builtin meshes only to rounding.
- **The default** (step 15), last, so that it can be dropped or deferred
  without the rest. Tests which pin how compiled values are saved ask for
  them with `savecompiled="true"`.

What saving as written found, each fixed in its own unit unless noted:

- A `quat` was ignored on a geom, site or camera whose default class gives an
  orientation as an alternative: the reader kept the alternative in force.
- The writer saved the elements which are directly in a body before those in
  its frames, and a nested frame after the first body of the frame around it,
  which changed the ids of geoms, sites, cameras, lights, joints and bodies,
  and with them what the saved keyframes refer to (`model/welcome`).
- An element with `class="main"`, or a body with `childclass="main"`, inside
  another childclass was saved without it and took on the class around it.
- The `nchannel` of a texture was never saved.
- The energy sensors were saved as `potential` and `kinetic`, which the
  reader rejects (found with the inline models of the tests).
- `inheritrange` had no attribute on `<general>`, which is how every actuator
  is saved, so an inherited range could only be saved as its value. The
  attribute is added. No model of the corpus inherits a range; this was found
  by checking that every field of the spec structs, and every attribute of
  the schema, is written somewhere.
- URDF gives a full inertia in a rotated frame, which MJCF cannot say: it is
  saved rotated into the frame of the body (in the writer unit).
- The elements of a default class had an uninitialized pointer to compiler
  settings, and the plugin instance which a composite makes did not name its
  plugin (both in the writer unit).
- Left as it is: the compiler converts the range of a hinge or ball joint
  from degrees only if the range limits, so `jnt_range` of an unlimited joint
  in a model which uses degrees holds degrees. Copy-back and the canonical
  notation follow the compiler.

Left open:

- Actuator shortcuts are saved as `<general>` (section 5, notation).
- A mesh or texture of an attached spec which has another `meshdir` is saved
  with its file as it was written, so the saved file does not find it. This
  is the same for compiled values, and older than this work.
- `MjSpec.to_xml()` in Python compiles the spec before it saves it, which an
  authored save does not need.
- `mj_setConst` recomputes `mjModel.stat`, so copying back after it writes
  explicit statistics.
- A flexcomp with `dof="quadratic"` is saved, in either way, as a flex which
  compiles to another stiffness. Older than this work; noted as a separate
  task.
- The compiler settings of an attached spec other than its angle unit and
  Euler sequence (`autolimits`, `inertiafromgeom`, `inertiagrouprange`,
  `boundmass`, ...) have no place in one flat file: its elements are saved
  under the settings of the spec they are attached to. The same holds for
  compiled values.
- In a single-precision build, a value which is copied back from the model
  is the float as a double, and is saved with all its digits
  (`0.10000000149011612` for a size set to 0.1). Copy-back could write the
  shortest number which converts to the float instead.

**The combined stack.** The three stretches and the fork branches which touch
the same code are one linear series on upstream main 07dfe9165, the fork branch
`spec-compile-stack` (63 commits). The stretches stay on their own branches as
they were reviewed. Its order, chosen so that the branches meet with the
fewest conflicts:

1. Stretch A without its last commit (9 commits).
2. The keyframe and attachment branches (13): `replicate-keyframes`,
   `attach-destination-keyframes`, `keyframes-between-changes`,
   `attach-reference-addresses`, `reattach-by-reference`,
   `self-attach-by-reference` and `copy-compiled-spec`.
   `keyframes-between-changes` contains the last commit of stretch A, the same
   flag, guard and test, which is why that commit is left out.
3. One fix which the combination found (1): a copy of a compiled spec held
   the geoms of a contact pair and the bodies of an exclude in the order of
   the bodies rather than as they were written, and took a tendon which wraps
   a cylinder again from its spec. Saving the copy as written showed the
   first; the copy-back sweep showed the second.
4. Stretch B (6) and stretch C without the change of default (15). With
   `copy-compiled-spec` underneath, a copy of a compiled spec holds what was
   compiled, so `mj_copyBack` accepts it; it refuses a copy in which an
   element had to be taken again from its spec, for example an equality whose
   body was renamed since the compilation.
5. Fixes which stand alone on upstream main (18): `save-after-structural-edit`
   (the refusal applies where compiled values are saved; a spec which is saved
   as written needs no compilation), `xml-vector-precision`,
   `flex-empty-cells`, `delete-plugin-ids`, `plugin-pointer-refs`,
   `plugin-instance-lifetime` (whose one rule `mjs_discardVisual` now follows
   too, in place of its own deletion of plugin instances),
   `attach-skin-namespace`, `delete-after-compile-references`,
   `writer-inertiagrouprange`, `writer-inertiafromgeom`,
   `write-read-exclusions`, `schema-dims` (`inheritrange` on `general` is
   dimensionless), `control-semantics`, `intvelocity-errors` and
   `texture-nchannel-overflow`.
6. The change of default (1), last, so that it can be left out. Its tests
   which pin how compiled values are saved include those of the branches
   above.

Not in the series: `find-element-after-edit`, the alternative to the lookup
commit of stretch A (section 9), and `discardvisual-find-element`, which
stretch A already has.

Where the branches met in more than the changelog:

- `StoreKeyframes`, between three branches: the addresses of elements which
  were moved by reference are set aside first, then a compiled model whose
  tree has changed is laid out by its lists. A copy of a compiled model has
  its addresses, so the test for a world body without one is gone.
- Copying the elements outside the tree: the rule of `copy-compiled-spec`
  (keep an element as compiled if its references resolve) is applied where
  `reattach-by-reference` resolves on a trial copy.
- The first stage of compilation no longer completes keyframes; they are
  completed from the compiled model.
- The `input` of an actuator in the writer, with the values which a save as
  written takes from the spec.
- `mjs_setToIntVelocity`: the error of the position setter is returned, the
  control is declared a velocity, and `inheritrange` is recorded.

Found in review of the series, and fixed in the commits of stretch C which
they belong to:

- A spec saved as written lost a keyframe vector which was what the model had
  without it, as the last compilation knew it: once the spec gave the model
  another reference configuration, the saved file gave the keyframe that one.
  Every vector which a keyframe has is saved now; no model of the corpus was
  saved differently for it.
- `mj_copyBack` accepted three changes which the next compilation does not
  keep, and now reports them: values which differ between the degrees of
  freedom of a ball or free joint, to which the spec gives one; elevation data
  whose lowest and highest values are not 0 and 1; and the anchor which a
  connect between bodies has in its second body, unless it is the one which
  `mj_setConst` computes from the model. A weld keeps the relative pose which
  compilation computed, unless that pose or its anchor was changed.

Found in the review of the rebased review series, and fixed in its commits:

- `mj_copyBack` wrote the position of a pose of which only the orientation
  was changed in the model, losing a position written in the spec since. The
  position and the orientation are now written each if it changed, except
  where compilation offsets the position by the orientation: a mesh which is
  not centered, a body aligned with its free joint. Other attributes are
  written whole, which its documentation now says.
- Saved as written with `saveinertial`, a body which infers its inertial got
  the mass which `settotalmass` had scaled, beside the authored masses and
  `settotalmass` itself, which scaled it again on loading. It is now saved
  before the scaling, as an authored inertial is.
- Saved as written, the elements of an attached model took the compiler
  settings of the model it is attached to. Where these differ in a setting
  which changes what compilation infers (`inertiafromgeom`,
  `inertiagrouprange`, `boundmass`, `boundinertia`, `balanceinertia`,
  `alignfree`, and `autolimits` one way), saving now fails with an error
  which names it. With `savecompiled` the example which showed this saved a
  file which did not load: the writer of compiled values decided whether to
  write an inertial by the `inertiafromgeom` of the model, not of the body.
  It now writes the compiled inertial of a body wherever the saved file would
  not give it alike, which for a single compiler is where it wrote it before.
  It decides by how the last compilation gave each body its inertia, which
  the body keeps, since the spec may have been edited since.
  Not addressed: the asset files of an attached model are saved relative to
  its own `meshdir` and `texturedir`, by both writers.
- The documentation said in places what saving did by the old default, and
  that saving as written never needs a compilation; `mj_compile` now says
  what compilation changes in a spec, and `MjSpec.to_xml` that it compiles.

**The review series** is the series to import: the same code as one commit
per unit of review, the fork branch `spec-compile-review`, 19 commits on
upstream main 73b63c92a, after the 3.15.0 release, which follow a first commit
that adds this document; its changelog is a new upcoming version. Commits of
the stack which rework the same code are one commit there, and a preparation,
or a fix which followed, is in the commit of its feature; each commit message
describes the unit as it ends up. Each of the 19 builds and passes the test
suite alone. The stack stays as the record of how the series was made, and
the numbers below are positions in it.

The review series was made on the base of the stack, 07dfe9165, where the two
ended in the same tree but for the changelog, and then rebased on 0f8bb88e9
and again on 73b63c92a. Three of its fixes had landed in 0f8bb88e9:
`writer-inertiagrouprange`,
`intvelocity-errors` and `texture-nchannel-overflow` (stack positions 54 and
60-62), so the series no longer has them; the control of `intvelocity` is
declared on top of the landed fix. Changes since then are made in the review
series only.

The stack has a changelog entry for every fix, about 285 lines. The review
series has about 150: its entries are in a section of their own, "Compiler",
with the breaking changes and the new functions and attributes first, and the
fixes in one list at the end of the section, a sentence or two for each group
of related fixes. What each fix was is in the commit messages.

| # | Commit of `spec-compile-review` | Stack |
|---|---|---|
| 1 | `CompareSpec`, the test utility | 1 |
| 2 | Order of contact pairs and excludes; assets named by the XML parser | 2-3 |
| 3 | Compilation leaves the spec as written; lookups by name | 5-9 |
| 4 | Keyframes through changes to the kinematic tree | 4, 10-17 |
| 5 | Attaching by reference | 18-21 |
| 6 | A copy of a compiled spec | 22-23 |
| 7 | `mjs_adoptInertial`; inferred inertia in `mjs_bodyToFrame` | 24-26 |
| 8 | `mjs_fuseStatic` and the body attribute `fuse` | 27-28 |
| 9 | `mjs_discardVisual` | 29 |
| 10 | `settotalmass` deprecated | 30 |
| 11 | `mj_copyBack`, with `MjSpec.copy_back` and the rule for copies | 31-33, 40, 44 |
| 12 | Reader and writer fixes which saving as written found | 35-37 |
| 13 | Saving as written: `savecompiled`, `savecanonical`, `inheritrange` on `general` | 34, 38-39, 41-43 |
| 14 | Saving compiled values: the refusal after a structural edit, five values lost in saving | 45-47, 55-57, review |
| 15 | Plugin instances when elements are deleted | 48-50 |
| 16 | Dimensions of MJCF attributes | 58 |
| 17 | What the control of an actuator is | 59 |
| 18 | Three small fixes: attach namespaces, deletion in a compiled spec | 51-53 |
| 19 | The change of default, last, so that it can be left out | 63 |

Units 3 and 4 are in the other order than in the stack, and units 16 and 17
come before 18. Unit 3 patches the fuse and the discard inside compilation,
which units 8 and 9 then replace: those two fixes matter only if unit 3 lands
without them.

## Appendix: why the spec changes today

Every element holds an authored `spec` beside a compiled working copy, and
every `Compile()` starts by regenerating the copy with `CopyFromSpec()`. The
design is sound; it leaks in four places.

- **Structure is shared.** There is one tree. `mjCModel::FuseStatic` and
  `IndexAssets(discardvisual)` run inside `TryCompile` and edit it while it is
  half compiled, patching compiled poses without updating the authored ones;
  hence the second-compile results in section 2. `stable_sort` reorders
  `pairs_` and `excludes_`.
- **Names have no authored copy.** `spec.name` points at the element's single
  name string, and `mjCMesh::CopyFromSpec` writes it. `mjs_findElement` uses
  the name-to-id maps once a spec is compiled, and `discardvisual` leaves them
  stale, hence the exception.
- **Compile-time members persist.** `mjCFrame::compiled` and the body BVH were
  the two found.
- **The inertia policy is switched on the wrong copy.** `discardvisual` writes
  the inertial into the authored body but switches `inertiafromgeom` on the
  compiler's working copy, which the writer reads and the bodies do not.

The writer reads the working copy too: `mjXWriter::Write` requires a compiled
model, `SetModel` routes `mj_copyBack` into working-copy fields, and
`Compiler()` writes `angle="radian"` and omits the settings it has already
applied. That is why saving writes compiled values today.
