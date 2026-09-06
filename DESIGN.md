# `mujoco.batch`: batched CPU simulation in the Python bindings

Status: proposal for discussion. Nothing here is implemented in this repository yet.
Kevin Zakka has a related implementation, currently private, which this design follows
and whose measurements it quotes; "the reference implementation" below refers to it.

## Summary

Add a `mujoco.batch` module to the Python bindings: N simulations of one model, stepped
on a C++ thread pool with the GIL released, whose entire user-facing API is `(N, ...)`
NumPy arrays over `mjData` and `mjModel` fields. Each environment is its
`mjSTATE_INTEGRATION` vector; `mjData` exists only per worker thread. The module is a
strict superset of `mujoco.rollout`, which becomes a thin wrapper over it, so the
bindings carry one thread pool, one error trap, and one batching primitive instead of
two.

## Why

MuJoCo has two batched CPU paths today, and neither is the one people need.

`mujoco.rollout` is a function on trajectories: initial states and open-loop control
sequences in, state and sensor trajectories out. It is the right shape for predictive
sampling and system identification and the wrong shape for anything closed-loop. A
policy in the loop needs the physics to persist between calls and needs derived
quantities out, and rollout offers neither: the per-thread `MjData` is scratch, so every
call re-injects `mjSTATE_FULLPHYSICS` and drops the solver warmstart, and the only
outputs are the state vector and `sensordata`. Reinforcement learning on CPU, which is
what mjlab and every vectorized-environment library want, is closed-loop.

The reference implementation fills that gap well as a standalone package, and its design
is the one proposed here. But a package outside the bindings pays for it:
`mjModel`/`mjData` layouts are its ABI, so it pins the exact MuJoCo version, needs a
release per MuJoCo release, cannot be used against a development build of the engine,
and duplicates machinery the bindings already have (the X-macro field enumeration, the
grouped named views, `mju_error` trapping, a thread pool). Inside the bindings all of
that cost is zero.

The third path, batching in the accelerated backends (MJX, MuJoCo Warp), covers the GPU.
A CPU batch that is exact, deterministic, and runs anywhere the bindings run is the
missing middle, for the same reasons rollout was added in the first place.

## What an environment is

An environment is an `mjSTATE_INTEGRATION` vector plus its `mjWarningStat` counters.
That spec exists to define exact continuation: it is everything `mj_step` reads that it
did not itself compute in the same call. Per call, a worker loads an environment's
vector into its own `mjData`, applies the caller's pending writes, runs the physics,
copies the requested outputs out, and saves the vector back. The consequences:

- Memory scales with threads, not environments. Measured `mjData` footprints (buffer
  plus touched arena; the 13 to 100 MB arena reservation is virtual): cart-pole 6 KB,
  humanoid 81 KB, Unitree Go1 467 KB, Unitree G1 910 KB. The integration vectors: 0.2,
  1.8, 1.3 and 2.8 KB. The reference implementation's measurement on switching: 4096 Go1
  environments from 2.5 GB resident to 53 MB.
- Each worker's `mjData` stays hot in cache whichever environment it steps. On a 24-core
  Threadripper at 48 threads, one substep per call went from 114k to 383k
  environment-substeps per second with the switch; on a laptop at 256 humanoids it is a
  wash, because the per-environment data already fit in cache.
- Results are bit-identical to a serial `mj_step` loop and independent of the thread
  count. The reference implementation tests this in lockstep against plain `MjData`
  across 50 calls with writes to every input kind, keyframe resets and subsets; a
  chaotic double pendulum stays at zero deviation over 5000 steps.
- What is not in `mjtState` cannot ride along. Sleeping is the important case: the sleep
  bookkeeping (`tree_asleep` and its countdown, the awake index arrays, the last-awake
  values that sleeping sensors keep reporting) is a history of one particular `mjData`,
  and waking is detected by bytewise comparison of `qpos`, `qvel` and the applied forces
  against what that data last held, so a scratch `mjData` that last held another
  environment would wake everything on every call. With per-worker data, models with
  sleep enabled are rejected; the persistent option below keeps sleeping possible, and
  whether it ships in the first version is open. Plugins that keep instance data outside
  `plugin_state` see one instance per worker, exactly as with rollout today.

Statelessness is preserved as a usage pattern, not lost as a property. The object owns
scratch and thread-local model copies; the per-environment vectors are data the caller
can read, write, or replace wholesale before any call. Writing them every call is
rollout; never writing them is a vectorized environment. Nothing survives a call except
the vectors the caller can see.

## Performance model

What to expect, from the reference implementation's measurements on a 24-core
Threadripper 7960X with fresh control noise every call (settled physics flatters every
number): throughput is near-linear in threads up to the physical core count once each
call carries enough work per thread, less the all-core clock drop, 11% on that part, so
24 cores cap at 21.4× and reach 20.7×; a second thread per core then adds 1.3 to 1.5×,
which is why the default is every logical CPU. The cost per environment-substep is the
model's: mesh collision is 55% of a G1 step, and the batch's own state round trip is
0.15 µs. The cost per call is a fixed cost per thread, roughly 20 to 40 µs of wake
latency plus up to one item of end-of-call imbalance, shared by the environments the
thread holds; a call should therefore carry at least about a millisecond of work per
thread, below which the fixed costs take 30 to 50%.

## API

```python
import mujoco
from mujoco.batch import Batch

batch = Batch(model, num_envs=4096)         # threads default to every logical CPU
qpos, ctrl = batch.bind("qpos"), batch.bind("ctrl")
touch = batch.sensor("touch")               # a view into bound sensordata
mass = batch.expand("body_mass")            # (4096, nbody), per environment
mass[:, 1] *= rng.uniform(0.8, 1.2, 4096)
batch.set_const()                           # derived constants follow, per environment
for _ in range(1000):
  ctrl[:] = policy(qpos)                    # rows you change are the rows that land
  batch.step()                              # every bound field is refreshed
```

### Construction

`Batch(model, num_envs, num_threads=0)`. The model is copied; edit it before. The size
signature is fixed for the object's lifetime; `num_envs` is fixed too, and construction
costs `num_threads` `MjData` plus `num_envs` small vectors, so a caller whose batch size
varies (sampling MPC) constructs per size or over-allocates and uses `env_ids`.
`num_threads=0` means every logical CPU, not every physical core, against the usual
instinct for a compute-bound library: MuJoCo's small dense linear algebra stalls on
memory latency, and a second thread per core measured 1.3 to 1.5× more physics on the
7960X for every model and batch size tried.

`persistent=True`, undecided for the first version, keeps one `mjData` per environment
instead of per worker, the reference implementation's original design. It is the only
way to support sleeping, since the sleep state is the history of a particular `mjData`,
and the element-wise write-back then does exactly what the bytewise wake check wants:
untouched elements are never rewritten, so nothing wakes spuriously. It also makes a
field bound between calls current immediately and gives plugins one instance per
environment. Against it: `num_envs` `mjData` of memory, the cold-cache step measured
above, 2.3× lower throughput at one substep per call on 24 threads (equal at ten), and a
second storage mode in the test matrix. See open question 5.

### Fields

`bind(name, dtype=None)` returns a persistent `(num_envs, ...)` array over any `mjData`
array field, in `MjData`'s layout (`xmat` is `(N, nbody, 9)`), refreshed on the worker
right after each call touches the environment. Input fields, the `mjtState` components
and `warning`, are written back before the call, element by element where they differ
from what the batch last wrote, so only what the caller changed lands and untouched
environments follow a plain `mj_step` loop bit for bit. `dtype=np.float32` on an
`mjtNum` field converts in C both ways; physics runs in `mjtNum`. A derived field bound
between calls is zero until the next call fills it.

Precision follows the library. The bindings build against a single-precision MuJoCo
(`mjtNum` is `float`, `MJTNUM_DTYPE` follows, and rollout already sizes its arrays by
it), and every per-environment buffer here is `mjtNum`, so on such a build the float32
presentation is the native layout and the conversion path is a plain copy; the reference
implementation already special-cases exactly this. The single-precision CI job runs the
batch tests like any other bindings test.

`state` is the bound `(num_envs, nstate)` integration vector itself: read it to
checkpoint, write it to restart, hand it to another batch. It is what `rollout`'s
`initial_state` argument writes and what its final state returns, warmstart included.

The grouped named views the bindings already generate for `MjData` (`data.body(name)`,
`joint`, `site`, `sensor`, `actuator`, ...) are generated for the batch from the same
X-macros, returning `(num_envs, ...)` slices of bound fields.

### Per-environment model fields

`expand(name, dtype=None)` returns a `(num_envs, ...)` array over an `mjModel` field,
seeded from the model. Before an environment's call the worker copies that environment's
rows into its own model copy, so environments differ in exactly the fields expanded.
Assets (mesh, heightfield, texture, BVH data) cannot be expanded, and the per-thread
copies must not carry them either: a Menagerie G1's model buffer is 76 MB, of which 81
KB is not asset data, so `mj_copyModel` per thread costs 3.8 GB at 48 threads where
per-thread structs that own only the non-asset arrays and point at the template's asset
buffers cost 3.8 MB. The reference implementation makes full copies today; the bindings
version makes the shallow ones. `set_const(env_ids=None)` runs `mj_setConst` per
environment and expands every field it changed, so derived constants are per environment
without a maintained list of which ones those are. The `mjOption` members are expandable
by name (`expand("opt.gravity")`), applied to the worker's model the same way.

### Calls

All calls release the GIL, are serialized, and accept `env_ids` (sorted unique ints or a
boolean mask) to restrict them to a subset.

```python
batch.step(nstep=1, *, env_ids=None,
           control=None, control_spec=mujoco.mjtState.mjSTATE_CTRL,
           record=(), out=None, fill_forward=False)
batch.forward(env_ids=None)
batch.reset(env_ids=None, keyframe=-1)      # mj_resetData[Keyframe], pending writes, mj_forward
batch.set_const(env_ids=None)
```

`step` is the one place where the two use cases meet, and the place where the reference
implementation today falls short of rollout: its `nstep > 1` holds inputs constant
across the window and copies outputs once at the end. The proposal makes multi-stepping
rollout-shaped:

- `control` is `(num_envs, nstep, ncontrol)` under `control_spec`, applied before each
  substep with `mj_setState`, as rollout does; absent, bound inputs hold.
- `record` names outputs to capture after every substep into `(num_envs, nstep, ...)`
  arrays: any `mjData` field, or an `mjtState` spec for a state vector. As with
  rollout's `state=` and `sensordata=`, the arrays are allocated per call when `out` is
  absent and written in place when the caller supplies them. Bound fields are still
  refreshed with the final substep.
- Simulation warnings are handled the way the `onwarn` branch has the engine handle
  them: `mj_step` returns an `mjtStatus` bitmask, mirrored in `mjData.status`, and
  `<option onwarn="auto|continue|stop"/>` selects the response (automatic recovery,
  record and continue, or stop the call at the first warning with the state left for
  inspection). The batch keeps the per-environment status as a bound scalar like `time`,
  and `record` can capture it per substep; under `stop` an environment whose substep
  returned nonzero skips its remaining substeps in the call, so the option's promise
  holds per environment. `fill_forward=True` adds rollout's rule on top, repeating the
  last recorded row after a stop. Rollout's own scan of the `warning` counters after
  every step is subsumed by the return value.

The per-call cost this removes is the per-thread fixed cost above, and it is measurable
whenever calls are short: at 256 humanoids on eight threads, 32 environments per thread,
four single-substep calls cost 19.2 µs per environment-step against 15.8 for one
four-substep call, about 1 µs per environment; at 2048 environments the same fixed cost
is a tenth of that per environment. For a humanoid step it is five percent; for a
cart-pole MPC, where 32 one-microsecond steps are less work than the wake latency, it is
a factor of two.

A model error on a worker raises `RuntimeError` naming the first failing environment;
the other environments ran, the failing one keeps its pre-call state with its writes
still pending, and the worker's `mjData` is reset. With the `noexit` proposal, errors
stop being a trapping problem for the bindings at all: in never-exit mode `mj_step`
returns `mjSTATUS_ERROR` with the worker's data poisoned until reset, which the batch
reports per environment exactly like a warning status.

### Rollout on top

`mujoco.rollout.rollout(model, data, initial_state, control, ...)` keeps its signature
and becomes:

```python
batch = _cached(model, nbatch, nthread)
batch.state[:] = expand_to_integration(initial_state, initial_warmstart)
outs = batch.step(nstep, control=control, control_spec=control_spec,
                  record=(mjSTATE_FULLPHYSICS, "sensordata"),
                  out=dict(state=state, sensordata=sensordata), fill_forward=True)
```

The existing `rollout_test.py` is the acceptance test for the wrapper. What a caller
gains for free: the final integration state (warmstart included) is available for exact
continuation, and `record` can capture any field along the trajectory, not only the
state and sensors. The list-of-models form of rollout is covered by expanded fields for
parameter differences; models that differ structurally (different meshes) need the
template-list extension below and remain on the old code path until then.

## Implementation

- `python/mujoco/batch.cc`, `batch.py`, next to `rollout.cc`, wired with one
  `mujoco_pybind11_module(_batch ...)` line and a `setup.py` entry. The reference
  implementation is nanobind; the port to pybind11 is mechanical (`nb::ndarray` views
  with a base object become `py::array` with `base`).
- Field tables from `MJDATA_POINTERS` and `MJMODEL_POINTERS`, as `structs.cc` already
  does; named views from `indexer_xmacro.h`.
- Per-thread models as shallow copies: an `mjModel` struct whose non-asset arrays live
  in a private buffer and whose asset pointers alias the template's, freed by the batch
  rather than `mj_deleteModel`. The asset set is the X-macro table's prefix
  classification, the same one that refuses to expand them.
- Thread pool: extend `threadpool.h` with contiguous per-worker slices and stealing when
  a worker finishes early, a default of every logical CPU, and no core affinity (a
  pinned worker that sleeps waits for its own core to wake; pinning measured 15%
  slower), then move rollout onto it. With `mjData` per worker there is no cross-call
  locality to preserve: the worker's cache holds its own `mjData`, an environment's
  state is a few kilobytes, and subset calls would break any stickiness anyway.
- Errors through `InterceptMjErrors`, replacing the reference implementation's
  process-global log handler and `setjmp`; once `noexit` lands, through the library's
  own boundaries and `mjSTATUS_ERROR`, with no trapping code in the bindings.
- Tests: the reference implementation's suite (lockstep bit-exactness across thread
  counts, float32 rows, `set_const` completeness, per-environment gravcomp, keyframe
  reset with expanded `qpos0`, subset validation, warning counters, error recovery,
  memory independent of `num_envs`) plus `rollout_test.py` unchanged.
- Docs: a section in `python.rst` beside rollout; the rollout section points at it.
- Free-threaded Python: declared `mod_gil_not_used`, as `_rollout` is.

The C library is untouched. Batching is a host-language concern of threads, arrays and
lifetimes, and `mj_step`, `mj_setState`, `mj_getState`, `mj_setConst`, `mj_copyModel`
were sufficient primitives for the reference implementation.

## Extensions, not in the first version

- Template lists: `Batch([model_a, model_b], num_envs, model_ids=...)` for structurally
  different models of one size signature, at `num_threads` copies per template. This is
  what mjlab's mesh variants need and what rollout's list of models provides today at N
  copies.
- Recording derived fields at reduced rate (every k substeps) for long horizons.

## What it changes for users

mjlab's classic backend becomes an adapter of a few hundred lines over `mujoco.batch`
with no new dependency; measured with a Python bridge over per-environment `MjData` it
already runs cartpole training 2.3× and the G1 velocity task 11× faster than Warp on
CPU, and the batch engine roughly doubles that bridge at eight threads. Sampling MPC,
sysid and parameter sweeps keep rollout's API and gain continuation and arbitrary
recorded fields. Engine development gets batched CPU simulation against any build, since
the module is built with the library.

## Alternatives

Upgrading rollout in place would mean giving it persistent per-roll state, outputs
beyond the state spec, sparse input writes and per-environment model fields, which is
this proposal with rollout's signature kept, so the two amount to the same code with a
different front door; the front door proposed here is the stateful object because the
stateless call is expressible on it and not the other way round.

Keeping the implementation external keeps the exact-version pin, the release lockstep,
the duplicate machinery, and the inability to run against a development engine.

Batching in the C library is rejected: the engine stays single-simulation.

## Open questions

1. Name: `mujoco.batch.Batch`, or `mujoco.Batch` at top level like `MjData`?
2. Should `rollout` be deprecated once it is a wrapper, or kept indefinitely as the
   stateless entry point? The wrapper costs nothing to keep.
3. Template lists in the first version, given mjlab's variants need them?
4. Fill-forward as rollout's default only, or also available on `step`?
5. Does `persistent=True` ship in the first version? It is the only route to sleeping
   batches, and it costs memory, throughput on short calls, and a second storage mode
   under test.
