# Never exit: errors recovered at the boundary of the pipeline call

*Design, 2026-09-09. Branch `mjok`, third commit. Supersedes the 2026-08 proposal on branch
`noexit`, which was written for the return-value design and generated wrappers; see
`DIAGNOSTICS_SURFACE_OPTIONS.md` (branch `diagnostics-design`) for how the field design was
chosen.*

## Summary

`mjERROR` inside a step used to be fatal by contract: the default log handler called `exit()`,
and a custom handler had to `longjmp` away or the behavior was undefined. A library that can
terminate the process from a physics step is hostile to every host that runs many
simulations in one process. The `onwarn` work made the *warnings* a per-call status in
`mjData.status`; this change makes the *errors* the negative range of the same status:

- An error raised inside a pipeline call is caught at the boundary of that call. The
  pipeline functions are the public functions of the *Main simulation*, *Components*, *Sub
  components* and *Derivatives* sections of `mujoco.h` that take a non-const `mjData` (53
  functions, from `mj_step` down to `mj_solveM` and `mjd_transitionFD`); a scanner test
  derives the set from the header and checks each definition. The message is delivered to the log
  handler exactly as before; then the `mjData` stack and arena are restored to their state at
  entry, `d->status` is set to a negative `mjtStatus` (`mjSTATUS_ERROR`, or a more specific
  kind recorded at the raise site: `mjSTATUS_OOM` for stack overflow), and the call returns.
- A data with a pending error refuses every pipeline call, raising an error that says so,
  until `mj_resetData` clears the status. Nothing computed by the abandoned call is trusted.
- Errors raised from callbacks (`mjcb_control` etc.) and from thread-pool workers are caught
  the same way.
- Outside a pipeline call — loaders, creators, utilities without an `mjData` — nothing
  changes: the default handler exits. A custom handler that transfers control keeps
  precedence everywhere (the bindings, the model compiler, the test fixture).

There is no mode flag. Recovery is what the default handler does *instead of* `exit()` when a
boundary is armed, and what a returning custom handler gets inside a pipeline call instead of
undefined behavior; outside a pipeline call a returning handler behaves as before (`mju_error`
returns, which the contract calls undefined).

## Mechanism

**The boundary is the entry macro.** `mjENTER(d)`, already present at the top of the
functions that raise warnings for the status semantics and now at the top of every pipeline
function, arms an error boundary: a `jmp_buf`, the snapshot `{pstack, pbase, parena,
threadlock}` of the data, and a link to the enclosing boundary, in a thread-local chain
(`engine_util_errmem.c`). `mjLEAVE(d)` pops it on every return path. The pair is balanced like
`mj_markStack`/`mj_freeStack`; the scanner test checks the pairing textually (an `mjLEAVE`
before every `return` and at the end), and the test that walks the warning-raising functions
under every policy asserts the chain is empty afterwards. The jump is `_setjmp`/`_longjmp`
(plain `setjmp` saves the signal mask with a system call on macOS: 200 ns against 2 ns), so
a boundary costs a few nanoseconds and the ~40 entries of a step are not measurable.

```c
#define mjENTER_(d, retval)                                        \
  mjBoundary mjenter_boundary_;                                    \
  const int mjenter_outermost_ = mji_enter(d, &mjenter_boundary_); \
  if (setjmp(mjenter_boundary_.env)) {                             \
    mji_recover(d, &mjenter_boundary_, mjenter_outermost_);        \
    return retval;                                                 \
  }                                                                \
  if (mji_refused(d, &mjenter_boundary_, mjenter_outermost_)) {    \
    return retval;                                                 \
  }
```

`mji_enter` pushes the boundary; only the outermost call on `d` clears the status and sets
`d->nested`. `mji_recover` restores the snapshot, records `mjSTATUS_ERROR` unless the raise
site recorded a kind, and exits the call. `mji_refused` handles a pending error: the
outermost call raises "mjData has a pending error, call mj_resetData" (caught at its own,
already-armed boundary), nested calls exit silently while the enclosing call unwinds.

**One hop.** `mju_error` records the message in a thread-local buffer, delivers it to the
active handler, and — if the handler returns — jumps to the innermost boundary on the thread;
when there is none, the default handler exits as before and a custom handler returns to the
raise site as before. The jump lands in the innermost pipeline
function on the stack, which recovers and returns to *its* caller; the callers unwind through
the ordinary `mji_stop` checks, which now also fire on a negative status, and the entries of
any later stages refuse. No intermediate frame is skipped by `longjmp` except engine frames
between the raise site and the innermost boundary, which hold no resources other than the
restored stack and arena.

**Which entries arm.** A user callback is a foreign frame (C++ destructors, Python) between
two engine frames, and a pipeline function called *from* a callback must catch its own errors
rather than let them unwind across that frame to the enclosing call's boundary — also so
that the error is attributed to the data the callback was operating on: a sensor callback
failing inside `mj_sensorPos(m, scratch)` called from a control callback must poison
`scratch`, not the simulation whose step invoked the callback. Arming at every entry would do
it, but costs a `setjmp` and a snapshot at each of the ~45 entries of a step: measured at
0.25 → 0.38 µs on a one-dof model. So the engine raises a thread-local *callback flag* around
every callback and plugin invocation (`mjCALLBACK`: the control, passive, sensor, actuator
and contact-filter callbacks, the timer callback, the actuator, passive, sensor and SDF plugin
hooks), and an entry arms a boundary only when it is the outermost call on its data or the
flag is set — a pipeline stage entered from engine code is covered by the enclosing boundary
and costs three loads. With this rule the one-dof model steps in 0.26 µs against 0.22 before
the change, and the humanoid in 14.2 µs against 14.1. A boundary clears the flag when pushed and restores it when popped, so
every entry made from inside a callback arms, and the stages that follow the callback do not.
The pipeline set must nonetheless be complete, since any of them can be called from a
callback; a scanner test derives the set from the header. An error raised *directly* by a
callback (its own `mju_error`) necessarily unwinds across the callback's frame to the
enclosing engine boundary, as it always has through the bindings' handler; that is the
callback author's frame to keep free of live destructors. The timer callback is invoked
before a call's boundary is armed and must return normally (documented with `mjcb_time`).
A test re-enters the pipeline on the same data from each callback that receives a mutable
`mjData` and checks that the statement after the nested call runs.

**A stop path restores what the call owns.** Drivers that mark the stack (`mj_inverseSkip`,
`mj_compareFwdInv`) free it on every stop path, as the actuation and integrator stages
already did; a test raises a warning from a position-stage sensor under the stop policy and
checks `pstack`/`pbase` after every driver.

**Interactive simulators.** `simulate` and Studio pause on a negative status and, under the
stop policy, on any warning that stopped the step, reporting the warning text or the error;
under the continue policy they keep pausing on divergence as before.

**A failed stage stops the call.** A driver runs every child stage through
`mjSTAGE(mj_fwdVelocity(m, d))`, which leaves the call (`mjLEAVE`, return) when the stage
stopped it with a pending error or a warning under the stop policy — the `RETURN_IF_ERROR`
idiom, so the drivers read as the list of their stages; `mjSTAGE_(call, cleanup)` frees a
stack frame the driver owns across its stages (`mj_inverseSkip`, `mj_compareFwdInv`). Nothing
runs on abandoned data, in particular no callback: a failing `mjcb_passive` inside
`mj_fwdVelocity` never reaches `mjcb_control`. The entries of later stages refuse as well,
which covers stages the drivers reach through infallible helpers.

**Handlers that transfer control.** A handler that `longjmp`s or throws out of a pipeline
call would leave the thread's boundary chain pointing at dead frames. So `mju_message`
*detaches* the chain while it delivers an error: the handler runs with no boundaries armed
(the bindings' handler asks `_mjPRIVATE__hasBoundary`, which also sees the detached chain),
and the chain is re-attached only if the handler returns. A handler that escapes leaves no
boundaries behind, dead or alive; the rule is that it must land outside every pipeline call
on the thread (the bindings and the test fixture do), and landing inside one costs the
enclosing boundaries their recovery, which degrades to the exit of before, never to a jump
into a dead frame. Nothing is checked on the hot path. The data such a handler
abandoned keeps its leaked stack and in-progress flag until `mj_resetData`, which clears both.

**Worker threads.** `longjmp` across threads is undefined, so `mju_dispatch` runs every task
inside its own boundary (`RunTask`): an error abandons the task, records the first message in
the pool, and the task returns normally. After the join, `mju_dispatch` raises the recorded
message on the calling thread, where the pipeline boundary catches it. This also closes a hole
that existed independently: a worker-thread `mjERROR` used to bypass the bindings'
thread-local handler and exit the process under Python.

**Kinds.** Every raise site names the kind of its error, in the macro it raises with:
`mjERROR_OOM` (-2) when a memory region is exhausted, `mjERROR_INPUT` (-3) when the argument,
the model or the combination of options cannot work, `mjERROR_INTERNAL` (-4) when an engine
invariant is violated. `mjSTATUS_ERROR` (-1) is what plain `mju_error` produces, which after
the sweep of the engine means code outside it: a plugin, a user callback, a host. The kind
travels in the `status` field of the `mjLogMessage`, so the log handler sees it too;
`mju_message` copies it to a thread-local for the recovery, which writes it into the status.
Nothing is written to the shared `mjData` from a worker thread; a worker's kind travels with
its message to the calling thread. The 250 engine sites split almost evenly, 117 `INPUT`
against 113 `INTERNAL` and 20 `OOM`, and that is the distinction which earns the taxonomy its
keep: half of what the engine raises is addressed to the caller and half to us, and the
message text alone does not say which. A scanner test refuses an unclassified `mjERROR` or
`mju_error` in `src/engine`, so a new raise site has to choose. Numerical degeneracies (a
rank-deficient Hessian, a zero-mass moving body) are classified as `INPUT`; several of them
would be better raised as warnings, which is a separate question.

**Bindings.** The bindings' handler used to `longjmp` out of the engine to raise
`FatalError`, leaking the `mjData` stack frame of the interrupted call. Now, when a boundary
is armed, the handler records the message and returns; the engine recovers, and the binding
raises `FatalError` after the call returns with the same message. Calls without a boundary
(loaders) keep the `longjmp` path. The data is left with a pending error, so a Python caller
that catches `FatalError` and keeps stepping gets a `FatalError` naming the pending error
instead of undefined behavior on a leaked stack, until `mj_resetData`.

A Python exception raised *in a callback* is not an engine failure and must not poison the
data: users interrupt a step this way and keep using the data (the bindings' tests do). The
callback trampoline therefore calls `_mjPRIVATE__interrupt()` instead of `mju_error`: it sets
a thread-local *interrupted* flag and jumps to the innermost boundary, which restores the
snapshot and writes the transient `mjSTATUS_INTERRUPTED` into the status; the enclosing
stages unwind through the same checks a pending error uses (`mji_stop`, and the entries
refuse a negative status), the outermost exit resets the status to `mjSTATUS_OK`, and the
binding takes the flag after the call returns and raises the Python exception. The hot
checks read only `d->status`; the flag is read once, at the recovery. Outside a pipeline call the trampoline falls through to
`mju_error` and the `longjmp` path as before.

## Contracts, restated

- `mjData.status < 0`: the last pipeline call was abandoned by an error. Valid operations:
  read (arrays are consistent with the entry snapshot only up to the last completed stage),
  `mj_resetData`, `mj_copyData`, delete. Everything in the pipeline is refused.
- A log handler that intercepts errors and transfers control is unaffected. One that returns
  now has defined behavior: recovery at the innermost boundary, or process exit.
- `mjOK(d)` is false for a pending error as for a warning; `d->status < 0` distinguishes.

## Follow-ups

- Creators (`mj_makeData`, `mjv_makeScene`) returning `NULL` on allocation failure; the
  data-less argument-validation errors in `mju_`/`mjv_` utilities stay fatal as programmer
  errors.
- WASM: `setjmp`/`longjmp` need Emscripten's longjmp support, on by default.
