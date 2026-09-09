# How the engine reports recoverable conditions: a design survey

*Options doc, 2026-09-09. Not a spec. The committed onwarn work (PR #5) and its imagined
noexit follow-up (PR #8) are **one point** in this space; this steps back to the motivation
and the user experience and lays out the alternatives, including Saran's `mjCHECK`.*

## What we are actually deciding

Separate two questions that the onwarn PR currently answers together:

1. **Policy** — what the *engine* does when it hits a recoverable condition: apply the
   per-event recovery and continue (`auto`), record it and continue without resetting
   (`continue`), or stop the step at the first event (`stop`). **This is settled.** Nobody in
   the discussion wants to remove the `onwarn` option or change its three modes, and the
   policy is expressible under every transport below (stop-mode needs an internal "bail"
   signal, which can be a return *or* an mjData flag).

2. **Reporting** — how the *caller* learns what happened, and how errors (the noexit
   follow-up) fold in. **This is the whole debate.** Everything below varies only this.

Holding (1) fixed and varying (2) is what makes the options comparable.

## Motivation (recap)

Three standing problems, unchanged:

- **No in-band per-call signal.** Today a caller learns a step was impaired only by polling
  `d->warning[].number` deltas or noticing `d->time` went backwards. Studio reimplements this
  downstream as a private `kDiverged`/`kAutoReset` enum — the engine's missing return value,
  built by hand.
- **`exit()` in a library is hostile.** One diverged rollout thread killing the whole process
  is unacceptable for RL farms and long-running services (the noexit motivation).
- **`autoreset` is a blunt hardcoded in-between** — neither recover-and-report nor
  stop-and-report.

## Who is asking, and what they need (personas)

| persona | needs | dominant access pattern |
|---|---|---|
| RL rollout farm (C/C++) | detect an impaired/diverged step cheaply, reset *that* episode, **never** exit the process | inline, per-step, hot loop |
| RL / batch via **MJX / Warp** | same, but there is **no return channel** in JAX — outcome must live in the data | field read, post-step |
| Interactive **Studio** | pause on divergence, show the user which warning | post-step, once per frame |
| Debugging | stop at the first event, inspect state at the scene | stop-mode + inspect |
| Embedders (**Python**, game engines) | convert to the host error model (exception / log), without the engine terminating | wrap the call |
| CI / correctness | die loudly on *unexpected* impairment | opt-in fail-fast |

Two facts fall out of this table and drive the whole analysis:

- **MJX/Warp cannot use a return value.** Whatever else we do, the outcome *must* be
  materialised in `mjData` for the batched backends. So "a field in mjData" is not optional in
  any option — it is a floor. The question is whether the return value exists *in addition*.
- **The hot-loop C persona is the one that most wants the terse inline `if (mj_step(...))`.**
  It is also the persona least likely to forget to check. The persona most likely to forget
  (a casual embedder) is best served by the process simply *not dying* — i.e. by the poison
  semantics, not by the transport.

## The reserved axes

- **Transport**: (a) return value per pipeline function; (b) a field in `mjData`; (c) a
  registered callback (push); (d) thread-local errno.
- **Inspection ergonomics**: raw field access (`d->warning[]`, `d->status`) vs a blessed
  accessor (`mj_status(d)` / `mjOK(d)`) vs an aborting macro (`mjCHECK(d)` → `mju_error`).
- **Error unification (noexit)**: do recoverable errors ride the *same* surface as warnings
  (one thing to check) or a separate one?

## Options

### A. Plumbed returns + `mjData.status` mirror — *the current onwarn, and its noexit follow-up*

Every public pipeline function returns `mjtStatus`; `d->status` mirrors the last top-level
call; `d->warning[]` counts. noexit later makes `mjERROR` return `mjSTATUS_ERROR` through the
same value (reserved sign bit), with a poison flag until reset.

```c
if (mj_step(m, d)) reset_episode();          // warnings and, later, errors
```

- **Buys**: self-documenting signatures (the type says the call can fail); terse hot-loop
  idiom; per-call granularity for free (each call's own return); errors and warnings on one
  scalar.
- **Costs**: ~30 public signatures change `void → mjtStatus`; breaks `void`-returning function
  pointers; bindings-signature regen; doc signature sweep; **and the recurring one we have
  actually paid — every rebase re-touches those signatures and the `status |=` plumbing.** It
  is a permanent widening of the API surface.
- Note: this option is *already* a hybrid — it ships the `d->status` mirror precisely because
  MJX needs the field. The return is the *added* thing over Option B, not the other way round.

### B. `mjData` field + `mjCHECK`, functions stay `void` — *Saran*

Functions revert to `void`. Each ORs its bit into `d->status`, cleared at top-level entry;
stop-mode reads `d->status` internally (`if (mji_stop(m, d)) return;`). The user inspects
through a blessed reader; noexit's poison flag lives in the same `mjData`.

```c
mj_step(m, d);
if (!mjOK(d)) reset_episode();               // or: mjCHECK(d);  // aborts if impaired
```

- **Buys**: **zero signature churn** — the thing that has cost us on every rebase and would
  cost every future reader of `mj_step`'s signature. One inspection surface for warnings +
  errors + (future) stats. Native MJX/Warp parity (the field *is* the channel; no C-returns /
  JAX-mirror asymmetry). It is the existing idiom — users already read `d->warning[]` —
  given a front door.
- **Costs**: the **errno footgun** — a `void` signature does not announce fallibility, so
  callers can forget to check. Mitigated by the poison flag (an unchecked impaired `mjData`
  stays flagged and, in stop/error mode, frozen — degrades safely, not silently) and by
  making `mjCHECK`/`mjOK` the one documented idiom. Loses the terse single-line `if`.
- **Key realisation that makes B cheap**: the return value in Option A is *redundant with
  `d->status`*, which A already maintains (`return (d->status = status)`). B removes the copy,
  not the information. onwarn semantics — three modes, stop-abort, the counters, the
  singular-inertia/qH warnings — survive unchanged.

#### B, fleshed out: the accessors

The three names floated (`mjOK`, `mjCHECK`, `mj_status`) implied three different
implementations because they *are* three different things. In MuJoCo's own idiom:

**The substrate — `mjData.status`.** One scalar of type `mjtStatus` — a **signed status
code** (see *The representation of `mjtStatus`* below): zero is OK, positive values are
warning kinds, negative values are error kinds set by noexit's boundary. The poison contract
is then simply "entries refuse to run while `d->status < 0`, until `mj_resetData`" — no second
field, and the poisoned data says *why*. It is a public struct field, so Python/MJX read
`d.status` directly; there is nothing to bind and no accessor function is needed (MuJoCo
exposes state as fields — you read `d->ncon`, not `mj_getNcon(d)`).

**`mjOK(d)` — an expression macro.** Can a macro "return a value" in C? Yes: a function-like
macro is textual substitution, so one that expands to an *expression* yields that expression's
value and works anywhere an expression does. `mjDISABLED(x)`/`mjENABLED(x)` in `mjmacro.h`
already do exactly this over `m->opt.*flags`; `mjOK` is the same pattern over `d->status`:

```c
#define mjOK(d) ((d)->status == mjSTATUS_OK)
```

Strict by design — a warning *or* an error makes it false, which is what the RL persona wants
("was this step impaired at all"). Finer questions read the value: `d->status < 0` is an
error, and the value names the kind. A `static inline` function is the type-safe alternative;
the `mjmacro.h` precedent says macro, and it is deliberately *not* bound to Python, where
`not d.status` is the idiom.

**`mjCHECK(d)` — a statement macro, necessarily.** It has to be a macro for the reason
`CHECK` is a macro in every codebase: to capture `__FILE__`/`__LINE__` at the call site. And
it is pass-or-abort, so it yields no value (what a macro *cannot* do is contain statements and
still be an expression, short of non-standard `({...})`):

```c
#define mjCHECK(d)                                                              \
  do {                                                                          \
    if ((d)->status) {                                                          \
      mju_error("mjCHECK failed (%s:%d): status %d", __FILE__, __LINE__,        \
                (d)->status);                                                   \
    }                                                                           \
  } while (0)
```

It routes through the log handler, so C exits and Python raises `FatalError` — the existing
error contract, used as opt-in fail-fast. Under noexit a *user-level* `mjCHECK` sits above any
engine boundary, so it stays genuinely fatal, which is exactly the intent. The name aligns
with absl/glog `CHECK` = abort.

**`mj_status(d)` — dropped.** It was a *function* name (`mj_lower` is the function namespace;
`mjUPPER` is the macro namespace, which is why the other two read as macros), and a function
accessor over a public field is redundant with `d->status` and un-MuJoCo-like. Removing it
resolves the apparent inconsistency: the reader and the check are macros; the truth is a field.

So B's whole user-facing surface is: **one field and two macros.** Nothing to bind, nothing
to document twice.

### C. Both, with a canonical/substrate split — *the "I want both" option*

Keep returns as the **canonical, ergonomic C path**; keep `d->status` + `mjCHECK` as the
**substrate** for the contexts a return cannot reach: MJX/Warp, callbacks (`mjcb_control`
sees `(m, d)` only), post-hoc/cross-call/batched inspection, and the aborting macro.

```c
if (mj_step(m, d)) reset_episode();          // C hot loop — canonical
// ... and in a callback, or from Python, or in MJX:
if (!mjOK(d)) ...                            // same information, via the substrate
```

- **The honest accounting**: C does **not** buy the blast-radius relief that motivates B. It
  keeps every cost of A (signatures, fn-pointers, bindings, doc sweep, rebase pain) **in
  full** and *adds* the reader. C = A-costs + a macro. What it buys over A is only the
  *blessed reader* and an explicit story for MJX/callbacks — both of which A can also add.
- **So C is only coherent if exactly one path is canonical.** If both are presented as
  co-equal "ways to check," it is the worst option: two surfaces, docs must teach both,
  reviewers decide per-site, users ask which is right. *Which* path is canonical is decided by
  noexit (next section): once the mjData is the home of a computation's health, the truthful
  statement is **the field is canonical and the return is documented C shorthand** — "status
  lives in `d->status`; in C you may take it back as the return value." Note this costs
  nothing in discoverability: the signature still says `mjtStatus`. Canonical-ness (what the
  docs teach, what MJX/callbacks use) and discoverability (what the type announces) are
  separable, and C-field-canonical keeps both.
- **We are ~90% at good-C already**: onwarn ships returns + `d->status`. The only missing
  pieces are the blessed `mjOK`/`mjCHECK` reader, the error bit, and the sentence in the docs
  that says where the truth lives — a small, additive delta, not a rewrite. This is the most
  likely place "convince ourselves back to returns" lands: keep what we built, name the
  reader, and be honest that the return hands back the field.

### D. Minimal — policy + a counter reader, no per-call status at all

Add `onwarn` and a convenience reader over the existing `d->warning[]` counters; add **no**
`d->status` and **no** returns. "Did this call warn?" is answered by a per-call flag or by
diffing counters; stop-mode uses an internal flag.

- **Buys**: smallest footprint; no new state beyond a flag; no API churn.
- **Costs**: no clean per-call granularity (counters accumulate; the caller diffs); **fails
  the MJX floor unless we add the field anyway** — at which point D collapses into B. Weakest
  on the core "tell me this step was impaired" ask. Listed for completeness; likely dominated
  by B.

### E. Push / callback — *dismissed, with reason*

A registered per-`mjData` "on-warning" callback (like `mjcb_control`); the engine calls it
when an event fires.

- **Why not**: awkward for "reset *this* episode" (control is inverted); reentrancy hazards
  inside the pipeline; **MJX/Warp cannot call host callbacks** — fails the batched floor. A
  push channel can always be layered on top of B/C later (the handler reads `d`), so it is not
  foreclosed; it is just not the primary surface.

*(Thread-local errno — a global `mju_lastStatus()` — is a non-starter: status belongs to the
`mjData`, of which there are many per thread and thousands in a batch. Noted only to close the
axis.)*

## noexit as a first-class goal

Treat it as a definite goal, not a maybe: **the library never terminates the process**;
`mjERROR` becomes a recoverable, per-`mjData` condition. Held as a goal from the start, it
changes the analysis in three specific ways — and leaves one thing unchanged.

**1. Health becomes a property of the mjData.** The noexit boundary snapshots and restores
`d->pstack`/`d->parena`; the poison state lives in `mjData`; entries gate on it. Whatever the
transport, a computation's health is *already* going to live in the data. Under B that is the
whole model — "the mjData carries its own health; readers inspect it" — and warnings simply
join errors there. Under A/C the return value is, in both the warning and the error case, a
courtesy copy of mjData state handed back to the caller. That is a coherence argument, not an
engineering one, but it is what flips the coherent hybrid to field-canonical.

**2. One entry macro serves both.** noexit needs an in-body macro at the top of every
fallible public function (Saran's review: no generated wrappers; a **single outermost
boundary** — "if the jmp_buf is already set, leave it alone" — rather than a per-call stack).
B needs a clear-of-`d->status` at top-level entry. These are the same macro:

```c
#define mjENTRY(d)  /* if outermost: clear d->status, arm the boundary */
```

and Saran's proposed test — "assert every fallible MJAPI function has the trampoline" — is
then one test covering both concerns. The same instinct at both scales: a macro in the
function body, not a change to the signature.

**3. Poison is the cushion for B's liability.** B's one real weakness is the errno footgun —
a `void` signature, a check the caller might forget. If noexit is definite, forgetting to check
an *error* leaves a poisoned mjData that refuses to run until reset: safe, not silent
corruption. And forgetting to check a *warning* only reproduces today's behaviors under each
mode (`auto`: reset-and-continue; `continue`: NaN propagation; `stop`: frozen state). So with
noexit first-class, B's liability reduces to *discoverability alone* — the signature does not
advertise fallibility — which is a documentation and taste question, not a safety one.

**What noexit does not change: the cost side.** The boundary catches at the outermost entry
via `longjmp`, so errors never need threading through intermediate `status |=` paths under A —
noexit's sweep (one in-body macro per fallible function) is comparable under A and B. It is
the *shape* of noexit that discriminates (points 1–3), not its cost.

**How errors surface, per option.** Under A/C the boundary's catch path returns a negative
error code and `if (mj_step(...))` catches warnings and errors alike (`< 0` distinguishes);
under B/D it writes the same code into `d->status` and `mjOK(d)`/`mjCHECK(d)` catches both.
In every option errors are the negative values of the same scalar that carries warnings —
one field is the aggregate, which is what keeps `mjOK` a trivial macro rather than a real
accessor.

## Where does an error land? A survey of exit points

The noexit doc's boundary "writes `d->status`" — which assumes an `mjData` in scope.
Warnings never tested that assumption, because `mj_warning` takes `mjData` by construction.
Errors do not. A survey of every `mjERROR`/`mju_error` site in `src/engine`:

| enclosing function has | sites |
|---|---|
| `mjData` | 161 (65%) |
| `mjModel` only | 36 |
| neither | 49 |

So no — 85 of 246 sites sit in functions with no `mjData`. But the leaf-level count
overstates the problem, because of one fact:

**The sink is a property of the boundary, not the leaf.** Errors unwind by `longjmp` to the
outermost public entry. Most data-less *leaves* are helpers reached from d-bearing entries:
`mj_checkDiscrete` from `mj_step1`, `mju_factorLUSparse` from `mj_implicitSkip`,
`mju_makeFrame` from collision, `mj_wakeIsland`/`mj_dsuMerge` from the island and sleep
machinery inside the step, `computeY_fill` from `mj_projectConstraint`. The boundary that
catches them has `d` and writes `d->status`. Only **18 of the 37** data-less
public-or-internal functions are actual `mujoco.h` entry points, and those split cleanly by
what their messages say:

| class | entry points | sites | messages |
|---|---|---|---|
| creators / OOM | `mju_malloc`, `mjv_makeScene` | 4 | "Could not allocate memory"; "could not allocate geom / flex / skin buffers" |
| argument validation | `mj_copyModel`, `mjv_copyModel`, `mj_stateSize`, `mj_extractState`, `mju_euler2Quat`, `mju_rayGeom`, `mjv_connector`, eight `mjv_` camera/interaction functions | ~20 | "different buffer size"; "invalid state signature"; "seq must contain exactly 3 characters"; "unexpected action"; "cannot average perspective and orthographic cameras"; "unknown camera type" |

**Zero of the 18 are runtime physics failures.** Every divergence, singular factorization,
capacity exhaustion and degenerate-geometry failure in the engine lives under a d-bearing
boundary. That is the reassuring finding.

**The sink taxonomy that follows.**

- **`mjData`** — every pipeline and data-bearing entry, and everything transitively under
  it. All runtime physics failures. This is what noexit phase 1 covers and what `d->status`,
  `mjOK` and `mjCHECK` are for.
- **NULL return** — creators. `mju_malloc`, `mj_makeData`, `mj_copyData` return pointers;
  `mj_loadXML`/`mj_loadModel` *already* return NULL on failure with the message through the
  handler. noexit for creators is "return NULL instead of exit": the existing idiom made
  universal, no new mechanism.
- **Argument validation in data-less utilities** — programmer errors: "you called us wrong."
  Two defensible policies. **(a) Stay fatal.** Never-exit is about the process surviving
  *runtime failures*, not surviving the caller's bugs; an assert is the right response, and
  Python still gets `FatalError` through the handler exactly as today. **(b)** A thread-local
  `mju_lastStatus()` sink, so even these are catchable. Proposed: (a) for phase 1 — honest
  scope, no fourth mechanism — with (b) available as an escalation if anyone ever needs to
  catch `mjv_moveCamera("unexpected action")`, which nobody does. (The thread-local errno
  dismissed earlier for *status* is exactly the right tool here, for the one class of errors
  that owns no data.)

**Consequences for NOEXIT_DESIGN.md** (to fold in once we converge): the `mjENTRY(d)`
boundary is for d-bearing entries, so Saran's test becomes "every fallible **d-bearing**
MJAPI function has it" — a smaller, more precise set than "every function returning
`mjtStatus`"; creators get NULL paths; and the doc needs the sentence "the sink is a property
of the boundary, not the leaf."

**Two things the survey exposed, out of scope but worth logging.**

1. `mju_factorLUSparse` "diagonal element too small" under the `implicit` integrator is
   **fatal**, while the same singularity in `qH` under `implicitfast` is now an INERTIA
   **warning** (clamped). onwarn created an asymmetry between sibling integrators. The LU
   path is reached from `mj_implicitSkip` with `d` in scope and could warn instead.
2. `mj_checkDiscrete` raises six option-validation errors at `mj_step1` — *every step*. That
   is correct, not a smell: `mjOption` is user-modifiable at runtime, so the check cannot move
   to compile time. Under noexit they are INPUT-class errors caught at a d-bearing boundary
   and report as such; a caller who flips an unsupported option mid-run gets a poisoned step
   until they fix it, which is the right behavior.

**Does it bear on the main question?** No — neutrally. The data-less residue is creators
(NULL) and validation (fatal or TLS), and neither the return-canonical nor the field-canonical
design touches those: nobody proposes converting `mju_` signatures to return codes, so A and B
share the identical residue. The survey scopes noexit; it does not reopen A vs B.

## The representation of `mjtStatus`: bitmask or signed status code?

The onwarn design chose a bitmask (`1 << mjtWarning`, sign bit reserved for "error") over a
plain enum on two grounds: OR-composition through the pipeline, and forward compatibility
("an int gains bits freely"). Relitigated with noexit first-class, the case flips.

**The problem with one error bit.** Errors are not one kind. Resource exhaustion (stack or
arena overflow: resize and reset) is not an invariant violation ("the next line would read
out of bounds": poison, trust nothing) is not a geometric impossibility ("these axes are
parallel, the frame cannot be completed": degenerate input, state untouched) is not an
argument-validation failure. A caller who wants to react differently — and a poisoned
`mjData` that ought to say *why* it is poisoned — needs kinds, not a bit. Under the bitmask
that means budgeting error bits alongside warning bits inside 32, asking "is there a bit
left" on every future addition, and the sign-bit hack on top.

**The proposal: a signed status code.** Zero is OK; positive values are warning kinds;
negative values are error kinds:

```c
typedef enum mjtStatus {
  mjSTATUS_OK           = 0,
  // warnings: positive, value = mjtWarning + 1
  mjSTATUS_INERTIA      = 1,  mjSTATUS_CONTACTFULL, mjSTATUS_CNSTRFULL,
  mjSTATUS_BADQPOS,           mjSTATUS_BADQVEL,     mjSTATUS_BADQACC,  mjSTATUS_BADCTRL,
  // errors: negative, by class
  mjSTATUS_ERR_STACK    = -1, // stack overflow: resize, reset
  mjSTATUS_ERR_ARENA    = -2, // arena overflow beyond the CNSTRFULL saves
  mjSTATUS_ERR_INPUT    = -3, // argument / option validation
  mjSTATUS_ERR_GEOMETRY = -4, // computation cannot complete on degenerate input
  mjSTATUS_ERR_INTERNAL = -5  // invariant violation: data poisoned
} mjtStatus;
```

- `if (status)` — anything happened. `if (status < 0)` — an error; `> 0` — a warning. The
  sign test is the idiom, and it is the *same* test on the return (A/C) and on the field (B).
- **No bit budget.** A new kind is a new enumerator, forever. (Yes, 32 bits is probably enough
  today; a budget is a permanent tax on every addition, and widening to 64 bits would cost the
  type its C-enum / Python-`IntEnum` simplicity.)
- **Conventional.** `int rc = mj_step(m, d)` is *the* C-library idiom — errno, LAPACK `info`,
  SQLite `rc`. Bitmask returns are rare and need explaining. In Python it is a plain `IntEnum`,
  not an `IntFlag`: `status < 0` and `==` just work, no `&`.
- It removes a confusion we have already tripped on: under the bitmask, `mjtWarning` values
  are *indices* and `mjtStatus` values are *shifted bits*, so `mjWARN_BADQPOS & 8` looks
  plausible and is wrong. Under the code, `mjSTATUS_BADQPOS` is a value you compare to. (Two
  enums related by a mapping exist either way; `+ 1` reads better than `1 <<`.)

**What is lost: compound warnings in the summary — and why that is not a big deal.** A
single value cannot say "CONTACTFULL *and* CNSTRFULL happened in this call." But the
information is not lost: **`d->warning[].number` and `.lastinfo` are the record; the status
is the summary.** Every event is still counted; a caller who wants the full set snapshots the
counters (they clear only on reset). The personas confirm that the summary is what is
consumed — the RL loop wants "impaired at all → reset", Studio wants one warning to display,
debugging wants the record and has it — and in `stop` mode exactly one event occurs anyway.

**Which value under compound events?** Two rules, both one-liners:

- *Warnings keep the first* (the causal root): `if (!d->status) d->status = code;` inside
  `mj_warning`. Deterministic — every warning site is synchronous.
- *Errors overwrite*: the noexit boundary sets `d->status = errcode` unconditionally. An error
  is by construction the last event of a call (it unwinds it), so it must win over an earlier
  auto-recovered warning.

This replaces `status |= child()` with keep-first at every accumulation site — the same
number of lines. The one thing OR did better, deterministic merging at parallel joins, has no
current instance (no warning fires on a worker thread), and for noexit's per-task error slots
"lowest task index wins" is fine because every error has already logged its own message.

**Correction to the onwarn design's reasoning.** It rejected the plain enum because it
"loses composition and drops later events." That neglected that the counters preserve every
event; only the *summary* is single-valued, and a summary is what every consumer wants. The
other argument there — a by-value struct freezes its ABI — still stands; the signed code keeps
everything an `int`.

**Does it bear on the main question?** Mildly, toward returns: a signed status code is the
most familiar return idiom in C libraries, so if `mjtStatus` is a *code* rather than a flag
word, keeping the return (A/C) reads as "MuJoCo functions return an rc, like everything else"
rather than as a novelty. It improves B too (`mjOK` is `== 0`, an error is `< 0`, and a
poisoned `mjData` says why). It does not decide A vs B.

## Comparison

| criterion | A (returns) | B (field+macro) | C (both) | D (minimal) |
|---|---|---|---|---|
| signature churn / rebase cost | high | **none** | high | none |
| fn-pointer / bindings-signature break | yes | **no** | yes | no |
| self-documenting fallibility | **yes** | no | yes | no |
| terse hot-loop `if` | **yes** | no | yes | no |
| per-call granularity | **yes** | via field | yes | weak |
| MJX/Warp parity | mirror field | **native** | native | needs field |
| errno "forgot to check" footgun | **low** | medium (poison-cushioned) | low | medium |
| one surface for warn+error+stats | scalar | **field** | two surfaces | field |
| net new API concepts | mjtStatus type + returns | mjtStatus type + `mjOK`/`mjCHECK` | both | reader only |

## Where I land (for the argument, not to close it)

The policy is settled; the transport turns on a single trade: **signature-level
discoverability + the terse RL idiom (A/C)** versus **permanent API-surface and rebase relief
(B)**. Two honest observations:

- The blast radius is real and we have *felt* it — but it is a **one-time** cost that ends the
  moment onwarn lands and imports; B's relief is mostly retrospective. Weigh "pain already
  largely paid" accordingly.
- The MJX floor means the field exists no matter what. So the return is genuinely *optional
  sugar* over a field we are keeping regardless. That reframes the choice as "is the sugar
  worth its signature cost," which is a smaller, cleaner question than "returns vs field."

A third observation, once noexit is first-class: canonical-ness and discoverability come
apart. The field can be the *documented truth* (what MJX, callbacks and Python use) while the
`mjtStatus` return type still *announces fallibility* in every C signature. You do not have to
choose between "the truth lives in mjData" and "the signature tells you it can fail."

My read, updated: **C with the field canonical and the return as C shorthand** is the
defensible home for a soft spot on returns. It keeps the self-documenting signature and the
hot-loop idiom the C audience actually wants, tells the truth about where health lives (which
noexit forces anyway), and is a *small additive step from what onwarn already is*: name the
reader, add the error bit, write the one sentence that says the return hands back the field.
It also makes the hybrid *safe*: the return is removable sugar, so pure **B** remains
reachable later by deletion, with no change in semantics — that reversibility is itself a
reason not to agonise. Pure **B now** wins only if we judge signature minimalism to outrank
discoverability for MuJoCo's audience — more respectable than it was before noexit was on the
table (points 1–3 above), but the RL hot loop is still exactly where a one-line
self-documenting failure check earns its keep. **D and E are dominated.**

## Open questions

1. **Open — the whole decision.** Is signature-level discoverability worth a permanent
   `void → mjtStatus` on ~30 public functions, given the field exists regardless? (A/C vs B.)
   Still on the fence; worth outside opinions.
2. *Resolved*: under C the docs make the **field canonical** and the return "C shorthand."
3. *Resolved*: the reader is two macros — `mjOK(d)` (expression, the `mjDISABLED` pattern)
   and `mjCHECK(d)` (statement, aborting via `mju_error` with file:line); `mj_status(d)`
   dropped.
4. *Resolved*: `mjtStatus` survives as a public type regardless — it is the type of
   `d->status` — even under B where it leaves the signatures.
5. *Resolved*: noexit uses Saran's single-outermost-boundary + in-body macro (no generated
   wrappers), merged with the clear-at-entry into one `mjENTRY(d)`, with one test asserting
   every fallible **d-bearing** entry has it (the exit-point survey narrows the set).
6. *Relitigated → proposed*: `mjtStatus` is a **signed status code**, not a bitmask (section
   above): zero OK, positive warning kinds, negative error kinds; warnings keep-first, errors
   overwrite; the counters remain the full record. Confirm the rule and the switch away from
   `|=`.
7. *New*: the error classes — STACK / ARENA / INPUT / GEOMETRY / INTERNAL as sketched, or
   fold GEOMETRY into INPUT? And their relationship to the noexit taxonomy (validation /
   exhaustion / invariant): INPUT+GEOMETRY ↔ validation, STACK+ARENA ↔ exhaustion,
   INTERNAL ↔ invariant.
8. *New, from the exit-point survey*: policy for argument-validation errors in data-less
   public utilities (~14 `mujoco.h` entries, all `mjv_`/`mju_`/copy/state functions) — stay
   fatal as programmer errors (proposed for phase 1) vs a thread-local `mju_lastStatus()`
   sink. Creators are settled: NULL return, the existing loader idiom.
9. *Follow-up logged by the survey, out of scope — confirmed as a fix*: `mju_factorLUSparse`
   singularity is fatal under `implicit` while the same condition warns under `implicitfast`;
   make the LU path an INERTIA warning. (The `mj_checkDiscrete` per-step check is correct as
   is — `mjOption` is runtime-modifiable — and is not a follow-up.)
