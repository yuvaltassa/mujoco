# eig3 assessment harness

Scaffolding behind the change "Converge the 3x3 eigensolvers to machine precision at any scale".
The findings are in [REPORT.md](REPORT.md).
Not meant to land; kept on this side branch so the measurements can be reproduced.

All tools link against a built `libmujoco` that exports `mjuu_eig3` (any commit of the
`eig3-convergence` branch; for the baseline, build upstream with `MJAPI` added to the declaration
of `mjuu_eig3` in `src/user/user_util.h`). Add `-DmjUSESINGLE` when linking a float32 library.

    clang++ -std=c++17 -O2 [-DmjUSESINGLE] -I<mujoco>/include <tool>.cc \
        -L<libdir> -lmujoco -Wl,-rpath,<libdir> -o <tool>

## eig3_bench.cc, eig3_template.h

`eig3_bench [ntrial] [lib|rules|scale|floor|time|stress]`

- `lib`: the library's `mju_eig3` (mjtNum) and `mjuu_eig3` (double) as they are, over twelve families
  of symmetric matrices (well separated, nearly repeated, exactly repeated, rank deficient, nearly
  diagonal, inertia of point clouds, smallest resolvable rotations, indefinite), at unit scale and
  over a scale sweep. Reports reconstruction error `max|Q diag(w) Q' - A| / max|A|`, off-diagonal
  residual of `Q'AQ`, principal-axis error, orthonormality of `Q`, eigenvalue error, iterations and
  hits of the 500-iteration cap.
- `rules`, `scale`: the same metrics for candidate stopping rules, using a templated copy of the
  algorithm in both precisions: current, cosine test dropped, `|t|` test, relative off-diagonal test
  at 4/8/16/32 epsilons, progress guard; each with the original and the cancellation-free half-angle.
- `floor`: roundoff floor of the largest off-diagonal element when iterating without a stopping rule.
- `stress`: iteration-count histogram of the relative rules over all families at random scales.
  Also compile with `-ffp-contract=off` and `-O0` to vary the arithmetic.
- `time`: ns per call. Only meaningful on a quiet machine.

## model_dump.cc, model_diff.py, iquat_detail.py

`model_dump <list.txt> <out.bin> [plugin_dir]` compiles every MJCF in the list and dumps all numeric
`mjModel` arrays; build it once per library. `model_diff.py base.bin new.bin` compares two dumps
field by field and, independently of the arbitrary choice of principal axes, compares the body
inertia tensor `R diag(I) R'` and mesh-geom vertices in the body frame. `iquat_detail.py` relates
the rotation of each changed inertial frame to the body's eigenvalue gap.

## timing/

- `broadphase_eig3.cc`: replays a model, rebuilds the covariance that `mj_broadphase` passes to
  `mju_eig3` at every step, times `mju_eig3` alone on those matrices and prints a bitwise hash of the
  final state. Feed both libraries the same baseline-compiled `.mjb` (`save_mjb.cc`) to isolate the
  engine from the compiler.
- `broadphase_rules.cc`: Jacobi rotations per broadphase call for the current rule, the converged
  rule and looser relative tolerances, on the same recorded covariances. Load-independent.
- `frame_rows.cc`: shows that `makeAAMM` projects onto the rows of the eigenvector matrix.
- `tiny_inertia.cc`, `tetra_golden.cc`: end-to-end compiler checks (small `fullinertia`, the
  tetrahedron golden values of `MjCMeshTest.FlippedFaceAllowed*`).

## results/

Output of the runs quoted in the assessment (Apple M-series, clang, double and `mjUSESINGLE`).
