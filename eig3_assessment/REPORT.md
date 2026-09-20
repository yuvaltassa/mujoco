# 3x3 eigensolver accuracy: assessment

Base: upstream 451e66a8d. Apple silicon, clang; double (Debug and Release) and `mjUSESINGLE` (Release).
Error metric: `max|Q diag(w) Q' - A| / max|A|` ("recon"); axis error is the angle to the true principal axes.
Raw output of every run is in `results/`.

## 1. The current solvers

200k matrices per family, unit scale, worst case:

| | recon | axis error | iterations avg (max) |
|---|---|---|---|
| `mju_eig3`, `mjuu_eig3`, double | 2.7e-6 .. 4.0e-6 | 1.4e-6 rad | 6.2 (8) |
| `mju_eig3`, float32 | 4.5e-4 .. 1.2e-3 | 1.3e-3 rad | 5.2 (7) |

The premise holds, and float32 is three decades worse: `c` rounds to exactly `1.0f` for rotations below ~4e-4 rad.

Two further defects:

- **The off-diagonal test is absolute**, so accuracy depends on the units of the matrix.

  | scale of A | recon | axis error |
  |---|---|---|
  | 1e-3 .. 1e9 | 2.8e-6 | 1.4e-6 rad |
  | 1e-6 | 5.9e-6 | 4.4e-5 rad |
  | 1e-9 | 7.5e-3 | 5.8e-2 rad |
  | 1e-12 | 0.88 | no diagonalization at all |

  End to end in the compiler: `fullinertia="2e-10 3e-10 4e-10 1e-11 2e-11 3e-11"` compiles to products of inertia
  9.96e-12, 2.0012e-11, 2.962e-11; at 1e-13 all three compile to 0, and a non-positive-definite matrix of that size is
  accepted. For large matrices, and always in float32, roundoff in `D` exceeds 1e-12 and the test can never fire; the
  cosine test is then the only thing that terminates the loop.
- **float32 already runs to the iteration cap**: 49% of nearly isotropic matrices take all 500 iterations (average 276).

## 2. Stopping rules

| rule (50k matrices per family) | double | float32 |
|---|---|---|
| drop the cosine test, or replace it by `\|t\| < eps` | recon 3.3e-8; **99.98% of well separated matrices run to the cap** | 100% run to the cap |
| drop the cosine test, cancellation-free half-angle | recon 4e-12, no cap hits at unit scale | 100% run to the cap: the absolute test cannot fire |
| `\|t\| < eps`, cancellation-free half-angle | recon 4e-12 at unit scale; 1.5% cap hits at scale >= 1e6; small-scale error unchanged (5.9e-6 at 1e-6, no diagonalization at 1e-12) | 1.3% cap hits, 87% for nearly isotropic |
| **off-diagonal <= k eps max\|A\|, cancellation-free half-angle** | recon 9e-15, at most 10 iterations, no cap hits, any scale | recon 3e-6, at most 8, no cap hits, any scale |

Why dropping the cosine test stalls: the half-angle sine is computed as `sqrt(0.5 - 0.5*c)`, which is exactly zero once
`c` rounds to 1. The rotation is then the identity, `D` does not change, and the loop spins. The cosine test was there
to hide this. A `t` test inherits the stall, and cannot terminate repeated eigenvalues at all: there `t` is a ratio of
two roundoff-level numbers and is O(1). What terminates them is recognizing that the off-diagonal is at the roundoff
level of `D = Q'AQ`, which needs a scale.

Choice of `k`: the roundoff floor of the largest off-diagonal element, iterating with no stopping rule, is at most
4.4 eps max|A| (6M samples per precision). Over 54M solves per precision (12 families, scales over 24 decades,
compiled with FMA, with `-ffp-contract=off`, and at `-O0`): k=2 stalls at 5e-4, k=4 stalls 13 times and only without
FMA, k=8, 16, 32 never, with at most 10 iterations. Worst case, the quaternion can still register the rotation when
k >= 6. The change uses 16 (`4e-15`, `2e-6f`).

Same algorithm elsewhere: PhysX `PxDiagonalize` is this quaternion Jacobi "with fix for precision issues": a
scale-free test on the rotation angle, a small-angle quaternion for `|w| > 1000`, and a cap of 24 iterations. Bullet
uses in-place Jacobi with a threshold relative to the diagonal; Drake uses Eigen's iterative solver after an
is-diagonal test at `4 eps` times the largest moment. Nobody uses an absolute threshold.

## 3. The change, measured

| | recon before -> after | axis error | iterations avg (max) | cap hits |
|---|---|---|---|---|
| `mju_eig3` double | 2.7e-6 -> 9.2e-15 | 1.4e-6 -> 3.7e-14 rad | 6.2 (8) -> 8.1 (10) | 0 -> 0 |
| `mjuu_eig3` | 2.7e-6 -> 1.6e-14 | same | same | 0 -> 0 |
| `mju_eig3` float32 | 7.8e-4 -> 3.0e-6 | 1.3e-3 -> 1.9e-5 rad | 5.2 (7) -> 6.1 (8) | 0 -> 0 |
| float32, nearly isotropic | 1.7e-6 -> 3.1e-6 | | 276 (500) -> 0.6 (6) | 98205 of 200k -> 0 |

Identical at every scale from 1e-12 to 1e9. Orthonormality of `Q` is unchanged (4.7e-15, 8e-7).

## 4. Blast radius

**Tests.** Five, all on the changed path:
- `MjCMeshTest.FlippedFaceAllowed{World,NoMass,Inertial,NegligibleArea}` compare `mesh_vert` with recorded values at
  `FLT_EPSILON`. The recorded values are rotated 7.3e-7 rad from the analytic principal frame of the tetrahedron
  (float64 `eigh` of its exact inertia); the new output matches it to 6e-8, i.e. float32 storage. Values regenerated,
  tolerance untouched.
- `FuseStaticTest.FuseStaticEquivalent` compares `qvel` of the fused and unfused model at `2e-17`, 46 ulp. The residual
  was 1.73e-17 and is now 2.30e-17 (40 -> 53 ulp), unchanged in float32 (4.7e-10). Recalibrated to `2e-16`. This is the
  only tolerance that was loosened.

**Compiled models.** 382 models load in both builds (repository `model/` and `test/`, mjlab's G1, Go1 and YAM); load
status and array sizes are identical, 289 are bit-identical (300 in float32).
- Body inertia tensor `R diag(I) R'` in the body frame: changes by at most 1.14e-6 of the largest moment; 3 models
  above 1e-6, none above 1e-5.
- Mesh geometry in the body frame: at most 1.0e-7 of the mesh size (1.6e-7 in float32), which is float32 storage.
- Raw fields: for bodies with well separated moments (relative gap > 1e-2) the inertial frame moves at most
  9.4e-7 rad. Of 1215 bodies whose `body_iquat` changed, 106 move more than 1e-6 rad and 101 more than 1e-5 rad:
  100 tessellated spheres (relative gap 3e-8, 0.05 rad, where the absolute test had stopped early) and one exactly
  symmetric tetrahedron body whose two equal axes swapped. `mesh_quat`, `geom_quat`, `mesh_vert` and the mesh BVH
  move with them. In `model/`:
  55 of 86 bit-identical, 24 change at roundoff, `helix`, `carousel`, `bunnies`, `sdf/cow` by 1e-8 .. 4e-7, and the
  meshes of `mug`, `welcome`, `sdf/mug` get new axes (tensor change <= 1.3e-8).
- So: yes, raw values of 16 models differ beyond 1e-6, but no physical quantity does by more than 1.1e-6.
- Behaviour change: a tiny `fullinertia` that is not positive definite used to compile and is now rejected.

**Engine.** `mj_broadphase` is the only engine caller: it orients the SAP frame along the covariance of geom
positions, every step. With the same compiled model, the old and new engine give bit-identical `qpos`/`qvel` on
boxes, humanoid, 22 humanoids, humanoid100, bunnies, particle and car, in both precisions: SAP is exact in any
orthonormal frame and the pairs are sorted. The cost is Jacobi rotations per step (load-independent):

| model (double) | upstream | converged | optional `eig3-broadphase-tol` |
|---|---|---|---|
| 3 boxes | 3.0 | 5.0 | 1.0 |
| humanoid | 1.0 | 2.6 | 1.0 |
| 22 humanoids | 3.0 | 5.0 | 0.9 |
| humanoid100 | 3.5 | 4.7 | 1.8 |
| bunnies | 4.9 | 6.9 | 3.0 |
| particle | 4.3 | 6.0 | 3.0 |
| car | 1.0 | 4.0 | 1.0 |

A rotation costs roughly 50-100 ns, so the converged solve adds 0.1-0.3 microseconds per step: 1-3% of a model that
steps in a few microseconds, nothing measurable beyond. Wall-clock numbers were taken on a heavily loaded machine
(load average above 100 on 10 cores) and are indicative only; the rotation counts are exact.

**Not assessed.** Menagerie (the local checkout is a blob-less sparse clone with only `i2rt_yam`), Python and MJX test
suites (no recorded values that depend on principal axes were found; the suites were not run), a full Release-double
`ctest` (Debug double and Release float32 were run in full; the affected tests also in Release double), and anything
downstream that records `body_iquat`, `mesh_quat` or mesh vertices of nearly symmetric bodies.

## 5. Left alone

- The threshold for sorting eigenvalues is still an absolute 1e-12, so eigenvalues of tiny matrices come back
  unsorted, as before. Making it relative would reorder axes of nearly symmetric bodies for no physical gain.
- `mjuu_normvec` does not normalize within 1e-14 of unit norm, which bounds the orthonormality of `mjuu_eig3`'s output.
- `makeAAMM` projects onto the rows of the eigenvector matrix, but the eigenvectors are its columns: SAP sweeps along
  the transposed frame (`timing/frame_rows.cc`). Correctness is unaffected, pruning is. Long-standing.
- The broadphase computes the covariance and its eigendecomposition even when fewer than two bodies can collide.
