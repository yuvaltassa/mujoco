// Experimental planar row projection/compaction for this contact-free, dense Newton lab.
// This is not a new MuJoCo equality type or a native performance implementation.
#ifndef LINKAGE_LAB_PLANAR_ROWS_H_
#define LINKAGE_LAB_PLANAR_ROWS_H_
#include <mujoco/mujoco.h>

#include <algorithm>
#include <vector>

inline int plane_frame;
inline bool plane_enabled;

inline void PlanarRows(const mjModel* m, mjData* d) {
  if (!plane_enabled || d->ne == 1) return;  // tendon already has one physical row
  int original = d->ne;
  int dim = m->eq_type[d->efc_id[0]] == mjEQ_WELD ? 6 : 3;
  if (original != dim || d->nefc != original || d->nisland || m->opt.jacobian != mjJAC_DENSE) {
    mju_error("Planar lab requires one connect/weld, no contacts, dense J, and no islands");
  }
  // Translation is expressed in the constant mechanism frame. Rotation residuals are
  // already in the second weld site's frame, whose z axis is the mechanism normal.
  const mjtNum* rotation = d->xmat + 9 * plane_frame;
  std::vector<mjtNum> jac(d->efc_J, d->efc_J + dim * m->nv);
  mjtNum pos[6], vel[6], aref[6];
  mju_copy(pos, d->efc_pos, dim);
  mju_copy(vel, d->efc_vel, dim);
  mju_copy(aref, d->efc_aref, dim);
  for (int r = 0; r < 2; ++r) {
    d->efc_pos[r] = d->efc_vel[r] = d->efc_aref[r] = 0;
    for (int k = 0; k < 3; ++k) {
      mjtNum p = rotation[3 * k + r];
      d->efc_pos[r] += p * pos[k];
      d->efc_vel[r] += p * vel[k];
      d->efc_aref[r] += p * aref[k];
    }
    for (int j = 0; j < m->nv; ++j) {
      d->efc_J[r * m->nv + j] = 0;
      for (int k = 0; k < 3; ++k)
        d->efc_J[r * m->nv + j] += rotation[3 * k + r] * jac[k * m->nv + j];
    }
  }
  int count = dim == 6 ? 3 : 2;
  if (dim == 6) {
    mju_copy(d->efc_J + 2 * m->nv, jac.data() + 5 * m->nv, m->nv);
    d->efc_pos[2] = pos[5];
    d->efc_vel[2] = vel[5];
    d->efc_aref[2] = aref[5];
    d->efc_R[2] = d->efc_R[5];
    d->efc_D[2] = d->efc_D[5];
    d->efc_diagA[2] = d->efc_diagA[5];
    mju_copy(d->efc_KBIP + 8, d->efc_KBIP + 20, 4);
  }
  // All retained rows have the same equality id/type and zero margin/frictionloss.
  // The primal solver needs only compact J, references, row weights and row metadata.
  d->ne = d->nefc = count;
  d->nJ = count * m->nv;
  if (m->opt.enableflags & mjENBL_DIAGEXACT) {
    std::vector<mjtNum> inverse(count * m->nv);
    mj_solveM(m, d, inverse.data(), d->efc_J, count);
    for (int r = 0; r < count; ++r) {
      mjtNum diagonal = mju_dot(d->efc_J + r * m->nv, inverse.data() + r * m->nv, m->nv);
      mjtNum imp = d->efc_KBIP[4 * r + 2];
      d->efc_R[r] = mju_max(mjMINVAL, (1 - imp) * diagonal / imp);
      d->efc_D[r] = 1 / d->efc_R[r];
      d->efc_diagA[r] = d->efc_R[r] * imp / (1 - imp);
    }
  }
}
#endif
