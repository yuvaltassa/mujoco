// Experimental controls for diagnosing near-null equality rows. No engine changes.
#ifndef LINKAGE_LAB_REGULARIZATION_H_
#define LINKAGE_LAB_REGULARIZATION_H_

#include <mujoco/mujoco.h>
#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <string>

inline std::string lab_regularization;
inline double lab_floor = 1e-6;

inline void LabRegularize(const mjModel* m, mjData* d) {
  for (int row = 0; row < d->ne;) {
    int type = m->eq_type[d->efc_id[row]];
    int dim = type == mjEQ_WELD ? 6 : type == mjEQ_CONNECT ? 3 : 1;
    if (dim > 1) {
      for (int block = row; block < row+dim; block += 3) {
        mjtNum largest = 0, mean = 0;
        for (int j = block; j < block+3; ++j) {
          largest = std::max(largest, d->efc_R[j]);
          mean += d->efc_R[j]/3;
        }
        for (int j = block; j < block+3; ++j) {
          d->efc_R[j] = lab_regularization == "block" ? mean :
              std::max(d->efc_R[j], static_cast<mjtNum>(lab_floor*largest));
          d->efc_D[j] = 1/d->efc_R[j];
          mjtNum imp = d->efc_KBIP[4*j+2];
          d->efc_diagA[j] = d->efc_R[j]*imp/(1-imp);
        }
      }
    }
    row += dim;
  }
  // The primal solver consumes island-local copies when islands are enabled.
  if (d->nisland) {
    for (int row = 0; row < d->nefc; ++row) {
      int source = d->map_iefc2efc[row];
      d->iefc_R[row] = d->efc_R[source];
      d->iefc_D[row] = d->efc_D[source];
    }
  }
}

inline void LabConfigure(const mjModel* m) {
  const char* mode = std::getenv("LINKAGE_REGULARIZATION");
  lab_regularization = mode ? mode : "raw";
  if (lab_regularization == "raw") return;
  if (lab_regularization != "floor" && lab_regularization != "block") {
    throw std::runtime_error("LINKAGE_REGULARIZATION must be raw, floor, or block");
  }
  if (const char* value = std::getenv("LINKAGE_FLOOR")) lab_floor = std::stod(value);
  if (lab_floor <= 0 || lab_floor > 1 || m->opt.integrator != mjINT_IMPLICITFAST ||
      m->opt.solver != mjSOL_NEWTON || m->opt.jacobian != mjJAC_DENSE) {
    throw std::runtime_error("Diagnostic regularization requires implicitfast, Newton, dense J, and 0 < floor <= 1");
  }
  mjcb_control = LabRegularize;
}
#endif
