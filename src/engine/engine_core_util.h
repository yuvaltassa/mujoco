// Copyright 2025 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_SRC_ENGINE_ENGINE_CORE_UTIL_H_
#define MUJOCO_SRC_ENGINE_ENGINE_CORE_UTIL_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>

#ifdef __cplusplus
extern "C" {
#endif


//-------------------------- model properties ------------------------------------------------------

// determine type of friction cone
MJAPI int mj_isPyramidal(const mjModel* m);

// determine type of constraint Jacobian
MJAPI int mj_isSparse(const mjModel* m);


//-------------------------- sparse chains ---------------------------------------------------------

// merge dof chains for two bodies
int mj_mergeChain(const mjModel* m, int* chain, int b1, int b2, int flg_skipcommon);

// merge dof chains for two simple bodies
int mj_mergeChainSimple(const mjModel* m, int* chain, int b1, int b2);

// get body chain
int mj_bodyChain(const mjModel* m, int body, int* chain);


//-------------------------- Jacobians -------------------------------------------------------------

// compute 3/6-by-nv Jacobian of global point attached to given body
MJAPI void mj_jac(const mjModel* m, const mjData* d,
                  mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);

// compute body frame Jacobian
MJAPI void mj_jacBody(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, int body);

// compute body center-of-mass Jacobian
MJAPI void mj_jacBodyCom(const mjModel* m, const mjData* d,
                         mjtNum* jacp, mjtNum* jacr, int body);

// compute subtree center-of-mass Jacobian
MJAPI void mj_jacSubtreeCom(const mjModel* m, mjData* d, mjtNum* jacp, int body);

// compute geom Jacobian
MJAPI void mj_jacGeom(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, int geom);

// compute site Jacobian
MJAPI void mj_jacSite(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, int site);

// compute translation Jacobian of point, and rotation Jacobian of axis
MJAPI mjtStatus mj_jacPointAxis(const mjModel* m, mjData* d,
                           mjtNum* jacPoint, mjtNum* jacAxis,
                           const mjtNum point[3], const mjtNum axis[3], int body);

// compute 3/6-by-nv sparse Jacobian of global point attached to given body
void mj_jacSparse(const mjModel* m, const mjData* d,
                  mjtNum* jacp, mjtNum* jacr, const mjtNum* point, int body,
                  int NV, const int* chain, int flg_skipcommon);

// sparse Jacobian difference for simple body contacts
void mj_jacSparseSimple(const mjModel* m, const mjData* d,
                        mjtNum* jacdifp, mjtNum* jacdifr, const mjtNum* point,
                        int body, int flg_second, int NV, int start);

// compute 3/6-by-NV sparse Jacobian time derivative of global point attached to given body
MJAPI void mj_jacDotSparse(const mjModel* m, const mjData* d,
                           mjtNum* jacp, mjtNum* jacr, const mjtNum* point, int body,
                           int NV, const int* chain);

// dense or sparse Jacobian difference for two body points: pos2 - pos1, global
MJAPI int mj_jacDifPair(const mjModel* m, const mjData* d, int* chain,
                        int b1, int b2, const mjtNum pos1[3], const mjtNum pos2[3],
                        mjtNum* jac1p, mjtNum* jac2p, mjtNum* jacdifp,
                        mjtNum* jac1r, mjtNum* jac2r, mjtNum* jacdifr,
                        int issparse, int flg_skipcommon);

// dense or sparse weighted sum of multiple body Jacobians at same point
int mj_jacSum(const mjModel* m, mjData* d, int* chain,
              int n, const int* body, const mjtNum* weight,
              const mjtNum point[3], mjtNum* jacp, mjtNum* jacr, int flg_rot);

// compute 3/6-by-nv Jacobian time derivative of global point attached to given body
MJAPI void mj_jacDot(const mjModel* m, const mjData* d,
                     mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);

// compute subtree angular momentum matrix
MJAPI void mj_angmomMat(const mjModel* m, mjData* d, mjtNum* mat, int body);


//-------------------------- coordinate transformation ---------------------------------------------

// compute object 6D velocity in object-centered frame, world/local orientation
MJAPI void mj_objectVelocity(const mjModel* m, const mjData* d,
                             int objtype, int objid, mjtNum res[6], int flg_local);

// compute material surface velocity of a geom at a point, in world frame
void mj_geomSurfaceVelocity(const mjModel* m, const mjData* d, int geomid,
                            const mjtNum point[3], mjtNum linear[3], mjtNum angular[3]);

// compute object 6D acceleration in object-centered frame, world/local orientation
MJAPI void mj_objectAcceleration(const mjModel* m, const mjData* d,
                                 int objtype, int objid, mjtNum res[6], int flg_local);

// map from body local to global Cartesian coordinates
MJAPI void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9],
                           const mjtNum pos[3], const mjtNum quat[4],
                           int body, mjtByte sameframe);


//-------------------------- miscellaneous ---------------------------------------------------------

// gather global node positions and velocities
MJAPI void mju_flexGatherState(const mjModel* m, const mjData* d, int f, mjtNum* xpos, mjtNum* vel);

// extract 6D force:torque for one contact, in contact frame
MJAPI void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum result[6]);

// count the number of length limit violations for tendon i (0, 1 or 2)
int tendonLimit(const mjModel* m, const mjtNum* ten_length, int i);

// return actuator damping contribution to joint or tendon
MJAPI mjtNum mj_actuatorDamping(const mjModel* m, mjtObj type, int id, mjtNum poly[mjNPOLY]);

// return actuator armature contribution to joint or tendon
MJAPI mjtNum mj_actuatorArmature(const mjModel* m, mjtObj type, int id);

// warn when a call discards the status it returns: on the engine's own declarations, and only in
// its C sources, where a status is either kept or discarded explicitly with a (void) cast. The
// tests and other C++ callers of these headers may ignore it as users do, and gcc does not honor
// the cast, so clang alone checks.
#if defined(__clang__) && !defined(__cplusplus)
  #define mjNODISCARD __attribute__((warn_unused_result))
#else
  #define mjNODISCARD
#endif

// high-level warning function: count warnings in mjData, print the first time; return the
// status of the warning, for the caller to compose into its own
MJAPI mjNODISCARD mjtStatus mj_warning(mjData* d, int warning, int info);

// compose a status with the one a call reported: a call keeps the first warning it ran into,
// and an error (negative) over any warning
static inline mjtStatus mji_join(mjtStatus status, mjtStatus reported) {
  if (status < 0) {
    return status;
  }
  return reported < 0 || !status ? reported : status;
}

// record the status of a pipeline call in the data it ran on, and return it. The mirror serves
// the consumers that never see a return value: callbacks and post-step hooks receiving only
// (m, d), and recorders snapshotting mjData. The outermost call returns last, so it is the one
// whose status the field ends up holding.
static inline mjtStatus mji_report(mjData* d, mjtStatus status) {
  return (mjtStatus)(d->status = status);
}

// run a pipeline stage, keep its status, and return if a warning under the stop policy stopped
// the call; mjSTAGE_ runs a cleanup before returning
#define mjSTAGE_(call, cleanup)                                                   \
  {                                                                               \
    status = mji_join(status, (call));                                            \
    if (mji_stop(m, status)) {                                                    \
      cleanup;                                                                    \
      return mji_report(d, status);                                               \
    }                                                                             \
  }
#define mjSTAGE(call) mjSTAGE_(call, (void)0)

// the pipeline call should unwind: an error, or a warning under the stop policy
static inline int mji_stop(const mjModel* m, mjtStatus status) {
  return status < 0 || (status && m->opt.onwarn == mjONWARN_STOP);
}


//-------------------------- effective-metric predicates ------------------------------------------

// the selected integrator performs the constraint solve in the effective metric
int mj_isMetric(const mjModel* m);

// do the tendon and actuator classes enter the metric (excluded under solver=PGS only;
// noslip atop a primal solver keeps them)
MJAPI int mj_effCouplings(const mjModel* m);

// tendon i has a spring: nonzero stiffness or stiffness polynomial
int mj_tendonHasStiffness(const mjModel* m, int i);

// tendon i has a damper: nonzero damping, damping polynomial, or an attached actuator
int mj_tendonHasDamping(const mjModel* m, int i);

// does flex f use the passive contact path (metric-carried contacts)
MJAPI int mj_effFlexContactPossible(const mjModel* m, int f);

// does flex f contribute elastic stiffness to the metric
int mj_effFlexStiffPossible(const mjModel* m, int f);

// does flex f need the implicit metric treatment: elastic stiffness or passive contact
MJAPI int mj_effFlexPossible(const mjModel* m, int f);

// can this tendon contribute to the metric (model-level; mirrored by island discovery
// and the sleep wake rule)
MJAPI int mj_effTendonPossible(const mjModel* m, int i);

// can this actuator contribute to the metric (model-level type check)
int mj_effActuatorPossible(const mjModel* m, int i);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_CORE_UTIL_H_
