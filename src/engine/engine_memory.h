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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_MEMORY_H_
#define MUJOCO_SRC_ENGINE_ENGINE_MEMORY_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjtype.h>
#include <mujoco/mjxmacro.h>

#ifdef __cplusplus
#include <cstddef>
extern "C" {
#else
#include <stddef.h>
#endif

// internal hash map size factor (2 corresponds to a load factor of 0.5)
#define mjLOAD_MULTIPLE 2

// mjData arena allocate
MJAPI void* mj_arenaAllocByte(mjData* d, size_t bytes, size_t alignment);

#ifndef mjUSEASAN

// mjData mark stack frame
MJAPI void mj_markStack(mjData* d);

// mjData free stack frame
MJAPI void mj_freeStack(mjData* d);

#else

MJAPI void mj__markStack(mjData* d) __attribute__((noinline));
MJAPI void mj__freeStack(mjData* d) __attribute__((noinline));

#endif  // ADDRESS_SANITIZER

// allocate bytes on the stack
MJAPI void* mj_stackAllocByte(mjData* d, size_t bytes, size_t alignment);

// mark a checked frame: an engine allocation on it that overflows returns NULL and marks the frame
// failed instead of raising an error, so its owner checks mj_stackFailed after allocating and
// reports mjSTATUS_OOM; the public allocators raise an error on any frame
#ifndef mjUSEASAN
MJAPI void mj_markStackChecked(mjData* d);
#else
MJAPI void mj__markStackChecked(mjData* d) __attribute__((noinline));
__attribute__((always_inline))
static inline void mj_markStackChecked(mjData* d) {
  __asm__ volatile("" ::: "memory");
  mj__markStackChecked(d);
  __asm__ volatile("" ::: "memory");
}
#endif

// nonzero if an allocation on the current frame failed
MJAPI int mj_stackFailed(const mjData* d);

// after allocating on a checked frame: if an allocation failed, run cleanup and return retval;
// the status form frees the frame and records the status in the data, as a public call does
#define mjSTACKCHECK_(d, cleanup, retval)                                         \
  if (mj_stackFailed(d)) {                                                        \
    cleanup;                                                                      \
    return retval;                                                                \
  }
#define mjSTACKCHECK(d) mjSTACKCHECK_(d, mj_freeStack(d), (mjtStatus)((d)->status = mjSTATUS_OOM))

// allocate bytes on the stack, with added caller information
MJAPI void* mj_stackAllocInfo(mjData* d, size_t bytes, size_t alignment,
                              const char* caller, int line);

// macro to allocate a stack array of given type, adds caller information
#define mjSTACKALLOC(d, num, type) \
(type*) mj_stackAllocInfo(d, (num) * sizeof(type), _Alignof(type), __func__, __LINE__)

// mjData stack allocate for array of mjtNums
MJAPI mjtNum* mj_stackAllocNum(mjData* d, size_t size);

// mjData stack allocate for array of ints
MJAPI int* mj_stackAllocInt(mjData* d, size_t size);

// clear arena pointers in mjData
static inline void mj_clearEfc(mjData* d) {
#define X(type, name, nr, nc) d->name = NULL;
  MJDATA_ARENA_POINTERS
#undef X
  d->nefc = 0;
  d->nisland = 0;
  d->nJ = d->nY = d->nA = 0;

  // deactivate the effective metric: its arena pointers were cleared above, so the
  // activity flag and counts consumers gate on must clear with them
  d->efm_active = 0;
  d->nefmT = d->nefmA = d->nefmK = d->nefmL = d->nefmdof = d->nefmcon = 0;
  d->contact = (mjContact*) d->arena;

  // if any contacts are allocated, clear their efc_address
  for (int i=0; i < d->ncon; i++) {
    d->contact[i].efc_address = -1;
  }
}

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_MEMORY_H_
