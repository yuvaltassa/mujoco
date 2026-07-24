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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_GEOM_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_GEOM_UTIL_H_

#include <mujoco/mjrfilament.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/scene_objects.h"
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

// Creates a Renderable from the given mjvGeom.
UniquePtr<mjrfRenderable> CreateGeomRenderable(const mjvGeom& geom,
                                               mjrfContext* ctx,
                                               ModelObjects* model_objs,
                                               SceneObjects* scene_objs);

// Assigns the mesh implied by the geom (builtin shape, model asset, or scene
// flex/skin mesh) to the renderable, along with its size and transform.
void ApplyGeomMesh(mjrfRenderable* renderable, const mjvGeom& geom,
                   ModelObjects* model_objs, SceneObjects* scene_objs);

// Applies the geom's size and transform to the renderable.
void ApplyGeomPose(mjrfRenderable* renderable, const mjvGeom& geom);

// Resolves the geom's material against the model and applies it.
void ApplyGeomMaterial(mjrfRenderable* renderable, const mjvGeom& geom,
                       ModelObjects* model_objs);

// Bitmask returned by DiffGeom describing which renderable state must be
// re-applied when moving from one mjvGeom to another.
enum : int {
  kDiffGeomMesh = 1 << 0,      // shape or asset changed: ApplyGeomMesh
  kDiffGeomPose = 1 << 1,      // transform or size changed: ApplyGeomPose
  kDiffGeomMaterial = 1 << 2,  // material inputs changed: ApplyGeomMaterial
};

// Returns the set of changes needed to turn a renderable displaying geom a
// into one displaying geom b.
int DiffGeom(const mjvGeom& a, const mjvGeom& b);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_GEOM_UTIL_H_
