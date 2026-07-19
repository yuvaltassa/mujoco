// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_OBJECTS_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_OBJECTS_H_

#include <unordered_map>

#include <mujoco/mjmodel.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/mjrfilament_cpp.h"

namespace mujoco {

// Creates and owns meshes read from an mjvScene.
class SceneObjects {
 public:
  explicit SceneObjects(mjrfContext* ctx);

  // Creates or updates the skin or flex mesh for the given geom from the
  // mjvScene. The mesh is created on first use (or when the frame's data
  // outgrows its buffers) and updated in place otherwise.
  bool UpdateSkinFlexMesh(const mjvScene* scene, const mjModel* model,
                          const mjvGeom& geom);

  // Returns the mesh for the given geom id, as set by UpdateSkinFlexMesh.
  const mjrfMesh* GetSkinMesh(int geom_id) const;
  const mjrfMesh* GetFlexMesh(int geom_id) const;

  // Returns the number of active indices in the flex mesh; indices past this
  // count belong to earlier frames with more faces and must not be drawn.
  int GetFlexIndexCount(int geom_id) const;

  SceneObjects(const SceneObjects&) = delete;
  SceneObjects& operator=(const SceneObjects&) = delete;

 private:
  // A mesh rebuilt from per-frame simulation output, updated in place.
  struct DynamicMesh {
    UniquePtr<mjrfMesh> mesh{nullptr, mjrf_destroyMesh};
    mjtSize vertex_capacity = 0;
    int num_attributes = 0;
    mjtSize num_indices = 0;
  };

  mjrfContext* ctx_ = nullptr;
  std::unordered_map<int, DynamicMesh> skins_;
  std::unordered_map<int, DynamicMesh> flexes_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_OBJECTS_H_
