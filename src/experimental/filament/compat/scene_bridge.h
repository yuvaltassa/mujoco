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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_BRIDGE_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_BRIDGE_H_

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <math/mat4.h>
#include <math/vec3.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/scene_objects.h"
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_lights.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

// Manages all mjModel data and updates a SceneView using an mjvScene.
class SceneBridge {
 public:
  SceneBridge(mjrfContext* ctx, mjrfScene* scene, const mjModel* model);
  ~SceneBridge();

  // Updates the Entities in the filament Scene to match the current mjvScene
  // state.
  void Update(const mjrRect& viewport, const mjvScene* scene);

  // Creates the filament objects from the mjModel.
  void UploadMesh(const mjModel* model, int id);
  void UploadTexture(const mjModel* model, int id);
  void UploadHeightField(const mjModel* model, int id);

  using DrawTextAtFn = std::function<void(const char*, float, float, float)>;
  void SetDrawTextFunction(DrawTextAtFn fn);

  // Returns the camera used for rendering the scene.
  mjrCamera GetCamera() const;

  SceneBridge(const SceneBridge&) = delete;
  SceneBridge& operator=(const SceneBridge&) = delete;

 private:
  // Converts a point in world space to clip space, eg. in the range [-1,-1, 0]
  // to [1, 1, 1]. Returns std::nullopt if the point is behind the camera.
  std::optional<filament::math::float3> ClipFromWorld(
      const filament::math::float3& pos) const;

  // Retained-scene path (see mjv_syncScene): the producer's change list
  // drives per-entry application; the bridge keeps no bookkeeping beyond one
  // renderable per scene position.
  struct RetainedEntry {
    UniquePtr<mjrfRenderable> renderable{nullptr, mjrf_destroyRenderable};
    int type = -1;  // mjtGeom of the last application; re-typing recreates
    bool has_mesh = false;  // builtin meshes may be assigned only once
    bool in_scene = false;
  };

  // Applies the change list of a retained scene.
  void UpdateRetained(const mjvScene* scene);

  // Applies changed state of the scene entry at idx to its renderable.
  void ApplyRetainedEntry(int idx, const mjvScene* scene, int bits);

  // Updates filament lights from the scene's light list.
  void UpdateLights(const mjvScene* scene);

  mjrfContext* ctx_ = nullptr;
  mjrfScene* scene_ = nullptr;
  std::unique_ptr<ModelObjects> model_objects_;
  std::unique_ptr<SceneObjects> scene_objects_;
  std::unique_ptr<ModelLights> model_lights_;
  mjrCamera camera_;
  DrawTextAtFn draw_text_callback_;
  std::vector<UniquePtr<mjrfRenderable>> renderables_;
  std::vector<RetainedEntry> retained_;
  int retained_ngeom_ = 0;
  filament::math::mat4 clip_from_world_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_BRIDGE_H_
