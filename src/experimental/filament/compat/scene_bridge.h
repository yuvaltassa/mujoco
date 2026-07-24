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

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include <math/mat4.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/scene_objects.h"
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_objects.h"
#include "render/filament/support/light_manager.h"

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

  // Retained state for one renderable: the renderable persists across frames
  // and state is re-applied only when its mjvGeom changes, judged against a
  // shadow copy of the last-applied geom.
  struct Slot {
    UniquePtr<mjrfRenderable> renderable{nullptr, mjrf_destroyRenderable};
    mjvGeom shadow;
    bool in_scene = false;
  };

  // A model element's slot, claimed at most once per frame by its key.
  struct KeyedSlot : Slot {
    uint64_t last_seen = 0;
  };

  // Reusable renderables for geoms without cross-frame identity (decor,
  // appended/ghost geoms), pooled by (type, dataid) and claimed in
  // generation order. mjv generation order is deterministic, so stable scene
  // content reclaims the same slots with no re-application needed.
  struct Pool {
    std::vector<Slot> slots;
    size_t used = 0;
  };

  // Reusable renderables for flex vertex/edge visualization, which is built
  // from scene flex data rather than from mjvGeoms.
  struct SwarmSlot {
    UniquePtr<mjrfRenderable> renderable{nullptr, mjrf_destroyRenderable};
    filament::math::float4 color{0, 0, 0, -1};
    bool in_scene = false;
  };
  struct SwarmPool {
    std::vector<SwarmSlot> slots;
    size_t used = 0;
  };

  // Applies the geom to the slot, creating the renderable on first use.
  void ApplySlot(Slot& slot, const mjvGeom& geom, bool reapply_all);

  // Returns the next free slot in the geom's pool, growing it if needed.
  Slot& ClaimPoolSlot(const mjvGeom& geom);

  // Returns the next free slot in a swarm pool, growing it if needed; new
  // renderables are assigned a builtin mesh of the given type.
  SwarmSlot& ClaimSwarmSlot(SwarmPool& pool, int geom_type);

  // Applies pose and color to a swarm slot and ensures it is in the scene.
  void UpdateSwarmSlot(SwarmSlot& slot, const filament::math::float4& color,
                       const float* size, const float* pos, const float* rot);

  // Removes unclaimed pooled renderables from the scene and resets claim
  // counts for the next frame.
  void SweepPools();

  mjrfContext* ctx_ = nullptr;
  mjrfScene* scene_ = nullptr;
  std::unique_ptr<ModelObjects> model_objects_;
  std::unique_ptr<SceneObjects> scene_objects_;
  std::unique_ptr<LightManager> light_manager_;
  mjrCamera camera_;
  DrawTextAtFn draw_text_callback_;

  // Model elements, keyed by (objtype, objid); persist across frames.
  std::unordered_map<uint64_t, KeyedSlot> keyed_;
  // Everything else (decor, appended geoms), pooled by (type, dataid).
  std::unordered_map<uint64_t, Pool> pools_;
  SwarmPool flex_vert_pool_;
  SwarmPool flex_edge_pool_;

  uint64_t frame_ = 0;
  // Set when uploaded assets replace the underlying mesh/texture objects, so
  // retained renderable state must be re-resolved.
  bool reapply_all_ = false;
  filament::math::mat4 clip_from_world_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_BRIDGE_H_
