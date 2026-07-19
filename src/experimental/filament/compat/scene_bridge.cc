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

#include "experimental/filament/compat/scene_bridge.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include <math/TMatHelpers.h>
#include <math/TVecHelpers.h>
#include <math/mat4.h>
#include <math/mathfwd.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/scene_geom_util.h"
#include "experimental/filament/compat/scene_objects.h"
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/filament_util.h"
#include "render/filament/support/light_manager.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat3;
using filament::math::mat4;

SceneBridge::SceneBridge(mjrfContext* ctx, mjrfScene* scene, const mjModel* model)
    : ctx_(ctx), scene_(scene) {
  model_objects_ = std::make_unique<ModelObjects>(model, ctx_);
  scene_objects_ = std::make_unique<SceneObjects>(ctx_);

  mjrf_configureSceneFromModel(scene_, model);

  auto clear_color = ReadElement(model, "filament.clearColor",
                                 filament::math::float4(0, 0, 0, 1));
  mjrf_setClearColor(ctx_, &clear_color[0]);

  light_manager_ = std::make_unique<LightManager>(scene_, model_objects_.get());
}

SceneBridge::~SceneBridge() {
  light_manager_.reset();
  for (auto& [key, slot] : keyed_) {
    if (slot.in_scene) {
      mjrf_removeRenderableFromScene(scene_, slot.renderable.get());
    }
  }
  keyed_.clear();
  for (auto& [key, pool] : pools_) {
    for (Slot& slot : pool.slots) {
      if (slot.in_scene) {
        mjrf_removeRenderableFromScene(scene_, slot.renderable.get());
      }
    }
  }
  pools_.clear();
  for (SwarmPool* pool : {&flex_vert_pool_, &flex_edge_pool_}) {
    for (SwarmSlot& slot : pool->slots) {
      if (slot.in_scene) {
        mjrf_removeRenderableFromScene(scene_, slot.renderable.get());
      }
    }
  }
}

// Model elements have exact (objtype, objid) identity in the mjvScene; decor
// and appended (e.g. mjv_addGeoms ghost) geoms do not. The first occurrence
// per frame claims the keyed slot; duplicates take the per-frame path.
static bool IsKeyedGeom(const mjvGeom& geom) {
  if (geom.objid < 0 || geom.category == mjCAT_DECOR) {
    return false;
  }
  return geom.objtype == mjOBJ_GEOM || geom.objtype == mjOBJ_SITE ||
         geom.objtype == mjOBJ_FLEX || geom.objtype == mjOBJ_SKIN;
}

static uint64_t GeomKey(const mjvGeom& geom) {
  return (uint64_t(uint32_t(geom.objtype)) << 32) | uint32_t(geom.objid);
}

// Pooled renderables are reusable across geoms of the same shape and asset,
// so the pool key is exactly the mesh-determining fields.
static uint64_t PoolKey(const mjvGeom& geom) {
  return (uint64_t(uint32_t(geom.type)) << 32) | uint32_t(geom.dataid);
}

std::optional<float3> SceneBridge::ClipFromWorld(const float3& pos) const{
  const float4 clip_pos = clip_from_world_ * float4(pos, 1.0f);
  if (clip_pos.w == 0.0f) {
    return std::nullopt;
  }
  return clip_pos.xyz / clip_pos.w;
}

mat4 CalculateClipFromWorld(const mjrRect& viewport, const mjrCamera& cam) {
  const float3 cam_pos(cam.pos[0], cam.pos[1], cam.pos[2]);
  const float3 cam_fwd(cam.forward[0], cam.forward[1], cam.forward[2]);
  const float3 cam_up(cam.up[0], cam.up[1], cam.up[2]);
  const float3 cam_at = cam_pos + cam_fwd;
  const float aspect_ratio = (float)viewport.width / (float)viewport.height;
  const float halfwidth =
      cam.frustum_width
          ? cam.frustum_width
          : 0.5f * aspect_ratio * (cam.frustum_top - cam.frustum_bottom);
  const float left = cam.frustum_center - halfwidth;
  const float right = cam.frustum_center + halfwidth;

  mat4 projection;
  if (cam.orthographic) {
    projection = mat4::ortho(left, right, cam.frustum_bottom, cam.frustum_top,
                             cam.frustum_near, cam.frustum_far);
  } else {
    projection = mat4::frustum(left, right, cam.frustum_bottom, cam.frustum_top,
                               cam.frustum_near, cam.frustum_far);
    projection[2][2] = -1.0f;
    projection[3][2] = -2.0f * cam.frustum_near;
  }
  mat4 look_at = mat4::lookAt(cam_pos, cam_at, cam_up);
  return projection * inverse(look_at);
}

void SceneBridge::Update(const mjrRect& viewport, const mjvScene* scene) {
  const mjModel* model = model_objects_->GetModel();

  mjtNum hpos[3], hfwd[3];
  float headpos[3], gazedir[3];
  mjv_cameraInModel(hpos, hfwd, nullptr, scene);
  mju_n2f(headpos, hpos, 3);
  mju_n2f(gazedir, hfwd, 3);

  camera_ = mjv_averageCamera(scene->camera, scene->camera + 1);
  clip_from_world_ = CalculateClipFromWorld(viewport, camera_);

  ++frame_;
  const bool reapply_all = reapply_all_;
  reapply_all_ = false;

  for (int i = 0; i < scene->ngeom; ++i) {
    const mjvGeom* geom = scene->geoms + i;

    if (draw_text_callback_ && geom->label[0] != 0) {
      if (auto pos = ClipFromWorld(ReadFloat3(geom->pos))) {
        draw_text_callback_(geom->label, pos->x, pos->y, pos->z);
      }
    }

    // Draw flex edges and vertices as separate renderables.
    if (geom->type == mjGEOM_FLEX &&
        (!scene->flexskinopt || model->flex_dim[geom->objid] == 1)) {
      const float4 flex_color(model->flex_rgba[4 * geom->objid + 0],
                              model->flex_rgba[4 * geom->objid + 1],
                              model->flex_rgba[4 * geom->objid + 2],
                              model->flex_rgba[4 * geom->objid + 3]);

      const int vertadr = scene->flexvertadr[geom->objid];
      const int vertnum = scene->flexvertnum[geom->objid];
      const float radius = model->flex_radius[geom->objid];

      if (scene->flexvertopt) {
        // Use small spheres to represent vertices.
        const float rot[] = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
        const float size[] = {radius, radius, radius};
        for (int v = vertadr; v < vertadr + vertnum; ++v) {
          SwarmSlot& slot = ClaimSwarmSlot(flex_vert_pool_, mjGEOM_SPHERE);
          UpdateSwarmSlot(slot, flex_color, size, scene->flexvert + 3*v, rot);
        }
      }

      if (scene->flexedgeopt) {
        const int edgeadr = scene->flexedgeadr[geom->objid];
        const int edgenum = scene->flexedgenum[geom->objid];

        // Use small thin cylinders to represent the edges.
        for (int e = edgeadr; e < edgeadr + edgenum; ++e) {
          const float* v1 = scene->flexvert + 3 * (vertadr + scene->flexedge[2*e]);
          const float* v2 = scene->flexvert + 3 * (vertadr + scene->flexedge[2*e+1]);
          const mjtNum vec[3] = {v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]};

          const float pos[3]{
              (v1[0] + v2[0]) * 0.5f,
              (v1[1] + v2[1]) * 0.5f,
              (v1[2] + v2[2]) * 0.5f,
          };

          mjtNum quat[4];
          mju_quatZ2Vec(quat, vec);
          mjtNum edgemat[9];
          mju_quat2Mat(edgemat, quat);
          const float rot[9] = {
            static_cast<float>(edgemat[0]),
            static_cast<float>(edgemat[1]),
            static_cast<float>(edgemat[2]),
            static_cast<float>(edgemat[3]),
            static_cast<float>(edgemat[4]),
            static_cast<float>(edgemat[5]),
            static_cast<float>(edgemat[6]),
            static_cast<float>(edgemat[7]),
            static_cast<float>(edgemat[8]),
          };

          const float len = static_cast<float>(mju_norm3(vec));
          const float size[3] = {radius, radius, len * 0.5f};

          SwarmSlot& slot = ClaimSwarmSlot(flex_edge_pool_, mjGEOM_CYLINDER);
          UpdateSwarmSlot(slot, flex_color, size, pos, rot);
        }
      }
    }

    if (geom->type == mjGEOM_FLEX || geom->type == mjGEOM_SKIN) {
      if (!scene_objects_->CreateSkinFlexMesh(scene, model_objects_->GetModel(),
                                              *geom)) {
        continue;
      }
    }

    // Model elements are reconciled against their persistent renderable.
    if (IsKeyedGeom(*geom)) {
      KeyedSlot& slot = keyed_[GeomKey(*geom)];
      if (slot.last_seen != frame_) {
        ApplySlot(slot, *geom, reapply_all);
        slot.last_seen = frame_;
        continue;
      }
      // The key was already claimed this frame (a ghost copy of a model
      // element); fall through to the pooled path.
    }

    // Everything else (decor, appended geoms) claims a pooled renderable.
    ApplySlot(ClaimPoolSlot(*geom), *geom, reapply_all);
  }

  // Sweep: remove keyed renderables that were not claimed this frame, e.g.
  // geoms hidden by a group toggle, and unclaimed pooled renderables. All
  // renderables are kept for reuse.
  for (auto& [key, slot] : keyed_) {
    if (slot.last_seen != frame_ && slot.in_scene) {
      mjrf_removeRenderableFromScene(scene_, slot.renderable.get());
      slot.in_scene = false;
    }
  }
  SweepPools();

  mjrfLight* headlight = nullptr;
  bool headlight_enabled = false;
  for (int i = 0; i < scene->nlight; ++i) {
    const mjvLight& scene_light = scene->lights[i];
    if (scene_light.id < 0 && scene_light.headlight) {
      // The headlight, if it exists, is assigned the id `scene->nlight`.
      headlight = light_manager_->GetLight(scene->nlight);
      if (!headlight) {
        continue;
      }
      // We position the headlight slightly behind the camera to avoid some
      // odd clipping issues.
      headlight_enabled = true;
      headpos[0] -= gazedir[0] * 0.05f;
      headpos[1] -= gazedir[1] * 0.05f;
      headpos[2] -= gazedir[2] * 0.05f;

      mjrf_setLightColor(headlight, scene_light.diffuse);
      mjrf_setLightTransform(headlight, headpos, gazedir);
      continue;
    } else if (mjrfLight* light = light_manager_->GetLight(scene_light.id)) {
      mjrf_setLightColor(light, scene_light.diffuse);
      mjrf_setLightTransform(light, scene_light.pos, scene_light.dir);
    } else {
      mju_error("Unexpected light id: %d", scene_light.id);
    }
  }

  // Enable/disable the headlight based on whether or not it's in the scene.
  if (headlight) {
    mjrf_setLightEnabled(headlight, headlight_enabled);
  }
}

void SceneBridge::ApplySlot(Slot& slot, const mjvGeom& geom,
                            bool reapply_all) {
  if (!slot.renderable) {
    mjrfRenderableParams params;
    mjrf_defaultRenderableParams(&params);
    slot.renderable = CreateRenderable(ctx_, params);
    reapply_all = true;
  }
  mjrfRenderable* renderable = slot.renderable.get();

  const int diff = reapply_all
                       ? (kDiffGeomMesh | kDiffGeomPose | kDiffGeomMaterial)
                       : DiffGeom(slot.shadow, geom);

  // Flex and skin meshes are recreated every frame, so the mesh is
  // unconditionally re-assigned; ApplyGeomMesh also sets pose and size.
  const bool deformable =
      geom.type == mjGEOM_FLEX || geom.type == mjGEOM_SKIN;
  if (deformable) {
    ApplyGeomMesh(renderable, geom, model_objects_.get(),
                  scene_objects_.get());
    if (diff & (kDiffGeomMesh | kDiffGeomMaterial)) {
      ApplyGeomMaterial(renderable, geom, model_objects_.get());
    }
  } else if (diff & kDiffGeomMesh) {
    ApplyGeomMesh(renderable, geom, model_objects_.get(),
                  scene_objects_.get());
    ApplyGeomMaterial(renderable, geom, model_objects_.get());
  } else {
    if (diff & kDiffGeomPose) {
      ApplyGeomPose(renderable, geom);
    }
    if (diff & kDiffGeomMaterial) {
      ApplyGeomMaterial(renderable, geom, model_objects_.get());
    }
  }

  slot.shadow = geom;
  if (!slot.in_scene) {
    mjrf_addRenderableToScene(scene_, renderable);
    slot.in_scene = true;
  }
}

SceneBridge::Slot& SceneBridge::ClaimPoolSlot(const mjvGeom& geom) {
  Pool& pool = pools_[PoolKey(geom)];
  if (pool.used == pool.slots.size()) {
    pool.slots.emplace_back();
  }
  return pool.slots[pool.used++];
}

SceneBridge::SwarmSlot& SceneBridge::ClaimSwarmSlot(SwarmPool& pool,
                                                    int geom_type) {
  if (pool.used == pool.slots.size()) {
    SwarmSlot& slot = pool.slots.emplace_back();
    mjrfRenderableParams params;
    mjrf_defaultRenderableParams(&params);
    slot.renderable = CreateRenderable(ctx_, params);
    mjrf_setRenderableGeomMesh(slot.renderable.get(), geom_type, 3, 3, 3);
  }
  return pool.slots[pool.used++];
}

void SceneBridge::UpdateSwarmSlot(SwarmSlot& slot, const float4& color,
                                  const float* size, const float* pos,
                                  const float* rot) {
  mjrf_setRenderableSize(slot.renderable.get(), size);
  mjrf_setRenderableTransform(slot.renderable.get(), pos, rot);
  if (slot.color != color) {
    mjrfMaterial material;
    mjrf_defaultMaterial(&material);
    material.color[0] = color.x;
    material.color[1] = color.y;
    material.color[2] = color.z;
    material.color[3] = color.w;
    mjrf_setRenderableMaterial(slot.renderable.get(), &material);
    slot.color = color;
  }
  if (!slot.in_scene) {
    mjrf_addRenderableToScene(scene_, slot.renderable.get());
    slot.in_scene = true;
  }
}

void SceneBridge::SweepPools() {
  auto sweep = [this](auto& pool) {
    for (size_t i = pool.used; i < pool.slots.size(); ++i) {
      auto& slot = pool.slots[i];
      if (slot.in_scene) {
        mjrf_removeRenderableFromScene(scene_, slot.renderable.get());
        slot.in_scene = false;
      }
    }
    pool.used = 0;
  };
  for (auto& [key, pool] : pools_) {
    sweep(pool);
  }
  sweep(flex_vert_pool_);
  sweep(flex_edge_pool_);
}

void SceneBridge::UploadMesh(const mjModel* model, int id) {
  model_objects_->UploadMesh(model, id);
  reapply_all_ = true;
}

void SceneBridge::UploadTexture(const mjModel* model, int id) {
  model_objects_->UploadTexture(model, id);
  reapply_all_ = true;
}

void SceneBridge::UploadHeightField(const mjModel* model, int id) {
  model_objects_->UploadHeightField(model, id);
  reapply_all_ = true;
}

void SceneBridge::SetDrawTextFunction(DrawTextAtFn fn) {
  draw_text_callback_ = std::move(fn);
}

mjrCamera SceneBridge::GetCamera() const { return camera_; }

}  // namespace mujoco
