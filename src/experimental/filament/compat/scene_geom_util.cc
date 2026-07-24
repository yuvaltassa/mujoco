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

#include "experimental/filament/compat/scene_geom_util.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include <mujoco/mjrfilament.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/scene_objects.h"
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

// Returns the tile size for infinite plane texture alignment.
// This is duplicated from engine_vis_visualize.c (re-center infinite plane)
// to ensure UV scaling matches the re-centering increments.
static float GetPlaneTileSize(const mjModel* model, int matid,
                              float texrepeat) {
  if (matid >= 0 && texrepeat > 0) {
    return 2.0f / texrepeat;
  } else {
    const float zfar = model->vis.map.zfar * model->stat.extent;
    return 2.1f * zfar / (mjMAXPLANEGRID - 2);
  }
}

void ApplyGeomMesh(mjrfRenderable* renderable, const mjvGeom& geom,
                   ModelObjects* model_objs, SceneObjects* scene_objs) {
  const mjModel* model = model_objs->GetModel();
  const int nstack = model->vis.quality.numstacks;
  const int nslice = model->vis.quality.numslices;
  const int nquad = model->vis.quality.numquads;

  float position[3];
  std::memcpy(position, &geom.pos, 3 * sizeof(float));
  float rotation[9];
  std::memcpy(rotation, &geom.mat, 9 * sizeof(float));

  const mjtGeom geom_type = (mjtGeom)geom.type;
  switch (geom_type) {
    case mjGEOM_MESH:
    case mjGEOM_SDF:
      mjrf_setRenderableMesh(renderable, model_objs->GetMesh(geom.dataid), 0, 0);
      break;
    case mjGEOM_HFIELD:
      mjrf_setRenderableMesh(renderable, model_objs->GetHeightField(geom.dataid), 0, 0);
      break;
    case mjGEOM_PLANE: {
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      // Planes only define an xy size, so set the z-dimension to 1.0f.
      const float size[3] = {geom.size[0], geom.size[1], 1.0f};
      mjrf_setRenderableSize(renderable, size);
      break;
    }
    case mjGEOM_SPHERE:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_ELLIPSOID:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_BOX:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_CAPSULE:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_CYLINDER:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_ARROW:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_ARROW1:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_ARROW2:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_LINE:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_LINEBOX:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_TRIANGLE:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    case mjGEOM_FLEX:
      // The flex mesh is updated in place; draw only the active faces.
      mjrf_setRenderableMesh(renderable, scene_objs->GetFlexMesh(geom.objid), 0,
                             scene_objs->GetFlexIndexCount(geom.objid));
      // Flexes are defined in global space.
      std::memset(position, 0, sizeof(position));
      std::memset(rotation, 0, sizeof(rotation));
      rotation[0] = 1.f;
      rotation[4] = 1.f;
      rotation[8] = 1.f;
      break;
    case mjGEOM_SKIN:
      mjrf_setRenderableMesh(renderable, scene_objs->GetSkinMesh(geom.objid), 0, 0);
      // Skins are defined in global space.
      std::memset(position, 0, sizeof(position));
      std::memset(rotation, 0, sizeof(rotation));
      rotation[0] = 1.f;
      rotation[4] = 1.f;
      rotation[8] = 1.f;
      break;
    case mjGEOM_NONE:
    case mjGEOM_LABEL:
      // Do nothing.
      break;
    case mjNGEOMTYPES:
      mju_warning("Unsupported geom type: %d", geom.type);
      break;
  }

  mjrf_setRenderableTransform(renderable, position, rotation);
}

void ApplyGeomPose(mjrfRenderable* renderable, const mjvGeom& geom) {
  const mjtGeom geom_type = (mjtGeom)geom.type;
  switch (geom_type) {
    case mjGEOM_FLEX:
    case mjGEOM_SKIN:
      // Vertices are in global space; the identity transform is set when the
      // mesh is (re)assigned by ApplyGeomMesh.
      return;
    case mjGEOM_PLANE: {
      // Planes only define an xy size, so set the z-dimension to 1.0f.
      const float size[3] = {geom.size[0], geom.size[1], 1.0f};
      mjrf_setRenderableSize(renderable, size);
      break;
    }
    case mjGEOM_SPHERE:
    case mjGEOM_ELLIPSOID:
    case mjGEOM_BOX:
    case mjGEOM_CAPSULE:
    case mjGEOM_CYLINDER:
    case mjGEOM_ARROW:
    case mjGEOM_ARROW1:
    case mjGEOM_ARROW2:
    case mjGEOM_LINE:
    case mjGEOM_LINEBOX:
    case mjGEOM_TRIANGLE:
      mjrf_setRenderableSize(renderable, geom.size);
      break;
    default:
      // Mesh-like geoms (mesh, sdf, hfield) bake their size into vertices.
      break;
  }
  mjrf_setRenderableTransform(renderable, geom.pos, geom.mat);
}

int DiffGeom(const mjvGeom& a, const mjvGeom& b) {
  int diff = 0;

  // objtype/objid are provenance and do not affect rendering; label, camdist,
  // modelrbound and transparent are not consumed by this renderer.

  // Shape and asset identity: requires a mesh rebind, which may also change
  // the material variant (e.g. availability of uv coordinates).
  if (a.type != b.type || a.dataid != b.dataid) {
    diff |= kDiffGeomMesh | kDiffGeomMaterial;
  }

  // Pose fields; size also feeds texture uv scaling.
  if (std::memcmp(a.pos, b.pos, sizeof(a.pos)) ||
      std::memcmp(a.mat, b.mat, sizeof(a.mat))) {
    diff |= kDiffGeomPose;
  }
  if (std::memcmp(a.size, b.size, sizeof(a.size))) {
    diff |= kDiffGeomPose | kDiffGeomMaterial;
  }

  // Material inputs.
  if (a.category != b.category || a.matid != b.matid || a.texid != b.texid ||
      a.texuniform != b.texuniform || a.texcoord != b.texcoord ||
      a.segid != b.segid ||
      std::memcmp(a.rgba, b.rgba, sizeof(a.rgba)) ||
      a.emission != b.emission || a.specular != b.specular ||
      a.shininess != b.shininess || a.reflectance != b.reflectance ||
      std::memcmp(a.texrepeat, b.texrepeat, sizeof(a.texrepeat))) {
    diff |= kDiffGeomMaterial;
  }

  return diff;
}

void ApplyGeomMaterial(mjrfRenderable* renderable, const mjvGeom& geom,
                       ModelObjects* model_objs) {
  const mjModel* model = model_objs->GetModel();

  mjrfMaterial material;
  mjrf_defaultMaterial(&material);

  if (geom.category == mjCAT_DECOR) {
    material.decor_ux = true;
  }

  material.color[0] = geom.rgba[0];
  material.color[1] = geom.rgba[1];
  material.color[2] = geom.rgba[2];
  material.color[3] = geom.rgba[3];

  if (geom.matid >= 0 && geom.matid < model->nmat) {
    auto get_texture = [&](int role) -> const mjrfTexture* {
      const int tex_id = model->mat_texid[geom.matid * mjNTEXROLE + role];
      return tex_id >= 0 ? model_objs->GetTexture(tex_id) : nullptr;
    };
    material.color_texture = get_texture(mjTEXROLE_RGB);
    material.normal_texture = get_texture(mjTEXROLE_NORMAL);
    material.opacity_texture = get_texture(mjTEXROLE_OPACITY);
    material.emissive_texture = get_texture(mjTEXROLE_EMISSIVE);
    material.orm_texture = get_texture(mjTEXROLE_ORM);
    material.metallic_texture = get_texture(mjTEXROLE_METALLIC);
    material.roughness_texture = get_texture(mjTEXROLE_ROUGHNESS);
    material.occlusion_texture = get_texture(mjTEXROLE_OCCLUSION);
  }
  if (geom.texid >= 0) {
    material.color_texture = model_objs->GetTexture(geom.texid);
  }

  material.reflectance = geom.reflectance;
  material.emissive = geom.emission;
  material.specular = geom.specular;
  material.glossiness = geom.shininess;
  if (geom.matid >= 0) {
    material.metallic = model->mat_metallic[geom.matid];
    material.roughness = model->mat_roughness[geom.matid];
  }

  material.segmentation_id = geom.segid;

  // Assume an emissive object is a selected object.
  if (geom.emission > 0 && geom.emission == model->vis.global.glow) {
    material.selected = true;
    material.emissive = 0.0f;
  }

  // UvScale only applies to objects that don't have explicit UV coordinates
  // in their vertex buffer. Instead, we set the UV coordinate to be the same
  // as the vertex position.
  //
  // The material's `texuniform` and `texrepeat` parameters allow us to scale
  // the programmatic UVs.

  if (material.color_texture) {
    bool tex_uniform;
    float tex_repeat[2];
    if (geom.texid >= 0) {
      tex_uniform = geom.texuniform;
      tex_repeat[0] = geom.texrepeat[0];
      tex_repeat[1] = geom.texrepeat[1];
    } else {
      tex_uniform = model->mat_texuniform[geom.matid];
      tex_repeat[0] = model->mat_texrepeat[(geom.matid * 2) + 0];
      tex_repeat[1] = model->mat_texrepeat[(geom.matid * 2) + 1];
    }
    if (mjrf_getTextureSamplerType(material.color_texture) == mjTEXTURE_2D) {
      // For 2D textures, `tex_repeat` specifies how many times the texture
      // image is repeated. The `tex_uniform` flag determines if the repetition
      // is applied at in object space (false) or in world space (true).
      material.uv_scale[0] = tex_repeat[0];
      material.uv_scale[1] = tex_repeat[1];

      if (geom.dataid >= 0 && geom.type != mjGEOM_PLANE) {
        if (geom.size[0] > mjMINVAL) {
          material.uv_scale[0] /= geom.size[0];
        }
        if (geom.size[1] > mjMINVAL) {
          material.uv_scale[1] /= geom.size[1];
        }
      }

      if (tex_uniform) {
        if (geom.size[0] > 0) {
          material.uv_scale[0] *= geom.size[0];
        }
        if (geom.size[1] > 0) {
          material.uv_scale[1] *= geom.size[1];
        }
      }
      const bool is_infinite_plane =
          geom.type == mjGEOM_PLANE && (geom.size[0] <= 0 || geom.size[1] <= 0);
      if (is_infinite_plane) {
        // Infinite planes are scaled to match the tile size used by
        // re-centering in engine_vis_visualize.c.
        const float plane_scale = static_cast<float>(mjMAXPLANEGRID) / 2.0f;
        const float tile_size_x =
            GetPlaneTileSize(model, geom.matid, tex_repeat[0]);
        const float tile_size_y =
            GetPlaneTileSize(model, geom.matid, tex_repeat[1]);
        material.uv_scale[0] = 2.0f * plane_scale / tile_size_x;
        material.uv_scale[1] = 2.0f * plane_scale / tile_size_y;
      }

      // We want to do the equivalent of:
      //   mjr_setf4(splane, 0.5 * scl.x,  0, 0, -0.5);
      //   mjr_setf4(tplane, 0, -0.5 * scl.y, 0, -0.5);
      //   glTexGenfv(GL_S, GL_OBJECT_PLANE, splane);
      //   glTexGenfv(GL_T, GL_OBJECT_PLANE, tplane);
      material.uv_scale[0] = 0.5f * material.uv_scale[0];
      material.uv_scale[1] = -0.5f * material.uv_scale[1];
      material.uv_offset[0] = -0.5f;
      material.uv_offset[1] = -0.5f;
    } else {
      // For cube maps, if `tex_uniform` is true, then scale the texture so that
      // it covers a 1x1 area of world space rather than the area of the object.
      if (tex_uniform) {
        material.uv_scale[0] = 1.0f / (geom.size[0] ? geom.size[0] : 1.0f);
        material.uv_scale[1] = 1.0f / (geom.size[1] ? geom.size[1] : 1.0f);
        material.uv_scale[2] = 1.0f / (geom.size[2] ? geom.size[2] : 1.0f);
      }
    }
  }

  // Apply material multipliers from the model.
  material.emissive *= model_objs->GetEmissiveMultiplier();
  material.specular *= model_objs->GetSpecularMultiplier();
  material.glossiness *= model_objs->GetShininessMultiplier();

  mjrf_setRenderableMaterial(renderable, &material);
}

UniquePtr<mjrfRenderable> CreateGeomRenderable(const mjvGeom& geom,
                                               mjrfContext* ctx,
                                               ModelObjects* model_objs,
                                               SceneObjects* scene_objs) {
  mjrfRenderableParams params;
  mjrf_defaultRenderableParams(&params);
  auto renderable = CreateRenderable(ctx, params);
  ApplyGeomMesh(renderable.get(), geom, model_objs, scene_objs);
  ApplyGeomMaterial(renderable.get(), geom, model_objs);
  return renderable;
}
}  // namespace mujoco
