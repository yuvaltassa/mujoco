// Copyright 2024 DeepMind Technologies Limited
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

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

static const char* const kModelPath = "testdata/model.xml";

class MjvSceneTest : public MujocoTest {
 protected:
  static constexpr int kMaxGeom = 10000;

  void InitSceneObjects(mjModel* model, int maxgeom = kMaxGeom) {
    mjv_defaultScene(&scn_);
    mjv_makeScene(model, &scn_, maxgeom);
    mjv_defaultOption(&opt_);
    mjv_defaultPerturb(&pert_);
    mjv_defaultFreeCamera(model, &cam_);

    // enable flags to exercise additional code paths
    for (int i = 0; i < mjNVISFLAG; ++i) {
      opt_.flags[i] = 1;
    }
  }

  void FreeSceneObjects() { mjv_freeScene(&scn_); }

  mjvScene scn_;
  mjvOption opt_;
  mjvPerturb pert_;
  mjvCamera cam_;
};

TEST_F(MjvSceneTest, UpdateScene) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull()) << "Failed to load model from " << kModelPath;

  InitSceneObjects(model);

  mjData* data = mj_makeData(model);
  while (data->time < .2) {
    mj_step(model, data);
  }

  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
  EXPECT_EQ(scn_.status, 0);
  EXPECT_GT(scn_.ngeom, 0);
  EXPECT_GT(scn_.nlight, 0);
  if (model->nskin) EXPECT_GT(scn_.nskin, 0);
  if (model->nflex) EXPECT_GT(scn_.nflex, 0);

  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);

  // call mj_copyData to expose any memory leaks in mjv_updateScene
  mjData* data_copy = mj_copyData(nullptr, model, data);

  mj_deleteData(data_copy);
  mj_deleteData(data);
  FreeSceneObjects();
  mj_deleteModel(model);
}

TEST_F(MjvSceneTest, MakeRetainedScene) {
  static constexpr char kFlexSkinPath[] = "engine/testdata/skingroup.xml";
  const std::string xml_path = GetTestDataFilePath(kFlexSkinPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull());

  // request retained mode between defaultScene and makeScene
  mjv_defaultScene(&scn_);
  scn_.retained = 1;
  mjv_makeScene(model, &scn_, kMaxGeom);

  // one slot per model element, all present but invisible before first sync
  int nslot = model->ngeom + model->nsite + model->nflex + model->nskin;
  EXPECT_EQ(scn_.nslot, nslot);
  EXPECT_EQ(scn_.ngeom, nslot);
  EXPECT_EQ(scn_.nchanged, 0);
  for (int k = 0; k < nslot; k++) {
    EXPECT_EQ(scn_.visible[k], 0);
    EXPECT_EQ(scn_.geoms[k].segid, k);
  }

  // slots are self-describing: geoms, sites, flexes, skins in model order
  EXPECT_EQ(scn_.geoms[0].objtype, mjOBJ_GEOM);
  EXPECT_EQ(scn_.geoms[0].objid, 0);
  int flexbase = model->ngeom + model->nsite;
  EXPECT_EQ(scn_.geoms[flexbase].objtype, mjOBJ_FLEX);
  EXPECT_EQ(scn_.geoms[flexbase].objid, 0);
  EXPECT_EQ(scn_.geoms[flexbase + model->nflex].objtype, mjOBJ_SKIN);

  // immediate-mode scenes are unaffected: remake without the request
  mjv_freeScene(&scn_);
  mjv_defaultScene(&scn_);
  mjv_makeScene(model, &scn_, kMaxGeom);
  EXPECT_EQ(scn_.retained, 0);
  EXPECT_EQ(scn_.nslot, 0);
  EXPECT_EQ(scn_.ngeom, 0);

  mjv_freeScene(&scn_);
  mj_deleteModel(model);
}

TEST_F(MjvSceneTest, UpdateSceneFlexSkin) {
  static constexpr char kFlexSkinPath[] = "engine/testdata/skingroup.xml";
  const std::string xml_path = GetTestDataFilePath(kFlexSkinPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull()) << "Failed to load model from "
                                << kFlexSkinPath;
  ASSERT_EQ(model->nflex, 2);  // in groups 2 and 4
  ASSERT_EQ(model->nskin, 3);  // in groups 0, 2 and 4 (flexcomps make skins)

  InitSceneObjects(model);
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // count per-element summary geoms of the given objtype
  auto count = [&](int objtype) {
    int n = 0;
    for (int i = 0; i < scn_.ngeom; i++) {
      if (scn_.geoms[i].objtype == objtype) {
        n++;
        EXPECT_EQ(scn_.geoms[i].category, mjCAT_DYNAMIC);
      }
    }
    return n;
  };

  // default groups enable 0-2: one flex and two skins are drawn
  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
  EXPECT_EQ(count(mjOBJ_FLEX), 1);
  EXPECT_EQ(count(mjOBJ_SKIN), 2);

  // enabling group 4 draws the second flex
  opt_.flexgroup[4] = 1;
  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
  EXPECT_EQ(count(mjOBJ_FLEX), 2);

  // disabling skin visualization removes the skin
  opt_.flags[mjVIS_SKIN] = 0;
  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
  EXPECT_EQ(count(mjOBJ_SKIN), 0);

  mj_deleteData(data);
  FreeSceneObjects();
  mj_deleteModel(model);
}

TEST_F(MjvSceneTest, SyncSceneEquivalence) {
  static const char* const kPaths[] = {"testdata/model.xml",
                                       "engine/testdata/skingroup.xml"};
  for (const char* path : kPaths) {
    const std::string xml_path = GetTestDataFilePath(path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
    ASSERT_THAT(model, NotNull()) << path;
    mjData* data = mj_makeData(model);
    while (data->time < .2) {
      mj_step(model, data);
    }

    // immediate scene (fixture members) and retained scene with its own camera
    InitSceneObjects(model);
    mjvScene retained;
    mjv_defaultScene(&retained);
    retained.retained = 1;
    mjv_makeScene(model, &retained, kMaxGeom);
    mjvCamera cam2 = cam_;

    // settle both twice so infinite-plane re-centering sees the same camera
    for (int it = 0; it < 2; it++) {
      mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
      mjv_syncScene(model, data, &opt_, &pert_, &cam2, mjCAT_ALL, &retained);
    }

    auto expect_equal = [&](const mjvGeom& a, const mjvGeom& b,
                            const std::string& ctx) {
      EXPECT_EQ(a.type, b.type) << ctx;
      EXPECT_EQ(a.dataid, b.dataid) << ctx;
      EXPECT_EQ(a.category, b.category) << ctx;
      EXPECT_EQ(a.matid, b.matid) << ctx;
      EXPECT_EQ(a.texid, b.texid) << ctx;
      EXPECT_EQ(a.texuniform, b.texuniform) << ctx;
      EXPECT_EQ(a.texcoord, b.texcoord) << ctx;
      EXPECT_EQ(std::memcmp(a.size, b.size, sizeof(a.size)), 0) << ctx;
      EXPECT_EQ(std::memcmp(a.pos, b.pos, sizeof(a.pos)), 0) << ctx;
      EXPECT_EQ(std::memcmp(a.mat, b.mat, sizeof(a.mat)), 0) << ctx;
      EXPECT_EQ(std::memcmp(a.rgba, b.rgba, sizeof(a.rgba)), 0) << ctx;
      EXPECT_EQ(std::memcmp(a.texrepeat, b.texrepeat, sizeof(a.texrepeat)), 0)
          << ctx;
      EXPECT_EQ(a.emission, b.emission) << ctx;
      EXPECT_EQ(a.specular, b.specular) << ctx;
      EXPECT_EQ(a.shininess, b.shininess) << ctx;
      EXPECT_EQ(a.reflectance, b.reflectance) << ctx;
      EXPECT_STREQ(a.label, b.label) << ctx;
    };

    // every model-element geom in the immediate scene matches its slot
    const int base[4] = {
        0, static_cast<int>(model->ngeom),
        static_cast<int>(model->ngeom + model->nsite),
        static_cast<int>(model->ngeom + model->nsite + model->nflex)};
    int nmatched = 0;
    for (int i = 0; i < scn_.ngeom; i++) {
      const mjvGeom& a = scn_.geoms[i];
      int slot = -1;
      switch (a.objtype) {
        case mjOBJ_GEOM: slot = base[0] + a.objid; break;
        case mjOBJ_SITE: slot = base[1] + a.objid; break;
        case mjOBJ_FLEX: slot = base[2] + a.objid; break;
        case mjOBJ_SKIN: slot = base[3] + a.objid; break;
        default: continue;  // decor
      }
      std::string ctx = std::string(path) + " objtype " +
                        std::to_string(a.objtype) + " objid " +
                        std::to_string(a.objid);
      ASSERT_TRUE(retained.visible[slot]) << ctx;
      expect_equal(a, retained.geoms[slot], ctx);
      nmatched++;
    }

    // and no other slots are visible
    int nvisible = 0;
    for (int k = 0; k < retained.nslot; k++) {
      nvisible += retained.visible[k];
    }
    EXPECT_EQ(nvisible, nmatched) << path;

    // the arena equals the immediate scene's non-model-element subsequence
    int k = retained.nslot;
    for (int i = 0; i < scn_.ngeom; i++) {
      const mjvGeom& a = scn_.geoms[i];
      if (a.objtype == mjOBJ_GEOM || a.objtype == mjOBJ_SITE ||
          a.objtype == mjOBJ_FLEX || a.objtype == mjOBJ_SKIN) {
        continue;
      }
      ASSERT_LT(k, retained.ngeom) << path;
      expect_equal(a, retained.geoms[k],
                   std::string(path) + " arena entry " + std::to_string(k));
      k++;
    }
    EXPECT_EQ(k, retained.ngeom) << path;

    mjv_freeScene(&retained);
    mj_deleteData(data);
    FreeSceneObjects();
    mj_deleteModel(model);
  }
}

TEST_F(MjvSceneTest, SyncSceneIncrementality) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);
  while (data->time < .2) {
    mj_step(model, data);
  }

  InitSceneObjects(model);
  opt_.flags[mjVIS_ISLAND] = 0;  // island colors depend on the contact set

  mjvScene retained;
  mjv_defaultScene(&retained);
  retained.retained = 1;
  mjv_makeScene(model, &retained, kMaxGeom);

  // settle: second sync sees the settled camera
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);

  // unchanged state: empty change list
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  EXPECT_EQ(retained.nchanged, 0);

  // move one dof: some but not all entries change; changed slots have the
  // pose bit (arena entries are unconstrained: insertions carry full bits)
  data->qpos[0] += 0.01;
  mj_forward(model, data);
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  EXPECT_GT(retained.nchanged, 0);
  EXPECT_LT(retained.nchanged, retained.ngeom);
  for (int k = 0; k < retained.nchanged; k++) {
    if (retained.changed[k] < retained.nslot) {
      EXPECT_TRUE(retained.changebits[k] & mjSYNC_POSE) << k;
    }
  }

  // disable a geom group: slot changes are pure visibility, content untouched
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  opt_.geomgroup[0] = 0;
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  int nhidden = 0;
  for (int k = 0; k < retained.nchanged; k++) {
    if (retained.changed[k] < retained.nslot) {
      EXPECT_EQ(retained.changebits[k], mjSYNC_VISIBLE) << k;
      EXPECT_EQ(retained.visible[retained.changed[k]], 0) << k;
      nhidden++;
    }
  }
  EXPECT_GT(nhidden, 0);

  // re-enable: the same slots reappear, marked for full re-application
  opt_.geomgroup[0] = 1;
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  int nshown = 0;
  for (int k = 0; k < retained.nchanged; k++) {
    if (retained.changed[k] < retained.nslot) {
      EXPECT_EQ(retained.changebits[k], mjSYNC_VISIBLE | mjSYNC_MESH |
                                            mjSYNC_POSE | mjSYNC_MATERIAL)
          << k;
      EXPECT_EQ(retained.visible[retained.changed[k]], 1) << k;
      nshown++;
    }
  }
  EXPECT_EQ(nshown, nhidden);

  mjv_freeScene(&retained);
  mj_deleteData(data);
  FreeSceneObjects();
  mj_deleteModel(model);
}

TEST_F(MjvSceneTest, SyncSceneArena) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);
  while (data->time < .2) {
    mj_step(model, data);
  }

  InitSceneObjects(model);
  opt_.flags[mjVIS_ISLAND] = 0;  // island colors depend on the contact set

  mjvScene retained;
  mjv_defaultScene(&retained);
  retained.retained = 1;
  mjv_makeScene(model, &retained, kMaxGeom);

  // settle, then confirm decor is present and stable: no changes at all
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  EXPECT_EQ(retained.nchanged, 0);
  int narena = retained.ngeom - retained.nslot;
  EXPECT_GT(narena, 0);  // the all-flags fixture produces decor

  // moving a dof changes arena entries too
  data->qpos[0] += 0.01;
  mj_forward(model, data);
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  bool arena_changed = false;
  for (int k = 0; k < retained.nchanged; k++) {
    arena_changed |= retained.changed[k] >= retained.nslot;
  }
  EXPECT_TRUE(arena_changed);

  // appends after a sync are the appender's to apply: no automatic change
  // entries, and no trace in the next sync's diff
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  narena = retained.ngeom - retained.nslot;
  int ngeom_before = retained.ngeom;
  int nchanged_before = retained.nchanged;
  mjv_addGeoms(model, data, &opt_, nullptr, mjCAT_DYNAMIC, &retained);
  EXPECT_GT(retained.ngeom, ngeom_before);        // ghost landed in the arena
  EXPECT_EQ(retained.nchanged, nchanged_before);  // no automatic entries
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  EXPECT_EQ(retained.ngeom - retained.nslot, narena);  // ghost vanished
  EXPECT_EQ(retained.nchanged, 0);                     // and left no trace

  // hiding static elements: slot entries are pure visibility changes
  opt_.flags[mjVIS_STATIC] = 0;
  mjv_syncScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &retained);
  int nhidden = 0;
  for (int k = 0; k < retained.nchanged; k++) {
    if (retained.changed[k] < retained.nslot) {
      EXPECT_EQ(retained.changebits[k], mjSYNC_VISIBLE) << k;
      EXPECT_EQ(retained.visible[retained.changed[k]], 0) << k;
      nhidden++;
    }
  }
  EXPECT_GT(nhidden, 0);

  mjv_freeScene(&retained);
  mj_deleteData(data);
  FreeSceneObjects();
  mj_deleteModel(model);
}

TEST_F(MjvSceneTest, UpdateSceneGeomsExhausted) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull()) << "Failed to load model from " << kModelPath;

  const int maxgeoms = 1;
  InitSceneObjects(model, maxgeoms);

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // clear handlers to avoid test failure; we are explicitly expecting a warning
  mju_clearHandlers();
  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
  EXPECT_EQ(scn_.status, 1);
  EXPECT_EQ(scn_.ngeom, maxgeoms);
  mj_deleteData(data);
  FreeSceneObjects();
  mj_deleteModel(model);
}

TEST_F(MjvSceneTest, PrincipalPointFrustumSign) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <camera name="cam" pos="0 0 1" zaxis="0 0 1"
              sensorsize="0.01 0.01" focal="0.01 0.01"
              principal="0 0.002"/>
    </worldbody>
  </mujoco>
  )";

  MjModelPtr model = LoadModelFromString(xml);
  ASSERT_THAT(model.get(), NotNull());
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());

  InitSceneObjects(model.get());

  // point camera at the fixed cam
  cam_.type = mjCAMERA_FIXED;
  cam_.fixedcamid = 0;
  mjv_updateCamera(model.get(), data.get(), &cam_, &scn_);

  float top = scn_.camera[0].frustum_top;
  float bottom = scn_.camera[0].frustum_bottom;

  // with cy > 0 the principal point is above center, so the frustum should
  // extend further downward than upward: |bottom| > top
  EXPECT_GT(-bottom, top);

  // verify exact values against the pinhole model
  float znear = model->vis.map.znear * model->stat.extent;
  float cy = model->cam_intrinsic[3];
  float fy = model->cam_intrinsic[1];
  float sh = model->cam_sensorsize[1];
  float half = znear / fy * (sh / 2);
  float offset = znear / fy * cy;

  EXPECT_FLOAT_EQ(top, half - offset);
  EXPECT_FLOAT_EQ(bottom, -(half + offset));

  FreeSceneObjects();
}

}  // namespace
}  // namespace mujoco
