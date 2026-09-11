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

// Tests for recoverable out-of-memory in the simulation pipeline: a scratch
// allocation that overflows the mjData arena ends the call with mjSTATUS_OOM.

#include <cstddef>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_util.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::NotNull;
using OomTest = MujocoTest;

// an error is kept over any warning, and stops the call under every policy
TEST_F(OomTest, ErrorOverridesWarningsAndStops) {
  EXPECT_EQ(mji_join(mjSTATUS_OK, mjSTATUS_OOM), mjSTATUS_OOM);
  EXPECT_EQ(mji_join(mjSTATUS_CONTACTFULL, mjSTATUS_OOM), mjSTATUS_OOM);
  EXPECT_EQ(mji_join(mjSTATUS_OOM, mjSTATUS_CONTACTFULL), mjSTATUS_OOM);
  EXPECT_EQ(mji_join(mjSTATUS_OK, mjSTATUS_CONTACTFULL), mjSTATUS_CONTACTFULL);
  EXPECT_EQ(mji_join(mjSTATUS_CONTACTFULL, mjSTATUS_BADQACC), mjSTATUS_CONTACTFULL);

  mjModel model = {0};
  for (int onwarn : {mjONWARN_AUTO, mjONWARN_CONTINUE, mjONWARN_STOP}) {
    model.opt.onwarn = onwarn;
    EXPECT_TRUE(mji_stop(&model, mjSTATUS_OOM));
    EXPECT_FALSE(mji_stop(&model, mjSTATUS_OK));
    EXPECT_EQ(mji_stop(&model, mjSTATUS_CONTACTFULL) != 0, onwarn == mjONWARN_STOP);
  }
}

// the public allocators raise an error on overflow, whatever the frame
TEST_F(OomTest, PublicAllocatorsRaiseAnError) {
  static constexpr char xml[] = R"(
  <mujoco><worldbody><body><joint/><geom size=".1"/></body></worldbody></mujoco>
  )";
  MjModelPtr model = LoadModelFromString(xml);
  ASSERT_THAT(model.get(), NotNull());
  MjDataPtr data = MakeData(model);
  mj_markStack(data.get());
  EXPECT_THAT(MjuErrorMessageFrom(mj_stackAllocNum)(data.get(), data->narena),
              HasSubstr("stack overflow"));
  EXPECT_THAT(MjuErrorMessageFrom(mj_stackAllocByte)(data.get(), data->narena, 8),
              HasSubstr("stack overflow"));
  mj_freeStack(data.get());
}

// run the pipeline on data of a given arena size: the status of the step, with
// the stack released and the data usable again after a failure
struct Outcome {
  int oom = 0;       // calls that ran out of memory
  int ok = 0;        // calls that completed
  int warned = 0;    // calls that completed with a warning
};

void Run(const mjModel* m, mjData* d, Outcome* outcome) {
  int nv = m->nv, na = m->na, nu = m->nu, ndx = 2*nv + na;
  std::vector<mjtNum> A(ndx*ndx), B(ndx*nu);

  auto record = [&](mjtStatus status) {
    EXPECT_GE(status, mjSTATUS_OOM);
    EXPECT_EQ(status, (mjtStatus)d->status);
    EXPECT_EQ(d->pstack, 0);
    EXPECT_EQ(d->pbase, 0);
    if (status < 0) {
      outcome->oom++;
    } else if (status > 0) {
      outcome->warned++;
    } else {
      outcome->ok++;
    }
    return status;
  };

  for (int i = 0; i < 3; i++) {
    if (record(mj_step(m, d)) < 0) {
      break;
    }
  }
  record(mj_inverse(m, d));
  if (m->opt.integrator != mjINT_RK4) {
    record(mjd_transitionFD(m, d, 1e-6, 1, A.data(), B.data(), nullptr, nullptr));
  }

  // a failed data is usable again after a reset
  mj_resetData(m, d);
  record(mj_forward(m, d));
}

// sweep the arena from its default size down to just above the reserve kept
// for stack frames: every size runs out of memory cleanly or completes, and the
// smallest sizes run out
constexpr int kSmallest = 2560;

void Sweep(mjModel* model, int nthread, Outcome* outcome) {
  int narena = model->narena;
  for (int size = narena; size > kSmallest; size = size / 2) {
    model->narena = size;
    mjData* data = mj_makeData(model);
    ASSERT_THAT(data, NotNull());
    if (nthread) {
      mju_threadpool(data, nthread);
    }
    Run(model, data, outcome);
    mju_threadpool(data, 0);
    mj_deleteData(data);
  }
  model->narena = narena;
}

// every allocation site of the pipeline fails cleanly, at every arena size
TEST_F(OomTest, PipelineRunsOutOfMemoryCleanly) {
  mock_warning_handler.ExpectWarnings();
  for (const char* path : {"engine/testdata/island/humanoid.xml",
                           "engine/testdata/island/island_efc.xml",
                           "testdata/flex.xml", "testdata/tendon_wrap.xml"}) {
    char error[1024];
    mjModel* model = mj_loadXML(GetTestDataFilePath(path).c_str(), nullptr, error, sizeof(error));
    ASSERT_THAT(model, NotNull()) << path << ": " << error;
    Outcome outcome;
    Sweep(model, 0, &outcome);
    EXPECT_GT(outcome.oom, 0) << path;
    EXPECT_GT(outcome.ok, 0) << path << " warned " << outcome.warned << " oom " << outcome.oom;
    mj_deleteModel(model);
  }
}

// the same under each solver with islands solved on worker threads
TEST_F(OomTest, WorkersRunOutOfMemoryCleanly) {
  mock_warning_handler.ExpectWarnings();
  char error[1024];
  mjModel* model = mj_loadXML(GetTestDataFilePath("engine/testdata/island/island_efc.xml").c_str(),
                              nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  for (int solver : {mjSOL_PGS, mjSOL_CG, mjSOL_NEWTON}) {
    model->opt.solver = solver;
    Outcome outcome;
    Sweep(model, 4, &outcome);
    EXPECT_GT(outcome.oom, 0) << "solver " << solver;
    EXPECT_GT(outcome.ok, 0) << "solver " << solver << " warned " << outcome.warned << " oom "
                             << outcome.oom;
  }
  mj_deleteModel(model);
}

// out of memory stops the step under every policy, before the state advances,
// and a data of sufficient size completes it: the recovery loop of a host
TEST_F(OomTest, StopsUnderEveryPolicyAndRecoversWithMoreMemory) {
  mock_warning_handler.ExpectWarnings();
  char error[1024];
  mjModel* model = mj_loadXML(GetTestDataFilePath("engine/testdata/island/humanoid.xml").c_str(),
                              nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  int narena = model->narena;

  // find an arena that runs out during the first step
  int small = narena;
  while (small > kSmallest) {
    small /= 2;
    model->narena = small;
    mjData* data = mj_makeData(model);
    mjtStatus status = mj_step(model, data);
    mj_deleteData(data);
    if (status == mjSTATUS_OOM) {
      break;
    }
  }
  ASSERT_EQ(model->narena, small);

  for (int onwarn : {mjONWARN_AUTO, mjONWARN_CONTINUE, mjONWARN_STOP}) {
    model->opt.onwarn = onwarn;
    mjData* data = mj_makeData(model);
    EXPECT_EQ(mj_step(model, data), mjSTATUS_OOM) << onwarn;
    EXPECT_FALSE(mjOK(data));
    EXPECT_EQ(data->time, 0);
    EXPECT_EQ(data->pstack, 0);
    EXPECT_EQ(data->pbase, 0);

    // the same again: the failure is repeatable, not accumulating
    EXPECT_EQ(mj_step(model, data), mjSTATUS_OOM);
    EXPECT_EQ(data->time, 0);
    mj_deleteData(data);

    // a larger data completes the step
    model->narena = narena;
    data = mj_makeData(model);
    EXPECT_EQ(mj_step(model, data), mjSTATUS_OK);
    EXPECT_GT(data->time, 0);
    mj_deleteData(data);
    model->narena = small;
  }
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
