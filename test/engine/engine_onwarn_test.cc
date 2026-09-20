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

// Tests for the onwarn option.

#include <cmath>
#include <csetjmp>
#include <cstddef>
#include <limits>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;
using OnWarnTest = MujocoTest;

constexpr mjtNum kNaN = std::numeric_limits<mjtNum>::quiet_NaN();

static constexpr char kModelXml[] = R"(
<mujoco>
  <worldbody>
    <body>
      <joint name="slide" type="slide"/>
      <geom size=".1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="slide"/>
  </actuator>
</mujoco>
)";

// the model overflows the contact arena during the position stage
constexpr char kContactFullXml[] = R"(
  <mujoco>
    <size memory="12K"/>
    <option ccd_iterations="5"/>
    <worldbody>
      <replicate count="10" offset="0.01 0.011 0">
        <body pos="0 0 .05">
          <freejoint/>
          <geom size=".1"/>
        </body>
      </replicate>
    </worldbody>
  </mujoco>
  )";

// stop: the step stops at the first warning, state untouched
TEST_F(OnWarnTest, StopAtFirstWarning) {
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in QVEL");

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  // inject bad qvel, save qpos
  data->qvel[0] = kNaN;
  mjtNum qpos = data->qpos[0];

  // the status names the warning, nothing was stepped
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_BADQVEL);
  EXPECT_EQ(data->time, 0);
  EXPECT_EQ(data->qpos[0], qpos);

  // stepping again without fixing anything stops again
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_BADQVEL);
  EXPECT_EQ(data->warning[mjWARN_BADQVEL].number, 2);
}

// stop: bad ctrl stops before actuation, d->ctrl left pristine for inspection
TEST_F(OnWarnTest, StopPreservesBadCtrl) {
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in CTRL");

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  data->ctrl[0] = kNaN;
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_BADCTRL);
  EXPECT_EQ(data->time, 0);
  EXPECT_TRUE(std::isnan(data->ctrl[0]));
}

// stop: the first half of a split step stops as the whole step does
TEST_F(OnWarnTest, StopInStep1) {
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in QPOS");

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  data->qpos[0] = kNaN;
  mj_step1(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_BADQPOS);
  EXPECT_EQ(data->time, 0);

  // fix the state: both halves complete healthy
  data->qpos[0] = 0;
  mj_step1(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
  mj_step2(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
  EXPECT_GT(data->time, 0);
}

// stop: arena exhaustion stops forward at the collision stage, the acceleration
// is never computed
TEST_F(OnWarnTest, StopAtArenaExhaustion) {
  mock_warning_handler.ExpectWarnings();
  char error[1024];
  MjModelPtr model = LoadModelFromString(kContactFullXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  mj_forward(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_CONTACTFULL);
  EXPECT_EQ(data->nefc, 0);
  EXPECT_EQ(data->qacc[2], 0);
}

// stop: the collision stage returns at the call that warned. The contacts
// accumulated so far have exhausted the arena, so the scratch of whatever
// would run next (filtering, sorting, the next pair) could not be allocated:
// for every arena size, no fatal error follows the warning
jmp_buf g_fatal;
mjfLogHandler g_fatal_prev = nullptr;

void FatalHandler(const mjLogMessage* msg) {
  if (msg->level == mjLOG_ERROR) {
    longjmp(g_fatal, 1);
  }
  if (g_fatal_prev) {
    g_fatal_prev(msg);
  }
}

TEST_F(OnWarnTest, StopLeavesTheCollisionStageAtOnce) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option onwarn="stop"/>
    <worldbody>
      <geom type="plane" size="1 1 .1"/>
      <geom type="plane" size="1 1 .1" pos="0 0 .001"/>
      <flexcomp name="flex" type="grid" dim="2" count="3 3 1" spacing=".1 .1 .1"
                radius=".02" mass="1" pos="0 0 .01">
        <edge equality="true"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  mock_warning_handler.ExpectWarnings();
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  // the arena a healthy forward pass uses
  size_t needed = 0;
  {
    MjDataPtr data = MakeData(model);
    ASSERT_EQ(mj_forward(model.get(), data.get()), mjSTATUS_OK);
    ASSERT_GT(data->ncon, 9);
    needed = data->maxuse_arena;
  }

  // shrink it until nothing fits
  int full_with_contacts = 0;
  for (size_t narena = 64 * (needed / 64); narena > 1024; narena -= 64) {
    model->narena = narena;
    mjData* d = mj_makeData(model.get());
    ASSERT_THAT(d, NotNull());
    volatile int fatal = 0;
    volatile mjtStatus status = mjSTATUS_OK;
    g_fatal_prev = mju_setLogHandler(FatalHandler);
    mjfLogHandler prev = g_fatal_prev;
    if (setjmp(g_fatal) == 0) {
      status = mj_forward(model.get(), d);
    } else {
      fatal = 1;
    }
    mju_setLogHandler(prev);

    // once the contact arena is full the call has returned
    EXPECT_FALSE(fatal && d->warning[mjWARN_CONTACTFULL].number) << "narena " << narena;
    if (!fatal && status == mjSTATUS_CONTACTFULL && d->ncon > 0) {
      EXPECT_EQ(d->pstack, 0) << "narena " << narena;
      full_with_contacts++;
    }
    mj_deleteData(d);
  }
  EXPECT_GT(full_with_contacts, 0);
}

// stop: a singular modified inertia stops the step before it advances, under
// both implicit integrators
TEST_F(OnWarnTest, StopBeforeAdvancingOnSingularInertia) {
  mock_warning_handler.ExpectWarnings(
      "Inertia matrix is too close to singular");

  // the affine velocity gain of 1 makes M - h*qDeriv exactly singular at h=1
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="1" integrator="implicit"/>
    <worldbody>
      <body>
        <joint name="slide" type="slide"/>
        <geom type="sphere" size="0.1" mass="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="slide" dyntype="integrator" gaintype="affine" gainprm="1 0 1"
               actearly="true"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  for (int integrator : {mjINT_IMPLICIT, mjINT_IMPLICITFAST}) {
    model->opt.integrator = integrator;
    mj_resetData(model.get(), data.get());
    data->ctrl[0] = 1;
    mj_step(model.get(), data.get());
    EXPECT_EQ(data->status, mjSTATUS_INERTIA) << integrator;
    EXPECT_EQ(data->time, 0);
  }
}

// XML: the onwarn keyword, its default, and the roundtrip through the writer
TEST_F(OnWarnTest, XmlOnwarn) {
  char error[1024];

  // auto is the default
  MjModelPtr model0 = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model0.get(), NotNull()) << error;
  EXPECT_EQ(model0->opt.onwarn, mjONWARN_AUTO);

  // parse onwarn
  static constexpr char xml[] = R"(
  <mujoco>
    <option onwarn="stop"/>
    <worldbody/>
  </mujoco>
  )";
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  EXPECT_EQ(model->opt.onwarn, mjONWARN_STOP);

  // writer emits the non-default value
  std::string saved = SaveAndReadXml(model.get());
  EXPECT_THAT(saved, HasSubstr("onwarn=\"stop\""));
}

// the removed autoreset flag is a schema error
TEST_F(OnWarnTest, RemovedAutoresetFlagIsAnError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag autoreset="disable"/>
    </option>
    <worldbody/>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error, HasSubstr("autoreset"));
}
// every driver frees the frame it owns before stopping, so the same data can be
// called again
TEST_F(OnWarnTest, StopPathsRestoreTheStack) {
  mock_warning_handler.ExpectWarnings();
  char error[1024];
  MjModelPtr model = LoadModelFromString(kContactFullXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  struct Entry {
    const char* name;
    mjtStatus (*fn)(const mjModel*, mjData*);
  };
  const Entry entries[] = {
      {"mj_step", mj_step},
      {"mj_step1", mj_step1},
      {"mj_forward", mj_forward},
      {"mj_inverse", mj_inverse},
      {"mj_fwdPosition", mj_fwdPosition},
      {"mj_collision", mj_collision},
  };
  for (const Entry& entry : entries) {
    for (int i = 0; i < 3; i++) {
      // the value a call returns is the value it recorded in the data
      EXPECT_EQ(entry.fn(model.get(), data.get()), mjSTATUS_CONTACTFULL)
          << entry.name;
      EXPECT_EQ(data->status, mjSTATUS_CONTACTFULL) << entry.name;
      EXPECT_EQ(data->pstack, 0) << entry.name << " call " << i;
      EXPECT_EQ(data->pbase, 0) << entry.name << " call " << i;
    }
  }
}

// a warning that stops the call stops the stage that raised it: no later
// callback of that stage runs on data the call has given up on
int g_ctrl_calls = 0;

TEST_F(OnWarnTest, StopEndsTheStageThatWarned) {
  mock_warning_handler.ExpectWarnings();
  char error[1024];
  MjModelPtr model = LoadModelFromString(kContactFullXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;
  g_ctrl_calls = 0;

  struct Guard {
    Guard() {
      prev_control = mjcb_control;
      mjcb_control = +[](const mjModel* m, mjData* d) { g_ctrl_calls++; };
    }
    ~Guard() { mjcb_control = prev_control; }
    mjfGeneric prev_control;
  } guard;

  // the position stage overflows the contact arena, so the step never reaches
  // the control callback
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_CONTACTFULL);
  EXPECT_EQ(g_ctrl_calls, 0);
  EXPECT_EQ(data->time, 0);
}

// under the stop policy, the finite-difference drivers stop perturbing after
// the first warning instead of running every column on data the warning
// abandoned
int g_fd_calls = 0;
int g_fd_calls_after = 0;

TEST_F(OnWarnTest, StopEndsFiniteDifferencing) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option onwarn="stop"/>
    <worldbody><body><joint name="j" type="slide"/><geom size=".1"/></body></worldbody>
    <actuator><motor joint="j"/></actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  g_fd_calls = g_fd_calls_after = 0;
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in CTRL");

  struct Guard {
    Guard() {
      prev_ = mjcb_control;
      mjcb_control = +[](const mjModel* m, mjData* d) {
        g_fd_calls++;
        if (d->status != mjSTATUS_OK) g_fd_calls_after++;
        d->ctrl[0] = std::numeric_limits<mjtNum>::quiet_NaN();
      };
    }
    ~Guard() { mjcb_control = prev_; }
    mjfGeneric prev_;
  } guard;

  mjtNum A[4] = {0}, B[2] = {0};
  mjd_transitionFD(model.get(), data.get(), 1e-6, 1, A, B, nullptr, nullptr);
  EXPECT_EQ(data->status, mjSTATUS_BADCTRL);
  EXPECT_EQ(g_fd_calls, 1);
  EXPECT_EQ(g_fd_calls_after, 0);
  EXPECT_EQ(data->pstack, 0);
}

// a warning first raised by a perturbed evaluation is reported, and under the stop policy it ends
// the perturbations
TEST_F(OnWarnTest, StopEndsFiniteDifferencingAtAPerturbation) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option onwarn="stop"/>
    <worldbody><body><joint name="j" type="slide"/><geom size=".1"/></body></worldbody>
    <actuator><motor joint="j"/></actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  g_fd_calls = 0;
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in CTRL");

  struct Guard {
    Guard() {
      prev_ = mjcb_control;
      // the nominal step is clean, the perturbed ones are not
      mjcb_control = +[](const mjModel* m, mjData* d) {
        if (g_fd_calls++) d->ctrl[0] = std::numeric_limits<mjtNum>::quiet_NaN();
      };
    }
    ~Guard() { mjcb_control = prev_; }
    mjfGeneric prev_;
  } guard;

  mjtNum A[4] = {0}, B[2] = {0};
  EXPECT_EQ(mjd_transitionFD(model.get(), data.get(), 1e-6, 1, A, B, nullptr, nullptr),
            mjSTATUS_BADCTRL);
  EXPECT_EQ(data->status, mjSTATUS_BADCTRL);
  EXPECT_EQ(g_fd_calls, 2);
  EXPECT_EQ(data->pstack, 0);
}

// keeping a returned status and honoring stop are separate obligations, and the
// compiler checks only the first: under the stop policy, an evaluation of a
// finite-difference driver ends at the stage that warned, as a step does
TEST_F(OnWarnTest, FiniteDifferenceStepStopsAtTheStageThatWarned) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option onwarn="stop"/>
    <worldbody><body><joint name="j" type="slide"/><geom size=".1"/></body></worldbody>
    <actuator><motor joint="j"/></actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  g_fd_calls = 0;
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in QPOS");

  struct Guard {
    Guard() {
      prev_ = mjcb_control;
      mjcb_control = +[](const mjModel* m, mjData* d) { g_fd_calls++; };
    }
    ~Guard() { mjcb_control = prev_; }
    mjfGeneric prev_;
  } guard;

  // the position check warns: neither the forward pass nor the acceleration
  // check that follow it in the evaluation runs
  data->qpos[0] = kNaN;
  mjtNum A[4] = {0}, B[2] = {0};
  EXPECT_EQ(mjd_transitionFD(model.get(), data.get(), 1e-6, 1, A, B, nullptr, nullptr),
            mjSTATUS_BADQPOS);
  EXPECT_EQ(g_fd_calls, 0);
  EXPECT_EQ(data->warning[mjWARN_BADQACC].number, 0);
  EXPECT_EQ(data->pstack, 0);
}

int g_gain_calls = 0;

// the same for the inverse driver: a warning in the inverse dynamics of an
// evaluation leaves its actuation unrun
TEST_F(OnWarnTest, FiniteDifferenceInverseStopsAtTheStageThatWarned) {
  // two coincident hinges make the mass matrix singular
  static constexpr char xml[] = R"(
  <mujoco>
    <option onwarn="stop"/>
    <worldbody>
      <body><joint name="a"/><joint/><geom size=".1" pos=".2 0 0"/></body>
    </worldbody>
    <actuator><general joint="a" gaintype="user"/></actuator>
  </mujoco>
  )";
  mock_warning_handler.ExpectWarnings("Inertia matrix is too close to singular");
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  g_gain_calls = 0;

  struct Guard {
    Guard() {
      prev_ = mjcb_act_gain;
      mjcb_act_gain = +[](const mjModel* m, const mjData* d, int id) -> mjtNum {
        g_gain_calls++;
        return 1;
      };
    }
    ~Guard() { mjcb_act_gain = prev_; }
    mjfAct prev_;
  } guard;

  mjtNum DfDa[4] = {0};
  EXPECT_EQ(mjd_inverseFD(model.get(), data.get(), 1e-6, /*flg_actuation=*/1, nullptr,
                          nullptr, DfDa, nullptr, nullptr, nullptr, nullptr),
            mjSTATUS_INERTIA);
  EXPECT_EQ(g_gain_calls, 0);
  EXPECT_EQ(data->pstack, 0);
}

// stop: a step under the IPC integrator ends at the stage that warned, before
// the integrator
TEST_F(OnWarnTest, StopDoesNotReachTheIpcIntegrator) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.002" integrator="discrete" solver="CG"><flag ipc="enable"/></option>
    <worldbody>
      <flexcomp name="cloth" type="grid" dim="2" count="3 3 1" spacing="0.05 0.05 1"
                radius="0.005" mass="0.05" pos="0 0 0.5">
        <edge equality="true"/>
      </flexcomp>
      <body pos="1 0 0"><joint name="slide" type="slide"/><geom size=".1"/></body>
    </worldbody>
    <actuator><motor joint="slide"/></actuator>
  </mujoco>
  )";
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in CTRL");
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  // healthy step
  EXPECT_EQ(mj_step(model.get(), data.get()), mjSTATUS_OK);
  mjtNum time = data->time;
  EXPECT_GT(time, 0);

  // the step ends at the actuation stage
  data->ctrl[0] = kNaN;
  EXPECT_EQ(mj_step(model.get(), data.get()), mjSTATUS_BADCTRL);
  EXPECT_EQ(data->time, time);
  EXPECT_EQ(data->pstack, 0);
}

// the length-range computation simulates until a total time: a step that stops
// under the stop policy cannot advance it, so the computation reports an error
// instead of waiting forever
TEST_F(OnWarnTest, StoppedLengthRangeSimulationIsAnError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option onwarn="stop"/>
    <worldbody><body><joint name="j" type="slide"/><geom size=".1"/></body></worldbody>
    <actuator><motor joint="j"/></actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in CTRL");

  // the first step is healthy, the ones after it stop at a bad control
  struct Guard {
    Guard() {
      prev_ = mjcb_control;
      mjcb_control = +[](const mjModel* m, mjData* d) {
        if (d->time > 0) d->ctrl[0] = std::numeric_limits<mjtNum>::quiet_NaN();
      };
    }
    ~Guard() { mjcb_control = prev_; }
    mjfGeneric prev_;
  } guard;

  mjLROpt opt;
  mj_defaultLROpt(&opt);
  opt.mode = mjLRMODE_ALL;
  EXPECT_EQ(mj_setLengthRange(model.get(), data.get(), 0, &opt, error, sizeof(error)), 0);
  EXPECT_THAT(error, HasSubstr("CTRL"));
  EXPECT_EQ(data->status, mjSTATUS_BADCTRL);
  EXPECT_GT(data->time, 0);
}

}  // namespace
}  // namespace mujoco
