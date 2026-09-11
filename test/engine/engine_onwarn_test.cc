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

// Tests for the onwarn option and mjData.status.

#include <cmath>
#include <csetjmp>
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

// auto (default): divergence resets the state, the status reports the warning
TEST_F(OnWarnTest, AutoResetsAndReportsStatus) {
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in QPOS");

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  EXPECT_EQ(model->opt.onwarn, mjONWARN_AUTO);

  // healthy step: OK status
  mj_step(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
  EXPECT_EQ(data->status, mjSTATUS_OK);
  EXPECT_GT(data->time, 0);

  // inject bad qpos: the step reports BADQPOS and resets the state
  data->qpos[0] = kNaN;
  mj_step(model.get(), data.get());
  EXPECT_FALSE(mjOK(data.get()));
  EXPECT_EQ(data->status, mjSTATUS_BADQPOS);
  EXPECT_TRUE(std::isfinite(data->qpos[0]));

  // warning statistics survive the reset, counted once
  EXPECT_EQ(data->warning[mjWARN_BADQPOS].number, 1);
  EXPECT_EQ(data->warning[mjWARN_BADQPOS].lastinfo, 0);

  // the next healthy step clears the status
  mj_step(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
}

// continue: no reset, the first warning is reported and NaNs propagate
TEST_F(OnWarnTest, ContinueRecordsWithoutReset) {
  mock_warning_handler.ExpectWarnings();

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_CONTINUE;

  // inject bad qvel: the step proceeds, bad velocity begets bad acceleration
  data->qvel[0] = kNaN;
  mj_step(model.get(), data.get());

  // the status names the first warning, both are counted
  EXPECT_EQ(data->status, mjSTATUS_BADQVEL);
  EXPECT_EQ(data->warning[mjWARN_BADQVEL].number, 1);
  EXPECT_EQ(data->warning[mjWARN_BADQACC].number, 1);

  // no reset: time advanced, state remains bad
  EXPECT_GT(data->time, 0);
  EXPECT_TRUE(std::isnan(data->qvel[0]));
}

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

// auto: bad ctrl is zeroed (in the local copy) and the step completes
TEST_F(OnWarnTest, AutoZeroesBadCtrl) {
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in CTRL");

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  data->ctrl[0] = kNaN;
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_BADCTRL);
  EXPECT_GT(data->time, 0);
  EXPECT_TRUE(std::isfinite(data->qpos[0]));
}

// mj_step1/mj_step2 report through the same channel
TEST_F(OnWarnTest, Step1Step2ReportStatus) {
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

// arena exhaustion: auto completes degraded, stop stops at the boundary
TEST_F(OnWarnTest, ContactFullModes) {
  mock_warning_handler.ExpectWarnings();

  static constexpr char xml[] = R"(
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
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  // auto: forward completes without the dropped contacts, the bodies are in
  // free fall
  mj_forward(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_CONTACTFULL);
  EXPECT_EQ(data->nefc, 0);
  EXPECT_LT(data->qacc[2], 0);

  // stop: forward stops at the collision stage, the acceleration is never
  // computed
  mj_resetData(model.get(), data.get());
  model->opt.onwarn = mjONWARN_STOP;
  mj_forward(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_CONTACTFULL);
  EXPECT_EQ(data->nefc, 0);
  EXPECT_EQ(data->qacc[2], 0);
}

// the status belongs to the outermost pipeline call: the stages it runs report
// into it, while the same stages called directly are calls in their own right
TEST_F(OnWarnTest, StatusBelongsToTheOutermostCall) {
  mock_warning_handler.ExpectWarnings();

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_CONTINUE;

  // mj_step: mj_checkVel warns, the forward stages that follow do not clear the
  // warning, mj_checkAcc warns again and the status keeps the first
  data->qvel[0] = kNaN;
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_BADQVEL);
  EXPECT_EQ(data->warning[mjWARN_BADQACC].number, 1);

  // called directly, each stage starts from a clean status
  mj_forward(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
  mj_checkAcc(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_BADQACC);
  mj_checkVel(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_BADQVEL);
  mj_fwdPosition(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
}

// a pipeline call made from a callback on another mjData is outermost for that
// data
mjData* g_scratch = nullptr;

TEST_F(OnWarnTest, CallbackOnAnotherDataIsIndependent) {
  mock_warning_handler.ExpectWarnings();

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  MjDataPtr scratch = MakeData(model);
  model->opt.onwarn = mjONWARN_CONTINUE;

  // the controller steps the (bad) scratch data from inside the step of the
  // (good) data
  g_scratch = scratch.get();
  g_scratch->qvel[0] = kNaN;
  struct Guard {
    Guard() {
      mjcb_control = +[](const mjModel* m, mjData* d) {
        if (d == g_scratch) {
          return;
        }
        mj_step(m, g_scratch);
        EXPECT_EQ(g_scratch->status, mjSTATUS_BADQVEL);
      };
    }
    ~Guard() { mjcb_control = nullptr; }
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
  EXPECT_EQ(scratch->status, mjSTATUS_BADQVEL);
  EXPECT_GT(scratch->time, 0);
}

// a copy carries the status, a reset clears it
TEST_F(OnWarnTest, CopyAndReset) {
  mock_warning_handler.ExpectWarnings();

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_CONTINUE;

  data->qvel[0] = kNaN;
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_BADQVEL);

  mjData* copy = mj_copyData(nullptr, model.get(), data.get());
  EXPECT_EQ(copy->status, mjSTATUS_BADQVEL);
  mj_deleteData(copy);

  mj_resetData(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_OK);
}

// every reporting function returns the status it records in the data
TEST_F(OnWarnTest, ReportingFunctionsReturnTheirStatus) {
  mock_warning_handler.ExpectWarnings();

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  const mjModel* m = model.get();
  mjData* d = data.get();

  struct Entry {
    const char* name;
    mjtStatus (*fn)(const mjModel*, mjData*);
  };
  const Entry entries[] = {
      {"mj_step", mj_step},
      {"mj_step1", mj_step1},
      {"mj_step2", mj_step2},
      {"mj_forward", mj_forward},
      {"mj_inverse", mj_inverse},
      {"mj_fwdPosition", mj_fwdPosition},
      {"mj_fwdActuation", mj_fwdActuation},
      {"mj_fwdAcceleration", mj_fwdAcceleration},
      {"mj_fwdConstraint", mj_fwdConstraint},
      {"mj_Euler", mj_Euler},
      {"mj_implicit", mj_implicit},
      {"mj_invPosition", mj_invPosition},
      {"mj_checkPos", mj_checkPos},
      {"mj_checkVel", mj_checkVel},
      {"mj_checkAcc", mj_checkAcc},
      {"mj_factorM", mj_factorM},
      {"mj_collision", mj_collision},
      {"mj_makeConstraint", mj_makeConstraint},
      {"mj_island", mj_island},
      {"mj_projectConstraint", mj_projectConstraint},
      {"mj_fwdKinematics", mj_fwdKinematics},
      {"mj_fwdVelocity", mj_fwdVelocity},
      {"mj_invVelocity", mj_invVelocity},
      {"mj_invConstraint", mj_invConstraint},
      {"mj_flex", mj_flex},
      {"mj_tendon", mj_tendon},
      {"mj_transmission", mj_transmission},
      {"mj_passive", mj_passive},
      {"mj_subtreeVel", mj_subtreeVel},
      {"mj_referenceConstraint", mj_referenceConstraint},
      {"mj_sensorPos", mj_sensorPos},
      {"mj_sensorVel", mj_sensorVel},
      {"mj_sensorAcc", mj_sensorAcc},
      {"mj_energyVel", mj_energyVel},
  };

  // healthy, and with a bad velocity under each policy (exercising the
  // unwinding paths); implicitfast: mj_implicit requires it, the other entries
  // accept any integrator
  model->opt.integrator = mjINT_IMPLICITFAST;
  for (int onwarn : {mjONWARN_AUTO, mjONWARN_CONTINUE, mjONWARN_STOP}) {
    model->opt.onwarn = onwarn;
    for (int bad = 0; bad < 2; bad++) {
      for (const Entry& entry : entries) {
        mj_resetData(m, d);
        if (bad) {
          d->qvel[0] = kNaN;
        }
        // the value a call returns is the value it recorded in the data
        EXPECT_EQ(entry.fn(m, d), (mjtStatus)d->status)
            << entry.name << " onwarn=" << onwarn << " bad=" << bad;
      }
      mj_resetData(m, d);
      if (bad) {
        d->qvel[0] = kNaN;
      }
      mj_forwardSkip(m, d, mjSTAGE_NONE, 0);
      mj_inverseSkip(m, d, mjSTAGE_NONE, 0);
      mj_RungeKutta(m, d, 4);
    }
  }

  // the contact buffer is empty on a reset data: adding one contact succeeds
  mj_resetData(m, d);
  mjContact con = {0};
  con.dim = 3;
  EXPECT_EQ(mj_addContact(m, d, &con), 0);
  EXPECT_TRUE(mjOK(d));
}

// a singular modified inertia M - h*qDeriv is an INERTIA warning under both
// implicit integrators: implicitfast clamps the LDL pivot, implicit clamps the
// LU pivot
TEST_F(OnWarnTest, SingularImplicitInertiaWarns) {
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

  for (int integrator : {mjINT_IMPLICIT, mjINT_IMPLICITFAST}) {
    model->opt.integrator = integrator;

    // auto: the step completes with a clamped pivot (the unit control makes the
    // early activation 1, so the velocity gain contributes h*1 to the modified
    // inertia)
    model->opt.onwarn = mjONWARN_AUTO;
    mj_resetData(model.get(), data.get());
    data->ctrl[0] = 1;
    mj_step(model.get(), data.get());
    EXPECT_EQ(data->status, mjSTATUS_INERTIA) << integrator;
    EXPECT_EQ(data->warning[mjWARN_INERTIA].lastinfo, 0);
    EXPECT_GT(data->time, 0);
    EXPECT_TRUE(std::isfinite(data->qvel[0]));

    // stop: the step stops before advancing
    model->opt.onwarn = mjONWARN_STOP;
    mj_resetData(model.get(), data.get());
    data->ctrl[0] = 1;
    mj_step(model.get(), data.get());
    EXPECT_EQ(data->status, mjSTATUS_INERTIA) << integrator;
    EXPECT_EQ(data->time, 0);
  }
}

// XML: onwarn keyword roundtrip and the removed autoreset flag
TEST_F(OnWarnTest, XmlOnwarnAndRemovedAutoreset) {
  char error[1024];

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

  // the removed autoreset flag is a schema error
  static constexpr char xml_removed[] = R"(
  <mujoco>
    <option>
      <flag autoreset="disable"/>
    </option>
    <worldbody/>
  </mujoco>
  )";
  MjModelPtr model2 = LoadModelFromString(xml_removed, error, sizeof(error));
  EXPECT_THAT(model2.get(), IsNull());
  EXPECT_THAT(error, HasSubstr("autoreset"));
}

// every driver frees the frame it owns before stopping, and the nesting state
// is balanced, so the same data can be called again
// the model overflows the contact arena during the position stage, which is a
// warning the engine raises itself: user code has no business raising one
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

// A log handler that transfers control out of a call skips the mj_freeStack of
// every stage it unwinds: the call is abandoned and its mjData needs
// mj_resetData before it is used again, which is what an intercepted error has
// always meant. What must survive is everything else on the thread.
jmp_buf g_escape;
mjData* g_other = nullptr;
mjModel* g_model = nullptr;

mjfLogHandler g_escape_prev = nullptr;

// escapes on an error, and leaves warnings to the handler it replaced
void EscapingHandler(const mjLogMessage* msg) {
  if (msg->level == mjLOG_ERROR) {
    longjmp(g_escape, 1);
  }
  if (g_escape_prev) {
    g_escape_prev(msg);
  }
}

constexpr char kTwoDataXml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="slide" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="slide" gaintype="user"/>
    </actuator>
    <sensor>
      <user dim="1" needstage="pos"/>
    </sensor>
  </mujoco>
  )";

TEST_F(OnWarnTest, AbandonedCallNeedsAResetAndLeavesTheRestAlone) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kTwoDataXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  MjDataPtr other = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  // an error from inside a stage that owns a stack frame, intercepted by a
  // handler that jumps out of the engine
  g_escape_prev = mju_setLogHandler(EscapingHandler);
  mjfLogHandler prev = g_escape_prev;
  mjcb_act_gain = +[](const mjModel* m, const mjData* d, int id) -> mjtNum {
    mju_error("host exception");
    return 0;
  };
  if (setjmp(g_escape) == 0) {
    mj_step(model.get(), data.get());
    ADD_FAILURE() << "the handler did not transfer control";
  }
  mjcb_act_gain = nullptr;
  mju_setLogHandler(prev);

  // the abandoned call left its stack frame behind as it always has, and
  // mj_resetData is what clears it
  EXPECT_GT(data->pstack, 0);
  mj_resetData(model.get(), data.get());
  EXPECT_EQ(data->pstack, 0);

  // another data on the same thread is untouched, before and after the reset
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in CTRL");
  mjcb_control = +[](const mjModel* m, mjData* d) {
    d->ctrl[0] = std::numeric_limits<mjtNum>::quiet_NaN();
  };
  mj_step(model.get(), other.get());
  EXPECT_EQ(other->status, mjSTATUS_BADCTRL);
  mjcb_control = nullptr;
  other->ctrl[0] = 0;
  mj_resetData(model.get(), other.get());
  mj_step(model.get(), other.get());
  EXPECT_EQ(other->status, mjSTATUS_OK);
  EXPECT_GT(other->time, 0);

  // and the reset data reports its own warnings again
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_OK);
  EXPECT_GT(data->time, 0);
}

// a call whose callback catches an error from a call on another data keeps its
// own warning
TEST_F(OnWarnTest, CatchingANestedFailureKeepsTheEnclosingWarning) {
  mock_warning_handler.ExpectWarnings();
  char error[1024];
  MjModelPtr model = LoadModelFromString(kContactFullXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  MjDataPtr scratch = MakeData(model);
  model->opt.onwarn =
      mjONWARN_CONTINUE;  // so the step runs on past the warning
  g_model = model.get();
  g_other = scratch.get();

  struct Guard {
    Guard() {
      prev_control = mjcb_control;
      // the callback fails a call on the scratch data and catches it there
      mjcb_control = +[](const mjModel* m, mjData* d) {
        if (d == g_other) return;
        g_escape_prev = mju_setLogHandler(EscapingHandler);
        mjfLogHandler prev = g_escape_prev;
        const int saved = m->opt.integrator;
        g_model->opt.integrator = 99;
        if (setjmp(g_escape) == 0) {
          mj_step(m, g_other);
        }
        g_model->opt.integrator = saved;
        mju_setLogHandler(prev);
      };
    }
    ~Guard() { mjcb_control = prev_control; }
    mjfGeneric prev_control;
  } guard;

  // the position stage warns before the callback runs; catching the nested
  // failure must not cost the enclosing call its own warning
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_CONTACTFULL);
  EXPECT_GT(data->warning[mjWARN_CONTACTFULL].number, 0);
}

// the pipeline functions that cannot raise a simulation warning do not report
// one, and leave the status recorded by the last reporting call alone
TEST_F(OnWarnTest, NonReportingFunctionsLeaveTheStatusAlone) {
  mock_warning_handler.ExpectWarnings();
  char error[1024];
  MjModelPtr model = LoadModelFromString(kContactFullXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_CONTINUE;
  mj_forward(model.get(), data.get());
  ASSERT_EQ(data->status, mjSTATUS_CONTACTFULL);

  void (*const functions[])(const mjModel*, mjData*) = {
      mj_kinematics, mj_comPos, mj_camlight, mj_crb, mj_makeM, mj_comVel, mj_rnePostConstraint,
      mj_energyPos,
  };
  for (auto function : functions) {
    function(model.get(), data.get());
    EXPECT_EQ(data->status, mjSTATUS_CONTACTFULL);
  }
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
