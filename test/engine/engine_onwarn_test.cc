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
  EXPECT_EQ(data->nested, 0);

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
        EXPECT_EQ(d->nested, 1);
        if (d == g_scratch) {
          return;
        }
        mj_step(m, g_scratch);
        EXPECT_EQ(g_scratch->status, mjSTATUS_BADQVEL);
        EXPECT_EQ(g_scratch->nested, 0);
      };
    }
    ~Guard() { mjcb_control = nullptr; }
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
  EXPECT_EQ(data->nested, 0);
  EXPECT_EQ(scratch->status, mjSTATUS_BADQVEL);
  EXPECT_GT(scratch->time, 0);
}

// a copy is not inside a pipeline call, a reset clears the status
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
  EXPECT_EQ(copy->nested, 0);
  mj_deleteData(copy);

  mj_resetData(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_OK);
  EXPECT_EQ(data->nested, 0);
}

// every pipeline function is a complete entry: after it returns, no call is in
// progress
TEST_F(OnWarnTest, PipelineFunctionsExitCleanly) {
  mock_warning_handler.ExpectWarnings();

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  const mjModel* m = model.get();
  mjData* d = data.get();

  struct Entry {
    const char* name;
    void (*fn)(const mjModel*, mjData*);
  };
  const Entry entries[] = {
      {"mj_step", mj_step},
      {"mj_step1", mj_step1},
      {"mj_step2", mj_step2},
      {"mj_forward", mj_forward},
      {"mj_inverse", mj_inverse},
      {"mj_fwdPosition", mj_fwdPosition},
      {"mj_fwdVelocity", mj_fwdVelocity},
      {"mj_fwdActuation", mj_fwdActuation},
      {"mj_fwdAcceleration", mj_fwdAcceleration},
      {"mj_fwdConstraint", mj_fwdConstraint},
      {"mj_Euler", mj_Euler},
      {"mj_implicit", mj_implicit},
      {"mj_invPosition", mj_invPosition},
      {"mj_invVelocity", mj_invVelocity},
      {"mj_invConstraint", mj_invConstraint},
      {"mj_checkPos", mj_checkPos},
      {"mj_checkVel", mj_checkVel},
      {"mj_checkAcc", mj_checkAcc},
      {"mj_factorM", mj_factorM},
      {"mj_collision", mj_collision},
      {"mj_makeConstraint", mj_makeConstraint},
      {"mj_island", mj_island},
      {"mj_projectConstraint", mj_projectConstraint},
      {"mj_referenceConstraint", mj_referenceConstraint},
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
        entry.fn(m, d);
        EXPECT_EQ(d->nested, 0)
            << entry.name << " onwarn=" << onwarn << " bad=" << bad;
      }
      mj_resetData(m, d);
      if (bad) {
        d->qvel[0] = kNaN;
      }
      mj_forwardSkip(m, d, mjSTAGE_NONE, 0);
      EXPECT_EQ(d->nested, 0);
      mj_inverseSkip(m, d, mjSTAGE_NONE, 0);
      EXPECT_EQ(d->nested, 0);
      mj_RungeKutta(m, d, 4);
      EXPECT_EQ(d->nested, 0);
    }
  }

  // the contact buffer is empty on a reset data: adding one contact succeeds
  mj_resetData(m, d);
  mjContact con = {0};
  con.dim = 3;
  EXPECT_EQ(mj_addContact(m, d, &con), 0);
  EXPECT_EQ(d->nested, 0);
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
    EXPECT_EQ(data->nested, 0);
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

}  // namespace
}  // namespace mujoco
