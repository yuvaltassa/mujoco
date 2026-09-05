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

// Tests for the onwarn option and mjtStatus returns.

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

// auto (default): divergence resets the state, status reports the warning
TEST_F(OnWarnTest, AutoResetsAndReturnsStatus) {
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in QPOS");

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  EXPECT_EQ(model->opt.onwarn, mjONWARN_AUTO);

  // healthy step: zero status, mirrored in mjData.status
  mjtStatus status = mj_step(model.get(), data.get());
  EXPECT_EQ(status, mjSTATUS_OK);
  EXPECT_EQ(data->status, mjSTATUS_OK);
  EXPECT_GT(data->time, 0);

  // inject bad qpos: step reports BADQPOS and resets the state
  data->qpos[0] = kNaN;
  status = mj_step(model.get(), data.get());
  EXPECT_EQ(status & mjSTATUS_BADQPOS, mjSTATUS_BADQPOS);
  EXPECT_EQ(data->status, status);
  EXPECT_TRUE(std::isfinite(data->qpos[0]));

  // warning statistics survive the reset, counted once
  EXPECT_EQ(data->warning[mjWARN_BADQPOS].number, 1);
  EXPECT_EQ(data->warning[mjWARN_BADQPOS].lastinfo, 0);
}

// continue: no reset, the warning is recorded and NaNs propagate
TEST_F(OnWarnTest, ContinueRecordsWithoutReset) {
  mock_warning_handler.ExpectWarnings();

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_CONTINUE;

  // inject bad qvel: the step proceeds, bad velocity begets bad acceleration
  data->qvel[0] = kNaN;
  mjtStatus status = mj_step(model.get(), data.get());
  EXPECT_EQ(status & mjSTATUS_BADQVEL, mjSTATUS_BADQVEL);
  EXPECT_EQ(status & mjSTATUS_BADQACC, mjSTATUS_BADQACC);

  // no reset: time advanced, state remains bad, each warning counted once
  EXPECT_GT(data->time, 0);
  EXPECT_TRUE(std::isnan(data->qvel[0]));
  EXPECT_EQ(data->warning[mjWARN_BADQVEL].number, 1);
  EXPECT_EQ(data->warning[mjWARN_BADQACC].number, 1);
}

// stop: the step returns at the first warning, state untouched
TEST_F(OnWarnTest, StopReturnsAtFirstWarning) {
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in QVEL");

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  // inject bad qvel, save qpos
  data->qvel[0] = kNaN;
  mjtNum qpos = data->qpos[0];

  // exactly one bit is set, nothing was stepped
  mjtStatus status = mj_step(model.get(), data.get());
  EXPECT_EQ(status, mjSTATUS_BADQVEL);
  EXPECT_EQ(data->status, mjSTATUS_BADQVEL);
  EXPECT_EQ(data->time, 0);
  EXPECT_EQ(data->qpos[0], qpos);

  // stepping again without fixing anything returns again
  status = mj_step(model.get(), data.get());
  EXPECT_EQ(status, mjSTATUS_BADQVEL);
  EXPECT_EQ(data->warning[mjWARN_BADQVEL].number, 2);
}

// stop: bad ctrl returns before actuation, d->ctrl left pristine for inspection
TEST_F(OnWarnTest, StopPreservesBadCtrl) {
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in CTRL");

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  data->ctrl[0] = kNaN;
  mjtStatus status = mj_step(model.get(), data.get());
  EXPECT_EQ(status, mjSTATUS_BADCTRL);
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
  mjtStatus status = mj_step(model.get(), data.get());
  EXPECT_EQ(status, mjSTATUS_BADCTRL);
  EXPECT_GT(data->time, 0);
  EXPECT_TRUE(std::isfinite(data->qpos[0]));
}

// mj_step1/mj_step2 report through the same channel
TEST_F(OnWarnTest, Step1Step2ReturnStatus) {
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in QPOS");

  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  data->qpos[0] = kNaN;
  EXPECT_EQ(mj_step1(model.get(), data.get()), mjSTATUS_BADQPOS);
  EXPECT_EQ(data->time, 0);
}

// arena exhaustion: auto completes degraded, stop returns at the boundary
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

  // auto: forward completes with a partial contact list
  mjtStatus status = mj_forward(model.get(), data.get());
  EXPECT_EQ(status & mjSTATUS_CONTACTFULL, mjSTATUS_CONTACTFULL);

  // stop: forward returns at the collision stage, before constraint assembly
  mj_resetData(model.get(), data.get());
  model->opt.onwarn = mjONWARN_STOP;
  status = mj_forward(model.get(), data.get());
  EXPECT_EQ(status, mjSTATUS_CONTACTFULL);
  EXPECT_EQ(data->nefc, 0);
}

// XML: onwarn keyword roundtrip and autoreset deprecation mapping
TEST_F(OnWarnTest, XmlOnwarnAndDeprecatedAutoreset) {
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
  EXPECT_THAT(model2.get(), ::testing::IsNull());
  EXPECT_THAT(error, HasSubstr("autoreset"));
}

}  // namespace
}  // namespace mujoco
