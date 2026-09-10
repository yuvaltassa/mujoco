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

// Tests for error recovery: an error raised inside a pipeline call is caught at
// the boundary of the call, which records it in mjData.status.

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_errmem.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::NotNull;

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

// records errors and returns, so that the engine recovers at its boundary;
// warnings still go to the test fixture
class RecoveryTest : public MujocoTest {
 protected:
  void SetUp() override {
    last_error_.clear();
    last_kind_ = 0;
    prev_handler_ = mju_setLogHandler(RecordErrors);
  }
  void TearDown() override { mju_setLogHandler(prev_handler_); }

  static void RecordErrors(const mjLogMessage* msg) {
    if (msg->level == mjLOG_ERROR) {
      last_error_ =
          std::string(msg->func ? msg->func : "") + ": " + msg->subject;
      last_kind_ = msg->status;
      return;
    }
    prev_handler_(msg);
  }

  static std::string last_error_;
  static int last_kind_;
  static mjfLogHandler prev_handler_;
};

std::string RecoveryTest::last_error_;
int RecoveryTest::last_kind_ = 0;
mjfLogHandler RecoveryTest::prev_handler_ = nullptr;

// an error in the top-level function: the call is abandoned, the data reports
// it
TEST_F(RecoveryTest, ErrorIsRecoveredAtTheBoundary) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  model->opt.integrator = 99;
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_INPUT);
  EXPECT_THAT(last_error_, HasSubstr("invalid integrator"));
  EXPECT_EQ(data->time, 0);
  EXPECT_EQ(data->nested, 0);
  EXPECT_EQ(data->pstack, 0);
}

// a pending error refuses every pipeline call until the data is reset
TEST_F(RecoveryTest, PendingErrorRefusesUntilReset) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  model->opt.integrator = 99;
  mj_step(model.get(), data.get());
  ASSERT_EQ(data->status, mjSTATUS_INPUT);

  // fixing the input is not enough: the data is refused, with an error naming
  // the cause
  model->opt.integrator = mjINT_EULER;
  last_error_.clear();
  mj_step(model.get(), data.get());
  EXPECT_THAT(last_error_, HasSubstr("pending error"));
  EXPECT_EQ(data->status, mjSTATUS_INPUT);
  EXPECT_EQ(data->time, 0);
  last_error_.clear();
  mj_forward(model.get(), data.get());
  EXPECT_THAT(last_error_, HasSubstr("pending error"));
  EXPECT_EQ(data->nested, 0);

  // reset clears the error, the data is usable again
  mj_resetData(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_OK);
  last_error_.clear();
  mj_step(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
  EXPECT_EQ(last_error_, "");
  EXPECT_GT(data->time, 0);
}

// an error raised from a callback unwinds to the boundary of the enclosing call
mjfGeneric g_prev_control = nullptr;

TEST_F(RecoveryTest, ErrorInCallbackUnwindsTheCall) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  struct Guard {
    Guard() {
      g_prev_control = mjcb_control;
      mjcb_control = +[](const mjModel* m, mjData* d) {
        mju_error("controller failed at time %g", d->time);
      };
    }
    ~Guard() { mjcb_control = g_prev_control; }
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_ERROR);
  EXPECT_THAT(last_error_, HasSubstr("controller failed at time 0"));
  EXPECT_EQ(data->time, 0);
  EXPECT_EQ(data->nested, 0);
}

// an interrupt (a host exception in a callback) unwinds the call without
// recording an error
TEST_F(RecoveryTest, InterruptUnwindsWithoutError) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  struct Guard {
    Guard() {
      g_prev_control = mjcb_control;
      mjcb_control =
          +[](const mjModel* m, mjData* d) { _mjPRIVATE__interrupt(); };
    }
    ~Guard() { mjcb_control = g_prev_control; }
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_TRUE(_mjPRIVATE__takeInterrupt());
  EXPECT_EQ(data->status, mjSTATUS_OK);
  EXPECT_EQ(last_error_, "");
  EXPECT_EQ(data->time, 0);
  EXPECT_EQ(data->nested, 0);
  EXPECT_EQ(data->pstack, 0);

  // the data is not poisoned: the next call runs
  mjcb_control = g_prev_control;
  mj_step(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
  EXPECT_GT(data->time, 0);
}

// recovery through the default handler keeps the enclosing boundaries armed: a
// nested call on another data fails inside a callback, then the enclosing call
// fails and is recovered
mjData* g_scratch = nullptr;
struct Probe {
  bool nested_recovered = false;
  bool armed_after = false;
  int control_calls = 0;
  int reentries = 0;
};
Probe g_probe;

TEST_F(RecoveryTest, DefaultHandlerKeepsEnclosingBoundaries) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  MjDataPtr scratch = MakeData(model);
  g_scratch = scratch.get();
  g_probe = Probe();

  // the default handler: prints the message and recovers at the boundary
  mju_setLogHandler(nullptr);

  struct Guard {
    Guard() {
      g_prev_control = mjcb_control;
      mjcb_control = +[](const mjModel* m, mjData* d) {
        // the scratch data uses the Euler integrator: mj_implicit fails and is
        // recovered
        mj_implicit(m, g_scratch);
        g_probe.nested_recovered = g_scratch->status < 0;

        // a further nested call, then the enclosing call must still be
        // protected
        mj_checkPos(m, d);
        g_probe.armed_after = _mjPRIVATE__hasBoundary();
        mju_error("the enclosing call fails after the nested recovery");
      };
    }
    ~Guard() { mjcb_control = g_prev_control; }
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_TRUE(g_probe.nested_recovered);
  EXPECT_TRUE(g_probe.armed_after);
  EXPECT_EQ(data->status, mjSTATUS_ERROR);
  EXPECT_EQ(data->nested, 0);
  EXPECT_FALSE(_mjPRIVATE__hasBoundary());
}

// a failed stage stops the call: nothing after it runs, in particular no
// callback
TEST_F(RecoveryTest, FailedStageStopsTheCall) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  g_probe = Probe();

  struct Guard {
    Guard() {
      g_prev_control = mjcb_control;
      prev_passive = mjcb_passive;
      mjcb_passive =
          +[](const mjModel* m, mjData* d) { mju_error("passive failed"); };
      mjcb_control =
          +[](const mjModel* m, mjData* d) { g_probe.control_calls++; };
    }
    ~Guard() {
      mjcb_control = g_prev_control;
      mjcb_passive = prev_passive;
    }
    mjfGeneric prev_passive;
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_ERROR);
  EXPECT_THAT(last_error_, HasSubstr("passive failed"));
  EXPECT_EQ(g_probe.control_calls, 0);
  EXPECT_EQ(data->time, 0);
  EXPECT_EQ(data->nested, 0);
}

// every pipeline function is a boundary: an error in a sensor callback of a
// nested call on another data is attributed to that data and returns to the
// callback that made the call
TEST_F(RecoveryTest, NestedCallOnAnotherDataIsItsOwnBoundary) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="slide" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <sensor>
      <user dim="1" needstage="pos"/>
    </sensor>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  MjDataPtr scratch = MakeData(model);
  g_scratch = scratch.get();
  g_probe = Probe();

  struct Guard {
    Guard() {
      g_prev_control = mjcb_control;
      prev_sensor = mjcb_sensor;
      mjcb_sensor = +[](const mjModel* m, mjData* d, int stage) {
        if (d == g_scratch) {
          mju_error("sensor failed");
        }
      };
      mjcb_control = +[](const mjModel* m, mjData* d) {
        mj_sensorPos(m, g_scratch);
        g_probe.control_calls++;  // the callback continues after the nested
                                  // failure
      };
    }
    ~Guard() {
      mjcb_control = g_prev_control;
      mjcb_sensor = prev_sensor;
    }
    mjfSensor prev_sensor;
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_EQ(scratch->status, mjSTATUS_ERROR);
  EXPECT_TRUE(mjOK(data.get()));
  EXPECT_EQ(g_probe.control_calls, 1);
  EXPECT_GT(data->time, 0);
  EXPECT_EQ(scratch->nested, 0);

  // called directly, the sensor stage is a boundary too
  mj_resetData(model.get(), scratch.get());
  last_error_.clear();
  mj_sensorPos(model.get(), scratch.get());
  EXPECT_EQ(scratch->status, mjSTATUS_ERROR);
  EXPECT_THAT(last_error_, HasSubstr("sensor failed"));
}

// every callback that receives a mutable mjData can re-enter the pipeline on
// it: the nested call arms its own boundary, its error returns to the callback,
// and the statement after it runs
static void Reenter(const mjModel* m, mjData* d) {
  g_probe.control_calls++;
  mj_implicit(m, d);  // the Euler integrator: mj_implicit fails at its entry
  g_probe.armed_after = _mjPRIVATE__hasBoundary();
  g_probe.nested_recovered = d->status < 0;
  g_probe.reentries++;
}

// a callback that abandons the call is not invoked again for the remaining
// elements of the stage that invokes it
int g_filter_calls = 0;
int g_filter_calls_stopped = 0;
mjfConFilt g_prev_filter = nullptr;

TEST_F(RecoveryTest, NoCallbackRunsOnAbandonedData) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option><flag midphase="disable"/></option>
    <worldbody>
      <body pos="0 0 .1">
        <freejoint/>
        <geom size=".1"/>
        <geom size=".1" pos=".3 0 0"/>
      </body>
      <body pos="0 0 .25">
        <freejoint/>
        <geom size=".1"/>
        <geom size=".1" pos=".3 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  g_filter_calls = g_filter_calls_stopped = 0;

  struct Guard {
    Guard() {
      g_prev_filter = mjcb_contactfilter;
      mjcb_contactfilter = +[](const mjModel* m, mjData* d, int g1, int g2) {
        g_filter_calls++;
        if (d->status < 0) g_filter_calls_stopped++;
        // the model integrates with Euler, so the nested call fails at its
        // first check, before it does any work on the data
        if (g_filter_calls == 1) mj_implicit(m, d);
        return 0;
      };
    }
    ~Guard() { mjcb_contactfilter = g_prev_filter; }
  } guard;

  // four geom pairs are candidates, but the filter fails on the first one
  mj_step(model.get(), data.get());
  EXPECT_LT(data->status, 0);
  EXPECT_EQ(g_filter_calls, 1);
  EXPECT_EQ(g_filter_calls_stopped, 0);
  EXPECT_EQ(data->nested, 0);
  EXPECT_EQ(data->pstack, 0);
}

// a chained observer keeps the default handler's termination outside a pipeline
// call, and its recovery inside one
mjfLogHandler g_chained_prev = nullptr;

TEST_F(RecoveryTest, ChainedDefaultHandlerRecoversInsideACall) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  // the observer pattern the programming guide recommends: chain to the
  // previous handler, which here is the default one
  g_chained_prev = mju_setLogHandler(nullptr);
  mju_setLogHandler(+[](const mjLogMessage* msg) { g_chained_prev(msg); });

  model->opt.integrator = 99;
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_INPUT);
  EXPECT_EQ(data->nested, 0);
  EXPECT_FALSE(_mjPRIVATE__hasBoundary());
}

// a callback that abandons the call stops the stage that invoked it, including
// the second callback of the same element
int g_gain_calls2 = 0;
int g_bias_calls2 = 0;

TEST_F(RecoveryTest, GainFailureStopsBeforeBias) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody><body><joint name="j" type="slide"/><geom size=".1"/></body></worldbody>
    <actuator><general joint="j" gaintype="user" biastype="user"/></actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  g_gain_calls2 = g_bias_calls2 = 0;

  struct Guard {
    Guard() {
      prev_gain_ = mjcb_act_gain;
      prev_bias_ = mjcb_act_bias;
      mjcb_act_gain = +[](const mjModel* m, const mjData* d, int id) -> mjtNum {
        g_gain_calls2++;
        mj_implicit(m, const_cast<mjData*>(d));  // fails at once under Euler
        return 1;
      };
      mjcb_act_bias = +[](const mjModel* m, const mjData* d, int id) -> mjtNum {
        g_bias_calls2++;
        return 0;
      };
    }
    ~Guard() {
      mjcb_act_gain = prev_gain_;
      mjcb_act_bias = prev_bias_;
    }
    mjfAct prev_gain_;
    mjfAct prev_bias_;
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_LT(data->status, 0);
  EXPECT_EQ(g_gain_calls2, 1);
  EXPECT_EQ(g_bias_calls2, 0);
  EXPECT_EQ(data->pstack, 0);
}

// the same for a callback invoked once per element of a stage
int g_gain_calls = 0;
int g_gain_calls_stopped = 0;

TEST_F(RecoveryTest, NoPerElementCallbackRunsOnAbandonedData) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body><joint name="j1" type="slide"/><geom size=".1"/></body>
      <body pos="1 0 0"><joint name="j2" type="slide"/><geom size=".1"/></body>
      <body pos="2 0 0"><joint name="j3" type="slide"/><geom size=".1"/></body>
    </worldbody>
    <actuator>
      <general joint="j1" gaintype="user"/>
      <general joint="j2" gaintype="user"/>
      <general joint="j3" gaintype="user"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  g_gain_calls = g_gain_calls_stopped = 0;

  struct Guard {
    Guard() {
      prev_ = mjcb_act_gain;
      mjcb_act_gain = +[](const mjModel* m, const mjData* d, int id) -> mjtNum {
        g_gain_calls++;
        if (d->status < 0) g_gain_calls_stopped++;
        // the model integrates with Euler, so the nested call fails at once
        if (g_gain_calls == 1) mj_implicit(m, const_cast<mjData*>(d));
        return 1;
      };
    }
    ~Guard() { mjcb_act_gain = prev_; }
    mjfAct prev_;
  } guard;

  // three actuators, but the gain callback abandons the call on the first
  mj_step(model.get(), data.get());
  EXPECT_LT(data->status, 0);
  EXPECT_EQ(g_gain_calls, 1);
  EXPECT_EQ(g_gain_calls_stopped, 0);
  EXPECT_EQ(data->nested, 0);
  EXPECT_EQ(data->pstack, 0);
}

TEST_F(RecoveryTest, CallbacksReenterTheirOwnData) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom size=".1"/>
      <body>
        <joint name="slide" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="slide" dyntype="user" gaintype="user" biastype="user"/>
    </actuator>
    <sensor>
      <user dim="1" needstage="pos"/>
    </sensor>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  struct Guard {
    Guard() {
      control = mjcb_control;
      passive = mjcb_passive;
      sensor = mjcb_sensor;
      filter = mjcb_contactfilter;
    }
    ~Guard() {
      mjcb_control = control;
      mjcb_passive = passive;
      mjcb_sensor = sensor;
      mjcb_contactfilter = filter;
    }
    mjfGeneric control, passive;
    mjfSensor sensor;
    mjfConFilt filter;
  } guard;

  struct Category {
    const char* name;
    void (*install)();
  };
  const Category categories[] = {
      {"control",
       +[] {
         mjcb_control = +[](const mjModel* m, mjData* d) { Reenter(m, d); };
       }},
      {"passive",
       +[] {
         mjcb_passive = +[](const mjModel* m, mjData* d) { Reenter(m, d); };
       }},
      {"sensor",
       +[] {
         mjcb_sensor =
             +[](const mjModel* m, mjData* d, int stage) { Reenter(m, d); };
       }},
      {"contactfilter",
       +[] {
         mjcb_contactfilter = +[](const mjModel* m, mjData* d, int g1, int g2) {
           Reenter(m, d);
           return 0;
         };
       }},
  };

  for (const Category& category : categories) {
    mjcb_control = mjcb_passive = nullptr;
    mjcb_sensor = nullptr;
    mjcb_contactfilter = nullptr;
    category.install();
    g_probe = Probe();
    mj_resetData(model.get(), data.get());
    last_error_.clear();

    mj_step(model.get(), data.get());
    EXPECT_GT(g_probe.control_calls, 0) << category.name;
    EXPECT_EQ(g_probe.reentries, g_probe.control_calls) << category.name;
    EXPECT_TRUE(g_probe.nested_recovered) << category.name;
    EXPECT_TRUE(g_probe.armed_after) << category.name;
    EXPECT_EQ(data->status, mjSTATUS_INPUT) << category.name;
    EXPECT_THAT(last_error_, HasSubstr("integrator must be implicit"))
        << category.name;
    EXPECT_EQ(data->nested, 0) << category.name;
    EXPECT_EQ(data->pstack, 0) << category.name;
    EXPECT_FALSE(_mjPRIVATE__hasBoundary()) << category.name;
  }
}

// the stop paths restore the stack frame of the call
TEST_F(RecoveryTest, StopPathsRestoreTheStack) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="slide" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <sensor>
      <user dim="1" needstage="pos"/>
    </sensor>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.onwarn = mjONWARN_STOP;

  // a position-stage sensor raises a warning: every driver stops after its
  // sensor stage
  struct Guard {
    Guard() {
      prev = mjcb_sensor;
      mjcb_sensor = +[](const mjModel* m, mjData* d, int stage) {
        mj_warning(d, mjWARN_BADCTRL, 0);
      };
    }
    ~Guard() { mjcb_sensor = prev; }
    mjfSensor prev;
  } guard;
  mock_warning_handler.ExpectWarnings("Nan, Inf or huge value in CTRL");

  struct Entry {
    const char* name;
    void (*fn)(const mjModel*, mjData*);
  };
  const Entry entries[] = {
      {"mj_step", mj_step},           {"mj_step1", mj_step1},
      {"mj_forward", mj_forward},     {"mj_inverse", mj_inverse},
      {"mj_sensorPos", mj_sensorPos},
  };
  for (const Entry& entry : entries) {
    for (int i = 0; i < 3; i++) {
      entry.fn(model.get(), data.get());
      EXPECT_EQ(data->status, mjSTATUS_BADCTRL) << entry.name;
      EXPECT_EQ(data->pstack, 0) << entry.name << " call " << i;
      EXPECT_EQ(data->pbase, 0) << entry.name << " call " << i;
      EXPECT_EQ(data->nested, 0) << entry.name;
    }
  }
}

// stack overflow is reported as out of memory, the stack frame is restored
TEST_F(RecoveryTest, StackOverflowIsOutOfMemory) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  struct Guard {
    Guard() {
      g_prev_control = mjcb_control;
      mjcb_control = +[](const mjModel* m, mjData* d) {
        mj_markStack(d);
        mj_stackAllocNum(d, d->narena);
        mj_freeStack(d);
      };
    }
    ~Guard() { mjcb_control = g_prev_control; }
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_OOM);
  EXPECT_EQ(last_kind_, mjSTATUS_OOM);
  EXPECT_THAT(last_error_, HasSubstr("stack overflow"));
  EXPECT_EQ(data->pstack, 0);
  EXPECT_EQ(data->pbase, 0);
  EXPECT_EQ(data->nested, 0);
}

// an error raised by code that does not classify it is generic in the status
TEST_F(RecoveryTest, UnclassifiedErrorIsGeneric) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  struct Guard {
    Guard() {
      g_prev_control = mjcb_control;
      mjcb_control = +[](const mjModel* m, mjData* d) {
        mju_error("failure in user code");
      };
    }
    ~Guard() { mjcb_control = g_prev_control; }
  } guard;

  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_ERROR);
  EXPECT_EQ(last_kind_, mjSTATUS_ERROR);
  EXPECT_THAT(last_error_, HasSubstr("failure in user code"));
}

// an engine invariant that breaks is a bug in MuJoCo, not the caller's to fix
TEST_F(RecoveryTest, BrokenInvariantIsInternal) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  // the transmission type is validated by the compiler, so the dispatch never
  // defaults unless something upstream of it is wrong
  model->actuator_trntype[0] = 99;
  mj_step(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_INTERNAL);
  EXPECT_EQ(last_kind_, mjSTATUS_INTERNAL);
  EXPECT_THAT(last_error_, HasSubstr("unknown transmission type"));
}

// an error in a worker thread is raised on the calling thread after the batch
int FailingCollision(const mjModel* m, mjData* d, mjPreContact* con, int g1,
                     int g2, mjtNum margin) {
  mju_error("collision of geoms %d and %d failed", g1, g2);
  return 0;
}

TEST_F(RecoveryTest, WorkerThreadErrorIsRaisedOnTheCallingThread) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <replicate count="4" offset="0.05 0 0">
        <body>
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
  mju_threadpool(data.get(), 2);

  struct Guard {
    Guard() {
      prev = mjCOLLISIONFUNC[mjGEOM_SPHERE][mjGEOM_SPHERE];
      mjCOLLISIONFUNC[mjGEOM_SPHERE][mjGEOM_SPHERE] = FailingCollision;
    }
    ~Guard() { mjCOLLISIONFUNC[mjGEOM_SPHERE][mjGEOM_SPHERE] = prev; }
    mjfCollision prev;
  } guard;

  mj_forward(model.get(), data.get());
  EXPECT_EQ(data->status, mjSTATUS_ERROR);
  EXPECT_THAT(last_error_, HasSubstr("collision of geoms"));
  EXPECT_EQ(data->threadlock, 0);
  EXPECT_EQ(data->pstack, 0);
  EXPECT_EQ(data->nested, 0);

  // reset, restore the collider: the data and its thread pool work again
  mj_resetData(model.get(), data.get());
  mjCOLLISIONFUNC[mjGEOM_SPHERE][mjGEOM_SPHERE] = guard.prev;
  mj_forward(model.get(), data.get());
  EXPECT_TRUE(mjOK(data.get()));
  EXPECT_GT(data->ncon, 0);
}

}  // namespace
}  // namespace mujoco
