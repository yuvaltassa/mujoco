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

// Tests that the models under model/ load and step.

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>  // NOLINT(build/c++17)
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/match.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

// compilation already takes one step, with contacts disabled
constexpr int kNumSteps = 10;

std::vector<std::string> GetModels() {
  std::vector<std::string> models;
  for (const auto& p :
       std::filesystem::recursive_directory_iterator(GetModelPath("."))) {
    std::string xml = p.path().string();
    if (p.path().extension() != ".xml" ||
        // models that need engine plugins, which this test does not load
        absl::StrContains(xml, "/plugin/") ||
        absl::StrContains(xml, "flex/cloth_sdf.xml") ||
        absl::StrContains(xml, "tactile/tactile.xml")) {
      continue;
    }
    models.push_back(xml);
  }
  return models;
}

class ModelsTest : public MujocoTest,
                   public ::testing::WithParamInterface<std::string> {};

TEST_P(ModelsTest, LoadAndStep) {
  const std::string& xml = GetParam();
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml.c_str(), nullptr, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << xml << ": " << error.data();
  mjData* d = mj_makeData(m);
  for (int i = 0; i < kNumSteps; i++) {
    mj_step(m, d);
  }
  mj_deleteData(d);
  mj_deleteModel(m);
}

INSTANTIATE_TEST_SUITE_P(
    AllModels, ModelsTest, ::testing::ValuesIn(GetModels()),
    [](const ::testing::TestParamInfo<std::string>& info) {
      std::string name = std::filesystem::path(info.param).filename().string();
      std::replace_if(
          name.begin(), name.end(), [](char c) { return !std::isalnum(c); },
          '_');
      return name + "_" + std::to_string(info.index);
    });

}  // namespace
}  // namespace mujoco
