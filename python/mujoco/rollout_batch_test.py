# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""rollout_test.py's tests, run on rollout_batch, mujoco.rollout over mujoco.batch."""

from absl.testing import absltest
from mujoco import rollout_batch
from mujoco import rollout_test
from mujoco.rollout_test import MuJoCoRolloutTest  # pylint: disable=unused-import

# the tests call the module through this global
rollout_test.rollout = rollout_batch

if __name__ == '__main__':
  absltest.main()
