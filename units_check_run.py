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
"""Runs units_check on a corpus (design scaffolding).

MUJOCO_SRC=<checkout at or after 1b7d9e83b> python units_check_run.py [model.xml ...]
"""
import os
import sys
import warnings

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import units_check as uc  # noqa: E402

WT = uc.WORKTREE
from scale_prototype_test import FEATURES  # noqa: E402

warnings.filterwarnings('ignore')

MODELS = [
    'model/humanoid/humanoid.xml',
    'model/humanoid/humanoid100.xml',
    'model/adhesion/active_adhesion.xml',
    'test/engine/testdata/actuation/orientation.xml',
    'test/engine/testdata/sensor/insidesite.xml',
    'model/flex/gripper.xml',
    'model/replicate/leaves.xml',
    'model/humanoid/22_humanoids.xml',
    'model/tendon_arm/arm26.xml',
    'model/slider_crank/slider_crank.xml',
    'model/car/car.xml',
    'model/balloons/balloons.xml',
    'test/engine/testdata/actuation/refsite.xml',
    'model/hammock/hammock.xml',
    'model/cube/cube_3x3x3.xml',
    'model/flex/trilinear.xml',
    'model/flex/flag.xml',
    'model/plugin/elasticity/cable.xml',
]

UNITS = [uc.Units(1.7, 0.6, 1.3), uc.Units(0.37, 2.9, 0.8)]

SENSORS = FEATURES.replace(
    '<option timestep="0.002"/>',
    '<option timestep="0.002"><flag energy="enable"/></option>').replace(
    '    <body name="crank"',
    '''    <body name="pend" pos="1 1 1">
      <joint name="ball" type="ball" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0 0 0.2 -0.3" size="0.03"/>
    </body>
    <body name="crank"''').replace('</keyframe>', '''</keyframe>
  <sensor>
    <touch site="prop"/> <accelerometer site="tip"/> <velocimeter site="tip"/> <gyro site="tip"/>
    <force site="rodtip"/> <torque site="rodtip"/>
    <jointpos joint="h1"/> <jointvel joint="h1"/> <jointpos joint="s1"/> <jointvel joint="s1"/>
    <tendonpos tendon="tf_lin"/> <tendonvel tendon="ts"/> <tendonpos tendon="tf_ang"/>
    <actuatorpos actuator="p_s"/> <actuatorvel actuator="m_h"/> <actuatorfrc actuator="p_s"/>
    <actuatorfrc actuator="m_h"/> <jointactuatorfrc joint="s1"/> <jointactuatorfrc joint="h1"/>
    <tendonactuatorfrc tendon="ts"/> <ballquat joint="ball"/> <ballangvel joint="ball"/>
    <jointlimitpos joint="s1"/> <jointlimitvel joint="s1"/> <jointlimitfrc joint="s1"/>
    <tendonlimitpos tendon="tf_lin"/> <tendonlimitvel tendon="tf_lin"/>
    <tendonlimitfrc tendon="tf_lin"/>
    <framepos objtype="body" objname="fore"/> <framequat objtype="body" objname="fore"/>
    <framexaxis objtype="site" objname="tip"/> <framelinvel objtype="site" objname="tip"/>
    <frameangvel objtype="site" objname="tip"/> <framelinacc objtype="site" objname="tip"/>
    <frameangacc objtype="site" objname="tip"/>
    <framepos objtype="site" objname="tip" reftype="body" refname="ball"/>
    <subtreecom body="base"/> <subtreelinvel body="base"/> <subtreeangmom body="base"/>
    <distance geom1="sphere" geom2="boxg" cutoff="10"/> <normal geom1="sphere" geom2="boxg" cutoff="10"/>
    <fromto geom1="sphere" geom2="boxg" cutoff="10"/>
    <e_potential/> <e_kinetic/> <clock/>
  </sensor>''')
SENSORS = SENSORS[:SENSORS.index('<keyframe>')] + SENSORS[SENSORS.index('</keyframe>') + 11:]


def run(name, load, key=None):
  for units in UNITS:
    try:
      qerr, serr, nsens_ok, nsens = uc.check(load, units, nstep=100, key=key,
                                             verbose=units is UNITS[0])
    except Exception as e:  # pylint: disable=broad-except
      print(f'{name:45s} FAILED {type(e).__name__}: {str(e)[:150]}')
      return
    print(f'{name:45s} L={units.L} M={units.M} T={units.T}: qpos err {qerr:.1e}, '
          f'sensors {nsens_ok}/{nsens} err {serr:.1e}')


run('SENSORS', lambda: mujoco.MjSpec.from_string(SENSORS))
run('FEATURES', lambda: mujoco.MjSpec.from_string(FEATURES))
run('FEATURES from keyframe', lambda: mujoco.MjSpec.from_string(FEATURES), key=0)
args = sys.argv[1:] or MODELS
for rel in args:
  path = os.path.join(WT, rel)
  if not os.path.exists(path):
    print(f'{rel:45s} missing')
    continue
  run(rel, lambda path=path: mujoco.MjSpec.from_file(path))
