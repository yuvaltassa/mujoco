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
"""Behavioural checks of the scaling rules in SCALE_DESIGN.md (scaffolding)."""

import os
import sys

import mujoco
import numpy as np

import scale_prototype as sp

REPO = os.path.dirname(os.path.abspath(__file__))

# every actuator shortcut on a hinge and on a slide, passive joint parameters,
# fixed and spatial tendons, joint and tendon equalities, a connect and a weld,
# an explicit contact pair, contacts with margins and torsional friction, a
# propeller-style site actuator with a moment arm, a slider-crank.
FEATURES = """
<mujoco>
  <option timestep="0.002"/>
  <default>
    <geom friction="1 0.01 0.001" margin="0.004" solimp="0.9 0.95 0.002 0.5 2" condim="6"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 .1"/>
    <site name="hook" pos="0.4 0 1.6"/>
    <body name="base" pos="0 0 1.2">
      <joint name="h1" axis="0 1 0" damping="0.2 0.05 0.01" armature="0.02" stiffness="3 0.5 0.1"
             springref="10" frictionloss="0.05" range="-120 120"/>
      <geom name="upper" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.04"/>
      <body name="fore" pos="0 0 -0.4">
        <joint name="h2" axis="0 1 0" damping="0.1" range="-100 100"/>
        <geom name="lower" type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03"/>
        <site name="tip" pos="0 0 -0.3"/>
        <body name="piston" pos="0.05 0 -0.15">
          <joint name="s1" type="slide" axis="1 0 0" range="0 0.2" damping="2 1 0.5" armature="0.1"
                 stiffness="50 20 10" springref="0.05" frictionloss="0.1" margin="0.01"
                 solimplimit="0.9 0.95 0.003"/>
          <geom name="rod" type="box" size="0.05 0.01 0.01" mass="0.3"/>
          <site name="rodtip" pos="0.05 0 0"/>
        </body>
      </body>
    </body>
    <body name="ball" pos="0.5 0.3 0.5">
      <freejoint name="free"/>
      <inertial pos="0.01 0 0" mass="2" fullinertia="0.02 0.03 0.04 0.001 0.002 0.003"/>
      <geom name="sphere" size="0.1"/>
      <site name="prop" pos="0.05 0 0.1"/>
    </body>
    <body name="box" pos="-0.5 0 0.2">
      <freejoint/>
      <geom name="boxg" type="box" size="0.1 0.15 0.05"/>
    </body>
    <body name="slider2" pos="-0.4 0.5 0.6">
      <joint name="s2" type="slide" axis="0 0 1" range="-0.3 0.3"/>
      <geom name="s2g" size="0.05"/>
    </body>
    <body name="crank" pos="0 -0.6 0.5">
      <joint name="hc" axis="0 1 0" damping="0.05"/>
      <geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.02"/>
      <site name="crankpin" pos="0.1 0 0"/>
    </body>
    <body name="sliderbody" pos="0.35 -0.6 0.5" euler="0 90 0">
      <site name="sliderpin"/>
      <geom size="0.01" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="sphere" geom2="floor" margin="0.006" gap="0.001" condim="6"
          friction="0.8 0.8 0.02 0.002 0.002" solimp="0.9 0.95 0.004"/>
    <exclude body1="base" body2="fore"/>
  </contact>
  <tendon>
    <fixed name="tf_ang" stiffness="0.5" damping="0.05" springlength="0.2" frictionloss="0.01">
      <joint joint="h1" coef="1"/> <joint joint="h2" coef="-0.5"/>
    </fixed>
    <fixed name="tf_lin" stiffness="20" damping="1" range="-0.1 0.4" limited="true" margin="0.01">
      <joint joint="s1" coef="1"/> <joint joint="s2" coef="0.5"/>
    </fixed>
    <spatial name="ts" stiffness="30 5 1" damping="1" springlength="0.2 0.4" armature="0.01">
      <site site="tip"/> <site site="rodtip"/>
    </spatial>
    <spatial name="bungee" stiffness="5" damping="0.1">
      <site site="tip"/> <site site="hook"/>
    </spatial>
  </tendon>
  <equality>
    <joint joint1="s2" joint2="s1" polycoef="0.01 0.5 0.8 0 0"/>
  </equality>
  <actuator>
    <motor name="m_h" joint="h1" gear="20" ctrlrange="-1 1" forcerange="-15 15" armature="0.001"
           damping="0.01 0.002"/>
    <position name="p_h" joint="h2" kp="20" kv="1" ctrlrange="-1 1"/>
    <position name="p_s" joint="s1" kp="300" kv="10" ctrlrange="0 0.2" forcerange="-40 40"/>
    <velocity name="v_s" joint="s2" kv="20" ctrlrange="-0.5 0.5"/>
    <intvelocity name="iv_s" joint="s1" kp="100" kv="5" actrange="0 0.2" ctrlrange="-0.3 0.3"/>
    <position name="p_dr" joint="h1" kp="5" dampratio="1" timeconst="0.05" ctrlrange="-1 1"/>
    <damper name="d_s" joint="s2" kv="10" ctrlrange="0 1"/>
    <cylinder name="c_s" joint="s1" area="0.01" timeconst="0.1" bias="1 -2 -0.5" ctrlrange="0 100"/>
    <general name="g_spring" joint="s2" gainprm="30" biastype="affine" biasprm="0 -100 -3"
             ctrlrange="-1 1"/>
    <general name="g_affine" joint="s1" gaintype="affine" gainprm="5 2 1" ctrlrange="-1 1"/>
    <motor name="m_t" tendon="ts" gear="2" ctrlrange="-1 1"/>
    <position name="p_tf" tendon="tf_ang" kp="2" ctrlrange="-1 1"/>
    <motor name="prop" site="prop" gear="0 0 1 0 0 0.05" ctrlrange="0 30"/>
    <motor name="spin" site="prop" gear="0 0 0 0 1 0" ctrlrange="-0.2 0.2"/>
    <motor name="freelin" joint="free" gear="1 0 0 0 0.1 0" ctrlrange="-2 2"/>
    <general name="sc" cranksite="crankpin" slidersite="sliderpin" cranklength="0.3" gainprm="5"
             biastype="affine" biasprm="0 -20 -1" ctrlrange="-1 1"/>
    <adhesion name="adh" body="box" gain="5" ctrlrange="0 1"/>
  </actuator>
  <keyframe>
    <key name="k" qpos="0.3 -0.2 0.05  0.5 0.3 0.5 1 0 0 0  -0.5 0 0.2 1 0 0 0  0.1  0.2"
         ctrl="0 0.2 0.1 0 0 0 0 0 0 0 0 0 10 0 0 0 0"/>
  </keyframe>
</mujoco>
"""

MODELS = [
    'model/humanoid/humanoid.xml',
    'model/tendon_arm/arm26.xml',
    'model/slider_crank/slider_crank.xml',
    'model/car/car.xml',
    'model/cube/cube_3x3x3.xml',
    'model/hammock/hammock.xml',
    'model/balloons/balloons.xml',
    'test/engine/testdata/actuation/refsite.xml',
]


def report(passed):
  print('  PASS' if passed else '  FAIL')
  return passed


BOUNDARY = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 .1"/>
    <site name="hook" pos="0 0 3"/>
    <body name="arm" pos="0 0 1">
      <joint name="hinge" axis="0 1 0"/>
      <geom name="hand" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05"/>
      <site name="tip" pos="0 0 -0.5"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="hand" geom2="floor" margin="0.01" friction="1 1 0.005 0.0001 0.0001"/>
  </contact>
  <equality>
    <connect body1="arm" body2="world" anchor="0 0 0.2"/>
  </equality>
  <tendon>
    <spatial name="bungee" stiffness="100" springlength="0.8">
      <site site="tip"/> <site site="hook"/>
    </spatial>
    <spatial name="inside" stiffness="100" springlength="0.3">
      <site site="tip"/> <site site="tip2"/>
    </spatial>
  </tendon>
</mujoco>
""".replace('<site name="tip" pos="0 0 -0.5"/>',
            '<site name="tip" pos="0 0 -0.5"/> <site name="tip2" pos="0.1 0 -0.2"/>')


def boundary_numbers(s=2.0):
  """Numbers for the boundary example in the design doc."""
  spec = mujoco.MjSpec.from_string(BOUNDARY)
  gauge = sp.Gauge.similar(s)
  sp.Scaler(spec, lambda n: gauge if n == 'arm' else sp.IDENTITY).apply()
  eq, pair = spec.equalities[0], spec.pairs[0]
  print('  connect anchor', np.array(eq.data)[:3])
  print('  pair margin', pair.margin, 'friction', np.array(pair.friction))
  for t in spec.tendons:
    print(f'  tendon {t.name}: springlength {np.array(t.springlength)} stiffness '
          f'{np.array(t.stiffness)[0]:.4g}')
  return True


PENDULUM = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 .1" pos="0 0 -3"/>
    <body name="arm" pos="0 0 1">
      <joint name="hinge" axis="0 1 0" damping="0.1" armature="0.01"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05"/>
    </body>
  </worldbody>
  <actuator>
    <position name="servo" joint="hinge" kp="100" kv="10" ctrlrange="-1 1" forcerange="-50 50"/>
  </actuator>
</mujoco>
"""


def pendulum_table(s=2.0):
  """Numbers for the worked example in the design doc; subtree scaling."""
  rows = {}
  for policy in ('unscaled', 'similar', 'geometry'):
    spec = mujoco.MjSpec.from_string(PENDULUM)
    if policy != 'unscaled':
      gauge = getattr(sp.Gauge, policy)(s)
      gauge_of = lambda name, g=gauge: g if name == 'arm' else sp.IDENTITY
      sp.Scaler(spec, gauge_of).apply()
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    inertia = np.zeros((m.nv, 1))
    mujoco.mj_mulM(m, d, inertia, np.ones((m.nv, 1)))
    inertia = inertia[0, 0]  # about the hinge, including armature
    mgl = m.body_mass[1] * 9.81 * np.linalg.norm(m.body_ipos[1])
    kp = m.actuator_gainprm[0, 0]
    rows[policy] = dict(
        body_pos=m.body_pos[1].copy(),
        mass=m.body_mass[1],
        inertia=inertia,
        armature=m.dof_armature[0],
        damping=m.dof_damping[0],
        kp=kp,
        kv=-m.actuator_biasprm[0, 2],
        forcerange=m.actuator_forcerange[0].copy(),
        ctrlrange=m.actuator_ctrlrange[0].copy(),
        gear=m.actuator_gear[0, 0],
        servo_hz=np.sqrt(kp / inertia) / (2 * np.pi),
        sag=mgl / kp,
        pendulum_hz=np.sqrt(mgl / inertia) / (2 * np.pi),
    )
  base = rows['unscaled']
  for policy, row in rows.items():
    print(f'  {policy}')
    for k, v in row.items():
      with np.errstate(divide='ignore', invalid='ignore'):
        ratio = np.asarray(v) / np.asarray(base[k])
      ratio = ratio[np.isfinite(ratio)]
      print(f'    {k:12s} {np.round(v, 5)!s:26s} x{np.round(ratio.mean(), 4) if ratio.size else 1}')

  # behaviour: the scaled arm in gravity g moves like the original in g / s
  spec0 = mujoco.MjSpec.from_string(PENDULUM)
  spec0.option.gravity = np.array(spec0.option.gravity) / s
  spec1 = mujoco.MjSpec.from_string(PENDULUM)
  gauge = sp.Gauge.similar(s)
  sp.Scaler(spec1, lambda n: gauge if n == 'arm' else sp.IDENTITY).apply()
  m0, m1 = spec0.compile(), spec1.compile()
  ctrl = lambda i: np.array([0.8 * np.sin(0.01 * i)])
  q0, q1 = sp.rollout(m0, 2000, ctrl), sp.rollout(m1, 2000, ctrl)
  err = np.abs(q0 - q1).max()
  print(f'  similar subtree in g vs original in g/s, same ctrl: max angle error {err:.2e}'
        f' over a swing of {np.ptp(q0):.2f} rad')
  return err < 1e-9


def main():
  np.set_printoptions(precision=4, suppress=True, linewidth=140)
  ok = True
  for s in (2.0, 0.37):
    print(f'FEATURES, whole model, similar s={s}')
    err = sp.check_similar(FEATURES, s, nstep=400, from_string=True)
    ok &= report(err[:50].max() < 1e-12)
    print(f'FEATURES from keyframe, s={s}')
    err = sp.check_similar(FEATURES, s, nstep=400, from_string=True, key=0)
    ok &= report(err[:50].max() < 1e-12)
  for rel in MODELS:
    path = os.path.join(REPO, rel)
    if not os.path.exists(path):
      print('missing', rel)
      continue
    print(rel)
    try:
      err = sp.check_similar(path, 1.7, nstep=300)
      ok &= report(err[:50].max() < 1e-6)
    except Exception as e:  # pylint: disable=broad-except
      print('  FAILED:', type(e).__name__, str(e)[:200])
      ok = False
  print('pendulum example')
  ok &= pendulum_table()
  print('boundary example')
  ok &= boundary_numbers()
  print('ALL OK' if ok else 'SOME CHECKS FAILED')
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())
