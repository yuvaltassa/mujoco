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
"""Reference prototype of the scaling rules in SCALE_DESIGN.md.

Design scaffolding, not for landing. Applies the rules destructively to an
mjSpec in Python, so that the dimension assignments and the boundary rules of
the design can be checked against simulation before any C++ is written.

The key behavioural check: a subtree scaled by s under the "similar" policy,
in gravity g, must move exactly like the original in gravity g / s, with
lengths multiplied by s and angles unchanged. Any field with a wrong
dimension breaks this.
"""

import dataclasses
import math
from typing import Callable, Optional

import mujoco
import numpy as np

mjtJoint = mujoco.mjtJoint
mjtTrn = mujoco.mjtTrn
mjtObj = mujoco.mjtObj
mjtEq = mujoco.mjtEq
mjtSensor = mujoco.mjtSensor


@dataclasses.dataclass(frozen=True)
class Gauge:
  """Accumulated transformation carried by an element.

  shape: length factor of the shape gauge (mass factor is shape**3).
  lf, mf: length and mass factors of the force gauge.
  """

  shape: float = 1.0
  lf: float = 1.0
  mf: float = 1.0

  def g(self, a, b=0):
    """Factor for a shape-class field of dimension L^a M^b."""
    return self.shape ** (a + 3 * b)

  def f(self, a, b):
    """Factor for a force-class field of dimension L^a M^b."""
    return self.lf**a * self.mf**b

  @staticmethod
  def similar(s):
    return Gauge(s, s, s**3)

  @staticmethod
  def geometry(s):
    return Gauge(s, 1.0, 1.0)

  @staticmethod
  def mean(gauges):
    n = len(gauges)
    gm = lambda xs: math.exp(sum(math.log(x) for x in xs) / n)
    return Gauge(
        gm([x.shape for x in gauges]),
        gm([x.lf for x in gauges]),
        gm([x.mf for x in gauges]),
    )


IDENTITY = Gauge()


def _defined(x):
  return not np.isnan(np.asarray(x, dtype=float).flat[0])


def _genforce(lin):
  """Dimension (a, b) of the generalized force conjugate to a coordinate."""
  return (1, 1) if lin else (2, 1)


class Scaler:
  """Applies gauges to a spec. gauge_of(body_name) -> Gauge."""

  def __init__(self, spec: mujoco.MjSpec, gauge_of: Callable[[str], Gauge]):
    self.spec = spec
    # structure queries only. Compiled from a copy: a frame's pose is cached at
    # its first compile, so edits to a compiled spec's frames would be ignored.
    self.model = spec.copy().compile()
    self.gauge_of = gauge_of
    self.ctrl_factor = np.ones(self.model.nu)
    self.act_factor = np.ones(self.model.na)

  # ---- ownership helpers ---------------------------------------------------

  def body_gauge(self, body):
    return self.gauge_of(body.name)

  def _joint_gauge(self, name):
    return self.body_gauge(self.spec.joint(name).parent)

  def _site_gauge(self, name):
    return self.body_gauge(self.spec.site(name).parent)

  def _geom_gauge(self, name):
    return self.body_gauge(self.spec.geom(name).parent)

  def _joint_lin(self, name):
    return self.spec.joint(name).type == mjtJoint.mjJNT_SLIDE

  def _tendon_info(self, name):
    """Returns (gauge, lin) of a tendon: mean gauge of its path elements."""
    m = self.model
    tid = mujoco.mj_name2id(m, mjtObj.mjOBJ_TENDON, name)
    gauges, lin, fixed = [], False, True
    for w in range(m.tendon_adr[tid], m.tendon_adr[tid] + m.tendon_num[tid]):
      wtype, obj = m.wrap_type[w], m.wrap_objid[w]
      if wtype == mujoco.mjtWrap.mjWRAP_JOINT:
        jname = mujoco.mj_id2name(m, mjtObj.mjOBJ_JOINT, obj)
        gauges.append(self._joint_gauge(jname))
        lin = lin or self._joint_lin(jname)
      elif wtype == mujoco.mjtWrap.mjWRAP_SITE:
        fixed = False
        gauges.append(
            self._site_gauge(mujoco.mj_id2name(m, mjtObj.mjOBJ_SITE, obj))
        )
      elif wtype in (
          mujoco.mjtWrap.mjWRAP_SPHERE,
          mujoco.mjtWrap.mjWRAP_CYLINDER,
      ):
        fixed = False
        gauges.append(
            self._geom_gauge(mujoco.mj_id2name(m, mjtObj.mjOBJ_GEOM, obj))
        )
    return Gauge.mean(gauges), (lin or not fixed)

  # ---- kinematic tree ------------------------------------------------------

  def scale_tree(self):
    spec = self.spec
    for body in spec.bodies[1:]:
      # own gauge: includes every frame that encloses the body. A body's pos is
      # content of those frames; the prototype's scaling frame sits at the
      # parent's origin.
      g = self.body_gauge(body)
      body.pos = body.pos * g.g(1)
      if body.explicitinertial:
        if _defined(body.ipos):
          body.ipos = body.ipos * g.g(1)
        body.mass = body.mass * g.g(0, 1)
        body.inertia = body.inertia * g.g(2, 1)
        if _defined(body.fullinertia):
          body.fullinertia = body.fullinertia * g.g(2, 1)

      for geom in body.geoms:
        self._scale_geom(geom, g)
      for site in body.sites:
        site.pos = site.pos * g.g(1)
        site.size = site.size * g.g(1)
        if _defined(site.fromto):
          site.fromto = site.fromto * g.g(1)
      for cam in body.cameras:
        cam.pos = cam.pos * g.g(1)
      for light in body.lights:
        light.pos = light.pos * g.g(1)
      for joint in body.joints:
        self._scale_joint(joint, g)

    # world-owned elements
    gw = self.gauge_of('world')
    for geom in spec.worldbody.geoms:
      self._scale_geom(geom, gw)
    for site in spec.worldbody.sites:
      site.pos = site.pos * gw.g(1)
      site.size = site.size * gw.g(1)
    # a frame's pos is content of its parent body (no frame scales here)
    for frame in spec.frames:
      frame.pos = frame.pos * self.body_gauge(frame.parent).g(1)

  def _scale_geom(self, geom, g):
    geom.pos = geom.pos * g.g(1)
    geom.size = geom.size * g.g(1)
    if _defined(geom.fromto):
      geom.fromto = geom.fromto * g.g(1)
    geom.margin *= g.g(1)
    geom.gap *= g.g(1)
    solimp = np.array(geom.solimp)
    solimp[2] *= g.g(1)
    geom.solimp = solimp
    friction = np.array(geom.friction)
    friction[1:] *= g.g(1)
    geom.friction = friction
    if _defined(geom.mass):
      geom.mass *= g.g(0, 1)
    # density M L^-3: factor 1

  def _scale_joint(self, joint, g):
    joint.pos = joint.pos * g.g(1)
    lin = joint.type == mjtJoint.mjJNT_SLIDE
    if joint.type == mjtJoint.mjJNT_FREE:
      return
    q = g.g(1) if lin else 1.0
    a, b = _genforce(lin)
    qa = 1 if lin else 0
    if lin:
      joint.range = joint.range * q
      joint.ref *= q
      joint.springref *= q
      joint.margin *= q
      solimp = np.array(joint.solimp_limit)
      solimp[2] *= q
      joint.solimp_limit = solimp
    stiffness = np.array(joint.stiffness)
    damping = np.array(joint.damping)
    for n in range(len(stiffness)):
      stiffness[n] *= g.f(a - (n + 1) * qa, b)
    for n in range(len(damping)):
      damping[n] *= g.f(a - (n + 1) * qa, b)
    joint.stiffness = stiffness
    joint.damping = damping
    joint.armature *= g.f(a - qa, b)  # inertia: energy / (Q/T)^2
    joint.frictionloss *= g.f(a, b)
    joint.actfrcrange = joint.actfrcrange * g.f(a, b)

  # ---- flexes --------------------------------------------------------------

  def scale_flexes(self):
    """Prototype: a flex takes the gauge of its first body (no spanning)."""
    for flex in self.spec.flexes:
      bodies = list(flex.nodebody) or list(flex.vertbody)
      g = self.gauge_of(bodies[0]) if bodies else self.gauge_of('world')
      flex.radius *= g.g(1)
      flex.vert = np.array(flex.vert) * g.g(1)
      flex.node = np.array(flex.node) * g.g(1)
      flex.margin *= g.g(1)
      flex.gap *= g.g(1)
      solimp = np.array(flex.solimp)
      solimp[2] *= g.g(1)
      flex.solimp = solimp
      friction = np.array(flex.friction)
      friction[1:] *= g.g(1)
      flex.friction = friction
      if flex.thickness > 0:
        flex.thickness *= g.g(1)
      flex.young *= g.f(-1, 1)  # pressure
      flex.edgestiffness *= g.f(0, 1)  # force / length
      flex.edgedamping *= g.f(0, 1)
      # damping: Rayleigh coefficient, a time

  # ---- assets --------------------------------------------------------------

  def scale_assets(self, g: Gauge):
    """Single-scale assets only (the prototype has no variants)."""
    for mesh in self.spec.meshes:
      mesh.scale = mesh.scale * g.g(1)
    for hfield in self.spec.hfields:
      hfield.size = hfield.size * g.g(1)

  # ---- tendons -------------------------------------------------------------

  def scale_tendons(self):
    for tendon in self.spec.tendons:
      g, lin = self._tendon_info(tendon.name)
      q = g.g(1) if lin else 1.0
      a, b = _genforce(lin)
      qa = 1 if lin else 0
      sl = np.array(tendon.springlength)
      sl[sl >= 0] *= q  # -1: computed at qpos0
      tendon.springlength = sl
      tendon.range = tendon.range * q
      tendon.margin *= q
      tendon.width *= g.g(1)
      solimp = np.array(tendon.solimp_limit)
      solimp[2] *= q
      tendon.solimp_limit = solimp
      stiffness = np.array(tendon.stiffness)
      damping = np.array(tendon.damping)
      for n in range(len(stiffness)):
        stiffness[n] *= g.f(a - (n + 1) * qa, b)
      for n in range(len(damping)):
        damping[n] *= g.f(a - (n + 1) * qa, b)
      tendon.stiffness = stiffness
      tendon.damping = damping
      tendon.armature *= g.f(a - qa, b)  # inertia: energy / (Q/T)^2
      tendon.frictionloss *= g.f(a, b)
      tendon.actfrcrange = tendon.actfrcrange * g.f(a, b)

  # ---- actuators -----------------------------------------------------------

  def _actuator_info(self, act):
    """Returns (gauge, lin, sixd) for an actuator's transmission."""
    trn = act.trntype
    gear = np.array(act.gear)
    if trn in (mjtTrn.mjTRN_JOINT, mjtTrn.mjTRN_JOINTINPARENT):
      joint = self.spec.joint(act.target)
      g = self._joint_gauge(act.target)
      if joint.type == mjtJoint.mjJNT_FREE:
        return g, bool(np.any(gear[:3])), True
      return g, joint.type == mjtJoint.mjJNT_SLIDE, False
    if trn == mjtTrn.mjTRN_TENDON:
      g, lin = self._tendon_info(act.target)
      return g, lin, False
    if trn == mjtTrn.mjTRN_SLIDERCRANK:
      return self._site_gauge(act.target), True, False
    if trn == mjtTrn.mjTRN_SITE:
      return self._site_gauge(act.target), bool(np.any(gear[:3])), True
    if trn == mjtTrn.mjTRN_BODY:
      return self.body_gauge(self.spec.body(act.target)), True, False
    raise NotImplementedError(f'transmission {trn}')

  def scale_actuators(self):
    m = self.model
    for i, act in enumerate(self.spec.actuators):
      g, lin, sixd = self._actuator_info(act)
      a, b = _genforce(lin)
      qa = 1 if lin else 0
      q = g.g(1) if lin else 1.0

      # transmission geometry
      if sixd and lin:
        gear = np.array(act.gear)
        gear[3:] *= g.g(1)  # moment arms
        act.gear = gear
      act.cranklength *= g.g(1)
      act.lengthrange = act.lengthrange * q

      gain = np.array(act.gainprm)
      bias = np.array(act.biasprm)
      gaintype, biastype = act.gaintype, act.biastype

      if gaintype == mujoco.mjtGain.mjGAIN_MUSCLE:
        if gain[2] > 0:
          gain[2] *= g.f(a, b)
        if bias[2] > 0:
          bias[2] *= g.f(a, b)
        ua = 0
      elif gaintype in (mujoco.mjtGain.mjGAIN_FIXED,
                        mujoco.mjtGain.mjGAIN_AFFINE):
        affine_bias = biastype == mujoco.mjtBias.mjBIAS_AFFINE
        fixed = gaintype == mujoco.mjtGain.mjGAIN_FIXED
        # control dimension: the force depends on ctrl only through
        # (ctrl - length) or (ctrl - velocity)
        position_form = fixed and affine_bias and gain[0] != 0 and (
            bias[1] == -gain[0])
        velocity_form = fixed and affine_bias and gain[0] != 0 and (
            bias[1] == 0 and bias[2] == -gain[0])
        ua = qa if (position_form or velocity_form) else 0
        gain[0] *= g.f(a - ua, b)
        if not fixed:
          gain[1] *= g.f(a - ua - qa, b)
          gain[2] *= g.f(a - ua - qa, b)
        if affine_bias:
          bias[0] *= g.f(a, b)
          bias[1] *= g.f(a - qa, b)
          if bias[2] < 0:  # positive: damping ratio, dimensionless
            bias[2] *= g.f(a - qa, b)
      else:
        raise NotImplementedError(f'gaintype {gaintype}')

      act.gainprm = gain
      act.biasprm = bias
      u = g.g(ua)
      act.ctrlrange = act.ctrlrange * u
      act.actrange = act.actrange * u
      act.forcerange = act.forcerange * g.f(a, b)
      damping = np.array(act.damping)
      for n in range(len(damping)):
        damping[n] *= g.f(a - (n + 1) * qa, b)
      act.damping = damping
      act.armature *= g.f(a - qa, b)  # inertia: energy / (Q/T)^2

      self.ctrl_factor[i] = u
      if m.actuator_actnum[i] > 0 and act.dyntype in (
          mujoco.mjtDyn.mjDYN_INTEGRATOR,
          mujoco.mjtDyn.mjDYN_FILTER,
          mujoco.mjtDyn.mjDYN_FILTEREXACT,
      ):
        adr = m.actuator_actadr[i]
        self.act_factor[adr : adr + m.actuator_actnum[i]] = u

  # ---- equality, pairs -----------------------------------------------------

  def scale_equalities(self):
    for eq in self.spec.equalities:
      data = np.array(eq.data)
      width = 1.0
      if eq.type == mjtEq.mjEQ_CONNECT and eq.objtype == mjtObj.mjOBJ_BODY:
        width = self.body_gauge(self.spec.body(eq.name1)).g(1)
        data[0:3] *= width
      elif eq.type == mjtEq.mjEQ_WELD and eq.objtype == mjtObj.mjOBJ_BODY:
        g1 = self.body_gauge(self.spec.body(eq.name1))
        g2 = self.body_gauge(self.spec.body(eq.name2)) if eq.name2 else (
            self.gauge_of('world'))
        data[0:3] *= g2.g(1)  # anchor, in body2
        data[3:6] *= g1.g(1)  # relpose position, in body1
        width = Gauge.mean([g1, g2]).g(1)
        data[10] *= width  # torquescale
      elif eq.type == mjtEq.mjEQ_JOINT:
        q1 = self._joint_gauge(eq.name1).g(1) if self._joint_lin(
            eq.name1) else 1.0
        q2 = 1.0
        if eq.name2:
          q2 = self._joint_gauge(eq.name2).g(1) if self._joint_lin(
              eq.name2) else 1.0
        for k in range(5):
          data[k] *= q1 / q2**k
        width = q1
      elif eq.type == mjtEq.mjEQ_TENDON:
        g1, lin1 = self._tendon_info(eq.name1)
        q1 = g1.g(1) if lin1 else 1.0
        q2 = 1.0
        if eq.name2:
          g2, lin2 = self._tendon_info(eq.name2)
          q2 = g2.g(1) if lin2 else 1.0
        for k in range(5):
          data[k] *= q1 / q2**k
        width = q1
      elif eq.type == mjtEq.mjEQ_FLEX:
        width = self.gauge_of('flex').g(1)  # edge-length residual
      eq.data = data
      # impedance width: dimension of the constraint residual
      solimp = np.array(eq.solimp)
      solimp[2] *= width
      eq.solimp = solimp

  def scale_pairs(self):
    for pair in self.spec.pairs:
      g = Gauge.mean([
          self._geom_gauge(pair.geomname1),
          self._geom_gauge(pair.geomname2),
      ])
      pair.margin *= g.g(1)
      pair.gap *= g.g(1)
      solimp = np.array(pair.solimp)
      solimp[2] *= g.g(1)
      pair.solimp = solimp
      friction = np.array(pair.friction)
      friction[2:] *= g.g(1)
      pair.friction = friction

  # ---- keyframes -----------------------------------------------------------

  def scale_keys(self):
    m = self.model
    for key in self.spec.keys:
      qpos, qvel = np.array(key.qpos), np.array(key.qvel)
      joints = list(self.spec.joints)  # spec order is id order
      assert len(joints) == m.njnt
      for j in range(m.njnt):
        q = self.body_gauge(joints[j].parent).g(1)
        qadr, vadr = m.jnt_qposadr[j], m.jnt_dofadr[j]
        if m.jnt_type[j] == mjtJoint.mjJNT_SLIDE:
          if qpos.size:
            qpos[qadr] *= q
          if qvel.size:
            qvel[vadr] *= q
        elif m.jnt_type[j] == mjtJoint.mjJNT_FREE:
          # prototype: scaling frame at the parent's origin
          if qpos.size:
            qpos[qadr : qadr + 3] *= q
          if qvel.size:
            qvel[vadr : vadr + 3] *= q
      if qpos.size:
        key.qpos = qpos
      if qvel.size:
        key.qvel = qvel
      if np.array(key.ctrl).size:
        key.ctrl = np.array(key.ctrl) * self.ctrl_factor
      if np.array(key.act).size:
        key.act = np.array(key.act) * self.act_factor

  def apply(self, asset_gauge: Optional[Gauge] = None):
    self.scale_tendons()  # before the tree: tendon info reads the spec
    self.scale_flexes()
    self.scale_actuators()
    self.scale_equalities()
    self.scale_pairs()
    self.scale_keys()
    self.scale_tree()
    if asset_gauge is not None:
      self.scale_assets(asset_gauge)
    return self.spec


# ------------------------------ checks ---------------------------------------


def qpos_factors(m, gauge_of_joint):
  f = np.ones(m.nq)
  for j in range(m.njnt):
    q = gauge_of_joint(mujoco.mj_id2name(m, mjtObj.mjOBJ_JOINT, j))
    adr = m.jnt_qposadr[j]
    if m.jnt_type[j] == mjtJoint.mjJNT_SLIDE:
      f[adr] = q
    elif m.jnt_type[j] == mjtJoint.mjJNT_FREE:
      f[adr : adr + 3] = q
  return f


def rollout(m, nstep, ctrl_fn, key=None):
  d = mujoco.MjData(m)
  if key is not None:
    mujoco.mj_resetDataKeyframe(m, d, key)
  out = []
  for i in range(nstep):
    if m.nu:
      d.ctrl[:] = ctrl_fn(i)
    mujoco.mj_step(m, d)
    out.append(d.qpos.copy())
  return np.array(out)


def check_similar(path_or_xml, s, nstep=300, scale_world=True, key=None,
                  amplitude=0.3, from_string=False, verbose=True):
  """Original in gravity g/s  vs  similar-scaled model in gravity g."""
  load = mujoco.MjSpec.from_string if from_string else mujoco.MjSpec.from_file
  spec0 = load(path_or_xml)
  spec1 = load(path_or_xml)

  gauge = Gauge.similar(s)
  def gauge_of(name):
    if name == 'world':
      return gauge if scale_world else IDENTITY
    return gauge

  scaler = Scaler(spec1, gauge_of)
  scaler.apply(asset_gauge=gauge)

  # unscaled surroundings are relatively weaker by 1/s
  spec0.option.gravity = np.array(spec0.option.gravity) / s
  spec0.option.wind = np.array(spec0.option.wind) / s
  spec0.option.viscosity = spec0.option.viscosity / s**2
  spec0.option.ccd_tolerance = spec0.option.ccd_tolerance / s  # a length

  m0, m1 = spec0.compile(), spec1.compile()
  rng = np.random.default_rng(0)
  phase = rng.uniform(0, 2 * np.pi, size=max(m0.nu, 1))

  def ctrl0(i):
    u = amplitude * np.sin(0.02 * i + phase[: m0.nu])
    lo, hi = m0.actuator_ctrlrange[:, 0], m0.actuator_ctrlrange[:, 1]
    limited = m0.actuator_ctrllimited.astype(bool)
    mid, half = (lo + hi) / 2, (hi - lo) / 2
    return np.where(limited, mid + half * np.sin(0.02 * i + phase[: m0.nu]), u)

  ctrl1 = lambda i: ctrl0(i) * scaler.ctrl_factor

  q0 = rollout(m0, nstep, ctrl0, key)
  q1 = rollout(m1, nstep, ctrl1, key)
  f = qpos_factors(m1, lambda _: s)
  err = np.abs(q1 / f - q0).max(axis=1)
  scale = max(1.0, np.abs(q0).max())
  if verbose:
    print(f'  nq={m0.nq} nu={m0.nu} max|err| first 50 steps: '
          f'{err[:50].max() / scale:.2e}, all {nstep}: {err.max() / scale:.2e}, '
          f'motion range: {np.ptp(q0, axis=0).max():.3g}')
  return err / scale
