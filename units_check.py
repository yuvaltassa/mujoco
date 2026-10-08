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
"""Checks the dim facets of mjcf.schema against simulation (design scaffolding).

Applies a change of units (sL, sM, sT) to every field of a model, with the factor of each field
computed from its schema dimension, and checks that the scaled model reproduces the original
exactly: generalized positions and sensor values map by their dimensions, step for step (the
timestep scales too). Contextual symbols are resolved per object (Q by joint and tendon kind and
actuator transmission, F = M L^2 T^-2 Q^-1, U by the actuator's algebraic form, Y by the sensor
element's output facet). dim=custom fields are resolved by hand below.

Usage: python units_check.py [model.xml ...]   (no arguments: the built-in corpus)
"""

import math
import os
import sys

import mujoco
import numpy as np

WORKTREE = os.environ.get('MUJOCO_SRC', '.')  # a checkout with the dim facets
sys.path.insert(0, os.path.join(WORKTREE, 'doc/generate'))
import mjcf_schema  # noqa: E402

SCHEMA = mjcf_schema.parse_file(os.path.join(WORKTREE, 'src/xml/mjcf.schema'))
mjtJoint, mjtTrn, mjtObj, mjtEq = mujoco.mjtJoint, mujoco.mjtTrn, mujoco.mjtObj, mujoco.mjtEq


class Units:
  def __init__(self, sl, sm, st):
    self.L, self.M, self.T = sl, sm, st

  def factor(self, comp, q=None, u=None, y=None):
    """Factor of one product component; Q, U, Y given as factors."""
    sym = {'L': self.L, 'M': self.M, 'T': self.T, 'A': 1.0, 'Q': q, 'U': u, 'Y': y}
    if q is not None:
      sym['F'] = self.M * self.L**2 * self.T**-2 / q
    f = 1.0
    for s, e in comp:
      if sym.get(s) is None:
        raise KeyError(f'unresolved symbol {s}')
      f *= sym[s] ** e
    return f


def field_attrs(element_names, groups=()):
  """{field name: Attr} for schema elements bound to one spec struct."""
  out = {}
  attrs = []
  for g in groups:
    attrs += SCHEMA._group_attrs(g)
  for name in element_names:
    attrs += SCHEMA.expanded_attrs(SCHEMA.elements[name])
  for a in attrs:
    if a.type not in mjcf_schema.REAL_TYPES:
      continue
    if a.dim.kind == 'product' and a.dim.symbols() <= {'A'}:
      continue  # dimensionless under any change of units
    field = str(a.facets.get('field', a.name))
    out.setdefault(field, a)
  return out


class Checker:
  def __init__(self, spec, units):
    self.spec, self.u = spec, units
    self.model = spec.copy().compile()
    self.log = []  # (object, field, note) for fields not transformed exactly
    self.ctrl_factor = np.ones(self.model.nu)
    self.act_factor = np.ones(self.model.na)

  # ---- helpers ---------------------------------------------------------------

  def scale_field(self, obj, field, attr, q=None, u=None, y=None, label=''):
    if not hasattr(obj, field):
      self.log.append((label, field, 'no such spec field'))
      return
    dim = attr.dim
    value = getattr(obj, field)
    arr = np.array(value, dtype=float)
    if dim.kind == 'opaque':
      return
    if dim.kind == 'custom':
      raise AssertionError(f'custom field {label}.{field} reached generic scaling')
    if dim.kind == 'solref':
      if arr.size >= 2 and arr[0] > 0:
        arr[0] *= self.u.T
      elif arr.size >= 2 and arr[0] < 0:
        arr[0] *= self.u.T**-2
        arr[1] *= self.u.T**-1
      setattr(obj, field, arr if np.ndim(value) else float(arr))
      return
    comps = dim.components
    if arr.ndim == 0:
      setattr(obj, field, float(arr) * self.u.factor(comps[0], q, u, y))
      return
    if np.all(np.isnan(arr)):
      return
    flat = arr.reshape(-1)
    for i in range(flat.size):
      comp = comps[0] if len(comps) == 1 else comps[i] if i < len(comps) else None
      if comp is None:
        continue
      flat[i] *= self.u.factor(comp, q, u, y)
    setattr(obj, field, flat.reshape(arr.shape))

  def scale_object(self, obj, attrs, skip=(), q=None, u=None, y=None, label=''):
    for field, attr in attrs.items():
      if field in skip or attr.dim.kind == 'custom':
        continue
      if attr.dim.kind == 'product' and attr.dim.symbols() & {'Q', 'F'} and q is None:
        value = np.array(getattr(obj, field, 0), dtype=float)
        if np.any(value[~np.isnan(value)] != 0):
          self.log.append((label, field, 'Q undefined, nonzero value not transformed'))
        continue
      self.scale_field(obj, field, attr, q, u, y, label)

  def joint_q(self, joint):
    if joint.type == mjtJoint.mjJNT_SLIDE:
      return self.u.L
    if joint.type in (mjtJoint.mjJNT_HINGE, mjtJoint.mjJNT_BALL):
      return 1.0
    return None  # free: mixed

  def tendon_q(self, tendon):
    """Spatial: L. Fixed: L if any slide joint, else angle (coefs absorb the rest)."""
    m = self.model
    tid = [t.name for t in self.spec.tendons].index(tendon.name) if tendon.name else (
        next(i for i, t in enumerate(self.spec.tendons) if t is tendon))
    adr, num = m.tendon_adr[tid], m.tendon_num[tid]
    types = m.wrap_type[adr:adr + num]
    if np.any(types != mujoco.mjtWrap.mjWRAP_JOINT):
      return self.u.L, None
    jtypes = [m.jnt_type[m.wrap_objid[w]] for w in range(adr, adr + num)]
    q = self.u.L if any(t == mjtJoint.mjJNT_SLIDE for t in jtypes) else 1.0
    coef = [q / (self.u.L if t == mjtJoint.mjJNT_SLIDE else 1.0) for t in jtypes]
    return q, coef

  # ---- elements --------------------------------------------------------------

  def run(self):
    s, u = self.spec, self.u
    self.scale_object(s.option, field_attrs(['option']), skip=('magnetic',), label='option')
    self.scale_object(s.compiler, field_attrs(['compiler']), label='compiler')
    # compiler/lengthrange: times scale with T; accel is a joint-space acceleration (angular
    # coordinates here) and maxforce an actuator force, both resolved by hand
    lr = s.compiler.LRopt
    for f in ('timeconst', 'timestep', 'inttotal', 'interval'):
      setattr(lr, f, getattr(lr, f) * u.T)
    lr.accel *= u.T**-2
    if lr.maxforce:
      self.log.append(('compiler', 'lengthrange maxforce', 'not transformed'))

    body_attrs = field_attrs(['body', 'inertial'], groups=('orientation',))
    for body in s.bodies[1:]:
      self.scale_object(body, body_attrs, label=f'body {body.name}')
    for frame in s.frames:
      self.scale_object(frame, field_attrs(['frame']), label='frame')

    geom_attrs = field_attrs(['geom'])
    for geom in s.geoms:
      self.scale_object(geom, geom_attrs, label=f'geom {geom.name}')
      shell = getattr(geom, 'typeinertia', None) == mujoco.mjtGeomInertia.mjINERTIA_SHELL
      geom.density *= u.M * u.L**(-2 if shell else -3)
    for site in s.sites:
      self.scale_object(site, field_attrs(['site']), label=f'site {site.name}')
    for cam in s.cameras:
      self.scale_object(cam, field_attrs(['camera']), label='camera')
    for light in s.lights:
      self.scale_object(light, field_attrs(['light']), label='light')

    joint_attrs = field_attrs(['joint'])
    for joint in s.joints:
      self.scale_object(joint, joint_attrs, q=self.joint_q(joint), label=f'joint {joint.name}')

    for mesh in s.meshes:
      self.scale_object(mesh, field_attrs(['mesh']), label=f'mesh {mesh.name}')
    for hfield in s.hfields:
      self.scale_object(hfield, field_attrs(['hfield']), label='hfield')
    for skin in s.skins:
      self.scale_object(skin, field_attrs(['skin']), label='skin')

    flex_attrs = field_attrs(['flex', 'flex_edge', 'elasticity', 'flexcomp_contact'])
    for flex in s.flexes:
      self.scale_object(flex, flex_attrs, label=f'flex {flex.name}')

    pair_attrs = field_attrs(['pair'])
    for pair in s.pairs:
      self.scale_object(pair, pair_attrs, label='pair')

    self.tendons()
    self.equalities()
    self.actuators()
    self.sensors()
    self.keys()
    return self.spec

  def tendons(self):
    for tendon in self.spec.tendons:
      q, coef = self.tendon_q(tendon)
      kind = 'fixed' if coef is not None else 'spatial'
      self.scale_object(tendon, field_attrs([kind]), skip=('springlength',), q=q,
                        label=f'tendon {tendon.name}')
      sl = np.array(tendon.springlength)
      sl[sl >= 0] *= q  # negative: computed at qpos0
      tendon.springlength = sl
      if coef is not None:
        for wrap, factor in zip(tendon.path, coef):
          if factor != 1:
            self.log.append((f'tendon {tendon.name}', 'coef', 'not settable from Python'))

  def equalities(self):
    s, u = self.spec, self.u
    for eq in s.equalities:
      data = np.array(eq.data)
      width = 1.0
      if eq.type == mjtEq.mjEQ_CONNECT:
        width = u.L
        if eq.objtype == mjtObj.mjOBJ_BODY:
          data[0:3] *= u.L
      elif eq.type == mjtEq.mjEQ_WELD:
        width = u.L
        if eq.objtype == mjtObj.mjOBJ_BODY:
          data[0:3] *= u.L
          data[3:6] *= u.L
        data[10] *= u.L
      elif eq.type in (mjtEq.mjEQ_JOINT, mjtEq.mjEQ_TENDON):
        if eq.type == mjtEq.mjEQ_JOINT:
          q1 = self.joint_q(s.joint(eq.name1))
          q2 = self.joint_q(s.joint(eq.name2)) if eq.name2 else 1.0
        else:
          q1 = self.tendon_q(s.tendon(eq.name1))[0]
          q2 = self.tendon_q(s.tendon(eq.name2))[0] if eq.name2 else 1.0
        for k in range(5):
          data[k] *= q1 / q2**k
        width = q1
      elif eq.type == mjtEq.mjEQ_FLEX:
        width = u.L
      eq.data = data
      self.scale_field(eq, 'solref', field_attrs([], groups=('equality_base',))['solref'])
      solimp = np.array(eq.solimp)
      solimp[2] *= width
      eq.solimp = solimp

  def actuator_q(self, act):
    s, u, m = self.spec, self.u, self.model
    gear = np.array(act.gear)
    trn = act.trntype
    if trn in (mjtTrn.mjTRN_JOINT, mjtTrn.mjTRN_JOINTINPARENT):
      joint = s.joint(act.target)
      if joint.type == mjtJoint.mjJNT_FREE:
        return (u.L if np.any(gear[:3]) else 1.0), True
      return self.joint_q(joint), joint.type == mjtJoint.mjJNT_BALL
    if trn == mjtTrn.mjTRN_TENDON:
      return self.tendon_q(s.tendon(act.target))[0], False
    if trn == mjtTrn.mjTRN_SITE:
      return (u.L if np.any(gear[:3]) else 1.0), True
    if trn in (mjtTrn.mjTRN_SLIDERCRANK, mjtTrn.mjTRN_BODY):
      return u.L, False
    return None, False

  def actuators(self):
    s, u, m = self.spec, self.u, self.model
    attrs = field_attrs(['general'], groups=('actuator_base', 'actuator_dynamics'))
    for i, act in enumerate(s.actuators):
      label = f'actuator {act.name}'
      q, sixd = self.actuator_q(act)
      if q is None:
        self.log.append((label, '*', 'transmission not handled'))
        continue
      gt, bt, dt = act.gaintype, act.biastype, act.dyntype
      gain, bias, dyn = np.array(act.gainprm), np.array(act.biasprm), np.array(act.dynprm)
      F = u.M * u.L**2 * u.T**-2 / q
      if gt in (mujoco.mjtGain.mjGAIN_FIXED, mujoco.mjtGain.mjGAIN_AFFINE):
        fixed = gt == mujoco.mjtGain.mjGAIN_FIXED
        affine = bt == mujoco.mjtBias.mjBIAS_AFFINE
        if fixed and affine and gain[0] != 0 and bias[1] == -gain[0]:
          uu = q
        elif fixed and affine and gain[0] != 0 and bias[1] == 0 and bias[2] == -gain[0]:
          uu = q / u.T
        else:
          uu = 1.0
        gain[0] *= F / uu
        if not fixed:
          gain[1] *= F / (uu * q)
          gain[2] *= F * u.T / (uu * q)
        if affine:
          bias[0] *= F
          bias[1] *= F / q
          if bias[2] < 0:
            bias[2] *= F * u.T / q
        if dt in (mujoco.mjtDyn.mjDYN_FILTER, mujoco.mjtDyn.mjDYN_FILTEREXACT):
          dyn[0] *= u.T
      elif gt == mujoco.mjtGain.mjGAIN_SO3:
        uu = 1.0  # orientation setpoint: angle or quaternion
        gain[0] *= F
        bias[1] *= F
        if bias[2] < 0:
          bias[2] *= F * u.T
      elif gt == mujoco.mjtGain.mjGAIN_MUSCLE:
        uu = 1.0
        for prm in (gain, bias):
          if prm[2] > 0:
            prm[2] *= F
          prm[3] *= u.T**-2  # scale: force / acc0
          prm[6] *= u.T**-1  # vmax, L0 per second
        dyn[0] *= u.T
        dyn[1] *= u.T
      else:
        self.log.append((label, '*', f'gaintype {gt} not handled'))
        continue
      act.gainprm, act.biasprm, act.dynprm = gain, bias, dyn
      if sixd and q == u.L:
        gear = np.array(act.gear)
        gear[3:] *= u.L  # moment arms
        act.gear = gear
      # uu is the dimension of the gain's input: act if the actuator has dynamics, else ctrl.
      # An integrator's ctrl is the rate of its act.
      muscle = gt == mujoco.mjtGain.mjGAIN_MUSCLE
      act_f = 1.0 if muscle else uu
      ctrl_f = act_f / u.T if dt == mujoco.mjtDyn.mjDYN_INTEGRATOR else act_f
      act.actrange = np.array(act.actrange) * act_f
      self.scale_object(act, attrs, skip=('actrange',), q=q, u=ctrl_f, label=label)
      self.ctrl_factor[i] = ctrl_f
      if m.actuator_actnum[i]:
        adr = m.actuator_actadr[i]
        self.act_factor[adr:adr + m.actuator_actnum[i]] = act_f

  def sensor_element(self, sensor):
    name = mujoco.mjtSensor(sensor.type).name
    for element in SCHEMA.elements.values():
      for const in element.consts():
        if const.field == 'type' and const.value == name:
          return element
    # type set by the reader
    alias = {'mjSENS_GEOMDIST': 'distance', 'mjSENS_GEOMNORMAL': 'normal',
             'mjSENS_GEOMFROMTO': 'fromto'}
    return SCHEMA.elements.get(alias.get(name, name[len('mjSENS_'):].lower()))

  def sensor_y(self, sensor):
    """Output factor of a sensor, or None if not a product."""
    element = self.sensor_element(sensor)
    if element is None or 'Y' not in element.dims:
      return None, 'no element'
    dim = element.dims['Y']
    if dim.kind != 'product':
      return None, dim.kind
    q = None
    if 'Q' in dim.symbols() or 'F' in dim.symbols():
      s = self.spec
      ot = sensor.objtype
      if ot == mjtObj.mjOBJ_JOINT:
        q = self.joint_q(s.joint(sensor.objname))
      elif ot == mjtObj.mjOBJ_TENDON:
        q = self.tendon_q(s.tendon(sensor.objname))[0]
      elif ot == mjtObj.mjOBJ_ACTUATOR:
        q = self.actuator_q(s.actuator(sensor.objname))[0]
      if q is None:
        return None, 'Q undefined'
    return self.u.factor(dim.components[0], q=q), None

  def sensors(self):
    attrs = field_attrs([], groups=('sensor_base',))
    self.sensor_factor = []
    for sensor in self.spec.sensors:
      y, why = self.sensor_y(sensor)
      self.sensor_factor.append(y)
      if y is None:
        if sensor.cutoff or sensor.noise:
          self.log.append((f'sensor {sensor.name}', 'cutoff/noise', why))
        self.scale_object(sensor, attrs, skip=('cutoff', 'noise'), label='sensor')
      else:
        self.scale_object(sensor, attrs, y=y, label=f'sensor {sensor.name}')

  def keys(self):
    m, u = self.model, self.u
    joints = list(self.spec.joints)
    for key in self.spec.keys:
      key.time *= u.T
      qpos, qvel = np.array(key.qpos), np.array(key.qvel)
      for j, joint in enumerate(joints):
        qa, va = m.jnt_qposadr[j], m.jnt_dofadr[j]
        if joint.type == mjtJoint.mjJNT_FREE:
          if qpos.size:
            qpos[qa:qa + 3] *= u.L
          if qvel.size:
            qvel[va:va + 3] *= u.L / u.T
            qvel[va + 3:va + 6] /= u.T
        elif joint.type == mjtJoint.mjJNT_BALL:
          if qvel.size:
            qvel[va:va + 3] /= u.T
        else:
          qf = self.joint_q(joint)
          if qpos.size:
            qpos[qa] *= qf
          if qvel.size:
            qvel[va] *= qf / u.T
      if qpos.size:
        key.qpos = qpos
      if qvel.size:
        key.qvel = qvel
      if np.array(key.ctrl).size:
        key.ctrl = np.array(key.ctrl) * self.ctrl_factor
      if np.array(key.act).size:
        key.act = np.array(key.act) * self.act_factor
      if np.array(key.mpos).size:
        key.mpos = np.array(key.mpos) * u.L


# ---- comparison --------------------------------------------------------------


def qpos_factor(m, units):
  f = np.ones(m.nq)
  for j in range(m.njnt):
    a = m.jnt_qposadr[j]
    if m.jnt_type[j] == mjtJoint.mjJNT_SLIDE:
      f[a] = units.L
    elif m.jnt_type[j] == mjtJoint.mjJNT_FREE:
      f[a:a + 3] = units.L
  return f


def check(load, units, nstep=200, key=None, verbose=True, solver_tight=True):
  spec0, spec1 = load(), load()
  checker = Checker(spec1, units)
  checker.run()
  if solver_tight:
    for spec in (spec0, spec1):
      spec.option.solver = mujoco.mjtSolver.mjSOL_NEWTON
      spec.option.tolerance = 1e-15
      spec.option.iterations = 1000
      spec.option.ls_tolerance = 1e-8
      spec.option.ls_iterations = 200
  m0, m1 = spec0.compile(), spec1.compile()
  d0, d1 = mujoco.MjData(m0), mujoco.MjData(m1)
  if key is not None:
    mujoco.mj_resetDataKeyframe(m0, d0, key)
    mujoco.mj_resetDataKeyframe(m1, d1, key)
  fq = qpos_factor(m1, units)
  # perturb hinge, slide and free-joint positions off any exact contact (knife edges)
  rng = np.random.default_rng(1)
  for j in range(m0.njnt):
    a = m0.jnt_qposadr[j]
    n = {int(mjtJoint.mjJNT_FREE): 3, int(mjtJoint.mjJNT_HINGE): 1,
         int(mjtJoint.mjJNT_SLIDE): 1}.get(int(m0.jnt_type[j]), 0)
    delta = rng.uniform(-1e-3, 1e-3, n)
    d0.qpos[a:a + n] += delta
    d1.qpos[a:a + n] += delta * fq[a:a + n]
  fs = np.ones(m1.nsensordata)
  sens_ok = np.zeros(m1.nsensordata, dtype=bool)
  for i, y in enumerate(checker.sensor_factor):
    adr, dim = m1.sensor_adr[i], m1.sensor_dim[i]
    if y is not None:
      fs[adr:adr + dim] = y
      sens_ok[adr:adr + dim] = True
  rng = np.random.default_rng(0)
  phase = rng.uniform(0, 2 * np.pi, m0.nu)
  lo, hi = m0.actuator_ctrlrange[:, 0], m0.actuator_ctrlrange[:, 1]
  limited = m0.actuator_ctrllimited.astype(bool)
  qerr, serr = 0.0, 0.0
  for k in range(nstep):
    t = k * m0.opt.timestep
    wave = np.sin(10 * t + phase)
    c0 = np.where(limited, (lo + hi) / 2 + (hi - lo) / 2 * wave, 0.3 * wave)
    d0.ctrl[:] = c0
    d1.ctrl[:] = c0 * checker.ctrl_factor
    mujoco.mj_step(m0, d0)
    mujoco.mj_step(m1, d1)
    scale = max(1.0, np.abs(d0.qpos).max())
    qerr = max(qerr, np.abs(d1.qpos / fq - d0.qpos).max() / scale)
    if sens_ok.any():
      ref = d0.sensordata[sens_ok]
      e = np.abs(d1.sensordata[sens_ok] / fs[sens_ok] - ref) / np.maximum(1.0, np.abs(ref))
      serr = max(serr, e.max())
  if verbose:
    for item in sorted(set(checker.log)):
      print('   note:', *item)
  return qerr, serr, int(sens_ok.sum()), m1.nsensordata
