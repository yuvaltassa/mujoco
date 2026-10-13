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
"""Roll out open-loop trajectories from initial states, get subsequent states and sensor values.

Prototype: rollout over mujoco.batch, with the API of mujoco.rollout.
"""

import atexit
import threading
from collections.abc import Sequence
from typing import Optional, Union

import mujoco
from mujoco import batch as _batch
import numpy as np
from numpy import typing as npt

_FULLPHYSICS = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
_PLUGIN = mujoco.mjtState.mjSTATE_PLUGIN.value
_WARMSTART = mujoco.mjtState.mjSTATE_WARMSTART.value
_EQ_ACTIVE = mujoco.mjtState.mjSTATE_EQ_ACTIVE.value
_MOCAP_POS = mujoco.mjtState.mjSTATE_MOCAP_POS.value
_MOCAP_QUAT = mujoco.mjtState.mjSTATE_MOCAP_QUAT.value
_USERDATA = mujoco.mjtState.mjSTATE_USERDATA.value
_INTEGRATION = mujoco.mjtState.mjSTATE_INTEGRATION.value


class Rollout:
  """Rollout object containing a thread pool for parallel rollouts."""

  def __init__(self, *, nthread: Optional[int] = None):
    """Construct a rollout object containing a thread pool for parallel rollouts.

    Args:
      nthread: Number of threads in pool.
        If zero, this pool is not started and rollouts run on the calling thread.
    """  # fmt: skip
    self.nthread = 0 if nthread is None else nthread
    self.rollout_ = True  # None once closed
    self._batch = None  # made on the first rollout, grown and remade as needed
    self._lock = threading.Lock()  # a call writes the batch's rows, then runs
    self._layout_model = None
    self._layout_cache = None

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.close()

  def close(self):
    self._batch = None
    self.rollout_ = None

  def rollout(
      self,
      model: Union[mujoco.MjModel, Sequence[mujoco.MjModel]],
      data: Union[mujoco.MjData, Sequence[mujoco.MjData]],
      initial_state: npt.ArrayLike,
      control: Optional[npt.ArrayLike] = None,
      *,  # require subsequent arguments to be named
      control_spec: int = mujoco.mjtState.mjSTATE_CTRL.value,
      skip_checks: bool = False,
      nstep: Optional[int] = None,
      initial_warmstart: Optional[npt.ArrayLike] = None,
      state: Optional[npt.ArrayLike] = None,
      sensordata: Optional[npt.ArrayLike] = None,
      chunk_size: Optional[int] = None,
  ):
    """Rolls out open-loop trajectories from initial states, get subsequent state and sensor values.

    Python wrapper for rollout.cc, see documentation therein.
    Infers nbatch and nstep.
    Tiles inputs with singleton dimensions.
    Allocates outputs if none are given.

    Args:
      model: An instance or length nbatch sequence of MjModel with the same size signature.
      data: Associated mjData instance or sequence of instances with length nthread.
      initial_state: Array of initial states from which to roll out trajectories.
        ([nbatch or 1] x nstate)
      control: Open-loop controls array to apply during the rollouts.
        ([nbatch or 1] x [nstep or 1] x ncontrol)
      control_spec: mjtState specification of control vectors.
      skip_checks: Whether to skip internal shape and type checks.
      nstep: Number of steps in rollouts (inferred if unspecified).
      initial_warmstart: Initial qfrc_warmstart array (optional).
        ([nbatch or 1] x nv)
      state: State output array (optional).
        (nbatch x nstep x nstate)
      sensordata: Sensor data output array (optional).
        (nbatch x nstep x nsensordata)
      chunk_size: Determines threadpool chunk size. If unspecified,
                  chunk_size = max(1, nbatch / (nthread * 10))

    Returns:
      state:
        State output array, (nbatch x nstep x nstate).
      sensordata:
        Sensor data output array, (nbatch x nstep x nsensordata).

    Raises:
      RuntimeError: rollout requested after thread pool shutdown.
      ValueError: bad shapes or sizes.
    """  # fmt: skip

    if self.rollout_ is None:
      raise RuntimeError('rollout requested after thread pool shutdown')

    # skip_checks shortcut:
    #   don't infer nbatch or nstep
    #   don't support singleton expansion
    #   don't allocate output arrays
    #   just call rollout and return
    if skip_checks:
      self._run(
          model,
          data,
          nstep,
          control_spec,
          initial_state,
          initial_warmstart,
          control,
          state,
          sensordata,
      )
      return state, sensordata

    if not isinstance(model, mujoco.MjModel):
      model = list(model)

    # check control_spec
    if control_spec & ~mujoco.mjtState.mjSTATE_USER.value:
      raise ValueError('control_spec can only contain bits in mjSTATE_USER')

    # check types
    if nstep and not isinstance(nstep, (int, np.integer)):
      raise ValueError('nstep must be an integer')
    if chunk_size and not isinstance(chunk_size, (int, np.integer)):
      raise ValueError('chunk_size must be an integer')
    _check_must_be_numeric(
        initial_state=initial_state,
        initial_warmstart=initial_warmstart,
        control=control,
        state=state,
        sensordata=sensordata,
    )

    # check number of dimensions
    _check_number_of_dimensions(
        2, initial_state=initial_state, initial_warmstart=initial_warmstart
    )
    _check_number_of_dimensions(
        3, control=control, state=state, sensordata=sensordata
    )

    # ensure 2D, make contiguous, row-major (C ordering)
    initial_state = _ensure_2d(initial_state)
    initial_warmstart = _ensure_2d(initial_warmstart)

    # ensure 3D, make contiguous, row-major (C ordering)
    control = _ensure_3d(control)
    state = _ensure_3d(state)
    sensordata = _ensure_3d(sensordata)

    # infer nbatch, check for incompatibilities
    nbatch = _infer_dimension(
        0,
        1,
        initial_state=initial_state,
        initial_warmstart=initial_warmstart,
        control=control,
        state=state,
        sensordata=sensordata,
    )
    if isinstance(model, list) and nbatch == 1:
      nbatch = len(model)

    if isinstance(model, list) and len(model) > 1 and len(model) != nbatch:
      raise ValueError(
          f'nbatch inferred as {nbatch} but model is length {len(model)}'
      )
    elif not isinstance(model, list):
      model = [model]  # Use a length 1 list to simplify code below

    if not isinstance(data, list):
      data = [data]  # Use a length 1 list to simplify code below  # pyrefly: ignore[bad-assignment]

    # infer nstep, check for incompatibilities
    nstep = _infer_dimension(
        1, nstep or 1, control=control, state=state, sensordata=sensordata
    )

    # get nstate/ncontrol/nv/nsensordata
    # check that they are equal across models
    nstate = mujoco.mj_stateSize(
        model[0], mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    )
    ncontrol = mujoco.mj_stateSize(model[0], control_spec)
    nv = model[0].nv
    nsensordata = model[0].nsensordata
    for m in model[1:]:
      if (
          nstate
          != mujoco.mj_stateSize(m, mujoco.mjtState.mjSTATE_FULLPHYSICS.value)
          or ncontrol != mujoco.mj_stateSize(m, control_spec)
          or nv != m.nv
          or nsensordata != m.nsensordata
      ):
        raise ValueError('models are not compatible')

    # check trailing dimensions
    _check_trailing_dimension(nstate, initial_state=initial_state, state=state)
    _check_trailing_dimension(ncontrol, control=control)
    _check_trailing_dimension(nv, initial_warmstart=initial_warmstart)
    _check_trailing_dimension(nsensordata, sensordata=sensordata)

    # tile input arrays if required (singleton expansion); one model serves all
    model = model[0] if len(model) == 1 else _one_model(model)
    initial_state = _tile_if_required(initial_state, nbatch)
    initial_warmstart = _tile_if_required(initial_warmstart, nbatch)
    control = _tile_if_required(control, nbatch, nstep)

    # allocate output if not provided
    if state is None:
      state = np.empty((nbatch, nstep, nstate), dtype=mujoco.MJTNUM_DTYPE)  # pyrefly: ignore[no-matching-overload]
    if sensordata is None:
      sensordata = np.empty((nbatch, nstep, nsensordata), dtype=mujoco.MJTNUM_DTYPE)  # pyrefly: ignore[no-matching-overload]

    # call rollout
    self._run(
        model,
        data,
        nstep,
        control_spec,
        initial_state,
        initial_warmstart,
        control,
        state,
        sensordata,
    )

    # return outputs
    return state, sensordata

  def _run(
      self,
      model,
      data,
      nstep,
      control_spec,
      initial_state,
      initial_warmstart,
      control,
      state,
      sensordata,
  ):
    """Roll out on the batch: rows from the initial states, then mjb_rollout."""
    if not isinstance(data, list):
      data = [data]
    if not data:
      raise ValueError('The list of data instances is empty')
    if self.nthread == 0 and len(data) > 1:
      raise ValueError(
          'More than one data instance passed but rollout is configured to run'
          ' on main thread'
      )
    if self.nthread > 0 and self.nthread != len(data):
      raise ValueError(
          f'Length of data: {len(data)} not equal to nthread: {self.nthread}'
      )
    for t, d in enumerate(data):
      # rollout threads across simulations; the engine's pool threads within one
      if d.threadpool:
        raise ValueError(
            f'data[{t}] has a thread pool installed (mju_threadpool); rollout'
            ' parallelizes across simulations and cannot use it'
        )
    if nstep < 1:
      return
    with self._lock:
      self._run_locked(
          model,
          data,
          nstep,
          control_spec,
          initial_state,
          initial_warmstart,
          control,
          state,
          sensordata,
      )

  def _run_locked(
      self,
      model,
      data,
      nstep,
      control_spec,
      initial_state,
      initial_warmstart,
      control,
      state,
      sensordata,
  ):
    nbatch = initial_state.shape[0]
    if not isinstance(model, mujoco.MjModel):
      model = _one_model(list(model[:nbatch]))
    model0 = model if isinstance(model, mujoco.MjModel) else model[0]
    for remake in (False, True):
      b = self._batch_for(model0, nbatch, remake)
      if isinstance(model, mujoco.MjModel):
        models = model
      else:  # as many as the batch has
        models = model + [model[0]] * (b.nsim - nbatch)
      try:
        b.set_models(models)
        break
      except mujoco.FatalError as e:
        if remake:
          raise ValueError('models are not compatible') from e

    # rows: the initial states, and the inputs of each model's reset
    rows = b.state[:nbatch]
    layout = self._layout(model0)
    head, nplugin = layout.head, layout.nplugin
    rows[:, head:] = _inputs(model, model0, data[0], nbatch, layout)
    rows[:, :head] = initial_state[:, :head]
    if nplugin:
      rows[:, -nplugin:] = initial_state[:, head:]
    if initial_warmstart is not None:
      rows[:, head : head + model0.nv] = initial_warmstart
    b.warning[:nbatch] = 0

    records = []
    if state is not None:
      records.append((None, _FULLPHYSICS, state))
    if sensordata is not None:
      records.append(('sensordata', 0, sensordata))
    ids = None if nbatch == b.nsim else self._ids(nbatch)
    if b._b.rollout(ids, nstep, control, control_spec, records):
      failed = np.flatnonzero(b.status[:nbatch])[0]
      raise mujoco.FatalError(b._b.error(int(failed)))

  def _batch_for(self, model0, nbatch, remake=False):
    """A batch of at least nbatch simulations of models of model0's layout."""
    b = self._batch
    persistent = bool(
        model0.opt.enableflags
        & (mujoco.mjtEnableBit.mjENBL_SLEEP | mujoco.mjtEnableBit.mjENBL_IPC)
    )
    if (
        remake
        or b is None
        or b.nsim < nbatch
        or b.persistent != persistent
        or not _same_layout(b.model, model0)
    ):
      nsim = max(nbatch, b.nsim if b is not None else 0)
      b = _batch.Batch(model0, nsim, max(1, self.nthread), persistent)
      self._batch = b
      self._ranges = {}
    return b

  def _layout(self, model0):
    """Offsets of model0's state components, cached for the model."""
    if self._layout_model is not model0:
      self._layout_model = model0
      self._layout_cache = _Layout(model0)
    return self._layout_cache

  def _ids(self, n):
    ids = self._ranges.get(n)
    if ids is None:
      ids = self._ranges[n] = np.arange(n, dtype=np.int32)
    return ids


persistent_rollout = None


def shutdown_persistent_pool():
  """Shutdown the persistent thread pool that is optionally created by rollout.

  This is called automatically interpreter shutdown, but can also be called manually.
  """  # fmt: skip
  global persistent_rollout
  if persistent_rollout is not None:
    persistent_rollout.close()
  persistent_rollout = None


atexit.register(shutdown_persistent_pool)


def rollout(
    model: Union[mujoco.MjModel, Sequence[mujoco.MjModel]],
    data: Union[mujoco.MjData, Sequence[mujoco.MjData]],
    initial_state: npt.ArrayLike,
    control: Optional[npt.ArrayLike] = None,
    *,  # require subsequent arguments to be named
    control_spec: int = mujoco.mjtState.mjSTATE_CTRL.value,
    skip_checks: bool = False,
    nstep: Optional[int] = None,
    initial_warmstart: Optional[npt.ArrayLike] = None,
    state: Optional[npt.ArrayLike] = None,
    sensordata: Optional[npt.ArrayLike] = None,
    chunk_size: Optional[int] = None,
    persistent_pool: bool = False,
):
  """Rolls out open-loop trajectories from initial states, get subsequent states and sensor values.

  Python wrapper for rollout.cc, see documentation therein.
  Infers nbatch and nstep.
  Tiles inputs with singleton dimensions.
  Allocates outputs if none are given.

  Args:
    model: An instance or length nbatch sequence of MjModel with the same size signature.
    data: Associated mjData instance or sequence of instances with length nthread.
    initial_state: Array of initial states from which to roll out trajectories.
      ([nbatch or 1] x nstate)
    control: Open-loop controls array to apply during the rollouts.
      ([nbatch or 1] x [nstep or 1] x ncontrol)
    control_spec: mjtState specification of control vectors.
    skip_checks: Whether to skip internal shape and type checks.
    nstep: Number of steps in rollouts (inferred if unspecified).
    initial_warmstart: Initial qfrc_warmstart array (optional).
      ([nbatch or 1] x nv)
    state: State output array (optional).
      (nbatch x nstep x nstate)
    sensordata: Sensor data output array (optional).
      (nbatch x nstep x nsensordata)
    chunk_size: Determines threadpool chunk size. If unspecified,
                chunk_size = max(1, nbatch / (nthread * 10))
    persistent_pool: Determines if a persistent thread pool is created or reused.

  Returns:
    state:
      State output array, (nbatch x nstep x nstate).
    sensordata:
      Sensor data output array, (nbatch x nstep x nsensordata).

  Raises:
    ValueError: bad shapes or sizes.
  """  # fmt: skip
  if not isinstance(data, list):
    data = [data]  # Use a length 1 list to simplify code below  # pyrefly: ignore[bad-assignment]

  nthread = len(data) if len(data) > 1 else 0  # pyrefly: ignore[bad-argument-type]

  # Use a persistent thread pool if requested
  if persistent_pool:
    # Create or restart persistent threadpool
    global persistent_rollout
    if persistent_rollout is None:
      persistent_rollout = Rollout(nthread=nthread)
    if persistent_rollout.nthread != nthread:
      persistent_rollout.close()
      persistent_rollout = Rollout(nthread=nthread)
    rollout_ = persistent_rollout
  else:
    rollout_ = Rollout(nthread=nthread)

  try:
    return rollout_.rollout(
        model,
        data,
        initial_state,
        control,
        control_spec=control_spec,
        skip_checks=skip_checks,
        nstep=nstep,
        initial_warmstart=initial_warmstart,
        state=state,
        sensordata=sensordata,
        chunk_size=chunk_size,
    )
  finally:
    if not persistent_pool:
      rollout_.close()


def _same_layout(a, b):
  """Whether models a and b give MjData the same sizes, as far as cheap to see."""
  return a is b or (
      a.nq == b.nq
      and a.nv == b.nv
      and a.nu == b.nu
      and a.na == b.na
      and a.nbody == b.nbody
      and a.ngeom == b.ngeom
      and a.nsensordata == b.nsensordata
      and a.nmocap == b.nmocap
      and a.neq == b.neq
  )


def _one_model(models):
  """models[0] if every entry is that model, else models."""
  first = models[0] if models else None
  return first if models and all(m is first for m in models) else models


class _Layout:
  """Sizes and offsets of a model's state components in a batch's rows."""

  def __init__(self, model):
    # the row is time..history (FULLPHYSICS but the plugin state), then the
    # rest of mjSTATE_INTEGRATION, ending with the plugin state
    self.head = mujoco.mj_stateSize(model, _FULLPHYSICS & ~_PLUGIN)
    self.nplugin = mujoco.mj_stateSize(model, _PLUGIN)
    self.tail = mujoco.mj_stateSize(model, _INTEGRATION) - self.head

    def offset(spec):  # of the component spec after the head
      return mujoco.mj_stateSize(model, (spec - 1) & ~_PLUGIN) - self.head

    self.eq_active = offset(_EQ_ACTIVE)
    self.mocap_pos = offset(_MOCAP_POS)
    self.mocap_quat = offset(_MOCAP_QUAT)
    self.userdata = offset(_USERDATA)
    mocap = np.flatnonzero(model.body_mocapid >= 0)
    self.mocap_bodies = mocap[np.argsort(model.body_mocapid[mocap])]


def _inputs(model, model0, data0, nbatch, layout):
  """Row entries after time..history, as mj_resetData sets them.

  Zero warmstart and inputs, each model's mocap poses and active constraints,
  and data0's userdata: (size,) for one model, (nbatch, size) for a list.
  """
  row = np.zeros(layout.tail, mujoco.MJTNUM_DTYPE)
  if model0.nuserdata:
    o = layout.userdata
    row[o : o + model0.nuserdata] = data0.userdata
  if not (model0.neq or model0.nmocap):
    return row
  models = [model] if isinstance(model, mujoco.MjModel) else model
  rows = np.tile(row, (len(models), 1))
  mocap = layout.mocap_bodies
  for i, m in enumerate(models):
    if m.neq:
      o = layout.eq_active
      rows[i, o : o + m.neq] = m.eq_active0
    if m.nmocap:
      o = layout.mocap_pos
      rows[i, o : o + 3 * m.nmocap] = m.body_pos[mocap].ravel()
      o = layout.mocap_quat
      rows[i, o : o + 4 * m.nmocap] = m.body_quat[mocap].ravel()
  return rows if len(models) > 1 else rows[0]


def _check_must_be_numeric(**kwargs):
  for key, value in kwargs.items():
    if value is None:
      continue
    if not isinstance(value, np.ndarray) and not isinstance(value, float):
      raise ValueError(f'{key} must be a numpy array or float')


def _check_number_of_dimensions(ndim, **kwargs):
  for key, value in kwargs.items():
    if value is None:
      continue
    if value.ndim > ndim:
      raise ValueError(f'{key} can have at most {ndim} dimensions')


def _check_trailing_dimension(dim, **kwargs):
  for key, value in kwargs.items():
    if value is None:
      continue
    if value.shape[-1] != dim:
      raise ValueError(
          f'trailing dimension of {key} must be {dim}, got {value.shape[-1]}'
      )


def _ensure_2d(arg):
  if arg is None:
    return None
  else:
    return np.ascontiguousarray(np.atleast_2d(arg), dtype=mujoco.MJTNUM_DTYPE)


def _ensure_3d(arg):
  if arg is None:
    return None
  else:
    # np.atleast_3d adds both leading and trailing dims, we want only leading
    if arg.ndim == 0:
      arg = arg[np.newaxis, np.newaxis, np.newaxis, ...]
    elif arg.ndim == 1:
      arg = arg[np.newaxis, np.newaxis, ...]
    elif arg.ndim == 2:
      arg = arg[np.newaxis, ...]
    return np.ascontiguousarray(arg, dtype=mujoco.MJTNUM_DTYPE)


def _infer_dimension(dim, value, **kwargs):
  """Infers dimension `dim` given guess `value` from set of arrays.

  Args:
    dim: Dimension to be inferred.
    value: Initial guess of inferred value (1: unknown).
    **kwargs: List of arrays which should all have the same size (or 1) along
      dimension dim.

  Returns:
    Inferred dimension.

  Raises:
    ValueError: If mismatch between array shapes or initial guess.
  """
  for name, array in kwargs.items():
    if array is None:
      continue
    if array.shape[dim] != value:
      if value == 1:
        value = array.shape[dim]
      elif array.shape[dim] != 1:
        raise ValueError(
            f'dimension {dim} inferred as {value} '
            f'but {name} has {array.shape[dim]}'
        )
  return value


def _tile_if_required(array, dim0, dim1=None):
  if array is None:
    return
  reps = np.ones(array.ndim, dtype=int)
  if array.shape[0] == 1:
    reps[0] = dim0
  if dim1 is not None and array.shape[1] == 1:
    reps[1] = dim1
  return np.tile(array, reps)
