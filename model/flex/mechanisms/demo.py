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


"""Drive and optionally record the elastic mechanism examples.

Run with a current MuJoCo Python build. Recording also requires imageio[ffmpeg].
The XML files can be opened directly in simulate; use actuator sliders to drive
slidercrank.xml and dipper.xml. All flexible coordinates remain passive.
"""

import argparse
import contextlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np


MODELS = ("cantilever", "slidercrank", "dipper")
BEAMS = ("soft", "stiff", "damped")


def control(name, data):
  """Smoothly start a slow periodic drive after one second of settling."""
  t = max(0.0, data.time - 1.0)
  ramp = min(t, 1.0)
  ramp = ramp * ramp * (3.0 - 2.0 * ramp)
  if name == "slidercrank":
    data.ctrl[0] = 2 * np.pi / 3 * (t - .5 * (1 - np.exp(-2 * t)))
  elif name == "dipper":
    data.ctrl[0] = .07 * ramp * np.sin(2 * np.pi * t / 2)


def deformation(name, data):
  """Measure bending independently of the driven rigid motion, in meters."""
  if name == "cantilever":
    return max(.85 - data.body(case + "_tip").xpos[2]
               for case in BEAMS)
  if name == "slidercrank":
    start = data.body("left_fitting").xpos
    end = data.body("right_fitting").xpos
    midpoint = np.mean([data.body(f"rod_6_{j}_{k}").xpos
                        for j in range(2) for k in range(2)], axis=0)
    return np.linalg.norm(midpoint - (start + end) / 2)
  payload = data.body("tip_fitting").xpos
  support = data.body("fulcrum")
  local_tip = support.xmat.reshape(3, 3).T @ (payload - support.xpos)
  return np.linalg.norm(local_tip - [1.2, 0, 0])


def run(name, seconds=10, output=None, timestep=None, stiffness=1):
  """Roll out a model; return measurements and the sampled payload trajectory."""
  path = Path(__file__).with_name(name + ".xml")
  spec = mujoco.MjSpec.from_file(str(path))
  if stiffness != 1:
    for flex in spec.flexes:
      flex.young *= stiffness
  model = spec.compile()
  if timestep is not None:
    model.opt.timestep = timestep
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  dt = model.opt.timestep
  steps = max(1, round(seconds / dt))
  peak_attachment = 0.0
  peak_deformation = 0.0
  samples = []
  step_wall = 0.0
  next_frame = 0
  fps = 30

  with contextlib.ExitStack() as stack:
    renderer = writer = None
    if output is not None:
      import imageio.v2 as imageio  # Optional video dependency.
      output = Path(output)
      output.mkdir(parents=True, exist_ok=True)
      renderer = stack.enter_context(mujoco.Renderer(model, height=720, width=1280))
      writer = stack.enter_context(imageio.get_writer(
          output / (name + ".mp4"), fps=fps, codec="libx264", quality=8,
          macro_block_size=1))
    for step in range(steps + 1):
      # mj_step leaves positional diagnostics at the beginning of its step.
      # Refresh them at sample times before measurements and rendering.
      if data.time + dt / 2 >= next_frame / fps or step == steps:
        mujoco.mj_forward(model, data)
        bend = deformation(name, data)
        peak_deformation = max(peak_deformation, float(bend))
        payload_name = {"cantilever": "soft_tip", "slidercrank": "slider"}.get(name, "payload")
        row = [data.time, *data.body(payload_name).xpos, bend]
        if name == "cantilever":
          row.extend(.85 - data.body(case + "_tip").xpos[2] for case in BEAMS)
        samples.append(row)
        if renderer is not None:
          renderer.update_scene(data, camera="overview")
          frame = renderer.render()
          writer.append_data(frame)
          if next_frame in (0, 60, 120, 180):
            imageio.imwrite(output / f"{name}-{next_frame:03d}.png", frame)
        next_frame += 1
      if step == steps:
        break
      control(name, data)
      start = time.perf_counter()
      mujoco.mj_step(model, data)
      step_wall += time.perf_counter() - start
      equality = data.efc_pos[data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
      peak_attachment = max(peak_attachment, float(np.max(np.abs(equality), initial=0)))
      if np.any(data.warning.number) or not np.isfinite(data.qpos).all():
        raise RuntimeError(f"{name}: invalid rollout at step {step}: {data.warning.number}")
    if not np.isclose(data.time, steps * dt, rtol=1e-4, atol=dt / 2):
      raise RuntimeError(f"{name}: simulation time reset unexpectedly")

  samples = np.asarray(samples)
  metrics = dict(
      model=name, dofs=model.nv, timestep_s=dt, simulated_s=float(data.time),
      step_wall_s=step_wall, realtime_factor=data.time / step_wall,
      max_attachment_component_m=peak_attachment,
      max_deformation_m=peak_deformation,
      final_deformation_m=float(samples[-1, 4]),
      payload_travel_m=np.ptp(samples[:, 1:4], axis=0).tolist(),
      warnings=data.warning.number.tolist())
  if output is not None:
    (output / (name + ".json")).write_text(json.dumps(metrics, indent=2) + "\n")
    np.savetxt(output / (name + ".csv"), samples, delimiter=",",
               header="time,x,y,z,deformation" +
               (",soft,stiff,damped" if name == "cantilever" else ""), comments="")
  return metrics, samples


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--model", choices=(*MODELS, "all"), default="all")
  parser.add_argument("--seconds", type=float, default=10)
  parser.add_argument("--output", type=Path, help="Write MP4s, PNGs and measurements here")
  args = parser.parse_args()
  if args.seconds <= 0:
    parser.error("--seconds must be positive")
  for name in MODELS if args.model == "all" else (args.model,):
    metrics, _ = run(name, args.seconds, args.output)
    print(json.dumps(metrics, indent=2), flush=True)


if __name__ == "__main__":
  main()
