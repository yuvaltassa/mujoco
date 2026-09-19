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


"""Compare the cantilever trio with small-deflection Euler-Bernoulli theory.

Requires matplotlib in addition to the demo dependencies. The reference includes
both the directly attached tip mass and uniformly distributed beam self-weight.
MIT 1.050 Solid Mechanics, Fall 2004, Problem Set 11, page 2:
https://ocw.mit.edu/courses/1-050-solid-mechanics-fall-2004/fd4eff39aec922b8c07660006f40686e_pset04_11.pdf
"""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

import demo


def reference(model, spec, name, gravity):
  """Return EI and the cantilever tip deflection under tip and self-weight."""
  fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, name)
  a = model.flex_vertadr[fid]
  n = model.flex_vertnum[fid]
  length, width, height = np.ptp(model.flex_vert[a:a+n], axis=0)
  a = model.flex_nodeadr[fid]
  n = model.flex_nodenum[fid]
  bodies = np.unique(model.flex_nodebodyid[a:a+n])
  bodies = bodies[model.body_dofnum[bodies] > 0]
  beam_mass = float(np.sum(model.body_mass[bodies]))
  tip_mass = float(model.body(name + "_tip").mass[0])
  material = next(f for f in spec.flexes if f.name == name)
  ei = material.young * width * height**3 / 12
  delta = gravity * length**3 / ei * (tip_mass / 3 + beam_mass / 8)
  return dict(young_pa=material.young, damping_s=material.damping,
              length_m=float(length), width_m=float(width), height_m=float(height),
              beam_mass_kg=beam_mass, tip_mass_kg=tip_mass,
              ei_nm2=float(ei), reference_tip_m=float(delta))


def compare(output):
  output.mkdir(parents=True, exist_ok=True)
  _, trace = demo.run("cantilever", seconds=12)
  spec = mujoco.MjSpec.from_file(str(Path(__file__).with_name("cantilever.xml")))
  model = spec.compile()
  gravity = -model.opt.gravity[2]
  results = {name: reference(model, spec, name, gravity) for name in demo.BEAMS}

  # Use a small load for the linear theory comparison. Additional damping only
  # accelerates settling; it does not change the static equilibrium equations.
  for flex in spec.flexes:
    flex.damping = .12
  spec.option.gravity[2] *= .1
  static_model = spec.compile()
  data = mujoco.MjData(static_model)
  for _ in range(round(10 / static_model.opt.timestep)):
    mujoco.mj_step(static_model, data)
    if np.any(data.warning.number):
      raise RuntimeError(f"Static comparison produced warnings: {data.warning.number}")
  mujoco.mj_forward(static_model, data)
  max_speed = float(np.max(np.abs(data.qvel)))
  if max_speed > 1e-5:
    raise RuntimeError(f"Static comparison has not settled: {max_speed=}")
  for name, row in results.items():
    measured = float(.85 - data.body(name + "_tip").xpos[2])
    expected = row["reference_tip_m"] * .1
    row.update(small_load_reference_m=expected, small_load_measured_m=measured,
               small_load_error_percent=100 * (measured / expected - 1))
  report = dict(gravity_scale=.1, settling_damping_s=.12,
                settling_max_generalized_speed=max_speed, beams=results)
  (output / "beam-comparison.json").write_text(json.dumps(report, indent=2) + "\n")
  np.savetxt(output / "cantilever.csv", trace, delimiter=",",
             header="time,x,y,z,deformation,soft,stiff,damped", comments="")

  colors = ["#168c83", "#dd6427", "#8365b7"]
  labels = ["Soft · E = 8 MPa, damping = 2 ms",
            "Stiff · E = 16 MPa, damping = 2 ms",
            "Damped · E = 8 MPa, damping = 40 ms"]
  fig, axes = plt.subplots(2, 1, figsize=(12, 8), layout="constrained")
  for i, (name, color, label) in enumerate(zip(demo.BEAMS, colors, labels)):
    axes[0].plot(trace[:, 0], 1000 * trace[:, 5+i], color=color, label=label, lw=1.7)
  axes[0].set(xlabel="Time (s)", ylabel="Tip deflection (mm)",
              title="Gravity release: same geometry and 0.2 kg tip mass")
  axes[0].legend(loc="upper right", fontsize=9)
  axes[0].grid(alpha=.2)
  x = np.arange(3)
  axes[1].bar(x-.18, [1000*results[n]["small_load_reference_m"] for n in demo.BEAMS],
              width=.34, color="#b7c3cc", label="Euler–Bernoulli reference")
  bars = axes[1].bar(x+.18, [1000*results[n]["small_load_measured_m"] for n in demo.BEAMS],
                     width=.34, color=colors, label="MuJoCo, 16 trilinear cells")
  axes[1].bar_label(bars, labels=[f'{results[n]["small_load_error_percent"]:+.1f}%'
                                for n in demo.BEAMS], padding=5)
  axes[1].set(xticks=x, xticklabels=["Soft", "Stiff", "Damped"],
              ylabel="Static tip deflection (mm)",
              title="Small-load check at 0.1g: simulation versus linear beam theory")
  axes[1].set_ylim(0, max(b.get_height() for b in bars)*1.3)
  axes[1].legend(loc="upper right", fontsize=9)
  axes[1].grid(axis="y", alpha=.2)
  axes[1].set_axisbelow(True)
  fig.suptitle("Cantilever comparison", fontsize=18, fontweight="bold")
  fig.savefig(output / "beam-comparison.png", dpi=160)
  fig.savefig(output / "beam-comparison.pdf")
  plt.close(fig)
  return report


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--output", type=Path, required=True)
  args = parser.parse_args()
  print(json.dumps(compare(args.output), indent=2))
