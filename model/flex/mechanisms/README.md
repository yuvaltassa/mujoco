# Elastic mechanisms

Three mechanisms inspired by [Miles Macklin's Reduced Elastic Links experiments](https://reports.mmacklin.com/newton-reduced/reduced_elastic_links_implementation.html), implemented with MuJoCo's multicell trilinear flexes. The geometry and MJCF are original; no Newton source or external mesh assets are required.

| Model | Mechanism | Interpolation cells | Total DOFs |
| --- | --- | --- | --- |
| `cantilever.xml` | A clamped beam released under a 0.3 kg tip load | 16 × 1 × 1 | 198 |
| `fourbar.xml` | Two hinged rigid arms joined by a flexible coupler carrying a central load | 8 × 1 × 1 | 118 |
| `dipper.xml` | An interior-supported flexible arm, raised and lowered by a telescoping cylinder | 12 × 1 × 1 | 171 |

Open an XML file directly in `simulate` and select the `overview` camera. The cantilever moves under gravity without controls. Use the `drive` actuator slider for the four-bar, or `cylinder` for the dipper. Their zero controls are useful starting poses.

For the slow demonstration trajectories and optional recording, use Python bindings built from current MuJoCo:

```sh
python model/flex/mechanisms/demo.py
python model/flex/mechanisms/demo.py --model dipper --output /path/to/previews
```

Recording additionally requires `imageio[ffmpeg]`. It produces a ten-second MP4, selected PNG frames, a CSV trajectory and JSON measurements for each model. Rendering is excluded from the reported simulation step timing.

## Construction

The fine tetrahedral grid provides the visible flex surface; `cellcount` independently controls the coarser deformation grid. The cantilever pins the four control nodes of its root cross-section. The moving mechanisms instead use world-frame control nodes, which support the discrete integrator's sparse Newton solve.

Four point equalities attach each selected cross-section to a rigid fitting. The fitting can then carry an ordinary hinge, as at the ends of the four-bar and the dipper fulcrum. These constraints clamp the section to the fitting; the hinge supplies its rotational freedom. In the dipper, a base hinge, actuated prismatic rod and tip point connection form the cylinder. The flex DOFs are never directly actuated.

Orange denotes flexible material, silver denotes fittings, and brass denotes pivots. Colors are constant, not strain measurements. Contact is disabled so these examples isolate elastic loading and mechanism motion.

## Validation and scope

Ten-second trajectories were checked in double and single precision, at 1 ms and 0.5 ms timesteps, without warnings. At the default timestep, the largest absolute Cartesian attachment-error component was below 0.15 mm. Halving the timestep changed the sampled payload paths by less than 0.7 mm in double precision. A tenfold Young's modulus increase reduced peak deformation by approximately tenfold in all three examples.

These are mechanism demonstrations, not calibrated material benchmarks. Bending compliance depends appreciably on the interpolation grid. For example, at one-tenth gravity, the cantilever's settled tip sag was 5.75, 9.79 and 11.91 mm with 8, 16 and 32 longitudinal cells, respectively. An Euler–Bernoulli reference including the tip load and uniformly distributed beam weight gives 8.47 mm. Thus timestep stability does not establish spatial convergence or agreement with beam theory; retain this distinction when changing geometry, resolution or material parameters.
