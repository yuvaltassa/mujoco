# Elastic mechanisms

Mechanisms inspired by [Miles Macklin's Reduced Elastic Links experiments](https://reports.mmacklin.com/newton-reduced/reduced_elastic_links_implementation.html), implemented with MuJoCo's multicell trilinear flexes. The geometry and MJCF are original; no Newton source or external mesh assets are required.

| Model | Mechanism | Interpolation cells | Total DOFs |
| --- | --- | --- | --- |
| `cantilever.xml` | Three beams with different stiffness or damping and directly attached tip masses | 16 × 1 × 1 per beam | 594 |
| `slidercrank.xml` | A crank wheel drives a sliding piston through a flexible connecting rod | 12 × 1 × 1 | 160 |
| `dipper.xml` | A cylinder-driven flexible arm with a freely swinging tendon-suspended payload | 12 × 1 × 1 | 177 |

Open an XML file directly in `simulate` and select the `overview` camera. The cantilevers move under gravity without controls. Use the `crank` actuator slider for the slider-crank, or `cylinder` for the dipper. The script provides continuous crank rotation and a 1 Hz dipper drive (a one-second period):

```sh
python model/flex/mechanisms/demo.py
python model/flex/mechanisms/demo.py --model dipper --seconds 12 --output /path/to/previews
```

Use Python bindings built from current MuJoCo. Recording additionally requires `imageio[ffmpeg]`; it produces MP4s, selected PNG frames, CSV trajectories and JSON measurements. Rendering is excluded from simulation step timing. The cantilever CSV includes a separate tip-deflection column for each beam.

## Cantilever comparison

All three beams have the same 0.9 m length, 0.07 × 0.055 m cross-section, 0.12 kg beam mass and 0.2 kg tip mass. Each tip mass is centered on the end cross-section, without a hanging link or eccentric load. Their material parameters are:

| Color / beam | Young's modulus | Stiffness-proportional Rayleigh damping coefficient |
| --- | --- | --- |
| Teal / soft | 8 MPa | 0.002 s |
| Orange / stiff | 16 MPa | 0.002 s |
| Violet / damped | 8 MPa | 0.04 s |

The two soft beams have the same static equilibrium, but different transient decay. The stiff beam bends less and oscillates faster. Node joint damping is zero so it does not obscure the specified material damping.

Generate response plots and a separate small-load theory comparison with:

```sh
python model/flex/mechanisms/compare_beams.py --output /path/to/previews
```

This also requires `matplotlib`. The Euler–Bernoulli reference includes the tip weight and uniformly distributed beam self-weight:

```text
I = b h³ / 12
δ_tip = m_tip g L³ / (3 E I) + m_beam g L³ / (8 E I)
```

See [MIT 1.050 Solid Mechanics, Fall 2004, Problem Set 11, page 2](https://ocw.mit.edu/courses/1-050-solid-mechanics-fall-2004/fd4eff39aec922b8c07660006f40686e_pset04_11.pdf) for the end-load and distributed-load solutions. The check uses one-tenth gravity to keep deflections small, and temporarily increases damping to reach equilibrium. Damping does not change the static force balance.

| Beam | Theory at 0.1g | Simulated, 16 cells | Difference |
| --- | --- | --- | --- |
| Soft | 7.522 mm | 8.758 mm | +16.4% |
| Stiff | 3.761 mm | 4.381 mm | +16.5% |
| Damped | 7.522 mm | 8.758 mm | +16.4% |

The agreement in relative stiffness is good, but the absolute bending compliance remains discrepant and resolution-dependent. These are mechanism demonstrations, not calibrated beam benchmarks. The comparison preserves the material inputs rather than fitting Young's modulus to the theoretical answer. The large-amplitude gravity-release animation is not used as the small-deflection validation.

## Attachments and suspension

The fine tetrahedral grid supplies the visible surface; `cellcount` controls the coarser deformation grid. The cantilevers pin the four nodes of their root cross-sections. The moving mechanisms use world-frame flex nodes, which support the discrete integrator's sparse Newton solve.

Four point equalities clamp each attached cross-section to a rigid fitting. The fitting can carry an ordinary hinge, as at the connecting rod ends and dipper fulcrum. In the dipper, a base hinge, actuated prismatic rod and tip point connection form the drive cylinder. The flex DOFs are passive. The slider joint has 30 N·s/m damping to resist piston motion and load the connecting rod. In the twelve-second drive, peak midpoint bending is about 0.25 m and peak centerline extension is approximately 1%.

The dipper's 0.5 kg payload is a separate free body. A 0.42 m spatial tendon with only an upper length limit connects its top to the arm's tip fitting: it transmits tension and permits slack, without a rigid suspension link or payload pose controller. At 1 Hz the suspension goes slack and the payload tumbles through large swings. In the twelve-second double-precision recording, sampled tendon extension reaches approximately 4 mm. Disabling that tendon causes the payload to fall freely.

Colors are constant, not strain measurements. Contact is disabled to isolate elastic loading and mechanism motion. The suspended payload remains above the ground throughout the demonstration.

## Validation

Twelve-second trajectories were checked in double and single precision, at 1 ms and 0.5 ms timesteps, without warnings. The three-beam scene uses a larger iteration budget to converge its combined solve in single precision. Maximum Cartesian attachment-error components reach 0.56 mm in the faster dipper and remain below 0.22 mm in the slider-crank. The dipper exceeds the earlier 0.5 mm attachment-error check. XML save/reload checks were performed before this parameter-only update.

The lightly damped cantilever phase is timestep-sensitive: its soft-beam tip differs by about 4.2 mm at twelve seconds between the two timesteps. The slider-crank final-position difference remains below 0.1 mm. The 1 Hz dipper has substantially different late-time trajectories across timesteps and precisions, including out-of-plane motion in single precision; its recorded trajectory is not converged. Warning-free rollouts do not establish spatial or temporal convergence.
