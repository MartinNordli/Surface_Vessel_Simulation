# Implementation and evidence for plan.md

The versioned configuration and independent Njord force model are implemented.
The supplied hull and coefficients remain an **uncalibrated analytical fixture**.
Actual Njord CAD, load properties and independent boat trials were not supplied;
there is no claim of calibrated Njord realism.

## Implemented scope

- Strict shared vessel/scenario/algorithm resolution, explicit legacy conversion,
  immutable resource copies, generated model/TF/bridges, public autonomy projection
  and configuration/manifests tied to run identity.
- Separate visual/collision/buoyancy geometry; validated convex triangular OBJ or
  analytical boxes; disjoint buoyancy volumes; geometry-derived scoring envelope
  and navigation inflation; complete mass/COM/inertia and sensor poses.
- Clipped-volume hydrostatics and centroid moments, native added mass with
  relative-current hydrodynamics, signed periodic wind tables, physical asymmetric
  thruster allocation, forward/reverse limits, response lag and command expiry.
- Fresh-process dynamics experiments, stationarity/incomplete measurement rules,
  timestep/repetition acceptance tooling and a physical measurement/holdout protocol.

The plan's `vessel.yaml` role is filled by versioned profiles (`njord_v1.yaml`,
`wamv_reference.yaml`) selected with `VESSEL_CONFIG`. The unversioned
`vessel.yaml` remains the default WAM-V input through the explicit legacy adapter.

See [physical configuration](physical-configuration.md) for exact schema, units,
frames and supported geometry, and [calibration protocol](njord-calibration.md)
for measurements and acceptance.

## Reproducible evidence

The final source fingerprint is
`246dbd711ca40a5c6cfd0753fffac1fa5b5334399f8e1103fdfb5cfa0c411c9f`.
The image is retained locally as `njord-sim:physics-validation-final`, identity
`sha256:9739e3c9b73d3be5bd15bace8ad4db4211f0fa365eff4c009c1b823123093264`.
This fingerprint includes uncommitted Docker inputs; the Git HEAD alone does not
identify this implementation. Image labels and per-run manifests retain both.
Generated evidence remains in ignored `outputs/`, not in Git.

| Check | Evidence and scope |
| --- | --- |
| Existing WAM-V baseline | `outputs/yaml-physics-baseline/`; 119 original-image tests passed, plus original sequential dynamics observations. Its coast followed a turn and is not isolated-coast acceptance. |
| Final container suite | `outputs/yaml-physics-final-container-tests.log`; 163 tests passed, including actual ROS transport, generated SDF and C++ hydrostatics. |
| Final GPU sensors/navigation | `outputs/njord-final-sensor-smoke.log`; both 640×360 cameras, finite lidar points, navigation and camera TF passed in running Gazebo. |
| Physical library loading | `outputs/njord-runtime-library-maps.txt`; actual process loads Gazebo8.15 hydrodynamics and Njord force plugin. Package versions and binary checksum are in `docker/dependencies.lock.json`. |
| Actuator behavior | `outputs/njord-actuator-evidence/actuator_probe.json`; applied response and invalid-input decay matched configured 0.1 s lag, both targets invalidated, wall expiry measured 0.506 s for 0.5 s setting. This earlier image has the same actuator implementation; the later change made world gravity explicit. |
| Njord runtime manoeuvres | `outputs/njord-runtime-final/summary.json`; one fresh 4 ms trial each at 150 N: flotation within 1 cm and restoring response, 3 m/s wind drifts east, 0.5 m/s current drifts north, unequal thruster arms yaw positive, reverse surges negative, left/right turns yaw with opposite sign and equal magnitude, coast stops. All eight completed with passing physical assertions. |
| Numerical acceptance (4/2/1 ms) | `outputs/njord-numerical-final/` (straight), `outputs/njord-numerical-manoeuvres/` (hydrostatic, turn_left, turn_right, coast) and `outputs/njord-numerical-reverse/` (reverse, 25 s); three fresh repetitions per step at 150 N. All six experiments passed the 2 %/absolute 2→1 ms criterion with no incomplete runs. Largest relative use of tolerance: coast distance 0.0044 m of 0.145 m allowed. Drift was not run as a convergence experiment. |
| WAM-V reference race | `outputs/wamv-reference-demo-final/run_metrics.json`; reference course completed on the final image, 3/3 gates in 111.5 s, no collision, minimum clearance 3.58 m. |
| Njord reference race | `outputs/njord-debug-race/run_metrics.json`; reference autonomy with WAM-V-tuned `algorithms.yaml`: 2/3 gates, 58.4 m, no collision, then `simulation_timeout` at 360 s. The earlier `outputs/njord-final-race/` wall timeout with zero progress ran concurrently with the runtime manoeuvre campaign and is not a representative result. |

The ROS regression fixture now drives its intended simulation clock from a
monotonic source. Earlier failing logs record real wall-clock jumps; production
freshness checks were not relaxed. The dynamics stationarity check uses world
vertical/Euler drift and bounded pose excursion, rather than confusing body-frame
heave at fixed pitch with changing waterline. Original failed artifacts remain.

## Limits and remaining acceptance

- Convex OBJ only; arbitrary concave CAD, rotating/scaling raw imported resources,
  and touching/overlapping buoyancy volume bounds are rejected. Preparation must
  produce accepted geometry; mass and coefficients never auto-scale.
- Njord supports constant wind and flat water. Nonzero wind variance and waves
  are rejected. WAM-V retains its existing wave model. Seeded gust support and
  small-wave loads remain extensions, not silently approximated physics.
- The analytical fixture exhibits late open-loop yaw instability in a long
  reverse trial. Reverse acceptance therefore uses a 25 s observation, in which
  the response is stationary and converged; longer reverse runs are not approved.
  Such incomplete reports are retained.
- The reference autonomy is not tuned for the Njord fixture. In the Njord race the
  planner repeatedly reported no feasible path, guidance saturated turn commands,
  and after gate 2 the mission lost its tracked gate while the vessel faced away
  from gate 3. A Njord-specific `ALGORITHMS_CONFIG` and mission tuning remain open;
  WAM-V-tuned gains and the WAM-V-sized mapper self-filter (5×2.8 m) are used as-is.
- Heavy parallel simulator load invalidates race evidence: run races alone.
- Scoring uses a conservative nominal horizontal envelope, with physical contacts
  reported independently. It is not a full six-dimensional swept-hull evaluator.
- Numerical convergence is established for straight, reverse, both turns, coast
  and hydrostatics of this fixture at 150 N only. It does not establish
  calibrated hydrodynamics, other thrust levels or tournament success.
- Importing actual Njord geometry, identifying mass/load/thrusters/hydrodynamics,
  and evaluating independent boat trials remain the later data-dependent step.
  Calibration status cannot be changed to "calibrated" merely by editing YAML.
