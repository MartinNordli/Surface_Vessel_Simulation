# Versioned physical configuration

The WAM-V reference remains the default. `njord_v1.yaml` is an **uncalibrated
analytical test vessel**, with invented engineering parameters; it is not measured
Njord geometry. Configuration is read at startup and changes require a fresh run.

```bash
./scripts/njord build
VESSEL_CONFIG=/config/njord_v1.yaml ./scripts/njord simulator
# In a separate terminal, run the checks described in njord-calibration.md.
# For the reference autonomy and evaluator:
VESSEL_CONFIG=/config/njord_v1.yaml ./scripts/njord demo
```

Three inputs have separate ownership:

| Input | Responsibility |
| --- | --- |
| `VESSEL_CONFIG` | Geometry, mass, inertia, damping, added mass, wind response, actuator positions/axes/limits/response, sensor poses and settings |
| `SCENARIO` | Course, initial six-value pose, seed, environmental profiles, water and physics timestep |
| `ALGORITHMS_CONFIG` | Guidance gains/speed/operating thrust limit, planner timing, mapping safety margin |

Defaults are the existing `vessel.yaml`, selected scenario, and `algorithms.yaml`.
`wamv_reference.yaml` is the explicit versioned WAM-V profile. The old vessel and
scenario formats pass through explicit legacy adapters. Unknown or duplicated
keys, nonfinite numbers, nonphysical inertia, invalid geometry and operating
limits above physical limits fail before Gazebo starts. Legacy WAM-V cannot
change its upstream thruster separation, water density/level or water current;
unsupported changes fail instead of changing only the controller. Legacy
`PROFILE=conservative` caps the resolved speed at 1 m/s. Versioned vessel profiles
use `algorithms.yaml` for speed. Team ROS parameter files cannot override the
resolved physical and algorithm parameters of reference nodes.

`njord_sim/config/njord_v1.yaml` and `algorithms.yaml` are complete schema examples.
All numbers are SI. World coordinates are ENU; body coordinates are forward,
left, up. `center_of_mass_m` is relative to the body origin; the inertia tensor is
about that center, with body-parallel axes, and includes all payload once.

The versioned scenario requires `schema_version: 1`. Each environment contains
`wind_speed_mps`, `wind_direction_to_deg_enu`, `wind_variance_gain`, `wave_gain`,
`wave_period_s`, `wave_direction_rad`, `wave_steepness`, `current_speed_mps`,
`current_direction_to_deg_enu`, `water_density_kg_m3`, `water_level_m`, and
`physics_step_s`. Direction 0° means flow toward east, 90° toward north. Legacy
scenarios gain explicit still-current, 1000 kg/m³ water, zero water level and
4 ms timestep through conversion. Njord accepts flat water and constant wind;
nonzero waves or wind variance are currently rejected. WAM-V retains VRX waves.

The three geometry roles are independent: visual, collision, buoyancy. A box
uses `type: box`, `size_m: [x,y,z]`, and `pose: [x,y,z,roll,pitch,yaw]`. A prepared
mesh uses `type: mesh`, `uri: hull.obj`, and `pose`. Resource paths are relative
to the vessel YAML. Import accepts closed, outward-oriented **convex triangular
OBJ solids**, already scaled to metres, without external materials. Nonconvex
CAD must be prepared as disjoint convex buoyancy solids; `buoyancy` can be a list.
Overlapping or touching axis-aligned bounds are conservatively rejected, even
when detailed surfaces might not intersect. Geometry pose rotation is currently
rejected; bake rotation into prepared resources. Geometry changes do not scale
mass, inertia or damping automatically. Revise these explicitly and invalidate
previous calibration when geometry or payload changes.

The evaluator rectangle encloses the collision geometry about the body origin.
The mapper uses its circumscribed radius plus `mapping.safety_margin_m`. This is
conservative, not a claim of exact collision shape. Sensor frames and model
poses come from the same vessel definition. Both camera poses are explicit.

Damping arrays follow surge, sway, heave, roll, pitch, yaw. Values are nonnegative
magnitudes; the generator emits Gazebo's negative derivative convention. Linear
translation coefficients have units kg/s; quadratic translation kg/m. Linear
rotation is N·m·s/rad; quadratic rotation N·m·s²/rad². Added-mass diagonal values
use kg for translation, kg·m² for rotation. Added mass is written only to SDF
`inertial/fluid_added_mass`; hydrodynamics' alternative added-mass calculation is
disabled. Relative-water damping and the native-added-mass Coriolis correction
require the verified Gazebo release pair; the Docker build refuses version drift.

Wind tables contain ordered unique angles in [0,360) and signed `cx`, `cy`, `cn`.
Angles describe the relative airflow velocity **toward** the body in its xy
plane. Interpolation wraps periodically. Loads use dynamic pressure, the two
reference areas and yaw reference length; yaw moment is about the center of
mass. Air density is currently fixed at 1.225 kg/m³. The recorded speed interval
is an intended test envelope, not evidence of calibration.

Actuator commands remain newtons. Each forward-facing planar unit axis and
position determines the actual force and moment. Guidance solves the same
surge/yaw allocation about the center of mass, including asymmetric arms and
separate forward/reverse limits. No propeller conversion is applied again.
The simulator watchdog uses 0.5 s steady wall time. Invalid or expired commands
set target thrust to zero; configured exponential response decays the actual
force in simulation time. Body inertia and drift remain. A simulation rewind
invalidates commands.

Every run saves source YAML, resolved values, copied/checksummed resources,
SDF/URDF/bridge files and `run_manifest.json`. `run_ready.json` binds the manifest
digest to the run ID. Reference autonomy receives `public_parameters.json` and
sensor settings, with no obstacle/gate truth. Autonomy, evaluator and recorder
record the manifest digest. The output directory is not an access-control
boundary; external teams with filesystem access can still inspect evaluation
files. Use separate mounts if preventing access is required.

See [calibration protocol](njord-calibration.md) for isolated experiments and
numerical acceptance. CPU or SDF tests do not establish runtime sensor delivery,
numerical convergence, or realism against the physical boat.
