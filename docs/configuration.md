# Configuration

Every setting has exactly one home. A run is defined by three YAML files, a few
environment variables that pick between them, and a small set of fixed platform
constants. Configuration is read once at startup; change a file and start a new
run. Nothing needs rebuilding unless noted.

## Where each setting lives

| I want to change… | Edit | Selected with |
| --- | --- | --- |
| WAM-V camera, lidar, GPS or IMU resolution, rate, range or noise; WAM-V thrust limit | `njord_sim/config/vessels/wamv.yaml` | `VESSEL_CONFIG` (default) |
| A few WAM-V sensor settings for one experiment | a partial override, e.g. `config/examples/sensors_low_bandwidth.yaml` | `VESSEL_CONFIG=/config/examples/…` |
| WAM-V sensor mounting poses | `njord_sim/config/vessels/wamv_sensors.xacro` (then `./scripts/njord build`) | always used for the WAM-V |
| Anything about the Njord boat: geometry, mass, damping, wind, thrusters, sensors and their poses | `njord_sim/config/vessels/njord_v1.yaml` | `VESSEL_CONFIG=/config/vessels/njord_v1.yaml` |
| The four-thruster Munin placeholder (same schema; layout assumed until Naval decides) | `njord_sim/config/vessels/munin_v0.yaml` | `VESSEL_CONFIG=/config/vessels/munin_v0.yaml` |
| Gates, obstacles, start pose, default seed, race time limit | `scenarios/<course>.yaml` | course name, e.g. `./scripts/njord demo slalom` |
| Wind, waves, current, water, physics time step | `environments:` in `scenarios/<course>.yaml` | `ENVIRONMENT=calm\|moderate` |
| Speed ceiling of each profile, and which profiles exist | `speed_profiles_mps` in `njord_sim/config/algorithms.yaml` | `PROFILE=<name>` (shipped: `fast`, `conservative`) |
| Guidance gains, lookahead, operating thrust limit, stopping model, planner timing, map safety margin, occupancy grid size and position | `njord_sim/config/algorithms.yaml` | `ALGORITHMS_CONFIG` (default) |
| Other reference-node parameters (perception thresholds, mission geometry) | a ROS parameter file, e.g. `config/examples/team_params.yaml` | `ROS_PARAMS_FILE=/config/examples/…` |
| EKF / GPS-transform settings | `njord_sim/config/localization.yaml` | always used |
| World origin (GPS datum), command timeout, thruster command topic pattern, pinned WAM-V hull and thruster geometry | `njord_sim/njord_sim/constants.py` (then rebuild) | fixed platform constants |
| Seed of a single run | — | `SEED=<n>` |
| Simulated seconds per wall second | — | `REAL_TIME_FACTOR=<x>` (default `1.0`) |
| Navigate on simulator ground truth instead of the GPS/IMU estimate | — | `STATE_SOURCE=truth` (default `estimate`) |

Each vessel file is self-contained: the WAM-V and Njord files each hold their
own sensor settings, so the same camera setting appears once per vessel, not
once per project.

`njord_sim/config/` on the host is mounted read-only at `/config` in every
container, and the defaults point there, so edits apply to the next run without
rebuilding. Code that needs a configuration file (the partial-override base,
node fallback defaults, `localization.yaml`) requires the mounted workspace
defaults at `/workspace-config`; after startup it reads their frozen run copies.
Missing mounted files fail explicitly. Scenarios,
`constants.py` and `wamv_sensors.xacro` are part of the image: rebuild after
changing them.

### Environment variables

`scripts/njord` passes these to all containers (defaults in `compose.yaml`):

| Variable | Default | Meaning |
| --- | --- | --- |
| `SCENARIO` | `/opt/njord/scenarios/reference.yaml` | Course file; a course name on the command line overrides it |
| `SEED` | scenario seed (otherwise `1`) | Positive integer; gate jitter, wind and sensor noise |
| `ENVIRONMENT` | `calm` | Preset name from the scenario's `environments` |
| `PROFILE` | `fast` | Speed profile name from `algorithms.yaml` |
| `REAL_TIME_FACTOR` | `1.0` | Gazebo's target of simulated seconds per wall second; a target, not a guarantee. Recorded as `run.real_time_factor` in `resolved_configuration.json` and as `real_time_factor_target` next to the achieved `real_time_factor` in `run_metrics.json`. See [the note on computation time](#real-time-factor-and-computation-time) |
| `VESSEL_CONFIG` | `/config/vessels/wamv.yaml` | Vessel file or partial WAM-V override |
| `ALGORITHMS_CONFIG` | `/config/algorithms.yaml` | Reference autonomy tuning |
| `ROS_PARAMS_FILE` | empty | Extra ROS parameters for reference nodes |
| `AUTONOMY`, `CONTROLLER`, `PERCEPTION`, `MAPPING` | `reference` | `external` leaves out reference nodes ([team integration](team-integration.md)) |
| `CONFIG_HOST` | `./njord_sim/config` | Host directory mounted at `/config`; with your own directory, point `VESSEL_CONFIG` and `ALGORITHMS_CONFIG` at files in it |
| `NJORD_CPU` | `0` | `1` selects software rendering |

Paths in `VESSEL_CONFIG`, `ALGORITHMS_CONFIG` and `ROS_PARAMS_FILE` are paths
inside the container: use `/config/...` for files under `njord_sim/config/`.

### Real-time factor and computation time

Command and heartbeat freshness is measured in simulation time, so the guard,
the actuator plugins and the evaluator behave the same at any real-time factor.
Computation is not simulated: an algorithm that needs 100 ms of CPU takes
100 ms of wall time, which is 30 ms of simulated time at `REAL_TIME_FACTOR=0.3`
and 300 ms at 3. Below 1, algorithms therefore get more computation per
simulated second than on the boat (optimistic latency); above 1, less
(pessimistic). Only runs that achieve a factor near 1 on hardware comparable to
the boat's represent its computation latency. Measured: the evaluator's
steady-time budget scales as `max(wall_timeout_s, 2 × timeout_s / factor)` so a
slow run is not cut short; a Njord reference run at 0.3 (seed 1, CPU
rendering) reached 1 of 3 gates because the pure-Python planner spent up to
15 s of wall time on infeasible searches, not because of actuation timing.

### Parameter precedence

For the reference nodes started by a run, lowest to highest:

1. defaults in node code, which are themselves read from `wamv.yaml`,
   `algorithms.yaml` and `constants.py` (`njord_sim/defaults.py`);
2. `ROS_PARAMS_FILE`;
3. values resolved from the vessel file and `algorithms.yaml`
   (`public_parameters.json` in the run directory);
4. `use_sim_time: true`, which is always enforced.

Algorithm tuning therefore belongs in `algorithms.yaml`, not in a ROS parameter
file: a value set in both places is taken from `algorithms.yaml`. Nodes started
on their own with `ros2 run` use level 1 plus their command-line parameters.

## Validation

`njord_sim/configuration.py` validates all three files before Gazebo starts.
Unknown or duplicated keys, missing values, non-finite numbers, invalid inertia
or geometry, and operating limits above physical limits all fail with an error
that names the key. Nothing falls back silently: a misspelled setting is an
error, never a no-op.

## Vessel files

### WAM-V (`vessels/wamv.yaml`)

```yaml
schema_version: 2
profile: wamv_reference
settings:
  camera_width: 640
  # ... sensors ...
  max_thrust_n: 500.0
```

The hull, hydrodynamics and thruster placement come from the pinned VRX model,
so only sensor settings and the thrust limit are configurable. The VRX WAM-V
uses fixed water properties and no current: a scenario that changes current,
water density or water level is rejected for this vessel.

**Partial overrides.** A flat YAML without `schema_version` is merged onto
`settings` in `wamv.yaml`; list only the keys to change:

```bash
VESSEL_CONFIG=/config/examples/sensors_low_bandwidth.yaml ./scripts/njord demo slalom
```

New sensor types need a model, a bridge and adapters; adding a key to YAML does
not add a sensor.

### Njord (`vessels/njord_v1.yaml`, `vessels/munin_v0.yaml`)

`njord_v1.yaml` is an **uncalibrated analytical test vessel** with invented
engineering parameters and two aft thrusters; it is not measured Njord
geometry. `munin_v0.yaml` uses the same hull numbers with four thrusters in an
assumed 'X' layout, so four-thruster allocation and dynamic positioning can be
tested before Munin's design is fixed. Neither is Munin's real design. See
[njord-calibration.md](njord-calibration.md) for how it will be calibrated and
[njord-model-evidence.md](njord-model-evidence.md) for what has been verified.

```bash
VESSEL_CONFIG=/config/vessels/njord_v1.yaml ./scripts/njord demo
```

All numbers are SI. World coordinates are ENU; body coordinates are forward,
left, up. `center_of_mass_m` is relative to the body origin; the inertia tensor
is about that centre, with body-parallel axes, and includes all payload once.

**Geometry.** The three roles `visual`, `collision` and `buoyancy` are
independent. A box uses `type: box`, `size_m: [x, y, z]` and
`pose: [x, y, z, roll, pitch, yaw]`. A prepared mesh uses `type: mesh`,
`uri: hull.obj` and `pose`; paths are relative to the vessel file. Import
accepts closed, outward-oriented **convex triangular OBJ solids** already
scaled to metres, without external materials. Non-convex CAD must be split into
disjoint convex buoyancy solids (`buoyancy` may be a list); overlapping or
touching bounding boxes are rejected. Geometry pose rotation must be zero:
bake rotations into the mesh. Geometry changes never rescale mass, inertia or
damping; revise those explicitly and invalidate earlier calibration.

The scoring envelope is a rectangle about the body origin enclosing the
collision geometry. The mapper inflates obstacles by its circumscribed radius
plus `mapping.safety_margin_m`. Both are conservative, not an exact collision
shape.

**Hydrodynamics.** Arrays follow surge, sway, heave, roll, pitch, yaw. Values
are nonnegative magnitudes; the generator emits Gazebo's negative derivative
convention. Linear translation coefficients are kg/s, quadratic kg/m; linear
rotation N·m·s/rad, quadratic N·m·s²/rad². Added mass uses kg for translation
and kg·m² for rotation and is written only to SDF `inertial/fluid_added_mass`.
Relative-water damping and the added-mass Coriolis correction rely on the
verified Gazebo release pair that the Docker build pins.

**Wind.** Ordered unique angles in [0, 360) with signed `cx`, `cy`, `cn`.
Angles describe the direction the relative air moves toward in body axes; interpolation
wraps periodically. Loads use dynamic pressure, the two reference areas and
the yaw reference length, with air density fixed at 1.225 kg/m³.

**Thrusters.** Two or more fixed thrusters. They are defined only here; the
model, command guard, guidance and recorder all derive their thruster data from
this list (`configuration.thruster_table`).

```yaml
thrusters:                       # list order = thruster index everywhere
- name: thruster_1               # command topic /thruster_1/command
  position_m: [1.1, 0.55, -0.1]  # body frame, m
  yaw_deg: -45.0                 # direction of positive thrust
  forward_limit_n: 500.0
  reverse_limit_n: 500.0
  response_time_s: 0.1           # first-order lag of the applied force
```

- `name` matches `[a-z][a-z0-9_]*` and is unique. The guard subscribes to
  `/<name>/command` (`std_msgs/Float64`, newtons).
- `yaw_deg` is the horizontal direction of positive thrust, counter-clockwise
  from forward: 0 pushes forward, 90 to port, -90 to starboard, 180 aft.
  Angled, tunnel and reverse-mounted thrusters are all allowed.
- The layout must be able to set surge and yaw independently. If it can also
  set sway (for example four angled thrusters, or aft thrusters plus tunnel
  thrusters) the vessel is fully actuated; that is required for dynamic
  positioning.
- Guidance allocates a wrench (surge, zero sway, yaw) about the centre of mass
  with the minimum-norm solution and scales all thrusters uniformly at a
  limit. With a layout that cannot set sway, only surge and yaw are solved.
- Invalid or expired commands target zero thrust after `COMMAND_TIMEOUT_S` of
  simulation time (or `PROCESS_LIVENESS_S` of steady time while simulation
  time stalls); the configured response then decays the force in simulation
  time. Body inertia and drift remain.

Njord vessel files are `schema_version: 3`. A schema 1 file, which gave each
thruster an `axis` unit vector instead of `yaw_deg`, is converted when loaded;
names and order are kept.

**Limits of the Njord model.** Flat water and constant wind only: waves or
wind variance in the selected environment are rejected. The reference autonomy
is tuned for the WAM-V; changing the vessel does not constitute controller
tuning or physical calibration. Self filtering follows the actual visual surfaces.

## Scenario files (`scenarios/<course>.yaml`)

A course file owns the world: gates, obstacles, start pose, seed, time limit
and environment presets. `./scripts/njord demo <name>` finds
`scenarios/<name>.yaml` automatically, so a new course is one new file (then
`./scripts/njord build`, because scenarios are copied into the image).

```yaml
schema_version: 1
name: my_course
seed: 1                               # default when SEED is unset
start: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # x, y, z, roll, pitch, yaw
timeout_s: 360.0                      # simulation-time race limit
gate_y_jitter_m: 0.75                 # seeded +/- y offset per gate
gates:                                # passed in order, red to port
- {name: gate_1, red: [25.0, 7.0], green: [25.0, -7.0], radius_m: 0.5}
obstacles:
- {name: obstacle_1, position: [38.0, -7.0], radius_m: 1.0}
environments:
  calm: {...}                         # every key listed below
```

Each environment lists `wind_speed_mps`, `wind_direction_to_deg_enu`,
`wind_variance_gain`, `wave_gain`, `wave_period_s`, `wave_direction_rad`,
`wave_steepness`, `current_speed_mps`, `current_direction_to_deg_enu`,
`water_density_kg_m3`, `water_level_m` and `physics_step_s`. Directions are the
direction the flow moves **toward**: 0° east, 90° north.

The scoring hull is not part of a course: it comes from the vessel. Gate and
obstacle coordinates are world-generation and scoring truth only; the
autonomy is told how many gates there are, never where.

Older scenario files without `schema_version` (JSON syntax with
`wind_direction_deg`) are still accepted and converted with still water,
1000 kg/m³, water level 0 and a 4 ms step.

## Algorithms file (`config/algorithms.yaml`)

Tuning for the reference guidance, planner and mapper, shared by all vessels.
`speed_profiles_mps` maps each `PROFILE` name to a speed ceiling; guidance then
reduces speed further for heading error, clearance and the braking distance in
the observed-free corridor. `guidance.max_thrust` is an operating limit and
must not exceed the vessel's physical limit. The braking model (0.25 m/s²,
1 s reaction) is an assumption until verified by dynamics measurements.

A new speed profile only needs a new `speed_profiles_mps` entry; the launch,
the autonomy runner and `benchmark --profiles` read the names from this file.

`mapping.grid_resolution_m`, `grid_size_m` and `grid_origin_m` fix the reference
mapper's square occupancy grid in the `map` frame (default 0.5 m cells, 160 m,
lower-left corner at (-40, -40) m, so it reaches x, y = 120 m). Nothing outside
it is mapped, so a course must fit inside with room for approach and exit;
`tests/test_algorithms_config.py` checks every race course in `scenarios/`
with a 10 m margin. A larger grid costs mapping and planning time.

## Fixed constants (`njord_sim/njord_sim/constants.py`)

Values that belong to the pinned platform rather than to an experiment: the
world origin shared by Gazebo and the GPS transform (63.4305 N, 10.3951 E), the
0.5 s simulation-time command timeout and 2 s steady-time liveness limit used by
the guard, both actuator plugins and the evaluator, the thruster
command topic pattern `/{name}/command`, and the VRX WAM-V hull envelope
(6 × 3.3 m) and thruster layout: `thruster_1` (port) and `thruster_2`
(starboard) at x = -2.373776 m, y = ±1.027135 m, pushing forward.

## What a run records

Every run directory under `outputs/` holds the source YAML
(`source_config/`), checksummed resource copies (`resources/`), the fully
resolved values (`resolved_configuration.json`), the generated
`vessel.sdf`/`vessel.urdf`/`bridges.yaml`/`njord_course.sdf`, the flat sensor
settings actually used (`vessel_config.yaml`), the autonomy's parameters
(`public_parameters.json`) and `run_manifest.json` with SHA-256 digests of all
of them. `run_ready.json` binds the manifest digest to the run ID; autonomy,
evaluator and recorder record the same digest.

The run directory is not an access-control boundary: a team process with
filesystem access can read evaluation files. Use separate mounts if that
matters.


## Parameter fidelity contracts

WAM-V schema 2 and Njord schema 3 explicitly own IMU rate noise
(`imu_angular_velocity_noise_rad_s`, assumed 0.009 rad/s) and acceleration
noise (`imu_linear_acceleration_noise_m_s2`, assumed 0.021 m/s²).
`convert_sensor_schema` upgrades WAM-V 1 or Njord 2 by adding these assumed
values; Njord 1 first converts its thruster axes to yaw angles. Existing files
are not overwritten. Neither conversion constitutes calibration. GPS,
orientation, gyro and acceleration noise are applied only in the sensor
adapter, with independent seeded streams and variance-derived covariance.
Raw Gazebo IMU/GPS noise and bias are removed; camera and lidar noise remain
in Gazebo. Acquisition timestamps and frames are retained.

Algorithms schema 2 adds `mapping.self_filter_margin_m` (0.02 m) and
`navigation`: `stale_after_s` (0.5 s), `processing_margin_s` (0.1 s),
`clock_stall_after_s` (0.5 wall seconds) and `sync_slop_s` (0.12 s).
`convert_algorithm_schema` adds these explicit defaults to schema 1.
Algorithms schema 3 adds `mapping.grid_resolution_m` (0.5 m), `grid_size_m`
(160 m) and `grid_origin_m` ([-40, -40] m), the values the mapper previously
built in; older files are converted step by step with those values.
Reference startup requires freshness budgets of two sensor periods plus the
processing margin. Sensor periods round up to a physics tick, recorded in
`resolved_configuration.json`; rates above the physics update rate fail.
These reference constraints do not apply to a simulator-only run. Clock
rollback clears freshness history; duplicate stamps never renew a measurement.
Command expiry uses simulation time; steady time is only the liveness limit.

The mapper uses the selected lidar range and message range limits. Its
`self_geometry.json` contains actual generated visual surfaces, with each
component's frame, scale and pose preserved. Meshes are loaded with Gazebo's
mesh loader, retaining spaces between pontoons. A point within
`self_filter_margin_m + 3 * lidar_noise_stddev` of a surface is discarded
before ray clearing. This is a narrow blind band: an external object inside
it cannot be distinguished from a self return. Missing geometry prevents
mapper startup; missing acquisition-time TF discards the observation.
Collision geometry remains authoritative for contacts and navigation margin;
changing visual geometry never changes mass, inertia or damping automatically.

The WAM-V generator verifies both expected force-mode plugins and joints,
and sets their Gazebo command bounds to ±`max_thrust_n` (default 500 N), as
well as setting the watchdog limit. Njord retains individual forward/reverse
limits, yaw and actuator response times. Wind direction specifies the
direction the air **moves toward**, measured counterclockwise from ENU east
(or body forward in the coefficient table).

Managed runs mount workspace defaults at `/workspace-config` and selected
configuration at `/config`, both read-only. A configured mount missing a file
fails; no image YAML fallback is allowed. Before readiness, the simulator
freezes default YAML under `frozen_config/` and selected inputs, localization
and optional ROS overrides under `source_config/`. Nodes read the snapshots.
The manifest hashes these inputs, generated visual geometry and mesh resources.

Full image verification (`check-image`, `test`, CI) uses all build inputs.
Runtime verification uses an additional digest excluding only regular YAML
under `njord_sim/config`; code, Xacro, mesh and built-in scenario changes
require a rebuild. Missing labels or mismatches fail with no validation bypass.
The wrapper and direct benchmark/campaign entrypoints pin an immutable image
ID before checking and starting the run.
