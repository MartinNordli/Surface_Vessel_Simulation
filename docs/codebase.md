# Codebase

What every directory and file in the repository does. For how the pieces
work together at runtime, see [ros2-interface.md](ros2-interface.md); for
where settings live, see [configuration.md](configuration.md).

## Top level

```
.
├── README.md                 Overview, installation and links
├── AGENTS.md                 Engineering rules for contributors and coding agents
├── Dockerfile                ROS 2 Jazzy + Gazebo Harmonic + VRX image
├── compose.yaml              The three run services: simulator, autonomy, evaluator (+ tests, rviz)
├── compose.*.yaml            Overlays: gui, wsl, cpu, record, film, team
├── ruff.toml                 Python lint settings
├── docker/                   Image dependencies, lock file, entrypoint, VRX licence
├── docs/                     Documentation (this folder)
├── njord_sim/                ROS 2 Python package: nodes, configuration, launch files
├── njord_gz_plugins/         C++ Gazebo plugins
├── scenarios/                Course files
├── scripts/                  The njord CLI and run, report and maintenance tools
├── tests/                    Unit and integration tests
├── validation/               Live checks against a running simulator, acceptance tools
├── sandbox/                  Headless planner demo without ROS or Gazebo
├── .github/workflows/        CI: unit suite, image build, container tests, image publishing
├── .codex/agents/            Role definitions for coding sub-agents
└── outputs/                  Run results (git-ignored)
```

The one rule for Python code in `njord_sim/njord_sim/`: `*_core.py` and the
other plain modules are pure Python and testable without ROS; `*_node.py`
only connects them to topics, parameters and timers.

## `njord_sim/`: the ROS 2 package

### `njord_sim/njord_sim/`

**Configuration and run setup**

| File | What it does |
|---|---|
| `configuration.py` | Loads, validates and resolves the vessel, scenario and algorithm YAML files into one configuration; rejects unknown keys and invalid values before anything starts |
| `constants.py` | Fixed platform constants: GPS datum, command timeouts, topic and frame names, WAM-V geometry |
| `defaults.py` | Fallback parameters for reference nodes started on their own, read from the configuration files |
| `run_manifest.py` | Atomic, checksummed handoff (`run_ready.json`) from the simulator to autonomy and evaluator |

**Model and world generation**

| File | What it does |
|---|---|
| `vessel.py` | Generates the WAM-V model from the VRX xacro with the configured sensors, and the ROS–Gazebo bridges |
| `njord_model.py` | Generates the Njord-profile model (`njord_v1`, `munin_v0`): mass, inertia, geometry and plugin parameters |
| `mesh_geometry.py` | Imports and validates convex triangular OBJ meshes |
| `visual_geometry.py` | Exports the vessel's visual surfaces, used by the mapper's self filter |
| `self_geometry.py` | Rejects lidar returns that hit the vessel itself |
| `scenario.py` | Generates the world SDF: buoys, obstacles, water, wind, waves, gravity and time step |
| `physics_core.py` | Python mirror of the Njord plugin's load equations, used only by tests |

**Estimation and sensors**

| File | What it does |
|---|---|
| `sensor_adapter_node.py` | Adds seeded metric GPS and IMU noise and publishes the navigation heartbeat |
| `sensor_noise_core.py` | Seeded, independent sensor noise streams |
| `truth_relay_node.py` | `STATE_SOURCE=truth`: republishes ground truth as `/njord/odometry` and TF |
| `tf_wait.py` | Holds sensor messages until TF at their stamp exists, instead of dropping them |
| `geometry.py` | Rigid transforms and timestamp helpers |

**Reference autonomy (replaceable)**

| File | What it does |
|---|---|
| `perception_core.py`, `perception_node.py` | Finds red and green buoys in both cameras, adds lidar depth, tracks them in `map` |
| `mapping_core.py`, `mapper_node.py` | Ray-traced occupancy grid with unknown, free and occupied cells, aging and inflation |
| `mission_core.py`, `mission_node.py` | Gate mission: picks the next gate, sets goals, searches and retries |
| `dstar_lite.py` | Incremental D* Lite search |
| `planner_core.py`, `planner_node.py` | Grid planning with D* Lite on the occupancy grid; publishes `/njord/path` |
| `control_core.py`, `guidance_node.py` | Line-of-sight guidance, speed limits, collision checks and thrust allocation |
| `setpoint_control_core.py`, `setpoint_controller_node.py` | Reference controller for setpoint courses: drive to the target, then hold position and heading |

**Safety and scoring**

| File | What it does |
|---|---|
| `guard_core.py`, `command_guard_node.py` | The command guard: the only path to the thrusters, zero thrust on stale input |
| `scenario_core.py` | Gate crossing, clearance and race scoring from ground truth |
| `setpoint_core.py` | Issues setpoint targets in order and scores reach, hold, overshoot and effort |
| `evaluator_node.py` | The referee: starts the race, scores it and writes `run_metrics.json` and `timeseries.csv` |
| `report_core.py` | Builds the self-contained `report.html` from the metrics and time series |

### `njord_sim/launch/`

| File | What it does |
|---|---|
| `simulation.launch.py` | Simulator service: resolves the configuration, generates model and world, starts Gazebo, the bridges and `robot_state_publisher` |
| `dstar_demo.launch.py` | Autonomy service: estimation, the command guard and the selected reference nodes |

### `njord_sim/config/`

| File | What it does |
|---|---|
| `vessels/wamv.yaml` | WAM-V reference vessel: sensors and thrust limit (default) |
| `vessels/wamv_sensors.xacro` | WAM-V sensor mounting poses |
| `vessels/njord_v1.yaml` | Uncalibrated analytic Njord test hull with two aft thrusters |
| `vessels/munin_v0.yaml` | Uncalibrated Munin placeholder with four thrusters in an assumed X layout |
| `algorithms.yaml` | Tuning of the reference autonomy and setpoint controller; speed profiles |
| `localization.yaml` | EKF and GPS-transform settings |
| `njord.rviz` | RViz layout used by `gui` |
| `examples/sensors_low_bandwidth.yaml` | Example partial sensor override |
| `examples/team_params.yaml` | Example ROS parameter file for reference nodes |

This folder is mounted into the containers at `/config`, so edits apply to
the next run without rebuilding.

## `njord_gz_plugins/`: Gazebo plugins (C++)

| File | What it does |
|---|---|
| `src/NjordPhysics.cc` | Njord-profile physics: buoyancy, thruster forces with response lag and command expiry, wind loads |
| `src/ActuatorWatchdog.cc` | WAM-V thrust with the same command expiry as the guard |
| `src/ContactMonitor.cc` | Contact heartbeat, so silence never means "no contact" |
| `src/MeshExport.cc` | Small command-line tool (not a plugin) that exports mesh geometry exactly as Gazebo loads it, for the self filter |
| `include/njord/Hydrostatics.hh` | Mesh validation, clipped submerged volume, actuator response (header-only, testable without Gazebo) |
| `include/njord/Loads.hh` | Wind-table interpolation, wind load and thruster wrench (header-only) |
| `test/hydrostatics_test.cc`, `test/loads_test.cc` | C++ tests of the two headers |
| `test/load_vectors.csv` | Reference vectors from `physics_core.py` that the C++ loads must match |

How the physics fits together is described in
[physics-model.md](physics-model.md).

## `scenarios/`: courses

| File | Course |
|---|---|
| `reference.yaml` | 3 gates, 2 obstacles (default) |
| `slalom.yaml` | 5 alternating gates, 5 obstacles |
| `goto_square.yaml` | 4 setpoints on a 20 m square |
| `goto_retarget.yaml` | Setpoints replaced before they are reached |
| `station_keeping.yaml` | Drive 10 m and hold 60 s |
| `hold_position.yaml` | Hold the start pose 60 s |
| `dynamics.yaml` | Open water for dynamics measurements |

Each course is described in [running.md](running.md#3-courses).

## `scripts/`

| File | What it does |
|---|---|
| `njord` | The single entry point: `pull`, `build`, `demo`, `gui`, `lab`, `goto`, `test`, `benchmark`, `film`, … (`./scripts/njord help`) |
| `setup-host.sh` | Installs Docker and the NVIDIA Container Toolkit on Ubuntu 24.04 (never a GPU driver) |
| `ca-nodes.sh` | Builds and runs Control & Autonomy's nodes from the `Njord2627` repository against the simulator |
| `run_autonomy.py` | Autonomy container entry: waits for the handoff, then launches `dstar_demo.launch.py` |
| `run_evaluator.py` | Evaluator container entry: waits for the handoff, then starts the evaluator |
| `run_recorder.py` | Optional MCAP recording of a run (`compose.record.yaml`) |
| `run_filmer.py` | Chase camera for the README animation (`compose.film.yaml`) |
| `make_film_gif.py` | Turns the filmer's frames into an animated GIF |
| `make_report.py` | Rebuilds `report.html` of a finished run (`./scripts/njord report`) |
| `send_setpoint.py` | Sends one target pose (`./scripts/njord goto`) |
| `ned_dp_standin.py` | Stand-in NED/FRD controller that shows the conversion Control & Autonomy does |
| `benchmark.py` | Seed × environment × profile race matrix in isolated Docker projects |
| `dynamics_campaign.py` | Fresh-process dynamics trials at 4, 2 and 1 ms time steps |
| `build_metadata.py` | Fingerprints the Docker inputs so an image can be matched to the checkout |
| `lock-dependencies.py` | Maintenance: resolves and pins upstream dependency versions |
| `make_load_vectors.py` | Maintenance: regenerates `njord_gz_plugins/test/load_vectors.csv` |
| `summarize_evidence.py` | Writes compact evidence summaries to `docs/evidence/` |

## `validation/`: live checks and acceptance tools

| File | What it does |
|---|---|
| `check_runtime.py` | Live check of cameras, lidar, navigation and TF (`selftest`, `smoke`) |
| `runtime_checks.py`, `runtime_smoke.py` | Logic and ROS transport smoke tests behind the live check |
| `sensor_parameter_probe.py` | Measures raw against adapted sensor noise and rates |
| `check_lidar.py` | Compares lidar ranges with a known cylinder |
| `check_dynamics.py`, `dynamics_metrics.py` | One open-loop manoeuvre per fresh simulator, and its metrics |
| `check_actuator_timeout.py`, `check_njord_actuator.py`, `check_wamv_limits.py` | Probes of command expiry, applied force and thrust limits |
| `numerical_acceptance.py` | Compares 4, 2 and 1 ms runs for numerical convergence |
| `physical_acceptance.py`, `hull_acceptance.py` | Independent analytic checks of forces, accelerations and flotation |
| `calibration_report.py`, `calibration/dataset_manifest.template.json` | Scaffold for comparing the model with real-boat measurements |
| `mapper_timing.py` | Times one mapper update |

## `tests/`

Run with `python3 -m unittest discover -s tests -p 'test_*.py'` on the host,
or `./scripts/njord test` for the full suite in the container. Tests that
need ROS or Gazebo skip on the host.

| Area | Files |
|---|---|
| Configuration and generation | `test_configuration.py`, `test_vessel_config.py`, `test_vessel.py`, `test_algorithms_config.py`, `test_parameter_contracts.py`, `test_parameter_sensors.py`, `test_physical_integration.py` |
| Physics | `test_njord_physics.py`, `test_hull_acceptance.py`, `test_dynamics_metrics.py` |
| Planning and control | `test_dstar_lite.py`, `test_autonomy.py`, `test_guard_core.py`, `test_setpoint_control.py`, `test_ned_standin.py` |
| Perception and mapping | `test_perception.py`, `test_mapping_raytrace.py`, `test_mapping_ordering.py`, `test_self_geometry.py`, `test_tf_wait.py` |
| Mission and courses | `test_mission_heading.py`, `test_mission_recovery.py`, `test_scenarios.py`, `test_slalom_course.py`, `test_setpoint_core.py` |
| Runs, scoring and reports | `test_run_manifest.py`, `test_evaluator_runtime.py`, `evaluator_ros_smoke.py`, `test_report_core.py`, `test_recorder.py`, `test_film.py`, `test_truth_relay.py` |
| ROS transport | `test_ros_runtime.py`, `test_sensor_runtime.py`, `test_team_launch.py`, `test_team_runner.py` |
| Build and evidence | `test_build_metadata.py`, `test_dependency_lock.py`, `test_domain_leases.py`, `test_evidence_summaries.py`, `test_validation_evidence.py` |

## `docker/`

| File | What it does |
|---|---|
| `entrypoint.sh` | Container entrypoint: sources ROS and the workspace, then runs the service command |
| `dependencies.lock.json` | Pinned base image digest, VRX commit and Gazebo vendor commits |
| `gz.repos` | Gazebo vendor source repositories built into the image |
| `fake-vendor.control` | Dummy package that satisfies apt dependencies on the ROS Gazebo vendor packages, which the image builds from source instead |
| `LICENSE.vrx` | Licence of the VRX Docker setup this image derives from |

## `sandbox/`

| File | What it does |
|---|---|
| `headless_demo.py` | Runs the planner and guidance closed loop against a simple 3-DOF boat and 2D lidar, with no ROS or Gazebo |
| `make_gif.py` | Animates that run |
| `headless_metrics.json` | Committed metrics of the illustrated run |

## `docs/`

| File | What it covers |
|---|---|
| [running.md](running.md) | Every run option: courses, vessels, GUI, settings, results |
| [configuration.md](configuration.md) | Every setting, file formats and parameter precedence |
| [ros2-interface.md](ros2-interface.md) | Topics, message types, frames, QoS and timing rules |
| [control-autonomy.md](control-autonomy.md) | Step-by-step guide for running Control & Autonomy's nodes |
| [codebase.md](codebase.md) | This file |
| [physics-model.md](physics-model.md) | How the vessel physics is computed and how it will be calibrated |
| [validation.md](validation.md) | Tests, CI and measured evidence |
| [agent-workflows.md](agent-workflows.md) | Build and verification steps per change area, for contributors |
| `evidence/` | Compact summaries of the runs the documentation cites (written by `scripts/summarize_evidence.py`) |
| `media/` | Images and animations used by the documentation |
| `archive/` | Earlier decision documents kept for reference |
