# ROS interfaces and ownership

All distances/forces are SI. World is local ENU, `map` origin at the configured
Trondheim datum (63.4305 N, 10.3951 E). ROS body axes: x forward, y left, z up.
Camera optical axes: x right, y down, z forward. Every autonomous node uses
simulation time. Source stamps are preserved through sensing/mapping.

| Topic | Type | Owner / meaning |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS, one bridge |
| `/wamv/sensors/cameras/front_{left,right}_camera_sensor/image_raw` | `sensor_msgs/Image` | Undistorted RGB, optical frame |
| same camera prefix + `/camera_info` | `sensor_msgs/CameraInfo` | Intrinsic calibration |
| `/wamv/sensors/lidars/lidar_wamv_sensor/points` | `sensor_msgs/PointCloud2` | 3D lidar, `wamv/lidar_wamv_link` |
| same lidar prefix + `/scan` | `sensor_msgs/LaserScan` | Planar no-return free-space supplement |
| `/wamv/sensors/gps/gps/fix` | `sensor_msgs/NavSatFix` | Metric Gaussian GPS measurement noise |
| `/wamv/sensors/imu/imu/data` | `sensor_msgs/Imu` | Noisy ENU attitude/angular velocity/acceleration |
| `/njord/odometry` | `nav_msgs/Odometry` | Navigation state, `map` → `wamv/base_link`: GPS/IMU EKF by default; ground truth with `STATE_SOURCE=truth` |
| `/njord/occupancy` | `nav_msgs/OccupancyGrid` | -1 unknown, 0 observed free, 100 inflated obstacle |
| `/njord/buoys` | `vision_msgs/Detection3DArray` | Fused red/green buoy tracks in map |
| `/njord/goal` | `geometry_msgs/PoseStamped` | Mission waypoint in map |
| `/njord/path` | `nav_msgs/Path` | D* Lite route; empty explicitly invalidates |
| `/njord/{planner,mission,navigation}_status` | `diagnostic_msgs/DiagnosticArray` | Fresh validity heartbeat |
| `/<thruster name>/command`, e.g. `/thruster_1/command` | `std_msgs/Float64` | Controller force per thruster in newtons along its axis, before guard; one topic per thruster in the vessel file (WAM-V: `thruster_1` port, `thruster_2` starboard) |
| `/njord/actuator_forces` | `ros_gz_interfaces/Float32Array` | Internal force envelope: one force in N per thruster, in vessel-file order |
| `/njord/race_active` | `std_msgs/Bool` | Evaluator heartbeat, transient local, 10 Hz |
| `/wamv/ground_truth/odometry` | `nav_msgs/Odometry` | Evaluation; fed to navigation only in explicit truth mode (`STATE_SOURCE=truth`) |
| `/njord/contacts` | `ros_gz_interfaces/Contacts` | Physics-verified contact heartbeat, 20 Hz |
| `/njord/plan_ms` | `std_msgs/Float64` | Monotonic wall-time search latency |

The force envelope is **not a velocity command**. Only the Gazebo watchdog
forwards it to upstream physical thruster force topics. No ROS bridge exposes
unguarded WAM-V thruster commands. Launch only one force-envelope authority:
the autonomy guard for races, or the standalone dynamics measurement script.

Sensor subscriptions use best-effort sensor QoS. Mission/status/control topics
are reliable. TF comes from robot_state_publisher and the navigation state
source; no Gazebo world-pose TF publisher is connected.

- `STATE_SOURCE=estimate` (default): the local EKF supplies attitude in `odom`
  (`odom` → `wamv/base_link`); the global EKF supplies position/velocity from
  GPS plus IMU attitude/rates in `map` (`map` → `odom`). Integrating
  uncorrected accelerometer bias into a velocity pseudo-sensor is deliberately
  avoided. This is a baseline estimator, not a calibrated INS.
- `STATE_SOURCE=truth`: `truth_relay` replaces both EKFs. It republishes
  `/wamv/ground_truth/odometry` unchanged (same stamp, frames and body-frame
  twist) on `/njord/odometry`, broadcasts `odom` → `wamv/base_link` from it and
  a static identity `map` → `odom`. Invalid truth is dropped, so navigation goes
  stale and the guard zeroes thrust. Noisy GPS/IMU and `/njord/gps/odometry`
  are still published for a team's own estimator. The mode is recorded as
  `state_source` in `autonomy_config.json` and `run_metrics.json`; a truth run
  says nothing about estimation.

## Replace the reference algorithms

Launch `dstar_demo.launch.py autonomy:=external` to keep estimation and the
command guard while leaving mapping, perception, mission, planner and guidance
to the team's nodes. With `autonomy:=reference` (default), select
`controller:=external`, `perception:=external`, and/or `mapping:=external` to
omit only guidance, camera/lidar buoy fusion, and/or the mapper. Mission and
planner remain active in these partial modes. The Compose equivalents are
`AUTONOMY`, `CONTROLLER`, `PERCEPTION` and `MAPPING`, each `reference|external`.
There must be one publisher authority per replaced output. The sensor adapter
still owns `navigation_status`; do not duplicate it in an external stack.

Publish the documented forces and status messages. The
planner diagnostic name is `njord/planner`; mission is `mission`; navigation
is `navigation`. Level OK means current valid inputs. Guard rejects non-OK,
missing/expired heartbeats and expired or nonfinite commands. It checks diagnostic
source stamps against simulation time (up to 1 s old, at most 0.1 s ahead), and
requires receipt of every heartbeat and both commands within 0.5 s steady time.
The evaluator requires all three health statuses within 0.5 s simulation and
steady time before starting. Publish these at 10 Hz; forces normally at 20 Hz.
A fresh evaluator race heartbeat is also required. The guard does not subscribe
to paths, maps or odometry: the controller must zero commands on invalid, empty
or stale input, and the planner must report invalid paths. Never publish
plausible-looking health status when the corresponding input has stopped.

External perception output must be reliable `Detection3DArray` in `map`, stamped
with the processed image acquisition time. Each detection needs a stable unique
`id`, `bbox.center.position` in metres, and a result with `class_id` of `red` or
`green` and score >= 0.35. Publish an empty array for a newly processed image
without detections; the reference mission uses the array stamp as image-processing
freshness (0.5 s maximum). Transform measurements with TF at acquisition time.
2D detections require a range/depth fusion adapter before this 3D interface.

`params_file:=/path/to/parameters.yaml` (Compose: `ROS_PARAMS_FILE`) applies ROS
parameters by node name to the launched nodes. Values resolved from the vessel
file and `algorithms.yaml` take precedence over it, and `use_sim_time=true` is
enforced last; see [configuration.md](configuration.md#parameter-precedence).
The runner snapshots any ROS parameter file and saves settings and digests in
`autonomy_config.json`. See [team-integration.md](team-integration.md) for exact
commands, recording and replay.

Use topic remapping/ROS parameters on the individual nodes for different sensor
names. Sensor geometry and models are in `njord_sim/config`; scenario truth goes
only to world generation and scoring. A mission knows the number of gates, not
their locations. Red-left/green-right is this demo's rule, not an assertion about
the official competition rules.
## Njord physics model

The Njord vessel keeps the `wamv` model, topic and frame namespace so all of the
interfaces above are unchanged. `njord::Physics` consumes the same atomic
`/njord/actuator_forces` envelope as the WAM-V watchdog; exactly one of these
plugins is loaded. Forces are newtons, applied at the configured thruster
positions. Invalid or expired commands target zero thrust; the configured
actuator response decays the force in simulation time, while expiry uses steady
wall time.

Gazebo-only `/njord/actuator_applied` (`gz.msgs.Float_V`) reports, for N
thrusters, N applied forces, then N target forces (newtons, vessel-file order),
then command validity (1/0), at up to 50 Hz. A command with the wrong number of
values or any non-finite value is invalid as a whole. Its header is simulation time at the end of the
response integration step. It is evaluation telemetry and has no autonomy bridge.

## Run handoff files

Before Gazebo starts, the simulator writes into the run directory:
`public_parameters.json` (guidance, planner, mapper, guard and sensor-adapter
parameters; no course coordinates), `vessel_config.yaml` (the flat sensor and
thrust settings in use), `run_manifest.json` (SHA-256 of every input and
generated file) and, last and atomically, `run_ready.json` with the run ID, gate
count and manifest digest. Autonomy, evaluator and recorder verify the public
artifacts against the manifest and record its digest. The full configuration
and scenario files are evaluation data, not autonomy inputs.
