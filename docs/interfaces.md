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
| `/njord/odometry` | `nav_msgs/Odometry` | GPS/IMU EKF, `map` → `wamv/base_link` |
| `/njord/occupancy` | `nav_msgs/OccupancyGrid` | -1 unknown, 0 observed free, 100 inflated obstacle |
| `/njord/buoys` | `vision_msgs/Detection3DArray` | Fused red/green buoy tracks in map |
| `/njord/goal` | `geometry_msgs/PoseStamped` | Mission waypoint in map |
| `/njord/path` | `nav_msgs/Path` | D* Lite route; empty explicitly invalidates |
| `/njord/{planner,mission,navigation}_status` | `diagnostic_msgs/DiagnosticArray` | Fresh validity heartbeat |
| `/njord/thrusters/{left,right}/thrust` | `std_msgs/Float64` | Controller forces in newtons, before guard |
| `/njord/actuator_forces` | `geometry_msgs/Twist` | Internal force envelope: linear.x left N, linear.y right N; all other fields zero |
| `/njord/race_active` | `std_msgs/Bool` | Evaluator heartbeat, transient local, 10 Hz |
| `/wamv/ground_truth/odometry` | `nav_msgs/Odometry` | Evaluation only; never fed to navigation |
| `/njord/contacts` | `ros_gz_interfaces/Contacts` | Physics-verified contact heartbeat, 20 Hz |
| `/njord/plan_ms` | `std_msgs/Float64` | Monotonic wall-time search latency |

The force envelope is **not a velocity command**. Only the Gazebo watchdog
forwards it to upstream physical thruster force topics. No ROS bridge exposes
unguarded WAM-V thruster commands. Launch only one force-envelope authority:
the autonomy guard for races, or the standalone dynamics measurement script.

Sensor subscriptions use best-effort sensor QoS. Mission/status/control topics
are reliable. TF comes from robot_state_publisher and the two EKFs; no Gazebo
world-pose TF publisher is connected. The local EKF supplies attitude in `odom`;
the global EKF supplies position/velocity from GPS plus IMU attitude/rates in
`map`. Integrating uncorrected accelerometer bias into a velocity pseudo-sensor
is deliberately avoided. This is a baseline estimator, not a calibrated INS.

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

`params_file:=/path/to/parameters.yaml` applies ROS parameters by node name to
the launched nodes, overriding defaults; `use_sim_time=true` is enforced last.
Compose exposes this as `ROS_PARAMS_FILE`. `VESSEL_CONFIG` selects a partial
sensor YAML override; `CONFIG_HOST` mounts a host directory at `/config` read-only.
The runner passes the simulator's resolved `vessel_config.yaml` to autonomy,
snapshots any ROS parameter file and saves settings/digests in
`autonomy_config.json`. Direct separate launches must use the same vessel file.
See the README team walkthrough for exact commands, recording and replay.

Use topic remapping/ROS parameters on the individual nodes for different sensor
names. Sensor geometry and models are in `njord_sim/config`; scenario truth goes
only to world generation and scoring. A mission knows the number of gates, not
their locations. Red-left/green-right is this demo's rule, not an assertion about
the official competition rules.
# Versioned physical configuration

The versioned Njord model retains the `wamv` model/topic/frame namespace for
compatibility with these interfaces. `njord::Physics` consumes the same atomic
`/njord/actuator_forces` envelope as the WAM-V watchdog; exactly one of these
plugins is loaded. Forces are newtons, applied at configured physical locations.
The Njord watchdog targets zero on invalid/expired commands; configured actuator
response decays force in simulation time while expiration uses steady wall time.
Gazebo-only `/njord/actuator_applied` (`gz.msgs.Twist`) reports applied newtons in
`linear.x/y`, target newtons in `angular.x/y`, and command validity (1/0) in
`angular.z`, at up to 50 Hz. Its header is simulation time at the end of the
response integration step. It is evaluation telemetry and has no autonomy bridge.

Launch resolves all three YAML files before Gazebo starts. It publishes
`public_parameters.json` (guidance, planner, mapper and guard settings, no course
coordinates), `vessel_config.yaml` (sensor compatibility settings), and an atomic
`run_ready.json` with run ID, gate count and manifest SHA-256. Consumers verify
the public artifacts against `run_manifest.json`. Reference node parameters from
this projection take precedence over team ROS overrides. Evaluator, autonomy and
recorder results carry the same manifest digest. Full configuration and scenario
artifacts are evaluation data; they are not autonomy inputs.
