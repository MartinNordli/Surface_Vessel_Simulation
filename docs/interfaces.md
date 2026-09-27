# ROS interfaces and ownership

All distances/forces are SI. World is local ENU, `map` origin at the configured
Trondheim datum (63.4305 N, 10.3951 E). ROS body axes: x forward, y left, z up.
Camera optical axes: x right, y down, z forward. Every autonomous node uses
simulation time. Source stamps are preserved through sensing/mapping.

| Topic | Type | Owner / meaning |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS, one bridge |
| `/sensors/cameras/{front_left,front_right}/image_raw` | `sensor_msgs/Image` | Undistorted RGB, in `{front_left,front_right}_camera_link_optical` |
| `/sensors/cameras/{front_left,front_right}/camera_info` | `sensor_msgs/CameraInfo` | Intrinsic calibration |
| `/sensors/lidar/points` | `sensor_msgs/PointCloud2` | 3D lidar, `lidar_link` |
| `/sensors/lidar/scan` | `sensor_msgs/LaserScan` | Planar no-return free-space supplement, `lidar_link` |
| `/sensors/gps/fix` | `sensor_msgs/NavSatFix` | Metric Gaussian GPS measurement noise, `gps_link` |
| `/sensors/imu/data` | `sensor_msgs/Imu` | Noisy ENU attitude/angular velocity/acceleration, `imu_link` |
| `/njord/odometry` | `nav_msgs/Odometry` | Navigation state, `map` → `base_link`: GPS/IMU EKF by default; ground truth with `STATE_SOURCE=truth` |
| `/njord/occupancy` | `nav_msgs/OccupancyGrid` | -1 unknown, 0 observed free, 100 inflated obstacle |
| `/njord/buoys` | `vision_msgs/Detection3DArray` | Fused red/green buoy tracks in map |
| `/njord/goal` | `geometry_msgs/PoseStamped` | Mission waypoint in map |
| `/njord/path` | `nav_msgs/Path` | D* Lite route; empty explicitly invalidates |
| `/njord/{planner,mission,navigation}_status` | `diagnostic_msgs/DiagnosticArray` | Fresh validity heartbeat |
| `/<thruster name>/command`, e.g. `/thruster_1/command` | `std_msgs/Float64` | Controller force per thruster in newtons along its axis, before guard; one topic per thruster in the vessel file (WAM-V: `thruster_1` port, `thruster_2` starboard) |
| `/njord/actuator_forces` | `ros_gz_interfaces/Float32Array` | Internal force envelope: one force in N per thruster, in vessel-file order |
| `/njord/race_active` | `std_msgs/Bool` | Evaluator heartbeat, transient local, 10 Hz steady time; required by the guard only with `RUN_MODE=race` |
| `/njord/guard_status` | `diagnostic_msgs/DiagnosticArray` | Command guard, 20 Hz steady time: status `command_guard`, OK while thrust may pass, otherwise WARN with the reason |
| `/sim/ground_truth/odometry` | `nav_msgs/Odometry` | Evaluation; fed to navigation only in explicit truth mode (`STATE_SOURCE=truth`) |
| `/sim/sensors/{gps/fix_raw,imu/data_raw}` | `sensor_msgs/NavSatFix`, `sensor_msgs/Imu` | Gazebo GPS/IMU before the sensor adapter adds noise; simulator internal |
| `/njord/contacts` | `ros_gz_interfaces/Contacts` | Physics-verified contact heartbeat, 20 Hz |
| `/njord/plan_ms` | `std_msgs/Float64` | Monotonic wall-time search latency |

The force envelope is **not a velocity command**. Only the Gazebo watchdog
forwards it to upstream physical thruster force topics. No ROS bridge exposes
unguarded WAM-V thruster commands. Launch only one force-envelope authority:
the autonomy guard for races, or the standalone dynamics measurement script.

### Names are the same for every vessel

Topic and frame names come from `njord_sim/njord_sim/constants.py` and do not
depend on the vessel file: WAM-V, `njord_v1` and `munin_v0` publish the same
topics in the same frames. Frames follow REP-105 without a vessel prefix
(`map` → `odom` → `base_link` → `lidar_link`, `imu_link`, `gps_link`,
`front_{left,right}_camera_link` → `…_optical`); the Gazebo model is called
`vessel` and its link names equal these frames. Everything under `/sim` exists
only in simulation and must not feed autonomy. Names before this change:

| Old | New |
|---|---|
| `/wamv/sensors/cameras/front_left_camera_sensor/{image_raw,camera_info}` | `/sensors/cameras/front_left/{image_raw,camera_info}` (same for `front_right`) |
| `/wamv/sensors/lidars/lidar_wamv_sensor/{points,scan}` | `/sensors/lidar/{points,scan}` |
| `/wamv/sensors/gps/gps/fix`, `/wamv/sensors/imu/imu/data` | `/sensors/gps/fix`, `/sensors/imu/data` |
| `/wamv/sensors/gps/gps/fix_raw`, `/wamv/sensors/imu/imu/data_raw` | `/sim/sensors/gps/fix_raw`, `/sim/sensors/imu/data_raw` |
| `/wamv/ground_truth/odometry` | `/sim/ground_truth/odometry` |
| `/wamv/joint_states` | `/joint_states` |
| `wamv/base_link`, `wamv/lidar_wamv_link`, `wamv/imu_wamv_link`, `wamv/gps_wamv_link` | `base_link`, `lidar_link`, `imu_link`, `gps_link` |
| `wamv/front_left_camera_link_optical` | `front_left_camera_link_optical` (same for `front_right`) |
| run files `wamv.sdf`, `wamv.urdf` | `vessel.sdf`, `vessel.urdf` |

Only Gazebo-internal topics of the pinned VRX WAM-V keep its `wamv`
namespace (e.g. `/wamv/thrusters/left/thrust`); they are not bridged to ROS.

Sensor subscriptions use best-effort sensor QoS. Mission/status/control topics
are reliable. TF comes from robot_state_publisher and the navigation state
source; no Gazebo world-pose TF publisher is connected.

- `STATE_SOURCE=estimate` (default): the local EKF supplies attitude in `odom`
  (`odom` → `base_link`); the global EKF supplies position/velocity from
  GPS plus IMU attitude/rates in `map` (`map` → `odom`). Integrating
  uncorrected accelerometer bias into a velocity pseudo-sensor is deliberately
  avoided. This is a baseline estimator, not a calibrated INS.
- `STATE_SOURCE=truth`: `truth_relay` replaces both EKFs. It republishes
  `/sim/ground_truth/odometry` unchanged (same stamp, frames and body-frame
  twist) on `/njord/odometry`, broadcasts `odom` → `base_link` from it and
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

Publish the documented forces. Which health statuses the guard requires comes
from `configuration.guard_requirements`: navigation always (the sensor adapter
or truth relay publishes it), planner and mission only while the reference
nodes run (`AUTONOMY=reference`), and the evaluator's race-active signal only
with `RUN_MODE=race` (`demo`, `benchmark`; `lab` uses `RUN_MODE=free`). An
external stack therefore needs no Njord status messages. The
planner diagnostic name is `njord/planner`; mission is `mission`; navigation
is `navigation`. Level OK means current valid inputs. Guard rejects non-OK,
missing/expired heartbeats and expired or nonfinite commands. It checks diagnostic
source stamps against simulation time (up to 1 s old, at most 0.1 s ahead), and
requires receipt of every heartbeat and every thruster command within 0.5 s of
simulation time and 2 s of steady time. The evaluator requires all three health
statuses at most 0.5 s old in simulation time, received within 2 s of steady
time, before starting (only navigation with `AUTONOMY=external`). Publish these at 10 Hz of simulation time; forces
normally at 20 Hz. The guard forwards a force set as soon as every thruster has
a new command, so the controller's rate and timing pass through unchanged.
In a race a live evaluator race-active signal is also required. The guard does not subscribe
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

The Njord vessels use the same model, topic and frame names as the WAM-V, so
all of the interfaces above are unchanged. `njord::Physics` consumes the same atomic
`/njord/actuator_forces` envelope as the WAM-V watchdog; exactly one of these
plugins is loaded. Forces are newtons, applied at the configured thruster
positions. Invalid or expired commands target zero thrust; the configured
actuator response decays the force in simulation time, and expiry uses the same
two limits as the guard (0.5 s of simulation time, 2 s of steady time).

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


The Njord physics plugin additionally publishes `/njord/actuator_wrench`
(`geometry_msgs/msg/WrenchStamped`, bridged from `gz.msgs.Wrench`). It is
**evaluation-only**: actual summed actuator force (N) and moment (N m) about
the configured center of mass, in body-parallel axes, stamped at the physics
step. It must not feed reference autonomy. The existing command topics and
newton units are unchanged. Sensor adapter outputs preserve raw acquisition
stamps and frames; IMU covariance follows the configured orientation, gyro
and accelerometer variances.
