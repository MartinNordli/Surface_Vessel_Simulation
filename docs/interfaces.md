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
to the team's nodes. Publish the documented forces and status messages. The
planner diagnostic name is `njord/planner`; mission is `mission`; navigation
is `navigation`. Level OK means current valid inputs. ERROR, missing heartbeats,
an empty path or expired commands removes commanded thrust. A fresh evaluator
race heartbeat is also required. Never publish plausible-looking health status
when the corresponding input has stopped.

Use topic remapping/ROS parameters on the individual nodes for different sensor
names. Sensor geometry and models are in `njord_sim/config`; scenario truth goes
only to world generation and scoring. A mission knows the number of gates, not
their locations. Red-left/green-right is this demo's rule, not an assertion about
the official competition rules.
