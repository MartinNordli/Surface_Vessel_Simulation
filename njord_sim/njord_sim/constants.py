"""Fixed platform constants shared by model generation, launch and scoring.

These values are properties of the pinned simulation platform, not tuning
knobs. Anything a user is expected to change lives in YAML instead:

- vessel sensors and actuator limits:  njord_sim/config/vessels/<vessel>.yaml
- reference autonomy tuning:          njord_sim/config/algorithms.yaml
- course, start pose and environment: scenarios/<course>.yaml

Keeping each constant here, once, prevents the generators, the evaluator and
the autonomy launch from silently disagreeing.
"""

# World origin (WGS84 latitude/longitude in degrees, altitude in metres).
# Gazebo's spherical coordinates and the navsat_transform datum must match,
# otherwise GPS positions and the ENU map frame drift apart.
WORLD_ORIGIN_WGS84 = (63.4305, 10.3951, 0.0)

# Actuation uses two clocks (docs/interfaces.md). A thrust command or health
# heartbeat is stale once it is older than COMMAND_TIMEOUT_S of simulation
# time: that decides how long it acts on the boat, at any real-time factor.
# PROCESS_LIVENESS_S of steady wall time is only a watchdog for a stopped
# /clock or a dead process; while simulation time stands still the boat does
# not move either, so it can be generous. Heartbeats at 10 Hz of simulation
# time stay live down to a real-time factor of about 0.05. Used by the ROS
# command guard, both Gazebo actuator plugins and the evaluator.
COMMAND_TIMEOUT_S = 0.5
PROCESS_LIVENESS_S = 2.0

# Gravitational acceleration (m/s^2) of the Njord world. scenario.py writes it
# as the world <gravity>; the NjordPhysics plugin reads that world value for
# buoyancy, so weight and buoyancy always use the same g. Draft does not depend
# on g (m g = rho g V), so the local Trondheim value (about 9.8215) would not
# change flotation.
GRAVITY_MPS2 = 9.81

# Health heartbeats of the processes the simulator starts itself, as
# key -> (topic, DiagnosticStatus name). configuration.guard_requirements
# decides which of them the guard and the evaluator require.
HEARTBEATS = {'navigation': ('/njord/navigation_status', 'navigation'),
              'planner': ('/njord/planner_status', 'njord/planner'),
              'mission': ('/njord/mission_status', 'mission'),
              'controller': ('/njord/controller_status', 'controller')}

# ROS topic on which the command guard receives one thruster's force in
# newtons (std_msgs/Float64), formatted with the thruster name from the vessel
# file. '/thruster_1/command' matches Control & Autonomy's allocation node.
THRUSTER_COMMAND_TOPIC = '/{name}/command'

# Setpoint courses ("go to this position"; docs/control-autonomy-setpoints.md).
# PROVISIONAL names until Control & Autonomy confirms them: rename a topic here
# and every node, launch file and test follows. The message types are fixed in
# the code that uses them (evaluator_node.py, setpoint_controller_node.py,
# scripts/send_setpoint.py).
#   SETPOINT_TOPIC           geometry_msgs/PoseStamped in map: the active target
#                            position and heading; a new message replaces it.
#                            Published reliable + transient local (depth 1);
#                            subscribe reliable (transient local to get the
#                            active target when joining late).
#   SETPOINT_SEQUENCE_TOPIC  nav_msgs/Path in map: the active target followed by
#                            the remaining scripted ones (optional lookahead).
# Under /sim, derived from ground truth for display only; autonomy must not
# subscribe to them:
#   SETPOINT_STATUS_TOPIC    diagnostic_msgs/DiagnosticArray, live score
#   SETPOINT_MARKERS_TOPIC   visualization_msgs/MarkerArray for RViz
#   TRAJECTORY_TOPIC         nav_msgs/Path, the travelled ground-truth track
SETPOINT_TOPIC = '/njord/setpoint'
SETPOINT_SEQUENCE_TOPIC = '/njord/setpoint_sequence'
SETPOINT_STATUS_TOPIC = '/sim/setpoint_status'
SETPOINT_MARKERS_TOPIC = '/sim/setpoint_markers'
TRAJECTORY_TOPIC = '/sim/trajectory'
# Thrust forces passed by the command guard (ros_gz_interfaces/Float32Array, N,
# vessel-file thruster order); the evaluator records them for the run report.
ACTUATOR_FORCES_TOPIC = '/njord/actuator_forces'
MAP_FRAME = 'map'
# Acceptance of targets sent by hand in a lab run (njord goto, RViz, a team's
# mission node), where no scenario entry describes them. The evaluator's
# observer mode scores every such target with these values.
OBSERVED_SETPOINT_ACCEPTANCE = {'tolerance_m': 1.5, 'heading_tolerance_deg': 15.0, 'hold_s': 5.0}

# ROS-facing sensor topics and frames, identical for every vessel profile so a
# team's code does not depend on which boat is simulated (docs/interfaces.md).
# Frames follow REP-105 without a vessel prefix. The Gazebo link names equal
# the frame names, so TF, sensor headers and Gazebo agree. Topics under /sim
# exist only in simulation: ground truth and raw GPS/IMU before the sensor
# adapter adds noise. Autonomy must not subscribe to them.
GZ_MODEL_NAME = 'vessel'  # Gazebo model name; contact names start with 'vessel::'
BASE_FRAME = 'base_link'
CAMERAS = ('front_left', 'front_right')
LIDAR_FRAME = 'lidar_link'
IMU_FRAME = 'imu_link'
GPS_FRAME = 'gps_link'
LIDAR_POINTS_TOPIC = '/sensors/lidar/points'
LIDAR_SCAN_TOPIC = '/sensors/lidar/scan'
IMU_TOPIC = '/sensors/imu/data'
GPS_TOPIC = '/sensors/gps/fix'
IMU_RAW_TOPIC = '/sim/sensors/imu/data_raw'
GPS_RAW_TOPIC = '/sim/sensors/gps/fix_raw'
GROUND_TRUTH_TOPIC = '/sim/ground_truth/odometry'

# Gazebo visibility mask of every lidar. The VRX sea surface (coast_waves) is
# a visual with visibility flag 8, drawn with shader waves the flat-water
# physics does not have; the pinned VRX WAM-V lidar uses mask 7 so it does not
# see it, and a real lidar returns very little from water. Without the mask the
# Njord lidar saw that surface up to about 0.6 m above the physical water level
# 3-10 m around the boat and the mapper marked it as obstacles.
LIDAR_VISIBILITY_MASK = 7


def camera_frame(camera, optical=False):
    """Body-aligned camera link of ``camera`` in CAMERAS, or its ROS optical frame."""
    return f'{camera}_camera_link' + ('_optical' if optical else '')


def camera_topic(camera, name):
    """Camera topic; ``name`` is 'image_raw' or 'camera_info'."""
    return f'/sensors/cameras/{camera}/{name}'

# The WAM-V hull, hydrodynamics and thruster placement come from the pinned
# VRX model. Only its sensors and thrust limit are configurable in YAML.
# Horizontal scoring envelope enclosing the VRX WAM-V hull (conservative).
WAMV_HULL = {'length_m': 6.0, 'beam_m': 3.3}
# The two aft thrusters of the VRX WAM-V 'H' layout (wamv_aft_thrusters.xacro),
# port then starboard, in body coordinates (m). yaw_deg 0 pushes forward.
WAMV_THRUSTERS = (
    {'name': 'thruster_1', 'position_m': [-2.373776, 1.027135, 0.318237], 'yaw_deg': 0.0},
    {'name': 'thruster_2', 'position_m': [-2.373776, -1.027135, 0.318237], 'yaw_deg': 0.0},
)
