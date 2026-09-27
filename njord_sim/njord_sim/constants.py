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

# Steady wall-clock time after which a missing thrust command is treated as
# stale. Used by the ROS command guard and both Gazebo actuator plugins.
COMMAND_TIMEOUT_S = 0.5

# ROS topic on which the command guard receives one thruster's force in
# newtons (std_msgs/Float64), formatted with the thruster name from the vessel
# file. '/thruster_1/command' matches Control & Autonomy's allocation node.
THRUSTER_COMMAND_TOPIC = '/{name}/command'

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
