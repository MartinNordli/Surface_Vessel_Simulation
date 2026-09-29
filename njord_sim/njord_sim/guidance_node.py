"""LOS guidance restricted to fresh, observed-free inflated-map corridors.

Line-of-sight (LOS) path follower for the reference autonomy stack. Each
control step picks a target point on the planner's path that is visible
through observed-free map cells, computes a surge force and a yaw moment
(PD on heading) and allocates them over the vessel's thrusters in newtons. The
collision and speed logic lives in the pure-Python ``control_core.py``.

Subscribes:
    ``path_topic`` (default ``/njord/path``, ``nav_msgs/Path``): planner path
        in ``map_frame``; its stamp is the age of the planner's inputs.
    ``odom_topic`` (default ``/njord/odometry``, ``nav_msgs/Odometry``):
        estimated pose in ``map_frame`` with ``child_frame_id == base_frame``;
        uses position, yaw, surge speed (``twist.linear.x``, body frame, m/s)
        and yaw rate (``twist.angular.z``, rad/s).
    ``grid_topic`` (default ``/njord/occupancy``, ``nav_msgs/OccupancyGrid``):
        inflated map; only cells equal to 0 (observed free) are drivable.
    ``status_topic`` (default ``/njord/planner_status``,
        ``diagnostic_msgs/DiagnosticArray``): must hold exactly one
        ``njord/planner`` status with level OK.

Publishes:
    ``thruster_topics`` (one per thruster, e.g. ``/thruster_1/command``,
        ``std_msgs/Float64``): thrust command per thruster in N, positive
        along the thruster's axis. The command guard is the only node that
        forwards these to the actuators.

Parameters:
    Topic names, ``map_frame`` and ``base_frame``. Tuning values (lookahead,
    gains, ``max_speed``, ``max_thrust``, ``control_hz``, ``stale_after_s``
    and the stopping model) come from ``algorithms.yaml``; the thruster
    layout (``thruster_topics``, ``thruster_positions`` relative to the COM,
    ``thruster_axes`` and forward/reverse limits) comes from the vessel file.
    Both arrive through ``node_defaults`` and the run's public parameters.
    Guidance requests zero sway, so a fully actuated vessel does not drift
    sideways while following the path.

Failure behaviour:
    Publishes zero thrust on every step where the path is empty, odometry or
    map is missing/invalid, the planner status is not OK, any input stamp is
    older than ``stale_after_s`` (or in the future) in /clock simulation time,
    the goal is reached, or no observed-free target is visible. Invalid
    messages also zero the thrusters immediately in their callback. Zero
    thrust does not stop the boat instantly; it coasts under the simulated
    dynamics. The speed cap in ``speed_limit`` is what keeps it able to stop.
"""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from njord_sim.constants import BASE_FRAME
from njord_sim.defaults import node_defaults
from njord_sim.control_core import allocate_thrusters, clearance, segment_is_free, speed_limit, tracking_corridor, wrap
from njord_sim.geometry import stamp_seconds
from njord_sim.planner_core import Geometry, fresh


class Guidance(Node):
    """ROS wrapper around the ``control_core`` helpers; see the module docstring."""

    def __init__(self):
        super().__init__('guidance')
        self.declare_parameters('', [
            ('path_topic', '/njord/path'), ('odom_topic', '/njord/odometry'),
            ('grid_topic', '/njord/occupancy'), ('status_topic', '/njord/planner_status'),
            ('map_frame', 'map'), ('base_frame', BASE_FRAME),
            # Tuning, speed ceiling and thruster layout: from algorithms.yaml
            # and the vessel file (see defaults.py); a run overrides them.
            *node_defaults('guidance'),
        ])
        # Shorthand for reading a parameter value.
        self.p = lambda name: self.get_parameter(name).value
        # Refuse to start with limits that would make the maths meaningless
        # (division by zero, zero timer period, no braking ability).
        if min(self.p('braking_deceleration_mps2'),
               self.p('control_hz'), self.p('stale_after_s'), self.p('max_thrust')) <= 0:
            raise ValueError('physical controller limits and frequencies must be positive')
        count = len(self.p('thruster_topics'))
        if (len(self.p('thruster_positions')) != 3 * count or len(self.p('thruster_axes')) != 3 * count
                or len(self.p('thruster_forward_limits')) != count
                or len(self.p('thruster_reverse_limits')) != count):
            raise ValueError('thruster topics, geometry and limits must describe the same thrusters')
        self.path = []  # map-frame (x, y) waypoints in m
        # state: ((x, y) m, yaw rad, surge m/s, yaw rate rad/s) in map frame.
        self.state = self.geometry = self.data = None
        self.path_stamp = self.odom_stamp = self.grid_stamp = self.status_stamp = None
        self.last_sim_time = None
        self.status_valid = False
        self.create_subscription(Path, self.p('path_topic'), self.on_path, 1)
        self.create_subscription(Odometry, self.p('odom_topic'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.p('grid_topic'), self.on_grid, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, self.p('status_topic'), self.on_status, 1)
        self.thrusters = [self.create_publisher(Float64, topic, 1) for topic in self.p('thruster_topics')]
        # Control loop on the node clock (/clock simulation time).
        self.create_timer(1.0 / self.p('control_hz'), self.step)

    def on_path(self, msg):
        """Store a finite map-frame path with its stamp; otherwise stop at once."""
        self.path = []
        self.path_stamp = None
        if msg.header.frame_id == self.p('map_frame'):
            points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
            if all(math.isfinite(v) for point in points for v in point):
                self.path = points
                self.path_stamp = stamp_seconds(msg.header.stamp)
        if not self.path:
            self.stop()

    def on_odom(self, msg):
        """Store pose, yaw, surge and yaw rate; stop at once on invalid odometry."""
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, q.x, q.y, q.z, q.w, twist.linear.x, twist.angular.z)
        if (msg.header.frame_id != self.p('map_frame') or msg.child_frame_id != self.p('base_frame')
                or not all(math.isfinite(v) for v in values)
                or abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 1e-3):
            self.state = self.odom_stamp = None
            self.stop()
            return
        # Yaw (rotation about map z, ENU: 0 = east, counter-clockwise positive).
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.state = ((p.x, p.y), yaw, twist.linear.x, twist.angular.z)
        self.odom_stamp = stamp_seconds(msg.header.stamp)

    def on_grid(self, msg):
        """Store a valid map-frame grid; an invalid grid clears it and stops."""
        try:
            geometry = Geometry.from_message(msg, self.p('map_frame'))
            geometry.validate_data(msg.data)
        except ValueError:
            self.geometry = self.data = self.grid_stamp = None
            self.stop()
            return
        self.geometry, self.data, self.grid_stamp = geometry, list(msg.data), stamp_seconds(msg.header.stamp)

    def on_status(self, msg):
        """Track the planner heartbeat; anything but a single OK record stops."""
        records = [status for status in msg.status if status.name == 'njord/planner']
        self.status_valid = (msg.header.frame_id == self.p('map_frame') and len(records) == 1
                             and records[0].level == DiagnosticStatus.OK)
        self.status_stamp = stamp_seconds(msg.header.stamp)
        if not self.status_valid:
            self.stop()

    def stop(self):
        """Command zero thrust (N) on every thruster."""
        for publisher in self.thrusters:
            publisher.publish(Float64(data=0.0))

    def step(self):
        """One control cycle: validate inputs, pick a target, publish thrust."""
        now = self.get_clock().now().nanoseconds * 1e-9  # /clock simulation time, s
        if self.last_sim_time is not None and now < self.last_sim_time:
            self.path = []
            self.state = self.geometry = self.data = None
            self.path_stamp = self.odom_stamp = self.grid_stamp = self.status_stamp = None
            self.status_valid = False
        self.last_sim_time = now
        # Every input must be present and fresh by its own acquisition stamp.
        if (not self.path or self.state is None or self.geometry is None or not self.status_valid
                or not all(fresh(now, stamp, self.p('stale_after_s')) for stamp in
                           (self.path_stamp, self.odom_stamp, self.grid_stamp, self.status_stamp))):
            return self.stop()
        position, yaw, surge, yaw_rate = self.state
        # Goal reached: stop commanding thrust and let the boat coast.
        if math.dist(position, self.path[-1]) < self.p('goal_tolerance_m'):
            return self.stop()
        target, free_distance = tracking_corridor(self.geometry, self.data, self.path, position, self.p('lookahead_m'))
        # No observed-free target, or one so close (< 5 cm) that its bearing is
        # meaningless: stop rather than steer on noise.
        if target is None or math.dist(position, target) < 0.05:
            return self.stop()
        # Heading error in rad: bearing to the target minus current yaw.
        error = wrap(math.atan2(target[1] - position[1], target[0] - position[0]) - yaw)
        margin = self.p('stopping_margin_m')
        # The goal is an intended stopping point, unlike an unknown-map boundary.
        # Permit arrival inside goal tolerance when its direct corridor is known.
        if (math.dist(position, self.path[-1]) <= self.p('lookahead_m')
                and segment_is_free(self.geometry, self.data, position, self.path[-1])):
            margin = min(margin, self.p('goal_tolerance_m') * 0.5)
        speed = speed_limit(self.p('max_speed'), error, free_distance,
                            clearance(self.geometry, self.data, position), surge,
                            self.p('braking_deceleration_mps2'), self.p('reaction_time_s'),
                            margin)
        # P control on surge speed (N) and PD control on heading (N*m); the
        # yaw-rate term damps the turn.
        force = self.p('kp_surge') * (speed - surge)
        moment = self.p('kp_yaw') * error - self.p('kd_yaw') * yaw_rate
        # Effective per-thruster limit: the smaller of the vessel's thruster
        # limit and the guidance max_thrust. Zero sway is requested.
        thrusts = allocate_thrusters((force, 0.0, moment), self.p('thruster_positions'),
            self.p('thruster_axes'),
            [min(v, self.p('max_thrust')) for v in self.p('thruster_forward_limits')],
            [min(v, self.p('max_thrust')) for v in self.p('thruster_reverse_limits')])
        for publisher, thrust in zip(self.thrusters, thrusts):
            publisher.publish(Float64(data=thrust))


def main():
    rclpy.init()
    node = Guidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
