"""LOS guidance restricted to fresh, observed-free inflated-map corridors."""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from njord_sim.control_core import clearance, mix_thrusters, segment_is_free, speed_limit, tracking_corridor, wrap
from njord_sim.planner_core import Geometry, fresh


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Guidance(Node):
    def __init__(self):
        super().__init__('guidance')
        self.declare_parameters('', [
            ('path_topic', '/njord/path'), ('odom_topic', '/njord/odometry'),
            ('grid_topic', '/njord/occupancy'), ('status_topic', '/njord/planner_status'),
            ('left_topic', '/njord/thrusters/left/thrust'), ('right_topic', '/njord/thrusters/right/thrust'),
            ('map_frame', 'map'), ('base_frame', 'wamv/base_link'), ('lookahead_m', 8.0),
            ('kp_yaw', 400.0), ('kd_yaw', 300.0), ('kp_surge', 200.0),
            ('max_speed', 1.5), ('max_thrust', 500.0), ('thruster_separation_m', 2.05427),
            ('goal_tolerance_m', 1.5), ('control_hz', 20.0), ('stale_after_s', 1.0),
            ('braking_deceleration_mps2', 0.25), ('reaction_time_s', 1.0), ('stopping_margin_m', 3.0),
        ])
        self.p = lambda name: self.get_parameter(name).value
        if min(self.p('braking_deceleration_mps2'), self.p('thruster_separation_m'),
               self.p('control_hz'), self.p('stale_after_s'), self.p('max_thrust')) <= 0:
            raise ValueError('physical controller limits and frequencies must be positive')
        self.path = []
        self.state = self.geometry = self.data = None
        self.path_stamp = self.odom_stamp = self.grid_stamp = self.status_stamp = None
        self.status_valid = False
        self.create_subscription(Path, self.p('path_topic'), self.on_path, 1)
        self.create_subscription(Odometry, self.p('odom_topic'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.p('grid_topic'), self.on_grid, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, self.p('status_topic'), self.on_status, 1)
        self.left = self.create_publisher(Float64, self.p('left_topic'), 1)
        self.right = self.create_publisher(Float64, self.p('right_topic'), 1)
        self.create_timer(1.0 / self.p('control_hz'), self.step)

    def on_path(self, msg):
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
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, q.x, q.y, q.z, q.w, twist.linear.x, twist.angular.z)
        if (msg.header.frame_id != self.p('map_frame') or msg.child_frame_id != self.p('base_frame')
                or not all(math.isfinite(v) for v in values)
                or abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 1e-3):
            self.state = self.odom_stamp = None
            self.stop()
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.state = ((p.x, p.y), yaw, twist.linear.x, twist.angular.z)
        self.odom_stamp = stamp_seconds(msg.header.stamp)

    def on_grid(self, msg):
        try:
            geometry = Geometry.from_message(msg, self.p('map_frame'))
            geometry.validate_data(msg.data)
        except ValueError:
            self.geometry = self.data = self.grid_stamp = None
            self.stop()
            return
        self.geometry, self.data, self.grid_stamp = geometry, list(msg.data), stamp_seconds(msg.header.stamp)

    def on_status(self, msg):
        records = [status for status in msg.status if status.name == 'njord/planner']
        self.status_valid = (msg.header.frame_id == self.p('map_frame') and len(records) == 1
                             and records[0].level == DiagnosticStatus.OK)
        self.status_stamp = stamp_seconds(msg.header.stamp)
        if not self.status_valid:
            self.stop()

    def stop(self):
        self.left.publish(Float64(data=0.0))
        self.right.publish(Float64(data=0.0))

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if (not self.path or self.state is None or self.geometry is None or not self.status_valid
                or not all(fresh(now, stamp, self.p('stale_after_s')) for stamp in
                           (self.path_stamp, self.odom_stamp, self.grid_stamp, self.status_stamp))):
            return self.stop()
        position, yaw, surge, yaw_rate = self.state
        if math.dist(position, self.path[-1]) < self.p('goal_tolerance_m'):
            return self.stop()
        target, free_distance = tracking_corridor(self.geometry, self.data, self.path, position, self.p('lookahead_m'))
        if target is None or math.dist(position, target) < 0.05:
            return self.stop()
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
        force = self.p('kp_surge') * (speed - surge)
        moment = self.p('kp_yaw') * error - self.p('kd_yaw') * yaw_rate
        left, right = mix_thrusters(force, moment, self.p('thruster_separation_m'), self.p('max_thrust'))
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))


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
