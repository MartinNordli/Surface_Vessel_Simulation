"""Persistent D* Lite with timestamp-based validity and explicit invalidation."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from njord_sim.planner_core import Geometry, IncrementalPlanner, fresh


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.declare_parameters('', [
            ('grid_topic', '/njord/occupancy'), ('odom_topic', '/njord/odometry'),
            ('goal_topic', '/njord/goal'), ('path_topic', '/njord/path'),
            ('status_topic', '/njord/planner_status'), ('map_frame', 'map'),
            ('stale_after_s', 1.0), ('publish_hz', 5.0),
        ])
        p = lambda name: self.get_parameter(name).value
        self.map_frame, self.timeout = p('map_frame'), p('stale_after_s')
        self.core = IncrementalPlanner()
        self.geometry = self.data = self.position = self.goal = None
        self.map_stamp = self.odom_stamp = None
        self.dirty = True
        self.points = []
        self.error = 'waiting for inputs'
        self.create_subscription(OccupancyGrid, p('grid_topic'), self.on_grid, qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, p('goal_topic'), self.on_goal, 1)
        self.path_pub = self.create_publisher(Path, p('path_topic'), 1)
        self.status_pub = self.create_publisher(DiagnosticArray, p('status_topic'), 1)
        self.latency_pub = self.create_publisher(Float64, '/njord/plan_ms', 1)
        self.create_timer(1.0 / p('publish_hz'), self.step)

    def invalidate(self, reason):
        self.points = []
        self.error = reason
        self.publish(False)

    def on_grid(self, msg):
        try:
            geometry = Geometry.from_message(msg, self.map_frame)
            geometry.validate_data(msg.data)
        except ValueError as error:
            self.geometry = self.data = self.map_stamp = None
            self.invalidate(str(error))
            return
        self.geometry, self.data = geometry, list(msg.data)
        self.map_stamp = stamp_seconds(msg.header.stamp)
        self.dirty = True
        self.step()

    def on_odom(self, msg):
        point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if msg.header.frame_id != self.map_frame or not all(math.isfinite(v) for v in point):
            self.position = self.odom_stamp = None
            self.invalidate('invalid odometry frame or position')
            return
        self.position, self.odom_stamp = point, stamp_seconds(msg.header.stamp)
        self.dirty = True

    def on_goal(self, msg):
        point = (msg.pose.position.x, msg.pose.position.y)
        if msg.header.frame_id != self.map_frame or not all(math.isfinite(v) for v in point):
            self.goal = None
            self.invalidate('invalid goal frame or position')
            return
        if point == self.goal:
            return  # mission heartbeat does not change route validity
        self.goal, self.dirty = point, True
        self.invalidate('goal changed; planning')
        self.step()

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not fresh(now, self.map_stamp, self.timeout) or not fresh(now, self.odom_stamp, self.timeout):
            self.dirty = True
            self.invalidate('map or odometry stale/unavailable')
            return
        if self.goal is None or self.geometry is None or self.position is None:
            self.invalidate('waiting for valid goal, map and odometry')
            return
        if self.dirty:
            start = time.perf_counter()
            try:
                self.points = self.core.plan(self.geometry, self.data, self.position, self.goal)
                self.error = 'valid path' if self.points else 'no feasible path or invalid endpoint'
            except ValueError as error:
                self.points, self.error = [], str(error)
            self.latency_pub.publish(Float64(data=(time.perf_counter() - start) * 1000))
            self.dirty = False
        self.publish(bool(self.points))

    def publish(self, valid):
        path = Path()
        path.header.frame_id = self.map_frame
        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        if valid:
            # Refresh only as new sensor input arrives, never from the timer alone.
            stamp = min(self.map_stamp, self.odom_stamp)
            path.header.stamp.sec = int(stamp)
            path.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
            if path.header.stamp.nanosec >= 1000000000:
                path.header.stamp.sec += 1
                path.header.stamp.nanosec = 0
            for x, y in self.points:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.path_pub.publish(path)
        status = DiagnosticStatus()
        status.name, status.hardware_id = 'njord/planner', 'dstar_lite'
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR
        status.message = self.error
        status.values = [KeyValue(key='path_valid', value=str(valid).lower()),
                         KeyValue(key='map_stamp', value=str(self.map_stamp)),
                         KeyValue(key='odom_stamp', value=str(self.odom_stamp))]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.status = [status]
        self.status_pub.publish(message)


def main():
    rclpy.init()
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.invalidate('planner shutting down')
        node.destroy_node()
        rclpy.try_shutdown()
