"""Persistent D* Lite with timestamp-based validity and explicit invalidation.

Plans a grid path from the vessel to the mission goal on the inflated
occupancy map, using the pure-Python core in ``planner_core.py`` /
``dstar_lite.py``. The search is kept between updates and repaired
incrementally when the map changes.

Subscribes:
    ``grid_topic`` (default ``/njord/occupancy``, ``nav_msgs/OccupancyGrid``):
        inflated map in ``map_frame``; values >= 50 are blocked.
    ``odom_topic`` (default ``/njord/odometry``, ``nav_msgs/Odometry``):
        estimated vessel pose in ``map_frame`` (start of the path).
    ``goal_topic`` (default ``/njord/goal``, ``geometry_msgs/PoseStamped``):
        goal from the mission node; only the x/y position is used.

Publishes:
    ``path_topic`` (default ``/njord/path``, ``nav_msgs/Path``): the path as
        map-frame cell centres. A valid path is stamped with the older of the
        map and odometry acquisition stamps, so its age reflects its inputs.
        An invalid state publishes an empty path.
    ``status_topic`` (default ``/njord/planner_status``,
        ``diagnostic_msgs/DiagnosticArray``): heartbeat with one status named
        ``njord/planner``; level OK only while a valid path exists, ERROR with
        the reason otherwise. The command guard requires this to be OK.
    ``/njord/plan_ms`` (``std_msgs/Float64``): wall-clock planning time in ms,
        published each time a replan runs.

Parameters:
    Topic names above, ``map_frame``, and ``stale_after_s`` (max input age in
    s) and ``publish_hz`` (path/status heartbeat rate), which come from
    ``algorithms.yaml`` through ``node_defaults``.

Failure behaviour:
    Stale (older than ``stale_after_s`` in /clock simulation time), missing,
    wrong-frame or non-finite map/odometry/goal, or no feasible path, all
    publish an empty path and an ERROR status. Guidance then stops the
    thrusters. Timestamps are never refreshed from the timer alone.
"""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from njord_sim.defaults import node_defaults
from njord_sim.geometry import stamp_seconds
from njord_sim.planner_core import Geometry, IncrementalPlanner, fresh


class Planner(Node):
    """ROS wrapper around ``IncrementalPlanner``; see the module docstring."""

    def __init__(self):
        super().__init__('planner')
        self.declare_parameters('', [
            ('grid_topic', '/njord/occupancy'), ('odom_topic', '/njord/odometry'),
            ('goal_topic', '/njord/goal'), ('path_topic', '/njord/path'),
            ('status_topic', '/njord/planner_status'), ('map_frame', 'map'),
            *node_defaults('planner'),  # stale_after_s, publish_hz from algorithms.yaml
        ])
        p = lambda name: self.get_parameter(name).value
        self.map_frame, self.timeout = p('map_frame'), p('stale_after_s')
        self.core = IncrementalPlanner()
        self.geometry = self.data = self.position = self.goal = None
        self.map_stamp = self.odom_stamp = None
        self.last_sim_time = None
        # dirty: inputs changed since the last plan, so the next step() replans.
        self.dirty = True
        self.points = []  # current path as map-frame (x, y) points; [] = invalid
        self.error = 'waiting for inputs'  # reason reported in the status
        self.create_subscription(OccupancyGrid, p('grid_topic'), self.on_grid, qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, p('goal_topic'), self.on_goal, 1)
        self.path_pub = self.create_publisher(Path, p('path_topic'), 1)
        self.status_pub = self.create_publisher(DiagnosticArray, p('status_topic'), 1)
        self.latency_pub = self.create_publisher(Float64, '/njord/plan_ms', 1)
        # Heartbeat timer on the node clock (/clock simulation time).
        self.create_timer(1.0 / p('publish_hz'), self.step)

    def invalidate(self, reason):
        """Drop the current path and immediately publish an empty path and ERROR."""
        self.points = []
        self.error = reason
        self.publish(False)

    def on_grid(self, msg):
        """Store a new map; an invalid map clears the stored one and invalidates."""
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
        """Store the vessel position and its acquisition stamp (replan on next step)."""
        point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if msg.header.frame_id != self.map_frame or not all(math.isfinite(v) for v in point):
            self.position = self.odom_stamp = None
            self.invalidate('invalid odometry frame or position')
            return
        self.position, self.odom_stamp = point, stamp_seconds(msg.header.stamp)
        self.dirty = True

    def on_goal(self, msg):
        """Accept a new goal; an unchanged goal is ignored, a new one replans now."""
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
        """Check input freshness, replan if needed and publish path + status.

        Runs on the timer and after every new map or goal.
        """
        now = self.get_clock().now().nanoseconds * 1e-9  # /clock simulation time, s
        if self.last_sim_time is not None and now < self.last_sim_time:
            self.geometry = self.data = self.position = self.goal = None
            self.map_stamp = self.odom_stamp = None
            self.dirty = True
        self.last_sim_time = now
        if not fresh(now, self.map_stamp, self.timeout) or not fresh(now, self.odom_stamp, self.timeout):
            # Force a full replan once fresh inputs return.
            self.dirty = True
            self.invalidate('map or odometry stale/unavailable')
            return
        if self.goal is None or self.geometry is None or self.position is None:
            self.invalidate('waiting for valid goal, map and odometry')
            return
        if self.dirty:
            # Planning latency is measured in wall time; it is a performance
            # metric only and never used for validity decisions.
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
        """Publish the path (empty unless ``valid``) and the diagnostic status."""
        path = Path()
        path.header.frame_id = self.map_frame
        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        if valid:
            # Refresh only as new sensor input arrives, never from the timer alone.
            # The path is as old as its oldest input, so downstream freshness
            # checks (guidance) see real data age rather than publish time.
            stamp = min(self.map_stamp, self.odom_stamp)
            path.header.stamp.sec = int(stamp)
            path.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
            # Rounding can give exactly 1e9 ns; carry it into the seconds.
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
        # The status itself is a heartbeat, so it carries the current sim time;
        # its level (not its stamp) says whether the path is valid.
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
        # Leave an empty path and ERROR status behind so nothing follows an
        # old route after the planner exits.
        node.invalidate('planner shutting down')
        node.destroy_node()
        rclpy.try_shutdown()
