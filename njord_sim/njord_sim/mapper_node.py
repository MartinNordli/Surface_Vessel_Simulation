"""Timestamped lidar mapping with full TF, observed free rays and aging."""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from njord_sim.geometry import stamp_seconds, transform_from_ros, yaw_from_quaternion
from njord_sim.mapping_core import OccupancyMapper


class Mapper(Node):
    def __init__(self):
        super().__init__("mapper")
        self.declare_parameters("", [
            ("points_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
            ("scan_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/scan"),
            ("grid_topic", "/njord/occupancy"), ("map_frame", "map"),
            ("base_frame", "wamv/base_link"), ("resolution", 0.5), ("size_m", 160.0),
            ("origin_x", -40.0), ("origin_y", -40.0), ("inflation_m", 4.0),
            ("observation_ttl_s", 5.0), ("min_height_m", 0.2), ("max_height_m", 5.0),
            ("max_range_m", 80.0), ("self_length_m", 5.0), ("self_width_m", 2.8),
            ("input_max_age_s", 0.5), ("publish_hz", 5.0),
        ])
        self.p = lambda name: self.get_parameter(name).value
        self.config = dict(resolution=self.p("resolution"), size_m=self.p("size_m"),
                           origin=(self.p("origin_x"), self.p("origin_y")),
                           inflation_m=self.p("inflation_m"), observation_ttl_s=self.p("observation_ttl_s"))
        self.mapper = OccupancyMapper(**self.config)
        self.last_stamp = None
        self.last_clock = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(OccupancyGrid, self.p("grid_topic"), 1)
        self.create_subscription(PointCloud2, self.p("points_topic"), self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p("scan_topic"), self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0/self.p("publish_hz"), self.publish_grid)

    def observe_clock(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.mapper = OccupancyMapper(**self.config)
            self.last_stamp = None
        self.last_clock = now
        return now

    def record_stamp(self, stamp):
        if self.last_stamp is None or stamp_seconds(stamp) > stamp_seconds(self.last_stamp):
            self.last_stamp = stamp

    def on_cloud(self, msg):
        now = self.observe_clock()
        stamp = stamp_seconds(msg.header.stamp)
        if not msg.header.frame_id or not 0 <= now-stamp <= self.p("input_max_age_s"):
            return
        try:
            when = Time.from_msg(msg.header.stamp)
            to_map = self.tf.lookup_transform(self.p("map_frame"), msg.header.frame_id, when)
            to_base = self.tf.lookup_transform(self.p("base_frame"), msg.header.frame_id, when)
        except TransformException:
            return
        raw = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw = np.asarray(raw).reshape(-1, 3)
        if not len(raw):
            return
        ranges = np.linalg.norm(raw, axis=1)
        raw = raw[np.isfinite(raw).all(axis=1) & (ranges > 0.1) & (ranges <= self.p("max_range_m"))]
        if not len(raw):
            return
        body = transform_from_ros(raw, to_base)
        # Reject self returns, without declaring the surrounding footprint free.
        outside_self = (np.abs(body[:, 0]) > self.p("self_length_m")/2) | (np.abs(body[:, 1]) > self.p("self_width_m")/2)
        filtered = raw[outside_self]
        world = transform_from_ros(filtered, to_map)
        has_return = np.linalg.norm(filtered, axis=1) < self.p("max_range_m")-0.01
        if not len(world):
            return
        origin = transform_from_ros([[0., 0., 0.]], to_map)[0]
        # Water returns establish free visibility only up to their contact point;
        # high returns can occlude objects and therefore do not clear the 2D map.
        height_valid = world[:, 2] <= self.p("max_height_m")
        world, has_return = world[height_valid], has_return[height_valid]
        hit = (world[:, 2] >= self.p("min_height_m")) & has_return
        if self.mapper.update(origin, world, hit, stamp, stream="cloud"):
            self.record_stamp(msg.header.stamp)

    def on_scan(self, msg):
        """Use explicit +inf range evidence; NaN/negative ranges stay unknown.

        The planar scan supplements sparse 3D rays. Like any projected 2D lidar
        map, it assumes obstacles intersect the sensing volume; overhanging and
        low objects require the 3D cloud and empirical sensor validation.
        """
        now = self.observe_clock()
        stamp = stamp_seconds(msg.header.stamp)
        if not msg.header.frame_id or not 0 <= now-stamp <= self.p("input_max_age_s"):
            return
        try:
            transform = self.tf.lookup_transform(self.p("map_frame"), msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        maximum = min(float(msg.range_max), self.p("max_range_m"))
        if not np.isfinite(maximum) or maximum <= 0:
            return
        valid = (np.isposinf(ranges) | np.isfinite(ranges)) & (ranges >= max(msg.range_min, 0.1))
        angles, ranges = angles[valid], ranges[valid]
        hit = np.isfinite(ranges) & (ranges < maximum)
        ranges = np.minimum(ranges, maximum)
        raw = np.column_stack((ranges*np.cos(angles), ranges*np.sin(angles), np.zeros(len(ranges))))
        world = transform_from_ros(raw, transform)
        origin = transform_from_ros([[0., 0., 0.]], transform)[0]
        # Only no-return rays supplement the 3D cloud, which owns obstacle hits
        # and self filtering. Finite planar hits may come from the vessel itself.
        world = world[~hit]
        if len(world) and self.mapper.update(origin, world, np.zeros(len(world), dtype=bool), stamp, stream="scan"):
            self.record_stamp(msg.header.stamp)

    def publish_grid(self):
        now = self.observe_clock()
        if self.last_stamp is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.last_stamp  # A heartbeat must never freshen old lidar.
        msg.header.frame_id = self.p("map_frame")
        msg.info.resolution = self.mapper.resolution
        msg.info.width = msg.info.height = self.mapper.size
        msg.info.origin.position.x, msg.info.origin.position.y = map(float, self.mapper.origin)
        msg.info.origin.orientation.w = 1.0
        msg.data = self.mapper.grid(now).ravel().tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
