"""Timestamped lidar mapping with full TF, observed free rays and aging.

ROS node ``mapper``: builds the 2D occupancy grid that the planner uses, from
lidar only (see ``mapping_core.OccupancyMapper`` for the grid logic).

Subscribes (best-effort sensor QoS):
    ``points_topic`` (sensor_msgs/PointCloud2): 3D lidar in the lidar frame.
        Owns obstacle hits and self-return filtering.
    ``scan_topic`` (sensor_msgs/LaserScan): planar lidar scan. Only its
        no-return rays are used, to add observed free space.

Publishes:
    ``grid_topic`` (nav_msgs/OccupancyGrid, default ``/njord/occupancy``) in
        ``map_frame`` at ``publish_hz`` (timer on the node clock, i.e.
        simulation time). Values: -1 unknown, 0 observed free, 100 obstacle
        inflated by ``inflation_m``. The header stamp is the acquisition time
        of the newest lidar data integrated, not the publish time. Nothing is
        published until the first measurement has been integrated.

Parameters:
    ``map_frame`` (``map``, ENU) and ``base_frame`` (vessel body, x forward,
    y left, z up); grid ``resolution``, ``size_m``, ``origin_x``/``origin_y``
    [m]; ``inflation_m`` [m] (default from the vessel hull and
    algorithms.yaml); ``observation_ttl_s`` [s]; ``min_height_m`` and
    ``max_height_m`` [m, map z] for obstacle hits; ``max_range_m`` [m];
    ``self_length_m``/``self_width_m`` [m], the body-frame box around
    ``base_frame`` whose returns are discarded as the vessel itself;
    ``input_max_age_s`` [s]; ``publish_hz`` [Hz].

Timing and failure behaviour:
    Each message is transformed with TF looked up at its own acquisition
    stamp. Messages with no frame, older than ``input_max_age_s`` or stamped
    in the future (relative to simulation time), or without TF at their
    stamp are dropped; they add no evidence. Old cells expire back to
    unknown, and because the grid keeps the newest integrated lidar stamp,
    a stalled lidar shows up as a stale map instead of being re-stamped.
    When simulation time jumps backwards (world reset) the map is discarded.
"""
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
from njord_sim.defaults import node_defaults
from njord_sim.mapping_core import OccupancyMapper


class Mapper(Node):
    """ROS wrapper around ``OccupancyMapper`` for the cloud and scan inputs."""

    def __init__(self):
        super().__init__("mapper")
        self.declare_parameters("", [
            ("points_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
            ("scan_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/scan"),
            ("grid_topic", "/njord/occupancy"), ("map_frame", "map"),
            ("base_frame", "wamv/base_link"), ("resolution", 0.5), ("size_m", 160.0),
            ("origin_x", -40.0), ("origin_y", -40.0),
            *node_defaults("mapper"),  # inflation_m: hull radius + algorithms.yaml margin
            ("observation_ttl_s", 5.0), ("min_height_m", 0.2), ("max_height_m", 5.0),
            ("max_range_m", 80.0), ("self_length_m", 5.0), ("self_width_m", 2.8),
            ("input_max_age_s", 0.5), ("publish_hz", 5.0),
        ])
        self.p = lambda name: self.get_parameter(name).value
        # Kept so the grid can be rebuilt identically after a simulation reset.
        self.config = dict(resolution=self.p("resolution"), size_m=self.p("size_m"),
                           origin=(self.p("origin_x"), self.p("origin_y")),
                           inflation_m=self.p("inflation_m"),
                           observation_ttl_s=self.p("observation_ttl_s"))
        self.mapper = OccupancyMapper(**self.config)
        # Header stamp (builtin_interfaces/Time) of the newest integrated lidar data.
        self.last_stamp = None
        # Last simulation time seen [s], to detect the clock jumping back.
        self.last_clock = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(OccupancyGrid, self.p("grid_topic"), 1)
        self.create_subscription(PointCloud2, self.p("points_topic"), self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p("scan_topic"), self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(1.0/self.p("publish_hz"), self.publish_grid)

    def observe_clock(self):
        """Return simulation time [s]; reset the map if time went backwards.

        A backwards jump means the Gazebo world was reset. Evidence stamped
        in the old timeline would otherwise look like it came from the future
        and block new data, so the whole grid starts over as unknown.
        """
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.mapper = OccupancyMapper(**self.config)
            self.last_stamp = None
        self.last_clock = now
        return now

    def record_stamp(self, stamp):
        """Remember ``stamp`` if it is the newest integrated acquisition time."""
        if self.last_stamp is None or stamp_seconds(stamp) > stamp_seconds(self.last_stamp):
            self.last_stamp = stamp

    def on_cloud(self, msg):
        """Integrate a 3D lidar cloud: hits for obstacles, free space along rays.

        Filtering, in order:

        * keep finite points with range from the lidar in (0.1, max_range_m];
        * drop self returns: points inside the ``self_length_m`` x
          ``self_width_m`` box around ``base_frame`` (body x forward, y left);
        * drop points higher than ``max_height_m`` in map z;
        * a remaining point is a hit if it is at least ``min_height_m`` high
          in map z and is a real return (range below ``max_range_m`` - 1 cm).
          Lower points (e.g. returns from the water surface) and max-range
          points only clear cells along their ray.
        """
        now = self.observe_clock()
        stamp = stamp_seconds(msg.header.stamp)
        if not msg.header.frame_id or not 0 <= now-stamp <= self.p("input_max_age_s"):
            return
        try:
            # Both transforms at the cloud acquisition time, not "now".
            when = Time.from_msg(msg.header.stamp)
            to_map = self.tf.lookup_transform(self.p("map_frame"), msg.header.frame_id, when)
            to_base = self.tf.lookup_transform(self.p("base_frame"), msg.header.frame_id, when)
        except TransformException:
            return
        raw = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw = np.asarray(raw).reshape(-1, 3)
        if not len(raw):
            return
        # Ranges are measured from the lidar origin (the cloud's own frame).
        ranges = np.linalg.norm(raw, axis=1)
        raw = raw[np.isfinite(raw).all(axis=1) & (ranges > 0.1) & (ranges <= self.p("max_range_m"))]
        if not len(raw):
            return
        body = transform_from_ros(raw, to_base)
        # Reject self returns, without declaring the surrounding footprint free.
        outside_self = ((np.abs(body[:, 0]) > self.p("self_length_m")/2)
                        | (np.abs(body[:, 1]) > self.p("self_width_m")/2))
        filtered = raw[outside_self]
        world = transform_from_ros(filtered, to_map)
        # Points at (within 1 cm of) max range are treated as "no return".
        has_return = np.linalg.norm(filtered, axis=1) < self.p("max_range_m")-0.01
        if not len(world):
            return
        # Lidar position in map at acquisition time: the start of every ray.
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

        Ranges below ``max(range_min, 0.1)`` m, NaN and -inf are ignored. A
        ``+inf`` range, or a range at or beyond ``min(range_max,
        max_range_m)``, is a "no return" ray; it is clipped to that maximum
        and clears the cells along it. Finite hits are not used here.
        """
        now = self.observe_clock()
        stamp = stamp_seconds(msg.header.stamp)
        if not msg.header.frame_id or not 0 <= now-stamp <= self.p("input_max_age_s"):
            return
        try:
            transform = self.tf.lookup_transform(self.p("map_frame"), msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp))
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
        # Polar -> Cartesian in the scan frame's x-y plane (z = 0).
        raw = np.column_stack((ranges*np.cos(angles), ranges*np.sin(angles), np.zeros(len(ranges))))
        world = transform_from_ros(raw, transform)
        origin = transform_from_ros([[0., 0., 0.]], transform)[0]
        # Only no-return rays supplement the 3D cloud, which owns obstacle hits
        # and self filtering. Finite planar hits may come from the vessel itself.
        world = world[~hit]
        if len(world) and self.mapper.update(origin, world, np.zeros(len(world), dtype=bool),
                                             stamp, stream="scan"):
            self.record_stamp(msg.header.stamp)

    def publish_grid(self):
        """Publish the aged and inflated grid (timer callback)."""
        now = self.observe_clock()
        if self.last_stamp is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.last_stamp  # A heartbeat must never freshen old lidar.
        msg.header.frame_id = self.p("map_frame")
        msg.info.resolution = self.mapper.resolution
        msg.info.width = msg.info.height = self.mapper.size
        # The grid is axis-aligned with map: identity orientation at the origin corner.
        msg.info.origin.position.x, msg.info.origin.position.y = map(float, self.mapper.origin)
        msg.info.origin.orientation.w = 1.0
        # Row-major, row 0 at origin_y, as OccupancyGrid expects.
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
