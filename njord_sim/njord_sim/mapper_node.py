"""Lidar point cloud -> inflated 2D occupancy grid.

Deliberately simple: a fixed world-frame grid, hits marked occupied, an
inflation disc around each hit, and the boat's own footprint forced free every
cycle. Nothing is ever un-marked apart from the footprint, so the map only
grows. That is enough for the demo and it keeps the failure modes few.

Points are transformed with the pose from odometry plus a static sensor offset
rather than through tf2. That is a deliberate shortcut for the proof of
concept: it removes a whole class of tf timing problems while the ground truth
odometry is exact anyway. Move to tf2 once a real state estimator replaces it.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


class Mapper(Node):
    def __init__(self):
        super().__init__("mapper")
        self.declare_parameters(
            "",
            [
                ("points_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
                ("odom_topic", "/wamv/ground_truth/odometry"),
                ("grid_topic", "/njord/occupancy"),
                ("map_frame", "map"),
                ("resolution", 2.0),
                ("size_m", 600.0),
                ("origin_x", -300.0),
                ("origin_y", -300.0),
                ("inflation_m", 6.0),
                ("footprint_m", 5.0),
                ("min_height_m", 0.4),   # world-frame z, rejects wave returns
                ("max_height_m", 6.0),
                ("max_range_m", 80.0),
                ("sensor_offset", [1.0, 0.0, 2.0]),
                ("publish_hz", 2.0),
            ],
        )
        p = self.get_parameter
        self.res = p("resolution").value
        self.n = int(p("size_m").value / self.res)
        self.origin = np.array([p("origin_x").value, p("origin_y").value])
        self.map_frame = p("map_frame").value
        self.min_z = p("min_height_m").value
        self.max_z = p("max_height_m").value
        self.max_range = p("max_range_m").value
        self.offset = np.array(p("sensor_offset").value, dtype=float)

        self.inflation = self._disc(int(round(p("inflation_m").value / self.res)))
        self.footprint = self._disc(int(round(p("footprint_m").value / self.res)))

        self.occ = np.zeros((self.n, self.n), dtype=np.int8)
        self.pose = None

        self.create_subscription(Odometry, p("odom_topic").value, self.on_odom, 10)
        self.create_subscription(PointCloud2, p("points_topic").value, self.on_cloud, 5)
        self.pub = self.create_publisher(OccupancyGrid, p("grid_topic").value, 1)
        self.create_timer(1.0 / p("publish_hz").value, self.publish_grid)

    @staticmethod
    def _disc(radius_cells):
        offsets = [
            (dr, dc)
            for dr in range(-radius_cells, radius_cells + 1)
            for dc in range(-radius_cells, radius_cells + 1)
            if dr * dr + dc * dc <= radius_cells**2
        ]
        return np.array(offsets, dtype=int) if offsets else np.zeros((1, 2), dtype=int)

    def on_odom(self, msg):
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def _stamp(self, cells, value):
        rows, cols = cells[:, 0], cells[:, 1]
        keep = (rows >= 0) & (rows < self.n) & (cols >= 0) & (cols < self.n)
        self.occ[rows[keep], cols[keep]] = value

    def on_cloud(self, msg):
        if self.pose is None:
            return
        x, y, z, yaw = self.pose

        raw = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if raw.size == 0:
            return
        rng = np.linalg.norm(raw[:, :2], axis=1)
        raw = raw[rng < self.max_range]
        if raw.size == 0:
            return

        body = raw + self.offset[None, :]
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.column_stack(
            [
                x + c * body[:, 0] - s * body[:, 1],
                y + s * body[:, 0] + c * body[:, 1],
                z + body[:, 2],
            ]
        )
        world = world[(world[:, 2] > self.min_z) & (world[:, 2] < self.max_z)]
        if world.size == 0:
            return

        cells = np.floor((world[:, :2] - self.origin[None, :]) / self.res).astype(int)
        cells = np.unique(cells[:, ::-1], axis=0)  # (row, col)
        if len(cells):
            self._stamp((cells[:, None, :] + self.inflation[None, :, :]).reshape(-1, 2), 100)

        robot = np.floor((np.array([x, y]) - self.origin) / self.res).astype(int)[::-1]
        self._stamp(robot[None, :] + self.footprint, 0)

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = self.n
        msg.info.height = self.n
        msg.info.origin.position.x = float(self.origin[0])
        msg.info.origin.position.y = float(self.origin[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = self.occ.ravel().tolist()
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
