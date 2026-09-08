"""Validation step 3: does the simulated lidar measure what is actually there?

Place a known object at a known position, hold the boat still, and compare the
closest lidar return against the distance you can compute from the world file.
If those disagree, the occupancy grid is wrong and every planner result built
on it is meaningless, however convincing the plot looks.

Also reports the fraction of returns that fall below the wave-rejection
height, which is the number that tells you whether your height filter is
throwing away real obstacles or letting wave tops through as buoys.

    ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
    python3 validation/check_lidar.py --ros-args -p target:="[-470.0, 210.0, 1.5]"
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

SAMPLES = 40


class LidarCheck(Node):
    def __init__(self):
        super().__init__("lidar_check")
        self.declare_parameter("points_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points")
        self.declare_parameter("odom_topic", "/wamv/ground_truth/odometry")
        self.declare_parameter("target", [0.0, 0.0, 1.0])  # x, y, radius in world frame
        self.declare_parameter("sensor_offset", [1.0, 0.0, 2.0])
        self.declare_parameter("min_height_m", 0.4)
        self.declare_parameter("use_sim_time", True)

        self.target = np.array(self.get_parameter("target").value, dtype=float)
        self.offset = np.array(self.get_parameter("sensor_offset").value, dtype=float)
        self.min_height = self.get_parameter("min_height_m").value

        self.pose = None
        self.errors = []
        self.low_fractions = []

        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)
        self.create_subscription(PointCloud2, self.get_parameter("points_topic").value, self.on_cloud, 5)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw)

    def on_cloud(self, msg):
        if self.pose is None or len(self.errors) >= SAMPLES:
            return
        x, y, z, yaw = self.pose
        raw = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if raw.size == 0:
            return

        body = raw + self.offset[None, :]
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.column_stack(
            [x + c * body[:, 0] - s * body[:, 1], y + s * body[:, 0] + c * body[:, 1], z + body[:, 2]]
        )
        self.low_fractions.append(float(np.mean(world[:, 2] <= self.min_height)))

        distance_to_target = np.linalg.norm(world[:, :2] - self.target[None, :2], axis=1)
        near = distance_to_target < self.target[2] + 2.0
        if not np.any(near):
            return

        measured = float(np.min(np.linalg.norm(world[near][:, :2] - np.array([[x, y]]), axis=1)))
        expected = float(np.linalg.norm(self.target[:2] - np.array([x, y])) - self.target[2])
        self.errors.append(measured - expected)

        if len(self.errors) >= SAMPLES:
            self.report(expected)

    def report(self, expected):
        errors = np.array(self.errors)
        print("\n--- simulated lidar against world geometry ---")
        print(f"  expected range to target   {expected:7.2f} m")
        print(f"  mean range error           {errors.mean():+7.3f} m")
        print(f"  std of range error         {errors.std():7.3f} m")
        print(f"  worst error                {np.abs(errors).max():7.3f} m over {SAMPLES} scans")
        print(f"  returns below {self.min_height:.1f} m height  {np.mean(self.low_fractions) * 100:5.1f} %")
        print("\nA bias of more than a cell width means the map is offset from the world.")
        raise SystemExit(0)


def main():
    rclpy.init()
    node = LidarCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
