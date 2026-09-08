"""Compare lidar with a known vertical cylinder using timestamped 3D TF.

Run with reference autonomy disabled and a stationary vessel. Ground-truth
odometry is interpolated in a private buffer; sensor extrinsics come from TF.
Expected range is measured from the sensor origin, not the boat origin.

python3 validation/check_lidar.py --ros-args -p target:='[25.0, 7.0, 0.5]'
Use the resolved scenario's marker coordinate (seeds may shift it).
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

SAMPLES = 40


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid transform quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def translation(t):
    return np.array([t.x, t.y, t.z])


class LidarCheck(Node):
    def __init__(self):
        super().__init__("lidar_check")
        self.declare_parameters("", [
            ("points_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
            ("odom_topic", "/wamv/ground_truth/odometry"), ("body_frame", "base_link"),
            ("target", [25.0, 7.0, 0.5]), ("min_height_m", 0.4),
            ("max_error_m", 0.5), ("wall_timeout_s", 120.0)])
        self.target = np.asarray(self.get_parameter("target").value, dtype=float)
        self.body_frame = self.get_parameter("body_frame").value
        self.min_height = self.get_parameter("min_height_m").value
        self.errors, self.low_fractions = [], []
        self.sensor_tf = Buffer(cache_time=Duration(seconds=10))
        self.truth_tf = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.sensor_tf, self)
        self.last_stamp = None
        self.tf_drops = 0
        self.wall_start = time.monotonic()
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.get_parameter("points_topic").value, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.5, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def watchdog(self):
        if time.monotonic() - self.wall_start > self.get_parameter("wall_timeout_s").value:
            self.get_logger().error(f"Lidar validation timed out: {len(self.errors)} valid scans, {self.tf_drops} TF drops")
            raise SystemExit(2)

    def on_odom(self, message):
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "validation_world"
        transform.child_frame_id = self.body_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.truth_tf.set_transform(transform, "ground_truth_validation")

    def on_cloud(self, message):
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds == 0 or (self.last_stamp is not None and stamp.nanoseconds <= self.last_stamp):
            return
        try:
            wb = self.truth_tf.lookup_transform("validation_world", self.body_frame, stamp).transform
            bs = self.sensor_tf.lookup_transform(self.body_frame, message.header.frame_id, stamp).transform
        except TransformException:
            self.tf_drops += 1
            return
        self.last_stamp = stamp.nanoseconds
        raw = point_cloud2.read_points_numpy(message, field_names=("x", "y", "z"), skip_nans=True)
        if not raw.size:
            return
        raw = np.asarray(raw).reshape(-1, 3)
        raw = raw[np.isfinite(raw).all(axis=1)]
        if not raw.size:
            return
        r_wb, r_bs = rotation(wb.rotation), rotation(bs.rotation)
        sensor_origin = translation(wb.translation) + r_wb @ translation(bs.translation)
        world = raw @ (r_wb @ r_bs).T + sensor_origin
        distance_to_target = np.linalg.norm(world[:, :2] - self.target[:2], axis=1)
        near = (np.abs(distance_to_target - self.target[2]) < 0.4) & (world[:, 2] > self.min_height)
        if not np.any(near):
            return
        self.low_fractions.append(float(np.mean(world[:, 2] <= self.min_height)))
        measured = float(np.min(np.linalg.norm(world[near, :2] - sensor_origin[:2], axis=1)))
        expected = float(np.linalg.norm(self.target[:2] - sensor_origin[:2]) - self.target[2])
        self.errors.append(measured - expected)
        if len(self.errors) >= SAMPLES:
            errors = np.asarray(self.errors)
            print(f"Expected horizontal range from lidar: {expected:.3f} m")
            print(f"Error mean/std/max: {errors.mean():+.3f}/{errors.std():.3f}/{np.abs(errors).max():.3f} m")
            print(f"Scans: {len(errors)}, TF drops: {self.tf_drops}, low returns: {np.mean(self.low_fractions):.1%}")
            raise SystemExit(0 if np.abs(errors).max() <= self.get_parameter("max_error_m").value else 2)


def main():
    rclpy.init()
    node = LidarCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
