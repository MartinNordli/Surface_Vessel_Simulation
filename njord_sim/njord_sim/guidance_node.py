"""Line-of-sight path following with differential thrust.

The same control law as sandbox/headless_demo.py, so a difference in behaviour
between the two points at the vessel model rather than at the controller.

Gains here are for the stock VRX WAM-V and will need retuning for Njord's own
hull. Tune against validation/check_dynamics.py, not by eye in the GUI.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64

from njord_sim.mapper_node import yaw_from_quaternion


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Guidance(Node):
    def __init__(self):
        super().__init__("guidance")
        self.declare_parameters(
            "",
            [
                ("path_topic", "/njord/path"),
                ("odom_topic", "/wamv/ground_truth/odometry"),
                ("left_topic", "/wamv/thrusters/left/thrust"),
                ("right_topic", "/wamv/thrusters/right/thrust"),
                ("lookahead_m", 15.0),
                ("kp_yaw", 400.0),
                ("kd_yaw", 300.0),
                ("kp_surge", 200.0),
                ("max_speed", 3.0),
                ("max_thrust", 500.0),
                ("goal_tolerance_m", 6.0),
                ("control_hz", 20.0),
                ("stale_after_s", 1.0),
            ],
        )
        p = self.get_parameter
        self.lookahead = p("lookahead_m").value
        self.kp_yaw = p("kp_yaw").value
        self.kd_yaw = p("kd_yaw").value
        self.kp_surge = p("kp_surge").value
        self.max_speed = p("max_speed").value
        self.max_thrust = p("max_thrust").value
        self.goal_tolerance = p("goal_tolerance_m").value
        self.stale_after = p("stale_after_s").value

        self.path = None
        self.state = None
        self.last_odom = None

        self.create_subscription(Path, p("path_topic").value, self.on_path, 1)
        self.create_subscription(Odometry, p("odom_topic").value, self.on_odom, 10)
        self.left = self.create_publisher(Float64, p("left_topic").value, 1)
        self.right = self.create_publisher(Float64, p("right_topic").value, 1)
        self.create_timer(1.0 / p("control_hz").value, self.step)

    def on_path(self, msg):
        if msg.poses:
            self.path = np.array([[q.pose.position.x, q.pose.position.y] for q in msg.poses])

    def on_odom(self, msg):
        self.state = (
            np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]),
            yaw_from_quaternion(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )
        self.last_odom = self.get_clock().now()

    def stop(self):
        self.left.publish(Float64(data=0.0))
        self.right.publish(Float64(data=0.0))

    def step(self):
        if self.path is None or self.state is None:
            return self.stop()
        age = (self.get_clock().now() - self.last_odom).nanoseconds * 1e-9
        if age > self.stale_after:
            self.get_logger().warn(f"odometry stale by {age:.1f}s, stopping", throttle_duration_sec=2.0)
            return self.stop()

        position, yaw, surge, yaw_rate = self.state
        if np.linalg.norm(self.path[-1] - position) < self.goal_tolerance:
            return self.stop()

        distances = np.linalg.norm(self.path - position[None, :], axis=1)
        i = int(np.argmin(distances))
        while i < len(self.path) - 1 and np.linalg.norm(self.path[i] - position) < self.lookahead:
            i += 1
        target = self.path[i]

        error = wrap(math.atan2(target[1] - position[1], target[0] - position[0]) - yaw)
        moment = self.kp_yaw * error - self.kd_yaw * yaw_rate
        speed_setpoint = self.max_speed * max(0.25, 1.0 - abs(error) / (math.pi / 2))
        thrust = self.kp_surge * (speed_setpoint - surge)

        left = float(np.clip(thrust / 2 - moment / 2, -self.max_thrust, self.max_thrust))
        right = float(np.clip(thrust / 2 + moment / 2, -self.max_thrust, self.max_thrust))
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
