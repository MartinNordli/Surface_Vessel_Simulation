"""Validation step 2: is the vessel model behaving like a boat?

Runs three open-loop manoeuvres and prints what the simulator actually did:

  straight   equal thrust on both sides   -> top speed and time to reach 95% of it
  turn       differential thrust          -> steady yaw rate and turning radius
  coast      thrust cut to zero           -> stopping distance

These are the same three numbers you can measure on the water in an afternoon.
Until simulated and measured agree, every result downstream of this is a
result about a simulator, not about your boat. Run it before the closed loop,
and re-run it after any change to the hull or thruster configuration.

Start the simulator service alone, then run this script on its ROS network.
The reference autonomy must be stopped so only this check commands thrusters.
The atomic force envelope is converted into newtons by the Gazebo watchdog.
"""

import math
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from dynamics_metrics import summarize

STRAIGHT_S = 40.0
TURN_S = 60.0
COAST_S = 40.0
THRUST = 300.0


class DynamicsCheck(Node):
    def __init__(self):
        super().__init__("dynamics_check")
        self.declare_parameter("odom_topic", "/wamv/ground_truth/odometry")
        self.declare_parameter("forces_topic", "/njord/actuator_forces")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self.forces = self.create_publisher(Twist, self.get_parameter("forces_topic").value, 1)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, qos_profile_sensor_data)

        self.samples = []
        self.t0 = None
        self.wall_start = time.monotonic()
        self.last_odom_wall = self.wall_start
        self.create_timer(0.05, self.step)
        self.create_timer(0.2, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.samples and t - self.t0 <= self.samples[-1][0]:
            return
        self.last_odom_wall = time.monotonic()
        if self.t0 is None:
            self.t0 = t
        self.samples.append(
            (
                t - self.t0,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
                msg.twist.twist.angular.z,
            )
        )

    def watchdog(self):
        if time.monotonic() - self.last_odom_wall > 10 or time.monotonic() - self.wall_start > 600:
            self.command(0.0, 0.0)
            self.get_logger().error("Dynamics check timed out waiting for advancing odometry")
            raise SystemExit(2)

    def command(self, left, right):
        # Transport envelope only: these fields are physical thrust in newtons,
        # consumed atomically by the simulator-side actuator watchdog.
        message = Twist()
        message.linear.x, message.linear.y = float(left), float(right)
        self.forces.publish(message)

    def step(self):
        if self.t0 is None:
            return
        t = self.samples[-1][0]
        if t < STRAIGHT_S:
            self.command(THRUST, THRUST)
        elif t < STRAIGHT_S + TURN_S:
            self.command(THRUST * 0.2, THRUST)
        elif t < STRAIGHT_S + TURN_S + COAST_S:
            self.command(0.0, 0.0)
        else:
            self.command(0.0, 0.0)
            self.report()
            raise SystemExit(0)

    def report(self):
        metrics = summarize(self.samples, STRAIGHT_S, TURN_S)
        metrics["thrust_per_side_n"] = THRUST
        print(json.dumps(metrics, indent=2, allow_nan=False))
        print("Compare with measured boat manoeuvres; these are simulator observations.")
        if not metrics["stopped_within_observation"]:
            print("Coast distance is a lower bound: the boat was still moving at the end.")


def main():
    rclpy.init()
    node = DynamicsCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.command(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
