"""Validation step 2: is the vessel model behaving like a boat?

Runs three open-loop manoeuvres and prints what the simulator actually did:

  straight   equal thrust on both sides   -> top speed and time to reach 95% of it
  turn       differential thrust          -> steady yaw rate and turning radius
  coast      thrust cut to zero           -> stopping distance

These are the same three numbers you can measure on the water in an afternoon.
Until simulated and measured agree, every result downstream of this is a
result about a simulator, not about your boat. Run it before the closed loop,
and re-run it after any change to the hull or thruster configuration.

    ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
    python3 validation/check_dynamics.py
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

STRAIGHT_S = 40.0
TURN_S = 60.0
COAST_S = 40.0
THRUST = 300.0


class DynamicsCheck(Node):
    def __init__(self):
        super().__init__("dynamics_check")
        self.declare_parameter("odom_topic", "/wamv/ground_truth/odometry")
        self.declare_parameter("left_topic", "/wamv/thrusters/left/thrust")
        self.declare_parameter("right_topic", "/wamv/thrusters/right/thrust")
        self.declare_parameter("use_sim_time", True)

        self.left = self.create_publisher(Float64, self.get_parameter("left_topic").value, 1)
        self.right = self.create_publisher(Float64, self.get_parameter("right_topic").value, 1)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)

        self.samples = []
        self.t0 = None
        self.create_timer(0.05, self.step)

    def on_odom(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
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

    def command(self, left, right):
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))

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
        straight = [s for s in self.samples if s[0] < STRAIGHT_S]
        turn = [s for s in self.samples if STRAIGHT_S + TURN_S * 0.5 < s[0] < STRAIGHT_S + TURN_S]
        coast = [s for s in self.samples if s[0] > STRAIGHT_S + TURN_S]

        top_speed = max(s[3] for s in straight)
        rise = next((s[0] for s in straight if s[3] > 0.95 * top_speed), float("nan"))
        yaw_rate = sum(s[4] for s in turn) / len(turn)
        radius = top_speed / abs(yaw_rate) if abs(yaw_rate) > 1e-6 else float("inf")
        entry_speed = coast[0][3]
        stopped = next((s for s in coast if s[3] < 0.05 * entry_speed), coast[-1])
        stopping = math.hypot(stopped[1] - coast[0][1], stopped[2] - coast[0][2])

        print("\n--- vessel dynamics as simulated ---")
        print(f"  top speed                {top_speed:6.2f} m/s   at {THRUST:.0f} N per thruster")
        print(f"  time to 95% of top speed {rise:6.1f} s")
        print(f"  steady yaw rate          {yaw_rate:6.3f} rad/s  at 20/100 differential")
        print(f"  turning radius           {radius:6.1f} m")
        print(f"  stopping distance        {stopping:6.1f} m     from {entry_speed:.2f} m/s")
        print("\nCompare these against the same manoeuvres on the real boat.")
        print("The turning radius is also the lower bound for your costmap inflation.")


def main():
    rclpy.init()
    node = DynamicsCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.command(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
