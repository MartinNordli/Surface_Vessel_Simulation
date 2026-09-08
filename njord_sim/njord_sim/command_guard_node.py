"""Single actuator authority; source/process freshness uses steady time."""
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')
        self.declare_parameters('', [('timeout_s', 0.5), ('max_thrust', 500.0),
                                     ('require_mission', True)])
        self.values = {}
        self.race_active = False
        self.race_received = 0.0
        self.create_subscription(Bool, '/njord/race_active', self.on_race,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(Twist, '/njord/actuator_forces', 1)
        for side in ('left', 'right'):
            self.create_subscription(Float64, f'/njord/thrusters/{side}/thrust',
                                     lambda m, s=side: self.command(s, m), 1)
        for component in ('planner', 'mission', 'navigation'):
            self.create_subscription(DiagnosticArray, f'/njord/{component}_status',
                                     lambda m, c=component: self.status(c, m), 1)
        self.create_timer(0.05, self.step, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def command(self, key, msg):
        self.values[key] = (time.monotonic(), msg.data if math.isfinite(msg.data) else None)

    def on_race(self, msg):
        self.race_active = msg.data
        self.race_received = time.monotonic()

    def status(self, key, msg):
        age = (self.get_clock().now().nanoseconds * 1e-9 -
               (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
        expected = 'njord/planner' if key == 'planner' else key
        matches = [s for s in msg.status if s.name == expected]
        valid = len(matches) == 1 and matches[0].level == DiagnosticStatus.OK
        valid = valid and -0.1 <= age <= 1.0
        self.values[key] = (time.monotonic(), True if valid else None)

    def step(self):
        keys = ['left', 'right', 'planner', 'navigation']
        if self.get_parameter('require_mission').value:
            keys.append('mission')
        now = time.monotonic()
        timeout = self.get_parameter('timeout_s').value
        valid = self.race_active and now-self.race_received <= timeout and all(k in self.values and now-self.values[k][0] <= timeout and
                    self.values[k][1] is not None for k in keys)
        msg = Twist()
        if valid:
            limit = self.get_parameter('max_thrust').value
            msg.linear.x = max(-limit, min(limit, self.values['left'][1]))
            msg.linear.y = max(-limit, min(limit, self.values['right'][1]))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
