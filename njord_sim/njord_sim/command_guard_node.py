"""Single actuator authority; source/process freshness uses steady time.

The command guard is the only node allowed to drive the thrusters. It
forwards the controller's thrust commands only while every required input is
fresh and healthy, and otherwise publishes zero force. It does not look at
paths, maps or odometry: controllers must zero their own commands on bad
input, and the planner, mission and sensor adapter must report problems in
their status heartbeats.

Subscribes:
    ``thruster_topics`` (one ``std_msgs/Float64`` per thruster, in vessel-file
        order, e.g. ``/thruster_1/command``): thrust command in N along the
        thruster's axis; non-finite values count as invalid.
    ``/njord/planner_status``, ``/njord/mission_status``,
        ``/njord/navigation_status`` (``diagnostic_msgs/DiagnosticArray``):
        health heartbeats. Each must hold exactly one status with the expected
        name (``njord/planner``, ``mission``, ``navigation``) and level OK.
    ``/njord/race_active`` (``std_msgs/Bool``, transient local): evaluator
        heartbeat; thrust is only allowed while it is true and fresh.

Publishes (20 Hz, steady-time timer):
    ``/njord/actuator_forces`` (``ros_gz_interfaces/Float32Array``): one force
        in N per thruster in ``data``, in vessel-file order (bridged to
        ``gz.msgs.Float_V``). Consumed by the Gazebo actuator watchdog or Njord
        physics plugin, which reject a length that does not match the vessel.

Parameters:
    ``thruster_topics``, ``timeout_s`` (max steady-time age of every input,
    s), ``max_thrust`` (N), ``forward_limits`` / ``reverse_limits``
    (per-thruster limits in N, reverse as magnitudes), from the vessel file
    and ``algorithms.yaml`` through ``node_defaults``; ``require_mission``
    (default True) makes the mission heartbeat mandatory.

Validity rules and failure behaviour:
    * Receipt freshness uses steady wall time (``time.monotonic``) and a
      steady-clock timer, so the guard keeps running and zeroes thrust even
      if /clock stops or a publishing process hangs.
    * Status content freshness uses /clock simulation time: the status
      header stamp must be at most 1.0 s old and at most 0.1 s in the future.
    * If any required input (every thruster command included) is missing,
      invalid or older than ``timeout_s``, or the race is not active, all
      forces are published as zero. They are published, not withheld, so
      downstream actuators see an explicit zero command.
    * Valid commands are clamped to the forward/reverse limits and
      ``max_thrust``. On shutdown a final zero command is published.
"""
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from ros_gz_interfaces.msg import Float32Array
from std_msgs.msg import Float64, Bool

from njord_sim.defaults import node_defaults


class CommandGuard(Node):
    """Gate between controller thrust commands and the actuators."""

    def __init__(self, **kwargs):
        # kwargs go to rclpy's Node, e.g. parameter_overrides in tests.
        super().__init__('command_guard', **kwargs)
        # thruster_topics, timeout_s, max_thrust and per-thruster limits come
        # from the vessel, algorithms.yaml and constants.py (see defaults.py).
        self.declare_parameters('', [*node_defaults('command_guard'), ('require_mission', True)])
        self.topics = list(self.get_parameter('thruster_topics').value)
        if (not self.topics or len(set(self.topics)) != len(self.topics)
                or len(self.get_parameter('forward_limits').value) != len(self.topics)
                or len(self.get_parameter('reverse_limits').value) != len(self.topics)):
            raise ValueError('thruster_topics must be unique and match the per-thruster limits')
        # Latest input per key (thruster index, 'planner', 'mission',
        # 'navigation'): (steady receipt time in s, value or None if invalid).
        self.values = {}
        self.status_stamps = {}
        self.last_sim_time = None
        self.race_active = False
        self.race_received = 0.0  # steady receipt time (s) of the race heartbeat
        self.create_subscription(Bool, '/njord/race_active', self.on_race,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(Float32Array, '/njord/actuator_forces', 1)
        for index, topic in enumerate(self.topics):
            self.create_subscription(Float64, topic, lambda m, i=index: self.command(i, m), 1)
        for component in ('planner', 'mission', 'navigation'):
            self.create_subscription(DiagnosticArray, f'/njord/{component}_status',
                                     lambda m, c=component: self.status(c, m), 1)
        # Steady-time timer: the guard must keep deciding (and zeroing) even
        # when simulation time is paused or no longer advancing.
        self.create_timer(0.05, self.step, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def command(self, key, msg):
        """Record a thrust command in N; non-finite values are stored as invalid."""
        self.values[key] = (time.monotonic(), msg.data if math.isfinite(msg.data) else None)

    def on_race(self, msg):
        """Record the evaluator's race-active heartbeat and its steady receipt time."""
        self.race_active = msg.data
        self.race_received = time.monotonic()

    def status(self, key, msg):
        """Record whether a health heartbeat is OK and its source stamp is current."""
        # Age of the status content in /clock simulation time (s).
        now = self.observe_clock()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now - stamp
        expected = 'njord/planner' if key == 'planner' else key
        matches = [s for s in msg.status if s.name == expected]
        valid = len(matches) == 1 and matches[0].level == DiagnosticStatus.OK
        # Allow 0.1 s of clock skew into the future, and at most 1 s of age.
        valid = valid and -0.1 <= age <= 1.0
        if stamp <= self.status_stamps.get(key, -math.inf):
            if not valid:
                self.values[key] = (time.monotonic(), None)
            return
        self.status_stamps[key] = stamp
        self.values[key] = (time.monotonic(), True if valid else None)

    def observe_clock(self):
        """Clear every previous-run input when simulation time moves backward."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_sim_time is not None and now < self.last_sim_time:
            self.values.clear()
            self.status_stamps.clear()
            self.race_active = False
        self.last_sim_time = now
        return now

    def forces(self, valid):
        """Forces message: clamped commands in N if ``valid``, otherwise zeros."""
        msg = Float32Array()
        msg.data = [0.0] * len(self.topics)
        if valid:
            limit = self.get_parameter('max_thrust').value
            forward = self.get_parameter('forward_limits').value
            reverse = self.get_parameter('reverse_limits').value
            # Clamp each thruster to [-min(max_thrust, reverse), min(max_thrust, forward)] N.
            msg.data = [max(-min(limit, reverse[i]), min(limit, forward[i], self.values[i][1]))
                        for i in range(len(self.topics))]
        return msg

    def step(self):
        """Publish clamped commands if every required input is valid, else zero."""
        self.observe_clock()
        keys = [*range(len(self.topics)), 'planner', 'navigation']
        if self.get_parameter('require_mission').value:
            keys.append('mission')
        now = time.monotonic()
        timeout = self.get_parameter('timeout_s').value
        # Every required input must have been received within timeout_s of
        # steady time and be valid, and the race must be active.
        valid = (self.race_active and now-self.race_received <= timeout
                 and all(k in self.values and now-self.values[k][0] <= timeout
                         and self.values[k][1] is not None for k in keys))
        self.pub.publish(self.forces(valid))


def main():
    rclpy.init()
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(node.forces(False))
        node.destroy_node()
        rclpy.try_shutdown()
