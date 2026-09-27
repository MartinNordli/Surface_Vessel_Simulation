"""Single actuator authority: data freshness on simulation time, liveness on steady time.

The command guard is the only node allowed to drive the thrusters. It
forwards the controller's thrust commands only while every required input is
fresh and healthy, and otherwise publishes zero force. It does not look at
paths, maps or odometry: controllers must zero their own commands on bad
input, and the planner, mission and sensor adapter must report problems in
their status heartbeats. The decision itself is ``guard_core.GuardCore``.

Subscribes:
    ``thruster_topics`` (one ``std_msgs/Float64`` per thruster, in vessel-file
        order, e.g. ``/thruster_1/command``): thrust command in N along the
        thruster's axis; non-finite values count as invalid.
    ``/njord/{navigation,planner,mission}_status``
        (``diagnostic_msgs/DiagnosticArray``): health heartbeats for the keys in
        ``required_status``. Each must hold exactly one status with the
        expected name (``navigation``, ``njord/planner``, ``mission``) and level OK.
    ``/njord/race_active`` (``std_msgs/Bool``, transient local): the
        evaluator's run-active signal, required if ``require_race_active``.

Publishes:
    ``/njord/actuator_forces`` (``ros_gz_interfaces/Float32Array``): one force
        in N per thruster in ``data``, in vessel-file order (bridged to
        ``gz.msgs.Float_V``). Published as soon as every thruster has a new
        command and all inputs are valid, so the controller's own rate passes
        through without a guard-imposed rate or delay. Explicit zeros are
        published on a 20 Hz steady-time timer while inputs are invalid, and
        immediately when a heartbeat invalidates a driving state. Consumed by
        the Gazebo actuator watchdog or Njord physics plugin, which reject a
        length that does not match the vessel.
    ``/njord/guard_status`` (``diagnostic_msgs/DiagnosticArray``, 20 Hz steady
        time): status ``command_guard``, OK while thrust may pass, otherwise
        WARN with the reason, e.g. ``planner status stale in simulation time``.

Parameters:
    ``thruster_topics``, ``timeout_s`` (simulation-time freshness of every
    input, s), ``liveness_s`` (steady-time process liveness, s),
    ``max_thrust`` (N), ``forward_limits`` / ``reverse_limits`` (per-thruster
    limits in N, reverse as magnitudes), from the vessel file, algorithms.yaml
    and constants.py through ``node_defaults``; ``required_status`` (default
    all three heartbeats) and ``require_race_active`` (default True).

Validity rules and failure behaviour:
    * An input is stale when it was received more than ``timeout_s`` ago in
      simulation time, or more than ``liveness_s`` ago in steady time. The
      steady-clock timer keeps deciding (and zeroing) even if /clock stops.
    * Status content must be at most 1.0 s old and at most 0.1 s in the future
      in simulation time. Simulation time moving backwards clears all inputs.
    * Valid commands are clamped to the forward/reverse limits and
      ``max_thrust``. On shutdown a final zero command is published.
"""
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from ros_gz_interfaces.msg import Float32Array
from std_msgs.msg import Float64, Bool

from njord_sim.defaults import node_defaults
from njord_sim.guard_core import GuardCore

# Heartbeat key -> (topic, expected DiagnosticStatus name).
STATUS_TOPICS = {'navigation': ('/njord/navigation_status', 'navigation'),
                 'planner': ('/njord/planner_status', 'njord/planner'),
                 'mission': ('/njord/mission_status', 'mission')}


class CommandGuard(Node):
    """Gate between controller thrust commands and the actuators."""

    def __init__(self, **kwargs):
        # kwargs go to rclpy's Node, e.g. parameter_overrides in tests.
        super().__init__('command_guard', **kwargs)
        # thruster_topics, timeouts, max_thrust and per-thruster limits come
        # from the vessel, algorithms.yaml and constants.py (see defaults.py).
        self.declare_parameters('', [*node_defaults('command_guard'),
                                     ('required_status', list(STATUS_TOPICS)),
                                     ('require_race_active', True)])
        p = lambda name: self.get_parameter(name).value
        self.topics = list(p('thruster_topics'))
        required = list(p('required_status'))
        if len(set(self.topics)) != len(self.topics) or set(required) - set(STATUS_TOPICS):
            raise ValueError('thruster_topics must be unique and required_status a subset of '
                             f'{sorted(STATUS_TOPICS)}')
        self.core = GuardCore(len(self.topics), required, p('require_race_active'), p('timeout_s'),
                              p('liveness_s'), p('forward_limits'), p('reverse_limits'), p('max_thrust'))
        self.driving = False  # the last published forces were a valid command
        self.create_subscription(Bool, '/njord/race_active', self.on_race,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(Float32Array, '/njord/actuator_forces', 1)
        self.status_pub = self.create_publisher(DiagnosticArray, '/njord/guard_status', 1)
        for index, topic in enumerate(self.topics):
            self.create_subscription(Float64, topic, lambda m, i=index: self.command(i, m), 1)
        for key in required:
            topic, name = STATUS_TOPICS[key]
            self.create_subscription(DiagnosticArray, topic, lambda m, k=key, n=name: self.status(k, n, m), 1)
        # Steady-time timer: the guard must keep deciding (and zeroing) even
        # when simulation time is paused or no longer advancing.
        self.create_timer(0.05, self.step, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):
        """(simulation time from /clock, steady time) in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9, time.monotonic()

    def publish(self, valid):
        """Publish forces: the clamped commands if ``valid``, otherwise zeros."""
        msg = Float32Array()
        msg.data = self.core.forces(valid)
        self.pub.publish(msg)
        self.driving = valid

    def command(self, index, msg):
        """Record a thrust command; forward the set once every thruster is new."""
        sim, wall = self.now()
        self.core.command(index, msg.data, sim, wall)
        if self.core.take_complete_set():
            self.publish(self.core.decide(sim, wall)[0])

    def on_race(self, msg):
        """Record the evaluator's run-active signal."""
        self.core.set_run_active(msg.data, time.monotonic())
        self.stop_if_invalid()

    def status(self, key, name, msg):
        """Record whether a heartbeat holds exactly one OK status named ``name``."""
        sim, wall = self.now()
        matches = [s for s in msg.status if s.name == name]
        ok = len(matches) == 1 and matches[0].level == DiagnosticStatus.OK
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.core.status(key, ok, stamp, sim, wall)
        self.stop_if_invalid()

    def stop_if_invalid(self):
        """Zero at once when a driving state became invalid, not at the next tick."""
        if self.driving and not self.core.decide(*self.now())[0]:
            self.publish(False)

    def step(self):
        """Steady tick: publish zeros while invalid, and the guard status."""
        valid, reason = self.core.decide(*self.now())
        if not valid:
            self.publish(False)
        status = DiagnosticArray()
        status.header.stamp = self.get_clock().now().to_msg()
        status.status = [DiagnosticStatus(name='command_guard', hardware_id='njord_sim',
                                          level=DiagnosticStatus.OK if valid else DiagnosticStatus.WARN,
                                          message=reason)]
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(False)
        node.destroy_node()
        rclpy.try_shutdown()
