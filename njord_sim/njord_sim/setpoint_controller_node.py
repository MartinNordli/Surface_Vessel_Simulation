"""Reference controller of setpoint courses: drive to the latest target pose.

ROS wrapper around ``setpoint_control_core.SetpointController``. Started by
dstar_demo.launch.py on a setpoint course (``course:=setpoints``) unless
``CONTROLLER=external``; a Control & Autonomy node replaces it by using the
same topics.

Subscribes:
    ``setpoint_topic`` (default constants.SETPOINT_TOPIC, ``/njord/setpoint``,
        ``geometry_msgs/PoseStamped``, reliable, volatile): target
        position and heading in ``map_frame``. The latest valid message is
        the target until another replaces it; it does not expire.
    ``odom_topic`` (default ``/njord/odometry``, ``nav_msgs/Odometry``):
        estimated pose in ``map_frame`` with ``child_frame_id == base_frame``
        and body-frame twist (surge, sway, yaw rate).

Publishes:
    ``thruster_topics`` (``std_msgs/Float64``, one per thruster, e.g.
        ``/thruster_1/command``): force in N along the thruster axis; the
        command guard forwards them.
    ``status_topic`` (default ``/njord/controller_status``,
        ``diagnostic_msgs/DiagnosticArray``): status ``controller`` at 10 Hz of
        simulation time; OK while odometry is valid and fresh, WARN otherwise.
        The guard requires it while this node is part of the run.

Parameters: topic names and frames; tuning, speed ceiling, thrust cap and
thruster layout come from algorithms.yaml ``setpoint_control`` and the vessel
file through the run's public parameters (``node_defaults`` when started on
its own).

Failure behaviour: zero thrust on every step without a target, without
odometry, with odometry older than ``stale_after_s`` (or from the future) in
simulation time, on a clock reset, and at once on an invalid message. A
target in another frame or with non-finite values is rejected and clears the
current one. Zero thrust does not stop the boat; it drifts.
"""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float64

from njord_sim.constants import BASE_FRAME, HEARTBEATS, MAP_FRAME, SETPOINT_TOPIC
from njord_sim.defaults import node_defaults
from njord_sim.geometry import stamp_seconds, yaw_from_quaternion
from njord_sim.planner_core import fresh
from njord_sim.setpoint_control_core import SetpointController

# Reliable and volatile: compatible with every reliable publisher, both the
# transient-local evaluator and send_setpoint.py and RViz's volatile 2D Goal
# Pose tool. The controller runs before the first target is issued.
SETPOINT_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)


def unit_quaternion(q):
    return abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) <= 1e-3


class SetpointControllerNode(Node):
    """ROS wrapper around ``SetpointController``; see the module docstring."""

    def __init__(self):
        super().__init__('setpoint_controller')
        self.declare_parameters('', [
            ('setpoint_topic', SETPOINT_TOPIC), ('odom_topic', '/njord/odometry'),
            ('status_topic', HEARTBEATS['controller'][0]),
            ('map_frame', MAP_FRAME), ('base_frame', BASE_FRAME),
            *node_defaults('setpoint_controller'),
        ])
        self.p = lambda name: self.get_parameter(name).value
        if min(self.p('control_hz'), self.p('stale_after_s')) <= 0:
            raise ValueError('control_hz and stale_after_s must be positive')
        count = len(self.p('thruster_topics'))
        if (len(self.p('thruster_positions')) != 3 * count or len(self.p('thruster_axes')) != 3 * count
                or len(self.p('thruster_forward_limits')) != count
                or len(self.p('thruster_reverse_limits')) != count):
            raise ValueError('thruster topics, geometry and limits must describe the same thrusters')
        settings = {key: self.p(key) for key in (
            'approach_radius_m', 'align_radius_m', 'kp_surge', 'kp_yaw', 'kd_yaw', 'kp_position',
            'kd_position', 'braking_deceleration_mps2', 'reaction_time_s', 'max_speed', 'max_thrust')}
        self.core = SetpointController(settings, self.p('thruster_positions'), self.p('thruster_axes'),
                                       self.p('thruster_forward_limits'), self.p('thruster_reverse_limits'))
        self.target = None      # (x, y, yaw) in map
        self.state = None       # ((x, y, yaw), (u, v, r))
        self.odom_stamp = None
        self.last_sim_time = None
        self.create_subscription(PoseStamped, self.p('setpoint_topic'), self.on_setpoint, SETPOINT_QOS)
        self.create_subscription(Odometry, self.p('odom_topic'), self.on_odom, qos_profile_sensor_data)
        self.thrusters = [self.create_publisher(Float64, topic, 1) for topic in self.p('thruster_topics')]
        self.status_pub = self.create_publisher(DiagnosticArray, self.p('status_topic'), 1)
        # Control and heartbeat on the node clock (/clock simulation time).
        self.create_timer(1.0 / self.p('control_hz'), self.step)
        self.create_timer(0.1, self.heartbeat)
        self.get_logger().info(f'{"fully actuated" if self.core.fully_actuated else "underactuated"} '
                               f'vessel with {count} thrusters; waiting for {self.p("setpoint_topic")}')

    def on_setpoint(self, msg):
        """Accept a finite map-frame target; anything else clears the target and stops."""
        p, q = msg.pose.position, msg.pose.orientation
        if (msg.header.frame_id != self.p('map_frame')
                or not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)) or not unit_quaternion(q)):
            self.get_logger().warning('Rejected setpoint: needs finite position and unit orientation in '
                                      f'{self.p("map_frame")}')
            self.target = None
            self.stop()
            return
        self.target = (p.x, p.y, yaw_from_quaternion(q))
        self.core.reset()
        self.get_logger().info('New setpoint x=%.2f y=%.2f heading=%.1f deg' % (
            p.x, p.y, math.degrees(self.target[2])))

    def on_odom(self, msg):
        """Store pose and body velocity; stop at once on invalid odometry."""
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, q.x, q.y, q.z, q.w, twist.linear.x, twist.linear.y, twist.angular.z)
        if (msg.header.frame_id != self.p('map_frame') or msg.child_frame_id != self.p('base_frame')
                or not all(math.isfinite(v) for v in values) or not unit_quaternion(q)):
            self.state = self.odom_stamp = None
            self.stop()
            return
        self.state = ((p.x, p.y, yaw_from_quaternion(q)), (twist.linear.x, twist.linear.y, twist.angular.z))
        self.odom_stamp = stamp_seconds(msg.header.stamp)

    def now(self):
        """Simulation time (s); a clock reset forgets odometry and mode."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_sim_time is not None and now < self.last_sim_time:
            self.state = self.odom_stamp = None
            self.core.reset()
        self.last_sim_time = now
        return now

    def odometry_ok(self, now):
        return self.state is not None and fresh(now, self.odom_stamp, self.p('stale_after_s'))

    def stop(self):
        """Command zero thrust (N) on every thruster."""
        for publisher in self.thrusters:
            publisher.publish(Float64(data=0.0))

    def step(self):
        """One control cycle: zero thrust unless target and fresh odometry exist."""
        if self.target is None or not self.odometry_ok(self.now()):
            return self.stop()
        for publisher, thrust in zip(self.thrusters, self.core.thrusts(*self.state, self.target)):
            publisher.publish(Float64(data=thrust))

    def heartbeat(self):
        """Publish the controller status (OK while odometry is valid and fresh)."""
        now = self.now()
        ok = self.odometry_ok(now)
        status = DiagnosticStatus(name=HEARTBEATS['controller'][1], hardware_id='reference_autonomy',
                                  level=DiagnosticStatus.OK if ok else DiagnosticStatus.WARN,
                                  message=('holding no target' if self.target is None else self.core.mode)
                                  if ok else 'odometry missing or stale')
        values = [KeyValue(key='mode', value=self.core.mode if self.target else 'idle')]
        if ok and self.target is not None:
            (x, y, _), _ = self.state
            values.append(KeyValue(key='distance_m', value=f'{math.dist((x, y), self.target[:2]):.2f}'))
        status.values = values
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self.status_pub.publish(output)


def main():
    rclpy.init()
    node = SetpointControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
