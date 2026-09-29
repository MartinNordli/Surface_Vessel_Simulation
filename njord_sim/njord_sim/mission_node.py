"""ROS wrapper of the sensor-only gate mission (logic in mission_core.GateMission).

The mission finds the next red-left/green-right gate in the fused buoy
detections and publishes goals that lead the vessel through it; when no gate
is in view it searches, and after a missed gate it retries from beside the
buoy (phases and rules: mission_core). No scenario coordinates or
ground-truth topics are consumed.

Subscribes:
    ``buoys_topic`` (default ``/njord/buoys``, ``vision_msgs/Detection3DArray``):
        fused buoy detections in ``map_frame``, stamped with image acquisition
        time. Each array (even an empty one) proves the camera pipeline is
        alive; detections need a stable ``id``, class ``red``/``green`` and
        score >= 0.35.
    ``odom_topic`` (default ``/njord/odometry``, ``nav_msgs/Odometry``):
        estimated vessel pose in ``map_frame``.

Publishes (at 10 Hz on /clock simulation time):
    ``goal_topic`` (default ``/njord/goal``, ``geometry_msgs/PoseStamped``):
        current goal in ``map_frame``, oriented along the gate's forward
        direction (or the search bearing). Republished every step while valid
        (the planner ignores an unchanged goal).
    ``status_topic`` (default ``/njord/mission_status``,
        ``diagnostic_msgs/DiagnosticArray``): one status named ``mission``
        with ``phase`` and ``gates_passed`` values. OK only when a goal was
        just published (``valid``, ``searching``, ``retrying gate``); WARN for
        ``complete``; ERROR for stale inputs, no gate yet or none found while
        searching, an expired gate or a missed crossing without retries left.

Parameters (m, s, rad):
    ``expected_gates`` (number of gates, not their positions),
    ``initial_heading_rad``, and from algorithms.yaml ``mission`` (through
    ``node_defaults``): gate width bounds, ``approach_m``, ``exit_m``,
    ``arrival_tolerance_m``, freshness and crossing-memory limits, and the
    search and retry settings; ``camera_max_age_s`` and ``odometry_max_age_s``
    from ``navigation.stale_after_s``.

Failure behaviour:
    Any non-OK status stops the boat: the command guard requires a fresh OK
    mission status before it forwards thrust. No goal is published while
    odometry or camera input is stale (by /clock simulation time). If the
    simulation clock jumps backwards, all mission state is reset.
"""
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from njord_sim.defaults import node_defaults
from njord_sim.geometry import estimate_initialized, stamp_seconds, yaw_from_quaternion
from njord_sim.mission_core import GateMission, OK

LEVELS = {0: DiagnosticStatus.OK, 1: DiagnosticStatus.WARN, 2: DiagnosticStatus.ERROR}


class Mission(Node):
    """Gate-sequencing mission node; see the module docstring."""

    def __init__(self):
        super().__init__("mission")
        self.declare_parameters("", [
            ("buoys_topic", "/njord/buoys"), ("odom_topic", "/njord/odometry"),
            ("goal_topic", "/njord/goal"), ("status_topic", "/njord/mission_status"),
            ("map_frame", "map"), ("initial_heading_rad", 0.0), ("expected_gates", 3),
            *node_defaults("mission"),
        ])
        self.p = lambda name: self.get_parameter(name).value
        settings = {name: self.p(name) for name, _ in node_defaults("mission")}
        self.core = GateMission(self.p("expected_gates"), self.p("initial_heading_rad"), settings)
        self.goal_pub = self.create_publisher(PoseStamped, self.p("goal_topic"), 1)
        self.status_pub = self.create_publisher(DiagnosticArray, self.p("status_topic"), 1)
        self.create_subscription(Odometry, self.p("odom_topic"), self.on_odom, 10)
        self.create_subscription(Detection3DArray, self.p("buoys_topic"), self.on_buoys, 10)
        self.create_timer(0.1, self.step)

    def on_odom(self, msg):
        """Pass a valid, initialized map-frame pose to the mission; others are ignored.

        An estimate whose x or y variance is 4 m^2 or more is still starting
        up (the navigation status reports it the same way) and can be far off;
        goals and gate crossings must not be derived from it. Ignored odometry
        is not cleared here; it ages out and the mission reports it as stale.
        """
        if msg.header.frame_id != self.p("map_frame") or not estimate_initialized(msg.pose.covariance):
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if (not np.isfinite([p.x, p.y]).all() or not np.isfinite(quaternion).all()
                or abs(float(quaternion@quaternion)-1.0) > 1e-3):
            return
        self.core.odometry((p.x, p.y), yaw_from_quaternion(q), stamp_seconds(msg.header.stamp))

    def on_buoys(self, msg):
        """Pass a detection array's confident red/green buoys to the mission."""
        if msg.header.frame_id != self.p("map_frame"):
            return
        detections = []
        for detection in msg.detections:
            if not detection.results:
                continue
            # Use the most confident class hypothesis for each detection.
            hypothesis = max(detection.results, key=lambda result: result.hypothesis.score).hypothesis
            if hypothesis.class_id not in ("red", "green") or hypothesis.score < 0.35:
                continue
            point = np.array([detection.bbox.center.position.x, detection.bbox.center.position.y])
            if np.isfinite(point).all():
                detections.append((detection.id, hypothesis.class_id, point))
        self.core.camera(stamp_seconds(msg.header.stamp), self.get_clock().now().nanoseconds*1e-9, detections)

    def status(self, level, message):
        """Publish the mission heartbeat with a mission_core level and text."""
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        item = DiagnosticStatus()
        item.name, item.hardware_id = "mission", "reference_autonomy"
        item.level, item.message = LEVELS[level], message
        item.values = [KeyValue(key="phase", value=self.core.phase),
                       KeyValue(key="gates_passed", value=str(len(self.core.passed)))]
        output.status = [item]
        self.status_pub.publish(output)

    def step(self):
        """Run one 10 Hz mission cycle and publish its goal and status."""
        decision = self.core.step(self.get_clock().now().nanoseconds*1e-9)
        if decision.level == OK:
            goal = PoseStamped()
            goal.header.frame_id, goal.header.stamp = self.p("map_frame"), self.get_clock().now().to_msg()
            x, y, yaw = decision.goal
            goal.pose.position.x, goal.pose.position.y = x, y
            goal.pose.orientation.z, goal.pose.orientation.w = float(np.sin(yaw/2)), float(np.cos(yaw/2))
            self.goal_pub.publish(goal)
        self.status(decision.level, decision.message)


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
