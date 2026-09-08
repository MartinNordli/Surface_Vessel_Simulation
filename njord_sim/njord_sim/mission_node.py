"""Sensor-only ordered red-left/green-right gate mission.

A bounded remembered crossing handles gates leaving the forward camera FOV;
actual camera frame freshness, odometry and downstream lidar guarding remain
required. No scenario coordinates or ground-truth topics are consumed.
"""
import math
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from njord_sim.geometry import stamp_seconds, yaw_from_quaternion
from njord_sim.perception_core import choose_gate


class Mission(Node):
    def __init__(self):
        super().__init__("mission")
        self.declare_parameters("", [
            ("buoys_topic", "/njord/buoys"), ("odom_topic", "/njord/odometry"),
            ("goal_topic", "/njord/goal"), ("status_topic", "/njord/mission_status"),
            ("map_frame", "map"), ("initial_heading_rad", 0.0), ("expected_gates", 3),
            ("min_gate_width_m", 8.0), ("max_gate_width_m", 30.0),
            ("approach_m", 5.0), ("exit_m", 6.0), ("arrival_tolerance_m", 2.0),
            ("detection_max_age_s", 2.0), ("camera_max_age_s", 0.5),
            ("odometry_max_age_s", 0.5), ("crossing_memory_s", 45.0),
            ("crossing_entry_m", 15.0),
        ])
        self.p = lambda name: self.get_parameter(name).value
        self.heading = self.p("initial_heading_rad")
        self.position = self.odom_stamp = self.camera_stamp = None
        self.observations = {}  # source camera/track id -> (color, position, source stamp)
        self.gate = None
        self.gate_seen = None
        self.phase = "seek"
        self.passed = []
        self.crossed = False
        self.previous_signed = None
        self.last_clock = None
        self.goal_pub = self.create_publisher(PoseStamped, self.p("goal_topic"), 1)
        self.status_pub = self.create_publisher(DiagnosticArray, self.p("status_topic"), 1)
        self.create_subscription(Odometry, self.p("odom_topic"), self.on_odom, 10)
        self.create_subscription(Detection3DArray, self.p("buoys_topic"), self.on_buoys, 10)
        self.create_timer(0.1, self.step)

    def on_odom(self, msg):
        if msg.header.frame_id != self.p("map_frame"):
            return
        stamp = stamp_seconds(msg.header.stamp)
        if self.odom_stamp is not None and stamp <= self.odom_stamp:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if (not np.isfinite([p.x, p.y]).all() or not np.isfinite(quaternion).all()
                or abs(float(quaternion@quaternion)-1.0) > 1e-3):
            return
        self.position, self.odom_stamp = np.array([p.x, p.y]), stamp
        if self.gate is None and not self.passed:
            self.heading = yaw_from_quaternion(q)

    def on_buoys(self, msg):
        if msg.header.frame_id != self.p("map_frame"):
            return
        stamp = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        if not 0 <= now-stamp <= self.p("camera_max_age_s"):
            return
        self.camera_stamp = max(stamp, self.camera_stamp or stamp)
        for detection in msg.detections:
            if not detection.results:
                continue
            hypothesis = max(detection.results, key=lambda result: result.hypothesis.score).hypothesis
            if hypothesis.class_id not in ("red", "green") or hypothesis.score < 0.35:
                continue
            p = detection.bbox.center.position
            point = np.array([p.x, p.y])
            if not np.isfinite(point).all():
                continue
            old = self.observations.get(detection.id)
            if old is None or stamp > old[2]:
                self.observations[detection.id] = (hypothesis.class_id, point, stamp)

    def status(self, level, message):
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        item = DiagnosticStatus()
        item.name, item.hardware_id = "mission", "reference_autonomy"
        item.level, item.message = level, message
        item.values = [KeyValue(key="phase", value=self.phase),
                       KeyValue(key="gates_passed", value=str(len(self.passed)))]
        output.status = [item]
        self.status_pub.publish(output)

    def step(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.position = self.odom_stamp = self.camera_stamp = None
            self.observations.clear()
            self.gate = self.gate_seen = None
            self.passed = []
            self.phase, self.crossed, self.previous_signed = "seek", False, None
            self.heading = self.p("initial_heading_rad")
        self.last_clock = now
        if len(self.passed) >= self.p("expected_gates"):
            return self.status(DiagnosticStatus.WARN, "complete")
        if self.odom_stamp is None or not 0 <= now-self.odom_stamp <= self.p("odometry_max_age_s"):
            return self.status(DiagnosticStatus.ERROR, "odometry unavailable or stale")
        if self.camera_stamp is None or not 0 <= now-self.camera_stamp <= self.p("camera_max_age_s"):
            return self.status(DiagnosticStatus.ERROR, "camera fusion unavailable or stale")
        self.observations = {key: value for key, value in self.observations.items()
                             if 0 <= now-value[2] <= self.p("detection_max_age_s")}
        # The overlapping cameras may report the same buoy with different IDs.
        observations = []
        for color, point, stamp in sorted(self.observations.values(), key=lambda value: -value[2]):
            if not any(c == color and np.linalg.norm(point-p) < 2.0 for c, p, _ in observations):
                observations.append((color, point, stamp))
        if self.gate is None:
            self.gate = choose_gate([(c, p) for c, p, _ in observations], self.position, self.heading,
                                    self.passed, self.p("min_gate_width_m"), self.p("max_gate_width_m"))
            if self.gate is None:
                return self.status(DiagnosticStatus.ERROR, "no unambiguous observed gate")
            # Preserve actual source age, even when selected from the short track cache.
            self.gate_seen = min(stamp for color, point, stamp in observations
                                 if np.linalg.norm(point-(self.gate.red if color == "red" else self.gate.green)) < 2.0)
            self.phase, self.crossed = "approach", False
            self.previous_signed = float((self.position-self.gate.center)@self.gate.forward)
        red_stamps = [s for c, p, s in observations if c == "red" and np.linalg.norm(p-self.gate.red) < 2.5]
        green_stamps = [s for c, p, s in observations if c == "green" and np.linalg.norm(p-self.gate.green) < 2.5]
        if red_stamps and green_stamps:
            self.gate_seen = max(self.gate_seen, min(max(red_stamps), max(green_stamps)))
        offset = self.position-self.gate.center
        signed = float(offset@self.gate.forward)
        lateral = abs(float(offset@np.array([-self.gate.forward[1], self.gate.forward[0]])))
        half_width = np.linalg.norm(self.gate.red-self.gate.green)/2
        memory = self.p("crossing_memory_s") if -self.p("crossing_entry_m") <= signed <= self.p("exit_m")+2.0 and lateral < half_width-1.0 else self.p("detection_max_age_s")
        if now-self.gate_seen > memory:
            return self.status(DiagnosticStatus.ERROR, "tracked gate expired")
        before = self.gate.center-self.p("approach_m")*self.gate.forward
        after = self.gate.center+self.p("exit_m")*self.gate.forward
        if self.phase == "approach" and (np.linalg.norm(self.position-before) <= self.p("arrival_tolerance_m") or signed >= -self.p("approach_m")):
            self.phase = "cross"
        if self.previous_signed is not None and self.previous_signed < 0 <= signed and lateral < half_width-1.0:
            self.crossed = True
        self.previous_signed = signed
        if self.phase == "cross" and self.crossed and np.linalg.norm(self.position-after) <= self.p("arrival_tolerance_m"):
            self.passed.append(self.gate.center)
            self.heading = math.atan2(self.gate.forward[1], self.gate.forward[0])
            self.gate, self.phase = None, "seek"
            return self.status(DiagnosticStatus.WARN, "gate crossed; selecting next")
        if signed > self.p("exit_m")+2 and not self.crossed:
            return self.status(DiagnosticStatus.ERROR, "gate crossing missed")
        target = before if self.phase == "approach" else after
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = self.p("map_frame"), self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = map(float, target)
        yaw = math.atan2(self.gate.forward[1], self.gate.forward[0])
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.goal_pub.publish(goal)
        self.status(DiagnosticStatus.OK, "valid")


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
