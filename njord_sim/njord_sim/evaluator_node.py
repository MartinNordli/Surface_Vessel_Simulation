"""Score ordered gate races using simulation-only ground truth and /clock.

A steady-clock timer ends the process if Gazebo never starts, freezes, or loses
odometry. Missing contact data cannot establish a collision-free run.
"""
import json
import math
import os
from pathlib import Path as FilePath
import sys
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64, Bool

from .scenario_core import RaceScorer, load_scenario, scenario_digest


class Evaluator(Node):
    def __init__(self):
        super().__init__("evaluator")
        self.declare_parameters("", [
            ("scenario_file", ""), ("odom_topic", "/wamv/ground_truth/odometry"),
            ("path_topic", "/njord/path"), ("contacts_topic", "/njord/contacts"),
            ("output", "outputs/run_metrics.json"), ("run_label", "run"),
            ("profile", "conservative"), ("wall_timeout_s", 600.0),
            ("odom_wall_timeout_s", 30.0),
            ("wait_for_ready", True),
            ("git_commit", os.environ.get("NJORD_IMAGE_SOURCE_COMMIT", "unknown")),
            ("image_source_digest", os.environ.get("NJORD_IMAGE_SOURCE_DIGEST", "unknown")),
            ("runner_git_commit", os.environ.get("RUNNER_GIT_COMMIT", "unknown")),
            ("image_identity", os.environ.get("IMAGE_ID", "unknown")),
        ])
        p = lambda name: self.get_parameter(name).value
        self.scenario = load_scenario(p("scenario_file"))
        self.scorer = RaceScorer(self.scenario)
        self.wall_start = time.monotonic()
        self.first_odom_wall = None
        self.last_odom_wall = None
        self.output = FilePath(p("output"))
        self.done = False
        self.exit_code = 2
        self.path_messages = 0
        self.latencies = []
        self.started = not p("wait_for_ready")
        self.readiness = {}
        self.latest_ground_truth = None
        self.contact_last_wall = None
        self.contact_last_stamp = None
        self.active_pub = self.create_publisher(Bool, "/njord/race_active",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for topic in ("mission_status", "planner_status", "navigation_status"):
            self.create_subscription(DiagnosticArray, "/njord/" + topic, self.on_readiness, 10)
        self.create_subscription(Odometry, p("odom_topic"), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Path, p("path_topic"), self.on_path, 10)
        self.create_subscription(Float64, "/njord/plan_ms", self.on_latency, 10)
        try:
            from ros_gz_interfaces.msg import Contacts
            self.create_subscription(Contacts, p("contacts_topic"), self.on_contacts, qos_profile_sensor_data)
        except ImportError:
            self.get_logger().warning("Contact message type unavailable; contact result will be null")
        self.create_timer(0.1, self.check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_readiness(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        for status in message.status:
            self.readiness[status.name] = (status.level == DiagnosticStatus.OK, stamp, time.monotonic())

    def contact_fresh(self):
        """Require both acquisition freshness and a live advancing stream."""
        if self.contact_last_stamp is None or self.contact_last_wall is None:
            return False
        age = self.get_clock().now().nanoseconds * 1e-9 - self.contact_last_stamp
        return 0 <= age <= 0.5 and time.monotonic() - self.contact_last_wall <= 0.5

    def ready(self):
        now_sim = self.get_clock().now().nanoseconds * 1e-9
        now_wall = time.monotonic()
        return self.latest_ground_truth is not None and self.last_odom_wall is not None and now_wall - self.last_odom_wall <= 0.5 and self.contact_fresh() and all(
            name in self.readiness and self.readiness[name][0]
            and 0 <= now_sim - self.readiness[name][1] <= 0.5
            and now_wall - self.readiness[name][2] <= 0.5
            for name in ("njord/planner", "mission", "navigation"))

    def on_path(self, _):
        self.path_messages += 1

    def on_latency(self, message):
        if math.isfinite(message.data) and message.data >= 0:
            self.latencies.append(message.data)

    def on_contacts(self, message):
        if self.done:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if (not 0 <= age <= 0.5
                or (self.contact_last_stamp is not None and stamp <= self.contact_last_stamp)):
            return  # Delayed, future and replayed messages cannot freshen evidence.
        self.contact_last_stamp, self.contact_last_wall = stamp, time.monotonic()
        def name(collision):
            return getattr(collision, "name", str(collision))
        involved = any("wamv" in name(c.collision1) or "wamv" in name(c.collision2)
                       for c in message.contacts)
        self.scorer.contact(involved)
        if involved:
            self.finish()

    def on_odom(self, message):
        if self.done:
            return
        wall = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.latest_ground_truth is None or stamp > self.latest_ground_truth[0]:
            self.last_odom_wall = wall
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pos = message.pose.pose.position
        self.latest_ground_truth = (stamp, pos.x, pos.y, yaw)
        if not self.started:
            return
        if self.first_odom_wall is None:
            self.first_odom_wall = wall
        self.scorer.update(stamp, pos.x, pos.y, yaw)
        if self.scorer.status != "running":
            self.finish()

    def check_timeout(self):
        if self.done:
            return
        now = time.monotonic()
        if not self.started and self.ready():
            self.started = True
            self.first_odom_wall = now
            self.scorer.update(*self.latest_ground_truth)
            self.get_logger().info("Navigation, mission and planner ready; race started")
        self.active_pub.publish(Bool(data=self.started and self.scorer.status == "running"))
        if now - self.wall_start >= self.get_parameter("wall_timeout_s").value:
            self.scorer.status = "wall_timeout"
        elif self.last_odom_wall is not None and now - self.last_odom_wall >= self.get_parameter("odom_wall_timeout_s").value:
            self.scorer.status = "odometry_timeout"
        elif self.started and not self.contact_fresh():
            self.scorer.status = "contact_monitor_timeout"
        elif self.scorer.start_time is not None:
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.scorer.start_time
            if elapsed >= self.scenario["timeout_s"]:
                self.scorer.elapsed = elapsed
                self.scorer.status = "simulation_timeout"
        if self.scorer.status != "running":
            self.finish()

    def finish(self):
        if self.done:
            return
        # Completion may arrive on odometry between watchdog timer ticks.
        if self.scorer.status == "completed" and not self.contact_fresh():
            self.scorer.status = "contact_monitor_timeout"
        metrics = self.scorer.metrics()
        elapsed_wall = time.monotonic() - self.wall_start
        simulation_wall = time.monotonic() - self.first_odom_wall if self.first_odom_wall else 0
        metrics.update({
            "label": self.get_parameter("run_label").value,
            "profile": self.get_parameter("profile").value,
            "seed": self.scenario["seed"], "environment": self.scenario["environment_name"],
            "scenario": self.scenario, "scenario_sha256": scenario_digest(self.scenario),
            "git_commit": self.get_parameter("git_commit").value,
            "image_source_digest": self.get_parameter("image_source_digest").value,
            "runner_git_commit": self.get_parameter("runner_git_commit").value,
            "image_identity": self.get_parameter("image_identity").value,
            "wall_time_s": elapsed_wall,
            "real_time_factor": self.scorer.elapsed / simulation_wall if simulation_wall > 0 else None,
            "path_messages": self.path_messages,
            "replans": len(self.latencies), "plan_samples": len(self.latencies),
            "max_plan_ms": max(self.latencies) if self.latencies else None,
            "mean_plan_ms": sum(self.latencies) / len(self.latencies) if self.latencies else None,
        })
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output.with_suffix(self.output.suffix + ".tmp")
        temporary.write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
        temporary.replace(self.output)
        self.done = True
        self.active_pub.publish(Bool(data=False))
        self.exit_code = 0 if self.scorer.status == "completed" else 2
        self.get_logger().info(f"Race {self.scorer.status}; metrics: {self.output}")


def main():
    rclpy.init()
    node = Evaluator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        node.scorer.status = "interrupted"
        node.finish()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
