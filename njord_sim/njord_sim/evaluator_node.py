"""Scores a run against ground truth and writes it to JSON.

This node is the argument. A video of a boat avoiding a buoy convinces nobody
for long; a table of clearance, path efficiency and replan latency across
repeated runs is something the team can act on and can regress against.

Obstacle positions are given as a parameter because they come from the world
file, not from any topic. Keep them in sync with the world you launch.
"""

import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64


class Evaluator(Node):
    def __init__(self):
        super().__init__("evaluator")
        self.declare_parameters(
            "",
            [
                ("odom_topic", "/wamv/ground_truth/odometry"),
                ("path_topic", "/njord/path"),
                ("goal", [100.0, 100.0]),
                ("goal_tolerance_m", 6.0),
                ("obstacles", [0.0]),  # flat [x, y, radius, x, y, radius, ...]
                ("hull_radius_m", 2.5),
                ("timeout_s", 300.0),
                ("output", "run_metrics.json"),
                ("run_label", "run"),
            ],
        )
        p = self.get_parameter
        self.goal = np.array(p("goal").value, dtype=float)
        self.goal_tolerance = p("goal_tolerance_m").value
        flat = p("obstacles").value
        self.obstacles = np.array(flat, dtype=float).reshape(-1, 3) if len(flat) >= 3 else np.empty((0, 3))
        self.hull_radius = p("hull_radius_m").value
        self.timeout = p("timeout_s").value
        self.output = p("output").value
        self.label = p("run_label").value

        self.start_time = None
        self.origin = None
        self.previous = None
        self.travelled = 0.0
        self.min_clearance = math.inf
        self.replans = 0
        self.latencies = []
        self.reached = False
        self.done = False

        self.create_subscription(Odometry, p("odom_topic").value, self.on_odom, 10)
        self.create_subscription(Path, p("path_topic").value, lambda _: self.count_replan(), 1)
        self.create_subscription(Float64, "/njord/plan_ms", lambda m: self.latencies.append(m.data), 10)

    def count_replan(self):
        self.replans += 1

    def on_odom(self, msg):
        if self.done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        if self.start_time is None:
            self.start_time = now
            self.origin = position.copy()

        if self.previous is not None:
            self.travelled += float(np.linalg.norm(position - self.previous))
        self.previous = position

        if len(self.obstacles):
            gaps = np.linalg.norm(self.obstacles[:, :2] - position[None, :], axis=1)
            gaps -= self.obstacles[:, 2] + self.hull_radius
            self.min_clearance = min(self.min_clearance, float(np.min(gaps)))

        elapsed = now - self.start_time
        if np.linalg.norm(position - self.goal) < self.goal_tolerance:
            self.reached = True
            self.finish(elapsed)
        elif elapsed > self.timeout:
            self.finish(elapsed)

    def finish(self, elapsed):
        self.done = True
        metrics = {
            "label": self.label,
            "reached_goal": self.reached,
            "time_s": round(elapsed, 2),
            "distance_travelled_m": round(self.travelled, 1),
            "straight_line_m": round(float(np.linalg.norm(self.goal - self.origin)), 1),
            "min_clearance_m": round(self.min_clearance, 2) if math.isfinite(self.min_clearance) else None,
            "collision": bool(math.isfinite(self.min_clearance) and self.min_clearance < 0.0),
            "replans": self.replans,
            "max_plan_ms": round(max(self.latencies), 1) if self.latencies else 0.0,
            "mean_plan_ms": round(sum(self.latencies) / len(self.latencies), 1) if self.latencies else 0.0,
        }
        with open(self.output, "w") as fh:
            json.dump(metrics, fh, indent=2)
        self.get_logger().info(json.dumps(metrics))
        self.get_logger().info(f"metrics written to {self.output}")


def main():
    rclpy.init()
    node = Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
