"""One isolated open-loop experiment per fresh simulator process.

Start simulator alone in the dynamics scenario, without autonomy. Invoke this
node before simulation time 5 s (or use a paused startup). Each repetition and
each experiment needs a new simulator process; this tool never resets poses.
Use --ros-args -p experiment:=coast -p manifest_path:=... -p output_path:=...
"""
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from dynamics_metrics import stationary, summarize_experiment


class DynamicsCheck(Node):
    def __init__(self):
        super().__init__("dynamics_check")
        defaults = {"odom_topic": "/wamv/ground_truth/odometry", "forces_topic": "/njord/actuator_forces",
                    "experiment": "straight", "thrust_n": 300.0, "duration_s": 60.0,
                    "stabilization_timeout_s": 60.0, "window_s": 10.0,
                    "fresh_start_limit_s": 5.0, "manifest_path": "", "output_path": "",
                    "repetition": 0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        self.experiment = self.settings["experiment"]
        if self.experiment not in {"straight", "reverse", "turn_left", "turn_right", "coast", "drift", "hydrostatic"}:
            raise ValueError("unknown experiment")
        for key in ("thrust_n", "duration_s", "stabilization_timeout_s", "window_s", "fresh_start_limit_s"):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        self.manifest = None
        self.forces = self.create_publisher(Twist, self.settings["forces_topic"], 1)
        self.create_subscription(Odometry, self.settings["odom_topic"], self.on_odom, qos_profile_sensor_data)
        self.samples, self.preparation = [], []
        self.t0 = self.phase_start = None
        self.phase = "measure" if self.experiment == "hydrostatic" else "settle"
        self.trajectory = []
        self.wall_start = self.last_odom_wall = time.monotonic()
        self.done = False
        self.create_timer(0.05, self.step)
        self.create_timer(0.2, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            path = self.settings["manifest_path"]
            if path:
                data = Path(path).read_bytes()
                self.manifest = {"sha256": hashlib.sha256(data).hexdigest(), "content": json.loads(data)}
            if t > self.settings["fresh_start_limit_s"]:
                self.finish("fresh simulator startup was not observed")
                return
            self.t0 = self.phase_start = t
        q = msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w*q.y - q.z*q.x))))
        row = (t, msg.pose.pose.position.x, msg.pose.pose.position.y,
               math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y), msg.twist.twist.angular.z,
               msg.pose.pose.position.z, roll, pitch, msg.twist.twist.linear.x,
               msg.twist.twist.linear.z, msg.twist.twist.angular.x, msg.twist.twist.angular.y)
        if not all(math.isfinite(v) for v in row):
            self.finish("nonfinite odometry")
            return
        target = self.samples if self.phase == "measure" else self.preparation
        if target and t <= target[-1][0]:
            self.finish("nonadvancing odometry or clock reset")
            return
        self.last_odom_wall = time.monotonic()
        target.append(row)
        self.trajectory.append(row)

    def watchdog(self):
        odom_timeout = 120 if self.t0 is None else 10
        if time.monotonic() - self.last_odom_wall > odom_timeout or time.monotonic() - self.wall_start > 600:
            self.finish("wall-time watchdog expired")

    def command(self, left, right):
        message = Twist()
        message.linear.x, message.linear.y = float(left), float(right)
        self.forces.publish(message)

    def step(self):
        if self.t0 is None or self.done:
            return
        rows = self.samples if self.phase == "measure" else self.preparation
        if not rows:
            return
        t = rows[-1][0]
        thrust = self.settings["thrust_n"]
        if self.phase != "measure":
            self.command(thrust, thrust) if self.phase == "accelerate" else self.command(0, 0)
            stable = stationary(rows, self.settings["window_s"])
            tail = [r for r in rows if r[0] >= t - self.settings["window_s"]]
            # Coasting must start with stable straight motion, never after a turn.
            straight = all(abs(r[4]) <= 0.001 for r in tail)
            ready = stable and (self.experiment == "drift" or straight)
            if self.phase == "settle" and self.experiment != "drift":
                ready = ready and all(r[3] <= 0.01 for r in tail)
            if self.phase == "accelerate":
                ready = ready and rows[-1][3] > 0.05
            if ready:
                self.phase = "accelerate" if self.phase == "settle" and self.experiment == "coast" else "measure"
                self.phase_start = t
                self.preparation = []
                if self.phase == "measure" and self.experiment == "coast":
                    self.command(0, 0)
            elif t - self.phase_start >= self.settings["stabilization_timeout_s"]:
                self.finish("initial stabilization or straight coast entry not established")
            return
        commands = {"straight": (thrust, thrust), "reverse": (-thrust, -thrust),
                    "turn_left": (0.2 * thrust, thrust), "turn_right": (thrust, 0.2 * thrust),
                    "coast": (0, 0), "drift": (0, 0), "hydrostatic": (0, 0)}
        self.command(*commands[self.experiment])
        if t - self.phase_start >= self.settings["duration_s"]:
            self.finish()

    def finish(self, reason=None):
        self.done = True
        self.command(0, 0)
        report = summarize_experiment(self.samples, self.experiment, self.settings["window_s"])
        if reason:
            report.update(complete=False, reason=reason)
        report.update(experiment=self.experiment, settings=self.settings, manifest=self.manifest,
                      samples=self.samples, trajectory=self.trajectory,
                      sample_columns=["time_s", "x_m", "y_m", "speed_mps", "yaw_rate_radps", "z_m", "roll_rad", "pitch_rad", "surge_mps", "body_heave_mps", "body_roll_rate_radps", "body_pitch_rate_radps"], evidence_level="running_simulator_observation")
        serialized = json.dumps(report, indent=2, allow_nan=False)
        if self.settings["output_path"]:
            path = Path(self.settings["output_path"])
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("x") as stream:
                stream.write(serialized + "\n")
        print(serialized)
        raise SystemExit(0 if report["complete"] else 2)


def main():
    rclpy.init()
    node = DynamicsCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish("interrupted")
    finally:
        node.command(0, 0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
