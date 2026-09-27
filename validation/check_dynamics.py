"""One isolated open-loop experiment per fresh simulator process.

Start simulator alone in the dynamics scenario, without autonomy. Invoke this
node before simulation time 5 s (or use a paused startup). Each repetition and
each experiment needs a new simulator process; this tool never resets poses.
Use --ros-args -p experiment:=coast -p manifest_path:=... -p output_path:=...

Where to run: inside the container (``autonomy`` or ``tests`` service) on the
same ROS domain as a running ``simulator`` service and no autonomy stack; this
node must be the only publisher of /njord/actuator_forces. See
docs/validation.md and docs/njord-calibration.md for the full protocol, and
scripts/dynamics_campaign.py, which automates fresh-process repetitions.

Experiments (``experiment`` parameter): straight, reverse (every thruster at
+/- thrust_n), turn_left, turn_right (20 % thrust on the inner side, 100 % on
the outer), coast (accelerate straight, then cut thrust), drift and
hydrostatic (zero thrust). Sides come from the thruster positions in the
run's resolved configuration: port thrusters (y > 0) get the left value,
starboard thrusters (y < 0) the right value and centreline thrusters their
mean. These open-loop experiments assume thrusters that push mainly forward. The run goes through phases:

    settle      zero thrust until a stationary window is observed
    accelerate  coast only: equal thrust until steady straight motion
    measure     the experiment's command for duration_s, then report

Parameters: odom_topic (default ground truth), forces_topic, thrust_n (N per
thruster, default 300), duration_s (simulation s, default 60),
stabilization_timeout_s (simulation s per preparation phase, default 60),
window_s (trailing stationarity window, default 10 s), fresh_start_limit_s
(default 5 s), manifest_path, resolved_path (default:
resolved_configuration.json next to manifest_path; one of the two is
required), output_path and repetition.

Output: the JSON report on stdout and, if output_path is set, in that file,
which must not already exist. It holds the metrics from
dynamics_metrics.summarize_experiment, the settings, the manifest (content and
SHA-256) and all raw samples. Exit code 0 if the experiment is complete, 2 if
not (including watchdog expiry or Ctrl-C). Thrust is set to zero on exit;
the vessel keeps its momentum.
"""
import hashlib
import json
import math
from pathlib import Path
import time
import threading
import sys

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Float32Array

from njord_sim.configuration import thruster_table
from njord_sim.constants import GROUND_TRUTH_TOPIC

from dynamics_metrics import stationary, summarize_experiment
from runtime_checks import valid_odometry


class DynamicsCheck(Node):
    """Open-loop force publisher and odometry recorder for one experiment.

    Ground-truth odometry is used on purpose: this measures the simulated
    vessel's dynamics, not the autonomy's estimates.
    """
    def __init__(self):
        super().__init__("dynamics_check")
        defaults = {"odom_topic": GROUND_TRUTH_TOPIC, "forces_topic": "/njord/actuator_forces",
                    "experiment": "straight", "thrust_n": 300.0, "duration_s": 60.0,
                    "stabilization_timeout_s": 60.0, "window_s": 10.0,
                    "fresh_start_limit_s": 5.0, "manifest_path": "", "resolved_path": "", "output_path": "",
                    "repetition": 0, "forces_n": [0.0], "applied_topic": "/njord/actuator_applied", "decay_s": 0.0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        self.experiment = self.settings["experiment"]
        if self.experiment not in {"straight", "reverse", "turn_left", "turn_right", "coast", "drift", "hydrostatic", "excitation", "oscillator_heave", "oscillator_roll", "oscillator_pitch"}:
            raise ValueError("unknown experiment")
        for key in ("thrust_n", "duration_s", "stabilization_timeout_s", "window_s", "fresh_start_limit_s"):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if not math.isfinite(self.settings['decay_s']) or self.settings['decay_s'] < 0:
            raise ValueError('decay_s must be finite and nonnegative')
        self.manifest = None
        resolved_path = self.settings["resolved_path"] or (
            self.settings["manifest_path"] and str(Path(self.settings["manifest_path"]).parent / "resolved_configuration.json"))
        if not resolved_path:
            raise ValueError("resolved_path or manifest_path is required for the thruster layout")
        # Side of each thruster in command order: +1 port, -1 starboard, 0 centreline.
        deadline = time.monotonic() + 120.0
        while not Path(resolved_path).is_file():
            if time.monotonic() >= deadline:
                raise TimeoutError('resolved configuration not produced before startup deadline')
            time.sleep(0.05)
        table = thruster_table(json.loads(Path(resolved_path).read_text())["vessel"])
        self.explicit_forces = self.settings['forces_n'] if self.experiment == 'excitation' else None
        if self.explicit_forces is not None and (len(self.explicit_forces) != len(table)
                or not all(math.isfinite(v) for v in self.explicit_forces)):
            raise ValueError('excitation requires one finite forces_n value per thruster')
        self.applied_samples = []
        self.wrench_samples = []
        self.invalid_telemetry = False
        self.applied_lock = threading.Lock()
        # Gazebo transport retains the acquisition stamp that Float32Array lacks.
        from gz.transport13 import Node as GzNode
        from gz.msgs10.float_v_pb2 import Float_V
        self.gz_node = GzNode()
        def receive_applied(message):
            stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
            values = list(message.data)
            if len(values) == 2*len(table)+1 and all(math.isfinite(x) for x in values):
                with self.applied_lock:
                    self.applied_samples.append({'time_s': stamp, 'forces_n': values[:len(table)],
                                                 'targets_n': values[len(table):-1], 'live':bool(values[-1])})
            else:
                self.invalid_telemetry = True
        self.gz_node.subscribe(Float_V, self.settings['applied_topic'], receive_applied)
        self.sides = [(t["position_m"][1] > 0) - (t["position_m"][1] < 0) for t in table]
        self.create_subscription(WrenchStamped, '/njord/actuator_wrench', self.on_wrench, qos_profile_sensor_data)
        self.forces = self.create_publisher(Float32Array, self.settings["forces_topic"], 1)
        self.create_subscription(Odometry, self.settings["odom_topic"], self.on_odom, qos_profile_sensor_data)
        self.samples, self.preparation = [], []
        self.t0 = self.phase_start = None
        self.phase = "measure" if self.experiment == "hydrostatic" or self.experiment.startswith("oscillator_") else "settle"
        self.trajectory = []
        self.wall_start = self.last_odom_wall = time.monotonic()
        self.done = False
        # Control step at 20 Hz on node time (simulation time with use_sim_time).
        self.create_timer(0.05, self.step)
        # Infrastructure watchdog on steady wall time, so it still fires if /clock stops.
        self.create_timer(0.2, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_wrench(self, message):
        force, moment = message.wrench.force, message.wrench.torque
        values = [force.x, force.y, force.z, moment.x, moment.y, moment.z]
        if all(math.isfinite(value) for value in values):
            self.wrench_samples.append({'time_s': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                                        'force_n': values[:3], 'moment_nm': values[3:], 'frame':message.header.frame_id})
        else:
            self.invalid_telemetry = True

    def on_odom(self, msg):
        """Store one odometry sample (see sample_columns in finish for the layout).

        The first message fixes t0; it must be stamped at or before
        fresh_start_limit_s, which shows the simulator was freshly started
        rather than already moving. Non-finite or non-advancing samples end
        the experiment as incomplete.
        """
        if not valid_odometry(msg, parent=msg.header.frame_id, child=msg.child_frame_id) or not msg.header.frame_id or not msg.child_frame_id:
            self.finish("invalid odometry pose, twist, frame or quaternion")
            return
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
        # Roll and pitch (rad) from the orientation quaternion.
        roll = math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w*q.y - q.z*q.x))))
        row = (t, msg.pose.pose.position.x, msg.pose.pose.position.y,
               math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y), msg.twist.twist.angular.z,
               msg.pose.pose.position.z, roll, pitch, msg.twist.twist.linear.x,
               msg.twist.twist.linear.z, msg.twist.twist.angular.x, msg.twist.twist.angular.y,
               msg.twist.twist.linear.y, math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
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
        """Fail on 120 s without first odometry, 10 s gaps later, or 600 s total (wall time)."""
        odom_timeout = 120 if self.t0 is None else 10
        if time.monotonic() - self.last_odom_wall > odom_timeout or time.monotonic() - self.wall_start > 600:
            self.finish("wall-time watchdog expired")

    def command(self, left, right):
        """Publish port/starboard thrust in newtons to every thruster by side."""
        message = Float32Array()
        message.data = [float(left) if side > 0 else float(right) if side < 0 else (left + right) / 2
                        for side in self.sides]
        self.forces.publish(message)

    def step(self):
        """Advance the phase machine using the latest sample time (simulation s)."""
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
            # Settle must reach rest (<= 0.01 m/s); drift only needs steadiness,
            # since wind or current may keep the boat moving.
            if self.phase == "settle" and self.experiment != "drift":
                ready = ready and all(r[3] <= 0.01 for r in tail)
            # Coast entry needs real forward speed (> 0.05 m/s).
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
                    "coast": (0, 0), "drift": (0, 0), "hydrostatic": (0, 0),
                    "oscillator_heave": (0,0), "oscillator_roll": (0,0), "oscillator_pitch": (0,0)}
        if self.explicit_forces is not None:
            message = Float32Array()
            message.data = ([0.0]*len(self.explicit_forces) if t-self.phase_start >= self.settings["duration_s"]
                            else list(map(float, self.explicit_forces)))
            self.forces.publish(message)
        else:
            self.command(*commands[self.experiment])
        if t - self.phase_start >= self.settings["duration_s"] + self.settings["decay_s"]:
            self.finish()

    def finish(self, reason=None):
        """Zero thrust, write the report and exit (0 complete, 2 incomplete).

        ``reason`` marks the report incomplete. The output file is opened in
        exclusive mode, so an existing result is never overwritten.
        """
        self.done = True
        self.command(0, 0)
        measure_samples = [row for row in self.samples if self.phase_start is not None
                           and row[0] <= self.phase_start + self.settings['duration_s']]
        report = summarize_experiment(measure_samples, self.experiment, self.settings["window_s"])
        if reason:
            report.update(complete=False, reason=reason)
        with self.applied_lock:
            applied_samples = list(self.applied_samples)
        if self.invalid_telemetry:
            report.update(complete=False, reason='malformed or nonfinite applied telemetry')
        if self.experiment == 'excitation' and not applied_samples:
            report.update(complete=False, reason='missing applied-force telemetry')
        report.update(experiment=self.experiment, settings=self.settings, manifest=self.manifest,
                      applied_samples=applied_samples, wrench_samples=self.wrench_samples,
                      samples=self.samples, trajectory=self.trajectory,
                      sample_columns=["time_s", "x_m", "y_m", "speed_mps", "yaw_rate_radps", "z_m", "roll_rad", "pitch_rad", "surge_mps", "body_heave_mps", "body_roll_rate_radps", "body_pitch_rate_radps", "sway_mps", "yaw_rad"], evidence_level="running_simulator_observation")
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
    node = None
    try:
        node = DynamicsCheck()
        rclpy.spin(node)
    except (Exception, KeyboardInterrupt) as error:
        if node is not None:
            node.finish(f"{type(error).__name__}: {error}")
        # Constructor failures still leave an exclusive incomplete artifact.
        result = {"complete": False, "reason": f"startup failed: {type(error).__name__}: {error}",
                  "metrics": {}, "samples": [], "evidence_level": "no_runtime_measurement"}
        serialized = json.dumps(result, indent=2, allow_nan=False)
        output = next((arg.split(":=", 1)[1] for arg in sys.argv if arg.startswith("output_path:=")), None)
        if output:
            path = Path(output)
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('x') as stream:
                stream.write(serialized + '\n')
        print(serialized)
        raise SystemExit(2)
    finally:
        if node is not None:
            node.command(0, 0)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
