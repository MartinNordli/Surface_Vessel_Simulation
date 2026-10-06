"""Score ordered gate races using simulation-only ground truth and /clock.

A steady-clock timer ends the process if Gazebo never starts, freezes, or loses
odometry. Missing contact data cannot establish a collision-free run.

Run flow (the node is started by scripts/run_evaluator.py with
use_sim_time:=true once run_ready.json exists):

1. Wait for readiness: fresh ground-truth odometry, a live contact-monitor
   heartbeat and OK diagnostics from mission, planner and navigation.
2. Start the race and publish ``/njord/race_active`` = true; the command guard
   only forwards thrust while this is true.
3. Feed every ground-truth pose to scenario_core.RaceScorer, which scores in
   simulation time.
4. Stop on the first final status and write the metrics JSON atomically.

Two clocks are used on purpose. Race time, the scenario timeout and data
freshness against message stamps use the node clock (/clock). Infrastructure
watchdogs (wall_timeout_s, odom_wall_timeout_s, stream liveness) use the
steady monotonic clock, so a frozen or never-started simulator still ends
the run. Ground truth is used here for scoring only; it is never forwarded
to autonomy.

Final statuses: those of RaceScorer plus ``wall_timeout``,
``odometry_timeout``, ``contact_monitor_timeout`` and ``interrupted``.
The process exits 0 only for ``completed``, otherwise 2.
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

from .configuration import guard_requirements
from .constants import GROUND_TRUTH_TOPIC, GZ_MODEL_NAME, HEARTBEATS, PROCESS_LIVENESS_S
from .scenario_core import RaceScorer, load_scenario, scenario_digest, wall_budget_s


class Evaluator(Node):
    """ROS wrapper around RaceScorer with readiness gating and watchdogs.

    Subscribes: ground-truth odometry (``odom_topic``), ``contacts_topic``,
    ``path_topic``, ``/njord/plan_ms`` and the ``/njord/*_status``
    diagnostics named in ``required_status`` (constants.HEARTBEATS keys, from
    configuration.guard_requirements). Publishes: ``/njord/race_active``
    (std_msgs/Bool, latched).
    """

    def __init__(self):
        super().__init__("evaluator")
        self.declare_parameters("", [
            ("scenario_file", ""), ("odom_topic", GROUND_TRUTH_TOPIC),
            ("path_topic", "/njord/path"), ("contacts_topic", "/njord/contacts"),
            ("output", "outputs/run_metrics.json"), ("run_label", "run"),
            ("profile", "conservative"), ("state_source", "estimate"), ("wall_timeout_s", 600.0),
            ("odom_wall_timeout_s", 30.0),
            ("required_status", guard_requirements({}, "race")["required_status"]),
            ("wait_for_ready", True),
            ("git_commit", os.environ.get("NJORD_IMAGE_SOURCE_COMMIT", "unknown")),
            ("image_source_digest", os.environ.get("NJORD_IMAGE_SOURCE_DIGEST", "unknown")),
            ("runner_git_commit", os.environ.get("RUNNER_GIT_COMMIT", "unknown")),
            ("image_identity", os.environ.get("IMAGE_ID", "unknown")),
        ])
        # wall_timeout_s: minimum steady-time budget for the whole run, startup
        #   included; see wall_budget_s for the scaling with the real-time factor.
        # odom_wall_timeout_s: steady time without advancing odometry.
        # wait_for_ready: False starts scoring on the first odometry message.
        p = lambda name: self.get_parameter(name).value
        self.scenario = load_scenario(p("scenario_file"))
        self.scorer = RaceScorer(self.scenario)
        self.wall_start = time.monotonic()
        self.first_odom_wall = None
        self.last_odom_wall = None
        self.output = FilePath(p("output"))
        self.wall_budget_s = wall_budget_s(p("wall_timeout_s"), self.scenario["timeout_s"],
                                           self.real_time_factor_target())
        self.done = False
        self.last_clock_s = None
        self.exit_code = 2  # nonzero unless the race completes
        self.path_messages = 0
        self.latencies = []
        self.started = not p("wait_for_ready")
        self.readiness = {}
        self.latest_ground_truth = None  # (stamp_s, x, y, yaw) of the last odometry
        self.contact_last_wall = None
        self.contact_last_stamp = None
        # Transient-local so a late-joining command guard still gets the state.
        self.active_pub = self.create_publisher(Bool, "/njord/race_active",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        required = list(p("required_status"))
        if not required or set(required) - set(HEARTBEATS):
            raise ValueError(f"required_status must be a nonempty subset of {sorted(HEARTBEATS)}")
        self.required_names = [HEARTBEATS[key][1] for key in required]
        for key in required:
            self.create_subscription(DiagnosticArray, HEARTBEATS[key][0], self.on_readiness, 10)
        self.create_subscription(Odometry, p("odom_topic"), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Path, p("path_topic"), self.on_path, 10)
        self.create_subscription(Float64, "/njord/plan_ms", self.on_latency, 10)
        try:
            from ros_gz_interfaces.msg import Contacts
            self.create_subscription(Contacts, p("contacts_topic"), self.on_contacts, qos_profile_sensor_data)
        except ImportError:
            self.get_logger().warning("Contact message type unavailable; contact result will be null")
        # 10 Hz watchdog on the steady clock: it keeps running even when
        # /clock never starts or stops advancing.
        self.create_timer(0.1, self.check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def real_time_factor_target(self):
        """REAL_TIME_FACTOR of a managed run (resolved_configuration.json), else 1."""
        resolved = self.output.parent / 'resolved_configuration.json'
        if resolved.is_file():
            return json.loads(resolved.read_text()).get('run', {}).get('real_time_factor', 1.0)
        return 1.0

    def on_readiness(self, message):
        """Store (ok, sim stamp, steady receive time) per diagnostic status name."""
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        for status in message.status:
            self.readiness[status.name] = (status.level == DiagnosticStatus.OK, stamp, time.monotonic())

    def contact_fresh(self):
        """Require both acquisition freshness and a live advancing stream.

        The last accepted contact message must be at most 0.5 s old in
        simulation time and have arrived within PROCESS_LIVENESS_S of steady
        time; the steady limit only catches a dead stream while /clock stalls.
        """
        if self.contact_last_stamp is None or self.contact_last_wall is None:
            return False
        age = self.get_clock().now().nanoseconds * 1e-9 - self.contact_last_stamp
        return 0 <= age <= 0.5 and time.monotonic() - self.contact_last_wall <= PROCESS_LIVENESS_S

    def ready(self):
        """True when every input needed to score a race is live.

        Odometry must have advanced within PROCESS_LIVENESS_S of steady time,
        contacts must be fresh, and every required diagnostic must be OK, at
        most 0.5 s old in simulation time and received within
        PROCESS_LIVENESS_S of steady time.
        """
        now_sim = self.get_clock().now().nanoseconds * 1e-9
        now_wall = time.monotonic()
        return self.latest_ground_truth is not None and self.last_odom_wall is not None and now_wall - self.last_odom_wall <= PROCESS_LIVENESS_S and self.contact_fresh() and all(
            name in self.readiness and self.readiness[name][0]
            and 0 <= now_sim - self.readiness[name][1] <= 0.5
            and now_wall - self.readiness[name][2] <= PROCESS_LIVENESS_S
            for name in self.required_names)

    def on_path(self, _):
        self.path_messages += 1

    def on_latency(self, message):
        if math.isfinite(message.data) and message.data >= 0:
            self.latencies.append(message.data)

    def on_contacts(self, message):
        """Accept a fresh contact message; end the race if it involves the vessel.

        Empty messages are heartbeats from the ContactMonitor plugin and only
        refresh liveness. A contact involves the vessel when either scoped
        collision name (``model::link::collision``) belongs to the vessel
        model, ``constants.GZ_MODEL_NAME``.
        """
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
        vessel = GZ_MODEL_NAME + "::"
        involved = any(name(c.collision1).startswith(vessel) or name(c.collision2).startswith(vessel)
                       for c in message.contacts)
        self.scorer.contact(involved)
        if involved:
            self.finish()

    def on_odom(self, message):
        """Record ground truth and, once the race has started, score it."""
        if self.done:
            return
        wall = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        # Only an advancing stamp proves the simulation is alive; a repeated
        # stamp (paused or frozen simulator) must not reset the watchdog.
        if self.latest_ground_truth is not None and stamp <= self.latest_ground_truth[0]:
            return  # Reordered/replayed transport samples cannot reset scoring or liveness.
        self.last_odom_wall = wall
        q = message.pose.pose.orientation
        # Yaw (rotation about world z, ENU) from the orientation quaternion.
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
        """Steady-clock tick: start the race when ready and apply the watchdogs.

        Checks in priority order: total wall budget, odometry silence,
        contact-monitor silence (only after the start), then the scenario's
        simulation-time limit, read here from /clock so that it does not
        depend on the next odometry message arriving.
        """
        if self.done:
            return
        now = time.monotonic()
        now_sim = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock_s is not None and now_sim < self.last_clock_s:
            self.scorer.status = "clock_reset"
            self.finish()
            return
        self.last_clock_s = now_sim
        if not self.started and self.ready():
            self.started = True
            self.first_odom_wall = now
            # The latest pose becomes the race start (time zero).
            self.scorer.update(*self.latest_ground_truth)
            self.get_logger().info("Required heartbeats and contacts ready; race started")
        self.active_pub.publish(Bool(data=self.started and self.scorer.status == "running"))
        if now - self.wall_start >= self.wall_budget_s:
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
        """Write the metrics once, publish race_active = false and set the exit code."""
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
            # 'truth' means autonomy navigated on ground truth, not on sensors.
            "state_source": self.get_parameter("state_source").value,
            "estimator_accuracy_validated": False,
            "evidence_scope": ("truth_navigation_scoring" if self.get_parameter("state_source").value == "truth"
                               else "sensor_navigation_scoring"),
            "seed": self.scenario["seed"], "environment": self.scenario["environment_name"],
            "scenario": self.scenario, "scenario_sha256": scenario_digest(self.scenario),
            "git_commit": self.get_parameter("git_commit").value,
            "image_source_digest": self.get_parameter("image_source_digest").value,
            "runner_git_commit": self.get_parameter("runner_git_commit").value,
            "image_identity": self.get_parameter("image_identity").value,
            "wall_time_s": elapsed_wall,
            # Simulated race seconds per steady second since the race started.
            "real_time_factor": self.scorer.elapsed / simulation_wall if simulation_wall > 0 else None,
            "path_messages": self.path_messages,
            "replans": len(self.latencies), "plan_samples": len(self.latencies),
            "max_plan_ms": max(self.latencies) if self.latencies else None,
            "mean_plan_ms": sum(self.latencies) / len(self.latencies) if self.latencies else None,
        })
        # Tie the metrics to the sealed run inputs when a manifest exists.
        manifest = self.output.parent / 'run_manifest.json'
        metrics['real_time_factor_target'] = self.real_time_factor_target()
        metrics['wall_budget_s'] = self.wall_budget_s
        if manifest.is_file():
            from njord_sim.run_manifest import sha256
            metrics['manifest_sha256'] = sha256(manifest)
            metrics['run_id'] = json.loads(manifest.read_text())['run_id']
        self.output.parent.mkdir(parents=True, exist_ok=True)
        # Temporary file plus rename: readers never see a partial result.
        temporary = self.output.with_suffix(self.output.suffix + ".tmp")
        temporary.write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
        temporary.replace(self.output)
        self.done = True
        self.active_pub.publish(Bool(data=False))
        self.exit_code = 0 if self.scorer.status == "completed" else 2
        self.get_logger().info(f"Race {self.scorer.status}; metrics: {self.output}")


def main():
    """Spin until the race finishes; exit 0 on ``completed``, otherwise 2."""
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
