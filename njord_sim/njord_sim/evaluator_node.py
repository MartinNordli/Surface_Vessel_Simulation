"""Score races and setpoint courses using simulation-only ground truth and /clock.

A steady-clock timer ends the process if Gazebo never starts, freezes, or loses
odometry. Missing contact data cannot establish a collision-free run.

Run flow (the node is started by scripts/run_evaluator.py with
use_sim_time:=true once run_ready.json exists):

1. Wait for readiness: fresh ground-truth odometry, a live contact-monitor
   heartbeat and OK diagnostics from the heartbeats in ``required_status``
   (configuration.guard_requirements: navigation, plus mission and planner on
   a gate course or the setpoint controller on a setpoint course).
2. Start the race and publish ``/njord/race_active`` = true; the command guard
   only forwards thrust while this is true.
3. Feed every ground-truth pose to the scorer, which scores in simulation
   time: scenario_core.RaceScorer for a gate course, or
   setpoint_core.SetpointCourse for a setpoint course. On a setpoint course
   the evaluator is the referee: it publishes each target on
   constants.SETPOINT_TOPIC (and the remaining sequence on
   SETPOINT_SEQUENCE_TOPIC) when the scorer issues it.
4. Stop on the first final status and write the metrics JSON atomically,
   then ``timeseries.csv`` and ``report.html`` next to it.

``mode:=observe`` (./scripts/njord lab) scores a free run instead: there is no
race, no readiness gate, no race-active signal and no run-ending watchdog.
Targets arrive on SETPOINT_TOPIC from outside (njord goto, RViz, a team's
mission node); setpoint_core.SetpointObserver scores each one with
constants.OBSERVED_SETPOINT_ACCEPTANCE until the next one replaces it.
Metrics and the report are rewritten after every target and when the node is
stopped.

For display only (both modes) the evaluator publishes the travelled track on
TRAJECTORY_TOPIC and, for setpoints, a live score on SETPOINT_STATUS_TOPIC and
RViz markers on SETPOINT_MARKERS_TOPIC. These are derived from ground truth and
live under /sim; autonomy must not subscribe to them.

Two clocks are used on purpose. Race time, the scenario timeout and data
freshness against message stamps use the node clock (/clock). Infrastructure
watchdogs (wall_timeout_s, odom_wall_timeout_s, stream liveness) use the
steady monotonic clock, so a frozen or never-started simulator still ends
the run. Ground truth is used here for scoring only; it is never forwarded
to autonomy.

Final statuses: those of the scorer plus ``wall_timeout``,
``odometry_timeout``, ``contact_monitor_timeout`` and ``interrupted``
(``stopped`` in observe mode). The process exits 0 only for ``completed``
(or a cleanly stopped observation), otherwise 2.
"""
import csv
import json
import math
import os
from pathlib import Path as FilePath
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Float64, Bool

from .configuration import guard_requirements
from .constants import (ACTUATOR_FORCES_TOPIC, GROUND_TRUTH_TOPIC, GZ_MODEL_NAME, HEARTBEATS, MAP_FRAME,
                        OBSERVED_SETPOINT_ACCEPTANCE, PROCESS_LIVENESS_S, SETPOINT_MARKERS_TOPIC,
                        SETPOINT_SEQUENCE_TOPIC, SETPOINT_STATUS_TOPIC, SETPOINT_TOPIC, TRAJECTORY_TOPIC)
from .geometry import yaw_from_quaternion
from .scenario_core import RaceScorer, load_scenario, scenario_digest, wall_budget_s
from .setpoint_core import SetpointCourse, SetpointObserver, commanded_yaw

MODES = ("race", "observe")
# Setpoint publishers are reliable and transient local, so a subscriber that
# asks for transient local and joins late still receives the active target.
# Subscribers are reliable and volatile, which matches any reliable publisher,
# including RViz's volatile 2D Goal Pose tool.
SETPOINT_PUBLISHER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
SETPOINT_SUBSCRIBER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.VOLATILE)
TIMESERIES_PERIOD_S = 0.1     # simulation time between rows of timeseries.csv
TRAJECTORY_STEP_M = 0.25      # spacing of the displayed track
TRAJECTORY_MAX_POSES = 20000  # halve the displayed track beyond this
# Marker colours (r, g, b, a) per target state.
COLORS = {"pending": (0.6, 0.6, 0.6, 0.35), "active": (1.0, 0.75, 0.0, 0.6),
          "reached": (0.1, 0.8, 0.2, 0.6), "advanced": (0.2, 0.5, 1.0, 0.5),
          "superseded": (0.2, 0.5, 1.0, 0.5), "timeout": (0.9, 0.1, 0.1, 0.6),
          "unfinished": (0.9, 0.1, 0.1, 0.6)}


def quaternion_z(yaw):
    """(z, w) of a rotation by ``yaw`` rad about +z."""
    return math.sin(yaw / 2), math.cos(yaw / 2)


class Evaluator(Node):
    """ROS wrapper around the race and setpoint scorers with readiness gating and watchdogs.

    Subscribes: ground-truth odometry (``odom_topic``), ``contacts_topic``,
    ``path_topic``, ``/njord/plan_ms``, the guard's thrust forces and the
    ``/njord/*_status`` diagnostics named in ``required_status``
    (constants.HEARTBEATS keys, from configuration.guard_requirements); in
    observe mode also SETPOINT_TOPIC. Publishes: ``/njord/race_active``
    (std_msgs/Bool, latched; race mode), the setpoint and sequence topics
    (setpoint race) and the /sim display topics.
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
            ("wait_for_ready", True), ("mode", "race"),
            ("git_commit", os.environ.get("NJORD_IMAGE_SOURCE_COMMIT", "unknown")),
            ("image_source_digest", os.environ.get("NJORD_IMAGE_SOURCE_DIGEST", "unknown")),
            ("runner_git_commit", os.environ.get("RUNNER_GIT_COMMIT", "unknown")),
            ("image_identity", os.environ.get("IMAGE_ID", "unknown")),
        ])
        # wall_timeout_s: minimum steady-time budget for the whole run, startup
        #   included; see wall_budget_s for the scaling with the real-time factor.
        # odom_wall_timeout_s: steady time without advancing odometry.
        # wait_for_ready: False starts scoring on the first odometry message.
        # mode: 'race' scores the scenario; 'observe' scores a free lab run.
        p = lambda name: self.get_parameter(name).value
        self.mode = p("mode")
        if self.mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        self.scenario = load_scenario(p("scenario_file"))
        self.kind = self.scenario["kind"]
        if self.mode == "observe":
            self.scorer = SetpointObserver(self.scenario, OBSERVED_SETPOINT_ACCEPTANCE)
        elif self.kind == "setpoints":
            self.scorer = SetpointCourse(self.scenario)
        else:
            self.scorer = RaceScorer(self.scenario)
        self.setpoints = not isinstance(self.scorer, RaceScorer)
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
        self.started = self.mode == "observe" or not p("wait_for_ready")
        self.readiness = {}
        self.latest_ground_truth = None  # (stamp_s, x, y, yaw) of the last odometry
        self.contact_last_wall = None
        self.contact_last_stamp = None
        self.forces = []                 # latest guard output, N per thruster
        self.rows = []                   # timeseries.csv rows
        self.trajectory = []             # displayed track: (x, y, yaw)
        self.ticks = 0
        self.publisher_check_done = False
        if self.mode == "race":
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
            from ros_gz_interfaces.msg import Contacts, Float32Array
            self.create_subscription(Contacts, p("contacts_topic"), self.on_contacts, qos_profile_sensor_data)
            self.create_subscription(Float32Array, ACTUATOR_FORCES_TOPIC, self.on_forces, 10)
        except ImportError:
            self.get_logger().warning("Contact message type unavailable; contact result will be null")
        self.trajectory_pub = self.create_publisher(Path, TRAJECTORY_TOPIC, 1)
        if self.setpoints:
            self.status_pub = self.create_publisher(DiagnosticArray, SETPOINT_STATUS_TOPIC, 1)
            self.markers_pub = self.marker_publisher()
        if self.mode == "observe":
            self.create_subscription(PoseStamped, SETPOINT_TOPIC, self.on_setpoint, SETPOINT_SUBSCRIBER_QOS)
        elif self.setpoints:
            self.setpoint_pub = self.create_publisher(PoseStamped, SETPOINT_TOPIC, SETPOINT_PUBLISHER_QOS)
            self.sequence_pub = self.create_publisher(Path, SETPOINT_SEQUENCE_TOPIC, SETPOINT_PUBLISHER_QOS)
        # 10 Hz watchdog on the steady clock: it keeps running even when
        # /clock never starts or stops advancing.
        self.create_timer(0.1, self.check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def marker_publisher(self):
        """RViz marker publisher, or None when visualization_msgs is unavailable."""
        try:
            from visualization_msgs.msg import MarkerArray
        except ImportError:
            return None
        return self.create_publisher(MarkerArray, SETPOINT_MARKERS_TOPIC, 1)

    def real_time_factor_target(self):
        """REAL_TIME_FACTOR of a managed run (resolved_configuration.json), else 1."""
        resolved = self.output.parent / 'resolved_configuration.json'
        if resolved.is_file():
            return json.loads(resolved.read_text()).get('run', {}).get('real_time_factor', 1.0)
        return 1.0

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def running(self):
        return self.scorer.status in ("running", "observing")

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
        age = self.sim_now() - self.contact_last_stamp
        return 0 <= age <= 0.5 and time.monotonic() - self.contact_last_wall <= PROCESS_LIVENESS_S

    def ready(self):
        """True when every input needed to score a race is live.

        Odometry must have advanced within PROCESS_LIVENESS_S of steady time,
        contacts must be fresh, and every required diagnostic must be OK, at
        most 0.5 s old in simulation time and received within
        PROCESS_LIVENESS_S of steady time.
        """
        now_sim = self.sim_now()
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

    def on_forces(self, message):
        """Latest thrust forces the guard passed (N per thruster; zeros when blocked)."""
        forces = [float(v) for v in message.data]
        self.forces = forces if all(math.isfinite(v) for v in forces) else []

    def on_contacts(self, message):
        """Accept a fresh contact message; end the race if it involves the vessel.

        Empty messages are heartbeats from the ContactMonitor plugin and only
        refresh liveness. A contact involves the vessel when either scoped
        collision name (``model::link::collision``) belongs to the vessel
        model, ``constants.GZ_MODEL_NAME``. In observe mode a contact is
        recorded but does not end the run.
        """
        if self.done:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.sim_now() - stamp
        if (not 0 <= age <= 0.5
                or (self.contact_last_stamp is not None and stamp <= self.contact_last_stamp)):
            return  # Delayed, future and replayed messages cannot freshen evidence.
        self.contact_last_stamp, self.contact_last_wall = stamp, time.monotonic()
        def name(collision):
            return getattr(collision, "name", str(collision))
        vessel = GZ_MODEL_NAME + "::"
        involved = any(name(c.collision1).startswith(vessel) or name(c.collision2).startswith(vessel)
                       for c in message.contacts)
        if self.setpoints:
            self.scorer.contact(involved, stamp)
        else:
            self.scorer.contact(involved)
        if involved and self.mode == "observe":
            self.get_logger().info(f"Contact with the vessel at {stamp:.2f} s (recorded)")
        elif involved:
            self.finish()

    def on_setpoint(self, message):
        """Observe mode: a target sent from outside replaces the active one."""
        p, q = message.pose.position, message.pose.orientation
        if (message.header.frame_id != MAP_FRAME
                or not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w))):
            self.get_logger().info(f"Ignored setpoint: needs finite values in the {MAP_FRAME} frame")
            return
        # Scored from receipt in simulation time: an RViz click is stamped
        # with RViz's own clock.
        self.handle_events(self.scorer.issue(self.sim_now(), p.x, p.y, yaw_from_quaternion(q)))

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
        self.score(stamp, pos.x, pos.y, yaw)
        self.record(stamp, pos.x, pos.y, yaw, message.twist.twist)
        if not self.running():
            self.finish()

    def score(self, stamp, x, y, yaw):
        """Feed one ground-truth pose to the scorer and act on its events."""
        if self.setpoints:
            force_sum = sum(abs(v) for v in self.forces)
            self.handle_events(self.scorer.update(stamp, x, y, yaw, force_sum))
        else:
            self.scorer.update(stamp, x, y, yaw)

    def record(self, stamp, x, y, yaw, twist):
        """Append a timeseries row (every TIMESERIES_PERIOD_S) and the display track."""
        start = getattr(self.scorer, "start_time", None)
        if start is None or not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            return
        t = stamp - start
        if not self.rows or t - self.rows[-1][0] >= TIMESERIES_PERIOD_S - 1e-9:
            track = self.scorer.active if self.setpoints else None
            self.rows.append((t, x, y, math.degrees(yaw), twist.linear.x, twist.linear.y,
                              math.degrees(twist.angular.z), track.index if track else None,
                              track.distance if track else None,
                              math.degrees(track.heading_error) if track else None, list(self.forces)))
        if not self.trajectory or math.dist(self.trajectory[-1][:2], (x, y)) >= TRAJECTORY_STEP_M:
            self.trajectory.append((x, y, yaw))
            if len(self.trajectory) > TRAJECTORY_MAX_POSES:
                self.trajectory = self.trajectory[::2]

    def handle_events(self, events):
        """Publish newly issued targets, log every event and refresh the display."""
        total = len(self.scorer.specs) if isinstance(self.scorer, SetpointCourse) else None
        for event in events:
            label = f"Setpoint {event['index'] + 1}{f'/{total}' if total else ''} '{event['name']}'"
            if event["type"] == "issued":
                x, y, yaw = self.scorer.tracks[event["index"]].spec["position"] + [
                    self.scorer.tracks[event["index"]].yaw]
                if self.mode == "race":
                    self.publish_setpoint(event["index"])
                self.get_logger().info(f"{label} issued at {event['time_s']:.1f} s: x={x:.2f} y={y:.2f} "
                                       f"heading={math.degrees(yaw):.1f} deg")
            else:
                track = self.scorer.tracks[event["index"]]
                self.get_logger().info(
                    f"{label} {event['outcome']} after {track.ended_at - track.issued_at:.1f} s: "
                    f"error {event['final_distance_m']:.2f} m / {abs(event['final_heading_error_deg']):.1f} deg")
        if events:
            self.publish_markers()
            if self.mode == "observe" and any(e["type"] == "ended" for e in events):
                self.write_outputs(self.collect_metrics())

    def publish_setpoint(self, index):
        """Publish target ``index`` and the remaining sequence, stamped now."""
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.scorer.target(index)
        message = PoseStamped()
        message.header.frame_id, message.header.stamp = MAP_FRAME, stamp
        message.pose.position.x, message.pose.position.y = float(x), float(y)
        message.pose.orientation.z, message.pose.orientation.w = quaternion_z(yaw)
        self.setpoint_pub.publish(message)
        sequence = Path()
        sequence.header = message.header
        sequence.poses = [message]
        previous = (x, y)
        for spec in self.scorer.specs[index + 1:]:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = map(float, spec["position"])
            pose.pose.orientation.z, pose.pose.orientation.w = quaternion_z(commanded_yaw(spec, previous))
            sequence.poses.append(pose)
            previous = spec["position"]
        self.sequence_pub.publish(sequence)
        if not self.publisher_check_done:
            self.publisher_check_done = True
            try:
                count = self.count_publishers(SETPOINT_TOPIC)
            except Exception:  # noqa: BLE001 - a missing graph must not stop the race
                count = 1
            if count > 1:
                self.get_logger().warning(f"{count} publishers on {SETPOINT_TOPIC}; the evaluator must be "
                                          "the only one during a setpoint race")

    def publish_markers(self):
        """RViz markers: tolerance disc, heading arrow and label per target."""
        if not self.setpoints or self.markers_pub is None:
            return
        from visualization_msgs.msg import Marker, MarkerArray
        items = [(track.index, track.spec, track.yaw,
                  "active" if track.outcome is None else track.outcome) for track in self.scorer.tracks]
        if isinstance(self.scorer, SetpointCourse) and self.scorer.status == "running":
            issued = len(self.scorer.tracks)
            previous = self.scorer.specs[issued - 1]["position"] if issued else self.scenario["start"][:2]
            for index, spec in enumerate(self.scorer.specs[issued:], issued):
                items.append((index, spec, commanded_yaw(spec, previous), "pending"))
                previous = spec["position"]
        array = MarkerArray()
        for index, spec, yaw, state in items:
            color = COLORS.get(state, COLORS["pending"])
            for k, kind in enumerate((Marker.CYLINDER, Marker.ARROW, Marker.TEXT_VIEW_FACING)):
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = MAP_FRAME, Time()
                marker.ns, marker.id, marker.type, marker.action = "setpoints", 3 * index + k, kind, Marker.ADD
                marker.pose.position.x, marker.pose.position.y = map(float, spec["position"])
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
                if kind == Marker.CYLINDER:
                    diameter = 2.0 * spec["tolerance_m"]
                    marker.scale.x = marker.scale.y = diameter
                    marker.scale.z = 0.05
                    marker.pose.orientation.w = 1.0
                elif kind == Marker.ARROW:
                    marker.pose.position.z = 0.1
                    marker.pose.orientation.z, marker.pose.orientation.w = quaternion_z(yaw)
                    marker.scale.x, marker.scale.y, marker.scale.z = 2.0, 0.2, 0.2
                    marker.color.a = 1.0
                else:
                    marker.pose.position.z = 1.5
                    marker.pose.orientation.w = 1.0
                    marker.scale.z = 0.8
                    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
                    marker.text = f"{index + 1} {spec['name']} ({state})"
                array.markers.append(marker)
        self.markers_pub.publish(array)

    def publish_status(self):
        """Live score of the active target on SETPOINT_STATUS_TOPIC (display only)."""
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        track = self.scorer.active
        failed = any(t.required and t.outcome not in (None, "reached") for t in self.scorer.tracks)
        status = DiagnosticStatus(name="setpoint", hardware_id="evaluator",
                                  level=DiagnosticStatus.WARN if failed else DiagnosticStatus.OK)
        reached = sum(t.outcome == "reached" for t in self.scorer.tracks)
        values = {"status": self.scorer.status, "reached": str(reached), "issued": str(len(self.scorer.tracks))}
        if track is not None:
            now = self.latest_ground_truth[0] if self.latest_ground_truth else track.issued_at
            values.update(index=str(track.index + 1), name=track.spec["name"],
                          distance_m=f"{track.distance:.2f}",
                          heading_error_deg=f"{math.degrees(track.heading_error):.1f}",
                          elapsed_s=f"{now - track.issued_at:.1f}",
                          inside_for_s=f"{now - track.inside_since:.1f}" if track.inside_since is not None else "0.0")
            status.message = (f"{track.spec['name']}: {track.distance:.2f} m, "
                              f"{abs(math.degrees(track.heading_error)):.1f} deg off")
        else:
            status.message = "no active setpoint"
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        output.status = [status]
        self.status_pub.publish(output)

    def publish_trajectory(self):
        """The travelled ground-truth track on TRAJECTORY_TOPIC (display only)."""
        path = Path()
        path.header.frame_id, path.header.stamp = MAP_FRAME, self.get_clock().now().to_msg()
        for x, y, yaw in self.trajectory:
            pose = PoseStamped()
            pose.header.frame_id = MAP_FRAME
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = quaternion_z(yaw)
            path.poses.append(pose)
        self.trajectory_pub.publish(path)

    def end_run(self, status, now_sim=None):
        """Set a final status from outside the scorer (watchdogs, interruption)."""
        if self.setpoints:
            self.scorer.stop(status, now_sim)
        else:
            self.scorer.status = status

    def check_timeout(self):
        """Steady-clock tick: start the race when ready and apply the watchdogs.

        Checks in priority order: total wall budget, odometry silence,
        contact-monitor silence (only after the start), then the scenario's
        simulation-time limit, read here from /clock so that it does not
        depend on the next odometry message arriving. Observe mode only
        refreshes the display and reacts to a clock reset.
        """
        if self.done:
            return
        self.ticks += 1
        now = time.monotonic()
        now_sim = self.sim_now()
        if self.last_clock_s is not None and now_sim < self.last_clock_s:
            self.end_run("clock_reset", now_sim)
            self.finish()
            return
        self.last_clock_s = now_sim
        self.refresh_display()
        if self.mode == "observe":
            return
        if not self.started and self.ready():
            self.started = True
            self.first_odom_wall = now
            # The latest pose becomes the race start (time zero).
            self.score(*self.latest_ground_truth)
            self.get_logger().info("Required heartbeats and contacts ready; race started")
        self.active_pub.publish(Bool(data=self.started and self.scorer.status == "running"))
        if now - self.wall_start >= self.wall_budget_s:
            self.end_run("wall_timeout", now_sim)
        elif self.last_odom_wall is not None and now - self.last_odom_wall >= self.get_parameter("odom_wall_timeout_s").value:
            self.end_run("odometry_timeout", now_sim)
        elif self.started and not self.contact_fresh():
            self.end_run("contact_monitor_timeout", now_sim)
        elif self.scorer.start_time is not None:
            elapsed = now_sim - self.scorer.start_time
            if elapsed >= self.scenario["timeout_s"] and self.scorer.status == "running":
                self.scorer.elapsed = elapsed
                self.end_run("simulation_timeout", now_sim)
        if self.scorer.status != "running":
            self.finish()

    def refresh_display(self):
        """Status at 2 Hz, markers and track at 1 Hz (steady time)."""
        try:
            if self.setpoints and self.ticks % 5 == 0:
                self.publish_status()
            if self.ticks % 10 == 0:
                self.publish_markers()
                self.publish_trajectory()
        except Exception as error:  # noqa: BLE001 - display must never end a run
            self.get_logger().info(f"Display update failed: {error}")

    def collect_metrics(self):
        """The scorer's metrics plus run provenance."""
        metrics = self.scorer.metrics()
        metrics.setdefault("course", self.kind)
        elapsed_wall = time.monotonic() - self.wall_start
        simulation_wall = time.monotonic() - self.first_odom_wall if self.first_odom_wall else 0
        metrics.update({
            "mode": self.mode,
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
        metrics['wall_budget_s'] = self.wall_budget_s if self.mode == "race" else None
        if manifest.is_file():
            from njord_sim.run_manifest import sha256
            metrics['manifest_sha256'] = sha256(manifest)
            metrics['run_id'] = json.loads(manifest.read_text())['run_id']
        return metrics

    def write_outputs(self, metrics):
        """Write the metrics atomically, then timeseries.csv and report.html.

        The metrics come first and alone decide the result; a failure in the
        time series or the report is logged and never loses them.
        """
        self.output.parent.mkdir(parents=True, exist_ok=True)
        # Temporary file plus rename: readers never see a partial result.
        temporary = self.output.with_suffix(self.output.suffix + ".tmp")
        temporary.write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
        temporary.replace(self.output)
        try:
            timeseries = self.output.with_name("timeseries.csv")
            count = max((len(row[-1]) for row in self.rows), default=0)
            temporary = timeseries.with_suffix(".csv.tmp")
            with temporary.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["t_s", "x_m", "y_m", "yaw_deg", "surge_mps", "sway_mps", "yaw_rate_dps",
                                 "setpoint_index", "distance_m", "heading_error_deg",
                                 *[f"force_{i + 1}_n" for i in range(count)]])
                for row in self.rows:
                    writer.writerow(["" if v is None else (f"{v:.4f}" if isinstance(v, float) else v)
                                     for v in row[:-1]] + [f"{f:.2f}" for f in row[-1]])
            temporary.replace(timeseries)
            from .report_core import write_report
            write_report(self.output, timeseries, self.output.with_name("report.html"))
        except Exception as error:  # noqa: BLE001 - the metrics are already safe
            self.get_logger().info(f"Time series or report not written: {error}")

    def finish(self):
        """Write the outputs once, publish race_active = false and set the exit code."""
        if self.done:
            return
        if self.mode == "observe" and self.scorer.status == "observing":
            self.scorer.stop("stopped", self.latest_ground_truth[0] if self.latest_ground_truth else None)
        # Completion may arrive on odometry between watchdog timer ticks.
        if self.scorer.status == "completed" and not self.contact_fresh():
            self.scorer.status = "contact_monitor_timeout"
        self.write_outputs(self.collect_metrics())
        self.done = True
        if self.mode == "race":
            try:
                self.active_pub.publish(Bool(data=False))
            except Exception:  # noqa: BLE001 - the ROS context may already be shut down
                pass
        self.exit_code = 0 if self.scorer.status in ("completed", "stopped") else 2
        self.get_logger().info(f"{'Observation' if self.mode == 'observe' else 'Race'} {self.scorer.status}; "
                               f"metrics: {self.output}, report: {self.output.with_name('report.html')}")

    def interrupt(self):
        """End the run on Ctrl-C or SIGTERM ('stopped' when observing, else 'interrupted')."""
        if self.done:
            return
        now = self.latest_ground_truth[0] if self.latest_ground_truth else None
        self.end_run("stopped" if self.mode == "observe" else "interrupted", now)
        self.finish()


def main():
    """Spin until the race finishes; exit 0 on ``completed``, otherwise 2."""
    rclpy.init()
    node = Evaluator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # SIGINT or SIGTERM (docker compose stop) ends the run with its result.
        node.interrupt()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
