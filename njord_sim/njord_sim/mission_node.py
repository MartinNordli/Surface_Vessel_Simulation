"""Sensor-only ordered red-left/green-right gate mission.

A bounded remembered crossing handles gates leaving the forward camera FOV;
actual camera frame freshness, odometry and downstream lidar guarding remain
required. No scenario coordinates or ground-truth topics are consumed.

The node finds the next gate (a red buoy on the left and a green buoy on the
right, seen from the current heading) in the fused buoy detections and
publishes goals that lead the vessel through it. For each gate it runs three
phases:

* ``seek``: no gate selected; pick the nearest unambiguous gate ahead.
* ``approach``: goal is ``approach_m`` before the gate centre, on the gate
  axis, so the boat lines up before entering.
* ``cross``: goal is ``exit_m`` past the gate centre. The gate counts as
  passed once the boat has crossed the gate line between the buoys and
  reached that exit point; then the mission returns to ``seek``.

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
        direction. Republished every step while valid (the planner ignores an
        unchanged goal).
    ``status_topic`` (default ``/njord/mission_status``,
        ``diagnostic_msgs/DiagnosticArray``): one status named ``mission``
        with ``phase`` and ``gates_passed`` values. OK only when a goal was
        just published; WARN for "complete" and "gate crossed"; ERROR for
        stale inputs, no gate, an expired gate or a missed crossing.

Parameters (m, s, rad):
    ``expected_gates`` (number of gates, not their positions),
    ``initial_heading_rad``, gate width bounds ``min/max_gate_width_m``,
    ``approach_m``, ``exit_m``, ``arrival_tolerance_m``, freshness limits
    ``detection_max_age_s``, ``camera_max_age_s``, ``odometry_max_age_s``,
    and the crossing-memory window ``crossing_memory_s`` /
    ``crossing_entry_m``.

Failure behaviour:
    Any non-OK status stops the boat: the command guard requires a fresh OK
    mission status before it forwards thrust. No goal is published while
    odometry or camera input is stale (by /clock simulation time). If the
    simulation clock jumps backwards, all mission state is reset.
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
    """Gate-sequencing mission node; see the module docstring."""

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
        # Direction (rad, map frame) in which the next gate is searched for.
        self.heading = self.p("initial_heading_rad")
        self.position = self.odom_stamp = self.camera_stamp = None
        self.observations = {}  # source camera/track id -> (color, position, source stamp)
        self.gate = None  # selected perception_core.Gate, or None while seeking
        self.gate_seen = None  # acquisition stamp (s) of the latest sighting of the gate
        self.phase = "seek"
        self.passed = []  # centres of passed gates, so they are not chosen again
        self.crossed = False  # vessel has crossed the current gate line
        # Signed distance (m) along the gate's forward axis at the last step;
        # negative before the gate line, positive after it.
        self.previous_signed = None
        self.last_clock = None  # sim time of the last step, to detect clock resets
        self.goal_pub = self.create_publisher(PoseStamped, self.p("goal_topic"), 1)
        self.status_pub = self.create_publisher(DiagnosticArray, self.p("status_topic"), 1)
        self.create_subscription(Odometry, self.p("odom_topic"), self.on_odom, 10)
        self.create_subscription(Detection3DArray, self.p("buoys_topic"), self.on_buoys, 10)
        self.create_timer(0.1, self.step)

    def on_odom(self, msg):
        """Store a valid, newer map-frame position; invalid messages are ignored.

        Ignored odometry is not cleared here; it simply ages out and the
        freshness check in ``step`` reports it as stale.
        """
        if msg.header.frame_id != self.p("map_frame"):
            return
        stamp = stamp_seconds(msg.header.stamp)
        # Drop out-of-order or duplicate messages.
        if self.odom_stamp is not None and stamp <= self.odom_stamp:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if (not np.isfinite([p.x, p.y]).all() or not np.isfinite(quaternion).all()
                or abs(float(quaternion@quaternion)-1.0) > 1e-3):
            return
        self.position, self.odom_stamp = np.array([p.x, p.y]), stamp
        # Until the first gate is chosen, search in the direction the boat is
        # pointing. Afterwards the heading is the previous gate's direction.
        if self.gate is None and not self.passed:
            self.heading = yaw_from_quaternion(q)

    def on_buoys(self, msg):
        """Record a fresh detection array and its red/green buoy observations."""
        if msg.header.frame_id != self.p("map_frame"):
            return
        stamp = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        # Reject old or future-stamped arrays; do not re-stamp them.
        if not 0 <= now-stamp <= self.p("camera_max_age_s"):
            return
        # Any fresh array, even with no detections, shows the camera pipeline
        # is running; step() requires this independently of gate memory.
        self.camera_stamp = max(stamp, self.camera_stamp or stamp)
        for detection in msg.detections:
            if not detection.results:
                continue
            # Use the most confident class hypothesis for each detection.
            hypothesis = max(detection.results, key=lambda result: result.hypothesis.score).hypothesis
            if hypothesis.class_id not in ("red", "green") or hypothesis.score < 0.35:
                continue
            p = detection.bbox.center.position
            point = np.array([p.x, p.y])
            if not np.isfinite(point).all():
                continue
            # Keep only the newest observation per detection id.
            old = self.observations.get(detection.id)
            if old is None or stamp > old[2]:
                self.observations[detection.id] = (hypothesis.class_id, point, stamp)

    def status(self, level, message):
        """Publish the mission heartbeat with a DiagnosticStatus level and text."""
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
        """Run one 10 Hz mission cycle: check inputs, advance the phase, publish."""
        now = self.get_clock().now().nanoseconds*1e-9  # /clock simulation time, s
        # Simulation clock went backwards (e.g. a world reset): nothing stored
        # is valid any more, so start the mission from scratch.
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
        # Forget observations older than detection_max_age_s.
        self.observations = {key: value for key, value in self.observations.items()
                             if 0 <= now-value[2] <= self.p("detection_max_age_s")}
        # The overlapping cameras may report the same buoy with different IDs.
        # Merge same-colour points within 2 m, keeping the newest.
        observations = []
        for color, point, stamp in sorted(self.observations.values(), key=lambda value: -value[2]):
            if not any(c == color and np.linalg.norm(point-p) < 2.0 for c, p, _ in observations):
                observations.append((color, point, stamp))
        # Phase "seek": select the nearest unambiguous gate ahead of `heading`.
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
        # Refresh the sighting time only when both buoys of this gate are seen
        # again (within 2.5 m of the stored positions); the refreshed time is
        # the older of the two, so the gate is never fresher than either buoy.
        red_stamps = [s for c, p, s in observations if c == "red" and np.linalg.norm(p-self.gate.red) < 2.5]
        green_stamps = [s for c, p, s in observations if c == "green" and np.linalg.norm(p-self.gate.green) < 2.5]
        if red_stamps and green_stamps:
            self.gate_seen = max(self.gate_seen, min(max(red_stamps), max(green_stamps)))
        # Vessel position in gate coordinates: `signed` is the distance (m)
        # along the gate's forward axis (negative before the gate line),
        # `lateral` the distance (m) from the gate centre line.
        offset = self.position-self.gate.center
        signed = float(offset@self.gate.forward)
        lateral = abs(float(offset@np.array([-self.gate.forward[1], self.gate.forward[0]])))
        half_width = np.linalg.norm(self.gate.red-self.gate.green)/2
        # Crossing memory: the forward cameras lose sight of the buoys as the
        # boat passes between them. Inside the crossing zone (from
        # crossing_entry_m before the gate line to exit_m + 2 m past it, and
        # at least 1 m inside the buoys) the gate stays valid for
        # crossing_memory_s after its last sighting. Elsewhere it must have
        # been seen within detection_max_age_s. Camera freshness is still
        # required above, and the command guard still needs every heartbeat.
        memory = (self.p("crossing_memory_s")
                  if -self.p("crossing_entry_m") <= signed <= self.p("exit_m")+2.0
                  and lateral < half_width-1.0
                  else self.p("detection_max_age_s"))
        if now-self.gate_seen > memory:
            return self.status(DiagnosticStatus.ERROR, "tracked gate expired")
        before = self.gate.center-self.p("approach_m")*self.gate.forward  # approach point
        after = self.gate.center+self.p("exit_m")*self.gate.forward  # exit point
        # approach -> cross once the approach point is reached, or once the
        # boat is already at or past it along the gate axis.
        if self.phase == "approach" and (np.linalg.norm(self.position-before) <= self.p("arrival_tolerance_m") or signed >= -self.p("approach_m")):
            self.phase = "cross"
        # The gate line was crossed forwards between the buoys (with a 1 m
        # margin to each buoy) since the previous step.
        if self.previous_signed is not None and self.previous_signed < 0 <= signed and lateral < half_width-1.0:
            self.crossed = True
        self.previous_signed = signed
        # Gate complete: crossed and at the exit point. Look for the next gate
        # in this gate's forward direction.
        if self.phase == "cross" and self.crossed and np.linalg.norm(self.position-after) <= self.p("arrival_tolerance_m"):
            self.passed.append(self.gate.center)
            self.heading = math.atan2(self.gate.forward[1], self.gate.forward[0])
            self.gate, self.phase = None, "seek"
            return self.status(DiagnosticStatus.WARN, "gate crossed; selecting next")
        # Past the exit without crossing between the buoys (e.g. went around
        # a buoy): report an error. The gate is not cleared, so the mission
        # stays in ERROR and the guard keeps the thrusters at zero.
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
