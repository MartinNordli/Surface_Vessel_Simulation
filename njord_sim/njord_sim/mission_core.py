"""Gate mission state machine of the reference autonomy, free of ROS (see mission_node.py).

Sensor-only: the mission sees fused buoy detections and the navigation
estimate, never scenario coordinates or ground truth. For each gate (a red
buoy on the left and a green buoy on the right, seen along the course
heading) it runs these phases:

* ``seek``: no gate selected; pick the nearest unambiguous gate ahead.
* ``search``: no gate has been found for ``search_after_s``. Goals at
  ``search_radius_m`` from where the search started, at bearings of 0, +40,
  -40, +80 and -80 degrees from the course heading, turn the boat so the
  forward cameras sweep the course. Every bearing points forward, so the
  boat never crosses a passed gate backwards. After ``search_timeout_s``
  without a gate the mission reports an error.
* ``approach``: goal ``approach_m`` before the gate centre, on the gate axis.
* ``cross``: goal ``exit_m`` past the gate centre. The gate counts as passed
  once the boat has crossed the gate line between the buoys and reached that
  exit point; the next gate is then selected in the same step.
* ``retry``: the boat was seen crossing the gate line outside the buoys and
  went past the exit. Up to ``max_gate_retries`` times per gate it first goes to a point beside
  the buoy it passed (``retry_clearance_m`` outside it, on the gate line) and
  then approaches again, this time until it reaches the approach point
  itself. That route stays outside the opening, so it never crosses the gate
  backwards.

``step`` returns a ``Decision``: a level (OK, WARN or ERROR), a message and,
when OK, the goal (x, y, yaw) in the map frame. Units: m, s, rad.
"""
from dataclasses import dataclass
import math

import numpy as np

from .perception_core import choose_gate

OK, WARN, ERROR = 0, 1, 2
# A position estimate moving faster than this (m/s) is a discontinuity, e.g.
# an initializing estimator, not vessel motion; gate crossings are not
# inferred across it.
MAX_PLAUSIBLE_SPEED_MPS = 15.0
# Search bearings relative to the course heading, in the order they are tried.
SEARCH_BEARINGS_DEG = (0.0, 40.0, -40.0, 80.0, -80.0)


@dataclass
class Decision:
    """Result of one mission step; ``goal`` is (x, y, yaw) when level is OK."""
    level: int
    message: str
    goal: tuple = None


class GateMission:
    """Ordered red-left/green-right gate mission; see the module docstring.

    ``settings`` holds the geometry, freshness and recovery values from
    algorithms.yaml ``mission`` plus ``camera_max_age_s`` and
    ``odometry_max_age_s``.
    """

    def __init__(self, expected_gates, initial_heading, settings):
        self.expected_gates = expected_gates
        self.initial_heading = initial_heading
        self.s = dict(settings)
        self.reset()

    def reset(self):
        """Start from scratch, e.g. after simulation time moved backwards."""
        self.heading = self.initial_heading  # course direction (rad, map) to search along
        self.position = self.odom_stamp = self.camera_stamp = None
        self.observations = {}  # detection id -> (color, position, source stamp)
        self.gate = None  # selected perception_core.Gate, or None while seeking
        self.gate_seen = None  # acquisition stamp (s) of the latest sighting of the gate
        self.phase = "seek"
        self.passed = []  # centres of passed gates, so they are not chosen again
        self.crossed = False  # vessel has crossed the current gate line
        # Signed distance (m) along the gate's forward axis at the last step;
        # negative before the gate line, positive after it.
        self.previous_signed = None
        self.crossing_side = None  # +1 left / -1 right of the gate centre at its line
        self.retries = 0  # retries of the current gate
        self.retry_point = None  # beside-the-buoy waypoint of a retry
        # After a retry the boat is level with the gate, so it must reach the
        # approach point before it may cross.
        self.reapproach = False
        self.seek_since = None  # time (s) the current seek began
        self.search = None  # dict(anchor, index, goal_since) while searching
        self.last_clock = None

    # --- inputs ---------------------------------------------------------------

    def odometry(self, position, yaw, stamp):
        """Record a newer map-frame position (m) and yaw (rad); older ones are ignored.

        A jump faster than MAX_PLAUSIBLE_SPEED_MPS forgets the previous gate
        coordinate, so no crossing is inferred across the discontinuity.
        """
        if self.odom_stamp is not None and stamp <= self.odom_stamp:
            return
        position = np.asarray(position, dtype=float)
        if (self.position is not None and np.linalg.norm(position-self.position)
                > MAX_PLAUSIBLE_SPEED_MPS*(stamp-self.odom_stamp)):
            self.previous_signed = None
        self.position, self.odom_stamp = position, stamp
        # Until the first gate is chosen, search in the direction the boat is
        # pointing. Afterwards the heading is the previous gate's direction.
        if self.gate is None and not self.passed:
            self.heading = yaw

    def camera(self, stamp, now, detections):
        """Record a detection array stamped ``stamp``; ``detections`` are
        (id, color, (x, y)) with class 'red' or 'green'. Old or future arrays
        are rejected, never re-stamped. Returns whether it was accepted."""
        if not 0 <= now-stamp <= self.s["camera_max_age_s"]:
            return False
        # Any fresh array, even with no detections, shows the camera pipeline
        # is running; step() requires this independently of gate memory.
        self.camera_stamp = max(stamp, self.camera_stamp or stamp)
        for identifier, color, point in detections:
            old = self.observations.get(identifier)
            if old is None or stamp > old[2]:
                self.observations[identifier] = (color, np.asarray(point, dtype=float), stamp)
        return True

    # --- decision -------------------------------------------------------------

    def step(self, now):
        """One mission cycle at simulation time ``now`` (s)."""
        # Simulation clock went backwards (e.g. a world reset): nothing stored
        # is valid any more, so start the mission from scratch.
        if self.last_clock is not None and now < self.last_clock:
            self.reset()
        self.last_clock = now
        if len(self.passed) >= self.expected_gates:
            return Decision(WARN, "complete")
        if self.odom_stamp is None or not 0 <= now-self.odom_stamp <= self.s["odometry_max_age_s"]:
            return Decision(ERROR, "odometry unavailable or stale")
        if self.camera_stamp is None or not 0 <= now-self.camera_stamp <= self.s["camera_max_age_s"]:
            return Decision(ERROR, "camera fusion unavailable or stale")
        # Forget observations older than detection_max_age_s.
        self.observations = {key: value for key, value in self.observations.items()
                             if 0 <= now-value[2] <= self.s["detection_max_age_s"]}
        observations = self.visible()
        if self.gate is None:
            decision = self.select(now, observations)
            if decision is not None:
                return decision
        return self.track(now, observations)

    def select(self, now, observations):
        """Choose the next gate, or search for one; None once a gate is selected."""
        gate = choose_gate([(c, p) for c, p, _ in observations], self.position, self.heading,
                           self.passed, self.s["min_gate_width_m"], self.s["max_gate_width_m"])
        if gate is None:
            return self.search_step(now)
        self.gate = gate
        # Preserve actual source age, even when selected from the short track cache.
        self.gate_seen = min(stamp for color, point, stamp in observations
                             if np.linalg.norm(point-(gate.red if color == "red" else gate.green)) < 2.0)
        self.phase, self.crossed, self.crossing_side = "approach", False, None
        self.retries, self.retry_point, self.seek_since, self.search = 0, None, None, None
        self.reapproach = False
        self.previous_signed = float((self.position-gate.center)@gate.forward)
        return None

    def search_step(self, now):
        """No gate in view: wait search_after_s, then sweep search goals."""
        if self.seek_since is None:
            self.seek_since = now
        self.phase = "seek"
        waited = now-self.seek_since
        if waited < self.s["search_after_s"]:
            return Decision(ERROR, "no unambiguous observed gate")
        if waited >= self.s["search_after_s"]+self.s["search_timeout_s"]:
            return Decision(ERROR, "no gate found while searching")
        if self.search is None:
            self.search = dict(anchor=self.position.copy(), index=0, goal_since=now)
        target = self.search_goal()
        # Next bearing once this goal is reached or has been tried long enough.
        if (np.linalg.norm(self.position-target) <= self.s["arrival_tolerance_m"]
                or now-self.search["goal_since"] >= self.s["search_goal_s"]):
            self.search["index"] += 1
            self.search["goal_since"] = now
            target = self.search_goal()
        self.phase = "search"
        bearing = math.atan2(*(target-self.search["anchor"])[::-1])
        return Decision(OK, "searching", (float(target[0]), float(target[1]), bearing))

    def search_goal(self):
        """Current search goal (x, y) in map."""
        bearing = self.heading + math.radians(SEARCH_BEARINGS_DEG[self.search["index"] % len(SEARCH_BEARINGS_DEG)])
        return self.search["anchor"] + self.s["search_radius_m"]*np.array([math.cos(bearing), math.sin(bearing)])

    def track(self, now, observations):
        """Approach, cross or retry the selected gate."""
        gate = self.gate
        # Refresh the sighting time only when both buoys of this gate are seen
        # again (within 2.5 m of the stored positions); the refreshed time is
        # the older of the two, so the gate is never fresher than either buoy.
        red_stamps = [s for c, p, s in observations if c == "red" and np.linalg.norm(p-gate.red) < 2.5]
        green_stamps = [s for c, p, s in observations if c == "green" and np.linalg.norm(p-gate.green) < 2.5]
        if red_stamps and green_stamps:
            self.gate_seen = max(self.gate_seen, min(max(red_stamps), max(green_stamps)))
        # Vessel position in gate coordinates: ``signed`` is the distance (m)
        # along the gate's forward axis (negative before the gate line),
        # ``lateral`` the signed distance (m) from the centre line, left positive.
        left = np.array([-gate.forward[1], gate.forward[0]])
        offset = self.position-gate.center
        signed, lateral = float(offset@gate.forward), float(offset@left)
        half_width = np.linalg.norm(gate.red-gate.green)/2
        # Crossing memory: the forward cameras lose sight of the buoys as the
        # boat passes between them. Inside the crossing zone (from
        # crossing_entry_m before the gate line to exit_m + 2 m past it, and
        # at least 1 m inside the buoys) the gate stays valid for
        # crossing_memory_s after its last sighting, and so it does during a
        # retry, when the boat faces away from the gate. Elsewhere it must
        # have been seen within detection_max_age_s. Camera freshness is still
        # required above, and the command guard still needs every heartbeat.
        in_zone = (-self.s["crossing_entry_m"] <= signed <= self.s["exit_m"]+2.0
                   and abs(lateral) < half_width-1.0)
        memory = (self.s["crossing_memory_s"] if self.phase == "retry" or in_zone
                  else self.s["detection_max_age_s"])
        if now-self.gate_seen > memory:
            return Decision(ERROR, "tracked gate expired")
        before = gate.center-self.s["approach_m"]*gate.forward  # approach point
        after = gate.center+self.s["exit_m"]*gate.forward  # exit point
        yaw = math.atan2(gate.forward[1], gate.forward[0])
        if self.phase == "retry":
            if np.linalg.norm(self.position-self.retry_point) > self.s["arrival_tolerance_m"]:
                return Decision(OK, "retrying gate", (float(self.retry_point[0]), float(self.retry_point[1]), yaw))
            # Beside the buoy on the gate line: approach again from before it.
            self.phase, self.crossed, self.retry_point, self.reapproach = "approach", False, None, True
            self.crossing_side = None
            self.previous_signed = signed
        # approach -> cross once the approach point is reached, or once the
        # boat is already at or past it along the gate axis (not after a
        # retry, which starts level with the gate).
        reached = np.linalg.norm(self.position-before) <= self.s["arrival_tolerance_m"]
        if self.phase == "approach" and (reached or (signed >= -self.s["approach_m"] and not self.reapproach)):
            self.phase, self.reapproach = "cross", False
        # The gate line was crossed forwards since the previous step: between
        # the buoys (with a 1 m margin to each buoy) it counts; outside them
        # the side is remembered for a retry.
        if self.previous_signed is not None and self.previous_signed < 0 <= signed:
            if abs(lateral) < half_width-1.0:
                self.crossed = True
            else:
                self.crossing_side = 1.0 if lateral > 0 else -1.0
        self.previous_signed = signed
        # Gate complete: crossed and at the exit point. Select the next gate
        # in this gate's forward direction within the same step.
        if self.phase == "cross" and self.crossed and np.linalg.norm(self.position-after) <= self.s["arrival_tolerance_m"]:
            self.passed.append(gate.center)
            self.heading = yaw
            self.gate, self.phase = None, "seek"
            if len(self.passed) >= self.expected_gates:
                return Decision(WARN, "complete")
            return self.select(now, observations) or self.track(now, observations)
        # Past the exit without crossing between the buoys (e.g. went around
        # a buoy): retry from beside that buoy, but only if the boat was seen
        # crossing the gate line outside the buoys. Otherwise (the estimate
        # jumped past the gate) or once the retries are used up, report an
        # error; the gate is kept, so the guard keeps the thrusters at zero
        # until the position is plausible again.
        if signed > self.s["exit_m"]+2 and not self.crossed:
            if self.crossing_side is None or self.retries >= self.s["max_gate_retries"]:
                return Decision(ERROR, "gate crossing missed")
            self.retries += 1
            self.retry_point = gate.center+self.crossing_side*(half_width+self.s["retry_clearance_m"])*left
            self.phase = "retry"
            return Decision(OK, "retrying gate", (float(self.retry_point[0]), float(self.retry_point[1]), yaw))
        target = before if self.phase == "approach" else after
        return Decision(OK, "valid", (float(target[0]), float(target[1]), yaw))

    def visible(self):
        """Observations as (color, position, stamp), newest first.

        The overlapping cameras may report the same buoy with different IDs:
        same-colour points within 2 m are merged, keeping the newest.
        """
        merged = []
        for color, point, stamp in sorted(self.observations.values(), key=lambda value: -value[2]):
            if not any(c == color and np.linalg.norm(point-p) < 2.0 for c, p, _ in merged):
                merged.append((color, point, stamp))
        return merged
