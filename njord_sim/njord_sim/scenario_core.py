"""Scenario resolution and race scoring without ROS or Gazebo dependencies.

Scenario geometry is evaluator-only. The vessel envelope is a conservative
horizontal rectangle, not the detailed WAM-V collision mesh. Contact events
and geometric envelope overlap are deliberately reported separately.
"""

import copy
import hashlib
import json
import math
from pathlib import Path
import random


def load_scenario(path, seed=None, environment=None):
    text = Path(path).read_text()
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        import yaml
        data = yaml.safe_load(text)
    data = copy.deepcopy(data)
    # Resolved files must never receive a second random offset.
    if data.get("resolved"):
        if seed is not None and seed != data["seed"]:
            raise ValueError("cannot reseed a resolved scenario")
        if environment is not None and environment != data["environment_name"]:
            raise ValueError("cannot change environment of a resolved scenario")
        validate_scenario(data)
        return data
    data["seed"] = int(data.get("seed", 1) if seed is None else seed)
    # USVWind interprets zero as a nondeterministic seed.
    if data["seed"] <= 0:
        raise ValueError("seed must be a positive integer")
    environment = environment or "calm"
    data["environment_name"] = environment
    data["environment"] = copy.deepcopy(data["environments"][environment])
    rng = random.Random(data["seed"])
    jitter = float(data.get("gate_y_jitter_m", 0.0))
    for gate in data["gates"]:
        offset = rng.uniform(-jitter, jitter)
        gate["red"][1] += offset
        gate["green"][1] += offset
    data["resolved"] = True
    validate_scenario(data)
    return data


def validate_scenario(data):
    # This also refuses NaN/Infinity anywhere in configuration and provenance.
    json.dumps(data, allow_nan=False)
    if not data.get("gates") or len(data["start"]) != 6:
        raise ValueError("scenario requires gates and a six-value start pose")
    if data["timeout_s"] <= 0 or min(data["hull"].values()) <= 0:
        raise ValueError("timeout and hull dimensions must be positive")
    names = set()
    for gate in data["gates"]:
        if len(gate["red"]) != 2 or len(gate["green"]) != 2:
            raise ValueError("gate positions must be 2D")
        if gate["radius_m"] <= 0 or math.dist(gate["red"], gate["green"]) <= 2 * gate["radius_m"]:
            raise ValueError("gate must have positive radius and non-overlapping markers")
    for obstacle in obstacles(data):
        if obstacle["name"] in names:
            raise ValueError("obstacle names must be unique")
        names.add(obstacle["name"])
        if len(obstacle["position"]) != 2 or obstacle["radius_m"] <= 0:
            raise ValueError("obstacles require 2D positions and positive radii")
    if data["environment"]["wave_period_s"] <= 0:
        raise ValueError("wave period must be positive")


def scenario_digest(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


def obstacles(data):
    result = []
    for gate in data["gates"]:
        for color in ("red", "green"):
            result.append({"name": gate["name"] + "_" + color,
                           "position": gate[color], "radius_m": gate["radius_m"],
                           "color": color})
    result.extend(data.get("obstacles", []))
    return result


def gate_crossing(previous, current, gate):
    """Return (direction, fraction) for a segment crossing the gate opening.

    +1 is red-to-port/green-to-starboard forward passage. The crossing test
    is swept between samples so a vessel cannot skip a gate at low odom rate.
    """
    red, green = gate["red"], gate["green"]
    dx, dy = green[0] - red[0], green[1] - red[1]
    width = math.hypot(dx, dy)
    tx, ty = dx / width, dy / width
    nx, ny = -ty, tx
    before = (previous[0] - red[0]) * nx + (previous[1] - red[1]) * ny
    after = (current[0] - red[0]) * nx + (current[1] - red[1]) * ny
    direction = 1 if before < 0 <= after else -1 if before > 0 >= after else 0
    if not direction:
        return None
    fraction = -before / (after - before)
    x = previous[0] + fraction * (current[0] - previous[0])
    y = previous[1] + fraction * (current[1] - previous[1])
    along = (x - red[0]) * tx + (y - red[1]) * ty
    if gate["radius_m"] <= along <= width - gate["radius_m"]:
        return direction, fraction
    return None


def hull_clearance(pose, obstacle, length, beam):
    """Signed horizontal rectangle-to-circle clearance in metres."""
    x, y, yaw = pose
    dx = obstacle["position"][0] - x
    dy = obstacle["position"][1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    qx, qy = abs(local_x) - length / 2, abs(local_y) - beam / 2
    signed_distance = math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)
    return signed_distance - obstacle["radius_m"]


class RaceScorer:
    def __init__(self, scenario):
        self.scenario = scenario
        self.obstacles = obstacles(scenario)
        self.previous = None
        self.previous_time = None
        self.start_time = None
        self.elapsed = 0.0
        self.distance = 0.0
        self.min_clearance = math.inf
        self.gate_events = []
        self.next_gate = 0
        self.status = "running"
        self.contact_messages = 0
        self.contact_events = 0

    def contact(self, vessel_contact):
        self.contact_messages += 1
        if vessel_contact:
            self.contact_events += 1
            if self.status == "running":
                self.status = "collision"

    def update(self, stamp, x, y, yaw):
        if self.status != "running":
            return
        if not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            self.status = "invalid_ground_truth"
            return
        if self.previous_time is not None and stamp <= self.previous_time:
            if stamp < self.previous_time:
                self.status = "clock_reset"
            return
        if self.start_time is None:
            self.start_time = stamp
        self.elapsed = stamp - self.start_time
        pose = (x, y, yaw)
        if self.previous is not None:
            travel = math.dist(self.previous[:2], pose[:2])
            self.distance += travel
            # Interpolate hull positions at <=0.1 m / 2 degrees to catch swept
            # envelope overlap between odometry messages, including rotation.
            delta_yaw = math.atan2(math.sin(yaw - self.previous[2]), math.cos(yaw - self.previous[2]))
            steps = max(1, math.ceil(travel / 0.1), math.ceil(abs(delta_yaw) / math.radians(2)))
            for i in range(steps):
                f = i / steps
                self._clearance((self.previous[0] + f * (x - self.previous[0]),
                                 self.previous[1] + f * (y - self.previous[1]),
                                 self.previous[2] + f * delta_yaw))
            crossings = []
            for idx, gate in enumerate(self.scenario["gates"]):
                crossing = gate_crossing(self.previous, pose, gate)
                if crossing:
                    crossings.append((crossing[1], idx, crossing[0]))
            for fraction, idx, direction in sorted(crossings):
                event_time = self.previous_time + fraction * (stamp - self.previous_time)
                self.gate_events.append({"gate": self.scenario["gates"][idx]["name"],
                                         "time_s": event_time - self.start_time,
                                         "direction": direction})
                if direction != 1 or idx != self.next_gate:
                    self.status = "invalid_gate_order"
                    break
                self.next_gate += 1
        self._clearance(pose)
        self.previous, self.previous_time = pose, stamp
        if self.min_clearance <= 0:
            self.status = "geometric_overlap"
        elif self.status == "running":
            if self.next_gate == len(self.scenario["gates"]):
                self.status = "completed"
                self.elapsed = self.gate_events[-1]["time_s"]
            elif self.elapsed >= self.scenario["timeout_s"]:
                self.status = "simulation_timeout"

    def _clearance(self, pose):
        hull = self.scenario["hull"]
        for obstacle in self.obstacles:
            self.min_clearance = min(self.min_clearance, hull_clearance(
                pose, obstacle, hull["length_m"], hull["beam_m"]))

    def metrics(self):
        return {
            "status": self.status,
            "reached_goal": self.status == "completed",
            "time_s": self.elapsed,
            "distance_travelled_m": self.distance,
            "min_clearance_m": self.min_clearance if math.isfinite(self.min_clearance) else None,
            "clearance_model": "horizontal_conservative_rectangle_swept_0.1m_2deg",
            "geometric_overlap": self.min_clearance <= 0,
            "collision": bool(self.contact_events) if self.contact_messages else None,
            "contact_status": "observed" if self.contact_messages else "unavailable",
            "contact_message_count": self.contact_messages,
            "contact_event_count": self.contact_events,
            "gates_passed": self.next_gate,
            "gates_total": len(self.scenario["gates"]),
            "gate_events": self.gate_events,
        }
