"""Static slalom feasibility checks; no sensor, dynamics or Gazebo claims.

The fully observed grid below deliberately uses scenario truth for offline
validation only. It does not model lidar visibility or live map uncertainty.
"""

import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "njord_sim"))

from njord_sim.control_core import observed_free, segment_is_free
from njord_sim.dstar_lite import astar
from njord_sim.perception_core import choose_gate
from njord_sim.planner_core import Geometry, IncrementalPlanner
from njord_sim.scenario import world_xml
from njord_sim.scenario_core import RaceScorer, load_scenario, obstacles, scenario_digest


SCENARIO = ROOT / "scenarios/slalom.yaml"
GEOMETRY = Geometry(320, 320, 0.5, -40.0, -40.0)
INFLATION_M = 4.0


def inflated_grid(markers):
    """Block every cell touched by an analytically inflated obstacle circle."""
    axis = (np.arange(GEOMETRY.width) + 0.5) * GEOMETRY.resolution
    x, y = np.meshgrid(axis + GEOMETRY.x, axis + GEOMETRY.y)
    blocked = np.zeros(x.shape, dtype=bool)
    half_cell = GEOMETRY.resolution / 2
    for marker in markers:
        dx = np.maximum(np.abs(x - marker["position"][0]) - half_cell, 0)
        dy = np.maximum(np.abs(y - marker["position"][1]) - half_cell, 0)
        blocked |= dx * dx + dy * dy <= (marker["radius_m"] + INFLATION_M) ** 2
    return np.where(blocked, 100, 0).ravel().tolist()


def gate_points(gate):
    red, green = np.asarray(gate["red"]), np.asarray(gate["green"])
    center = (red + green) / 2
    across = red - green
    forward = np.array([across[1], -across[0]]) / np.linalg.norm(across)
    # Existing mission approach/exit defaults, in metres.
    return center - 5 * forward, center, center + 6 * forward


class SlalomCourseTests(unittest.TestCase):
    def test_seeded_geometry_repeats_and_environment_preserves_course(self):
        digests = set()
        for seed in range(1, 11):
            with self.subTest(seed=seed):
                scenario = load_scenario(SCENARIO, seed=seed)
                self.assertEqual(scenario, load_scenario(SCENARIO, seed=seed))
                moderate = load_scenario(SCENARIO, seed=seed, environment="moderate")
                self.assertEqual(scenario["gates"], moderate["gates"])
                self.assertEqual(scenario["obstacles"], moderate["obstacles"])
                self.assertNotEqual(scenario["environment"], moderate["environment"])
                self.assertEqual(len(scenario["gates"]), 5)
                self.assertEqual(len(scenario["obstacles"]), 5)
                digests.add(scenario_digest(scenario))
        self.assertEqual(len(digests), 10)

    def test_world_collisions_and_contact_monitor_match_scoring_geometry(self):
        for seed in range(1, 11):
            with self.subTest(seed=seed):
                scenario = load_scenario(SCENARIO, seed=seed)
                world = ET.fromstring(world_xml(scenario)).find("world")
                models = {model.attrib["name"]: model for model in world.findall("model")}
                expected = obstacles(scenario)
                self.assertEqual(set(models), {marker["name"] for marker in expected})
                for marker in expected:
                    model = models[marker["name"]]
                    position = list(map(float, model.find("pose").text.split()))[:2]
                    for actual, wanted in zip(position, marker["position"]):
                        self.assertAlmostEqual(actual, wanted, places=8)
                    radius = model.find("link/collision/geometry/cylinder/radius")
                    self.assertEqual(float(radius.text), marker["radius_m"])
                    self.assertEqual(model.find("link/sensor/contact/topic").text,
                                     "/njord/raw_contacts/" + marker["name"])
                monitor = world.find("plugin[@name='njord::ContactMonitor']")
                self.assertEqual({marker.text for marker in monitor.findall("marker")}, set(models))

    def test_next_gate_is_unambiguous_from_start_and_each_previous_exit(self):
        for seed in range(1, 11):
            scenario = load_scenario(SCENARIO, seed=seed)
            detections = [(color, gate[color]) for gate in scenario["gates"]
                          for color in ("red", "green")]
            position, heading, passed = scenario["start"][:2], scenario["start"][5], []
            for index, gate in enumerate(scenario["gates"]):
                with self.subTest(seed=seed, gate=index):
                    chosen = choose_gate(detections, position, heading, passed)
                    self.assertIsNotNone(chosen)
                    np.testing.assert_allclose(chosen.red, gate["red"])
                    np.testing.assert_allclose(chosen.green, gate["green"])
                    _, center, position = gate_points(gate)
                    heading = math.atan2(chosen.forward[1], chosen.forward[0])
                    passed.append(center)

    def test_next_buoy_center_lines_are_clear_and_inside_nominal_camera_fov(self):
        # At start use the start heading; at each gate exit assume the boat is
        # aligned with that gate's forward normal. These planar center rays and
        # the standard 80-degree horizontal FOV do not establish live visibility:
        # actual yaw, camera pitch, rendered pixels and detection remain untested.
        sensors = ET.parse(ROOT / "njord_sim/config/sensors.xacro")
        cameras = sensors.findall(".//{http://ros.org/wiki/xacro}wamv_camera")
        self.assertEqual(len(cameras), 2)
        for seed in range(1, 11):
            scenario = load_scenario(SCENARIO, seed=seed)
            position = np.asarray(scenario["start"][:2])
            yaw = scenario["start"][5]
            for gate in scenario["gates"]:
                rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                                     [math.sin(yaw), math.cos(yaw)]])
                for camera in cameras:
                    offset = np.array([float(camera.attrib["x"]), float(camera.attrib["y"])])
                    origin = position + rotation @ offset
                    for color in ("red", "green"):
                        with self.subTest(seed=seed, gate=gate["name"],
                                          camera=camera.attrib["name"], buoy=color):
                            ray = np.asarray(gate[color]) - origin
                            local_ray = rotation.T @ ray
                            bearing = math.atan2(local_ray[1], local_ray[0])
                            self.assertLess(abs(bearing), math.radians(40),
                                            "buoy center outside nominal horizontal camera FOV")
                            for marker in obstacles(scenario):
                                if marker["name"] == gate["name"] + "_" + color:
                                    continue
                                relative = np.asarray(marker["position"]) - origin
                                fraction = float(np.clip(relative @ ray / (ray @ ray), 0, 1))
                                distance = float(np.linalg.norm(relative - fraction * ray))
                                self.assertGreater(distance, marker["radius_m"],
                                                   f"{marker['name']} blocks the planar buoy-center line")
                _, center, position = gate_points(gate)
                forward = position - center
                yaw = math.atan2(forward[1], forward[0])

    def test_each_extra_obstacle_requires_a_detour_between_mission_targets(self):
        for seed in range(1, 11):
            scenario = load_scenario(SCENARIO, seed=seed)
            previous = scenario["start"][:2]
            for gate, obstacle in zip(scenario["gates"], scenario["obstacles"]):
                with self.subTest(seed=seed, obstacle=obstacle["name"]):
                    before, _, after = gate_points(gate)
                    # Test this obstacle alone, so a gate buoy cannot satisfy
                    # the assertion that the extra obstacle causes an evasion.
                    data = inflated_grid([obstacle])
                    self.assertTrue(observed_free(GEOMETRY, data, GEOMETRY.cell(previous)))
                    self.assertTrue(observed_free(GEOMETRY, data, GEOMETRY.cell(before)))
                    self.assertFalse(segment_is_free(GEOMETRY, data, previous, before))
                    previous = after

    def test_inflated_routes_match_astar_and_score_all_five_gates_in_order(self):
        for seed in range(1, 11):
            with self.subTest(seed=seed):
                scenario = load_scenario(SCENARIO, seed=seed)
                data = inflated_grid(obstacles(scenario))
                position = tuple(scenario["start"][:2])
                route = [position]
                planner = IncrementalPlanner()
                for index, gate in enumerate(scenario["gates"]):
                    before, center, after = gate_points(gate)
                    self.assertTrue(segment_is_free(GEOMETRY, data, before, after),
                                    f"seed {seed}, gate {index}: obstructed crossing corridor")
                    for target in (before, center, after):
                        self.assertTrue(observed_free(GEOMETRY, data, GEOMETRY.cell(target)),
                                        f"seed {seed}, gate {index}: blocked mission target")
                        path = planner.plan(GEOMETRY, data, position, target)
                        self.assertTrue(path, f"seed {seed}, gate {index}: no inflated route")
                        self.assertAlmostEqual(planner.search.path_cost(), astar(
                            planner.search.grid, GEOMETRY.cell(position), GEOMETRY.cell(target)))
                        for point in path:
                            if math.dist(position, point) > 1e-9:
                                self.assertTrue(segment_is_free(GEOMETRY, data, position, point))
                                route.append(point)
                                position = point
                scorer = RaceScorer(scenario)
                for index, point in enumerate(route):
                    following = route[min(index + 1, len(route) - 1)]
                    yaw = math.atan2(following[1] - point[1], following[0] - point[0])
                    scorer.update(index * 0.1, *point, yaw)
                metrics = scorer.metrics()
                self.assertEqual(metrics["status"], "completed")
                self.assertEqual(metrics["gates_passed"], 5)
                self.assertEqual([event["gate"] for event in metrics["gate_events"]],
                                 [gate["name"] for gate in scenario["gates"]])
                self.assertTrue(all(event["direction"] == 1 for event in metrics["gate_events"]))
                self.assertGreater(metrics["min_clearance_m"], 0)
                # Synthetic geometry cannot establish live contact monitoring.
                self.assertIsNone(metrics["collision"])


if __name__ == "__main__":
    unittest.main()
