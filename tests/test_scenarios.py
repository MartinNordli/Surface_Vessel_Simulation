"""Independent deterministic geometry, race and benchmark regressions."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from contextlib import redirect_stdout
import io
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "njord_sim"))
from njord_sim.scenario import generate, world_xml
from njord_sim.scenario_core import RaceScorer, gate_crossing, hull_clearance, load_scenario, obstacles, scenario_digest


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


benchmark = module("scripts/benchmark.py", "benchmark")
dynamics = module("validation/dynamics_metrics.py", "dynamics_metrics")


class ScenarioTests(unittest.TestCase):
    def setUp(self):
        self.scenario = load_scenario(ROOT / "scenarios/reference.yaml", seed=1)

    def test_reproducible_seed_and_distinct_environment(self):
        self.assertEqual(self.scenario, load_scenario(ROOT / "scenarios/reference.yaml", seed=1))
        second = load_scenario(ROOT / "scenarios/reference.yaml", seed=2)
        self.assertNotEqual(scenario_digest(self.scenario), scenario_digest(second))
        moderate = load_scenario(ROOT / "scenarios/reference.yaml", seed=1, environment="moderate")
        self.assertEqual(moderate["gates"], self.scenario["gates"])
        self.assertNotEqual(moderate["environment"], self.scenario["environment"])

    def test_resolved_configuration_is_not_randomized_twice(self):
        with tempfile.TemporaryDirectory() as output:
            generate(ROOT / "scenarios/reference.yaml", output, seed=4)
            resolved = Path(output) / "resolved_scenario.json"
            first = json.loads(resolved.read_text())
            self.assertEqual(first, load_scenario(resolved))
            with self.assertRaises(ValueError):
                load_scenario(resolved, seed=5)

    def test_zero_seed_rejected(self):
        with self.assertRaises(ValueError):
            load_scenario(ROOT / "scenarios/reference.yaml", seed=0)

    def test_world_and_scoring_geometry_share_single_source(self):
        world = ET.fromstring(world_xml(self.scenario)).find("world")
        self.assertEqual(world.attrib["name"], "njord_course")
        self.assertEqual(world.find("include/uri").text, "coast_waves")
        self.assertNotIn("http", world_xml(self.scenario))
        models = {m.attrib["name"]: m for m in world.findall("model")}
        self.assertEqual(len(models), len(obstacles(self.scenario)))
        for obstacle in obstacles(self.scenario):
            model = models[obstacle["name"]]
            pose = list(map(float, model.find("pose").text.split()))
            for a, b in zip(pose[:2], obstacle["position"]):
                self.assertAlmostEqual(a, b, places=8)
            radius = model.find("link/collision/geometry/cylinder/radius")
            self.assertEqual(float(radius.text), obstacle["radius_m"])
            self.assertEqual(model.find("link/sensor/contact/topic").text, "/njord/raw_contacts/" + obstacle["name"])
        monitor = world.find("plugin[@name='njord::ContactMonitor']")
        self.assertEqual({m.text for m in monitor.findall("marker")}, set(models))

    def test_direction_and_opening(self):
        gate = {"red": [10, 7], "green": [10, -7], "radius_m": 0.5}
        self.assertEqual(gate_crossing((0, 0), (20, 0), gate), (1, 0.5))
        self.assertEqual(gate_crossing((20, 0), (0, 0), gate), (-1, 0.5))
        self.assertIsNone(gate_crossing((0, 8), (20, 8), gate))
        self.assertIsNone(gate_crossing((0, 0), (5, 0), gate))

    def test_ordered_gates_crossed_in_one_message(self):
        scorer = RaceScorer(self.scenario)
        scorer.update(10, 0, 0, 0)
        scorer.update(90, 80, 0, 0)
        self.assertEqual(scorer.status, "completed")
        self.assertEqual(scorer.next_gate, 3)
        self.assertAlmostEqual(scorer.elapsed, 75)
        self.assertEqual(scorer.metrics()["collision"], None)
        scorer.contact(False)
        self.assertIs(scorer.metrics()["collision"], False)

    def test_wrong_order_and_reverse_fail(self):
        for start, end in [(40, 60), (30, 20)]:
            scorer = RaceScorer(self.scenario)
            scorer.update(0, start, 0, 0)
            scorer.update(1, end, 0, 0)
            self.assertEqual(scorer.status, "invalid_gate_order")

    def test_rotated_rectangle_clearance(self):
        circle = {"position": [5, 0], "radius_m": 0.5}
        self.assertAlmostEqual(hull_clearance((0, 0, 0), circle, 6, 2), 1.5)
        self.assertAlmostEqual(hull_clearance((0, 0, math.pi/2), circle, 6, 2), 3.5)

    def test_swept_collision_not_missed_between_samples(self):
        scorer = RaceScorer(self.scenario)
        scorer.update(0, 30, -7, 0)
        scorer.update(1, 45, -7, 0)
        self.assertEqual(scorer.status, "geometric_overlap")
        self.assertLess(scorer.min_clearance, 0)
        self.assertIsNone(scorer.metrics()["collision"])

    def test_contacts_and_geometry_are_separate(self):
        scorer = RaceScorer(self.scenario)
        scorer.contact(True)
        self.assertEqual(scorer.status, "collision")
        self.assertIs(scorer.metrics()["collision"], True)
        self.assertIsNone(scorer.metrics()["min_clearance_m"])

    def test_clock_reset_duplicate_and_timeout(self):
        scorer = RaceScorer(self.scenario)
        scorer.update(10, 0, 0, 0)
        scorer.update(10, 1, 0, 0)
        self.assertEqual(scorer.distance, 0)
        scorer.update(9, 0, 0, 0)
        self.assertEqual(scorer.status, "clock_reset")
        scorer = RaceScorer(self.scenario)
        scorer.update(10, 0, 0, 0)
        scorer.update(10 + self.scenario["timeout_s"] + 1, 0, 0, 0)
        self.assertEqual(scorer.status, "simulation_timeout")

    def test_invalid_data_and_empty_run_produce_strict_json(self):
        scorer = RaceScorer(self.scenario)
        json.dumps(scorer.metrics(), allow_nan=False)
        scorer.update(0, float("nan"), 0, 0)
        self.assertEqual(scorer.status, "invalid_ground_truth")
        json.dumps(scorer.metrics(), allow_nan=False)

    def test_benchmark_requires_measured_contacts(self):
        run = {"label": "example", "seed": 1, "environment": "calm", "profile": "fast",
               "status": "completed", "collision": None, "geometric_overlap": False,
               "contact_status": "unavailable", "time_s": 20}
        self.assertFalse(benchmark.summarize([run])["all_runs_verified"])
        fast = run | {"collision": False, "contact_status": "observed"}
        slow = fast | {"label": "slow", "profile": "conservative", "time_s": 25}
        summary = benchmark.summarize([slow, fast])
        self.assertTrue(summary["all_runs_verified"])
        self.assertTrue(summary["comparisons"]["calm"]["fast_improves_median_without_failures"])

    def test_turn_uses_turn_speed_and_coast_integrates_arc(self):
        samples = [(0, 0, 0, 0, 0), (5, 0, 0, 10, 0),
                   (16, 0, 0, 2, 0.5), (19, 0, 0, 2, 0.5),
                   (20, 0, 0, 2, 0.5), (21, 1, 0, 1, 0.5),
                   (22, 1, 1, 0.5, 0.5), (23, 0, 1, 0, 0)]
        metrics = dynamics.summarize(samples, 10, 10)
        self.assertEqual(metrics["turning_radius_m"], 4)
        self.assertEqual(metrics["coast_distance_m"], 3)
        self.assertTrue(metrics["stopped_within_observation"])
        samples[-1] = (23, 0, 1, 0.5, 0)
        self.assertFalse(dynamics.summarize(samples, 10, 10)["stopped_within_observation"])

    def test_parallel_benchmark_holds_unique_domains_and_collects_all_runs(self):
        active, observed, collisions = set(), [], []
        lock = threading.Lock()
        barrier = threading.Barrier(2)

        def fake_run(index, item, total, output, env, timeout, domain, cancelled):
            with lock:
                if domain in active:
                    collisions.append(domain)
                active.add(domain)
                observed.append((index, domain))
            barrier.wait(timeout=2)
            time.sleep(0.02)
            with lock:
                active.remove(domain)
            environment, seed, profile = item
            return {"label": f"{environment}-{seed}-{profile}", "environment": environment,
                    "seed": seed, "profile": profile, "status": "completed", "collision": False,
                    "geometric_overlap": False, "contact_status": "observed",
                    "time_s": 20 if profile == "fast" else 25}

        def fake_output(command, env=None):
            if command[:3] == ["git", "status", "--porcelain"]:
                return ""
            return "test-identity"

        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "benchmark"
            argv = ["benchmark.py", "--jobs", "2", "--seeds", "1", "2",
                    "--environments", "calm", "--output-dir", str(output)]
            with patch.object(sys, "argv", argv), patch.object(benchmark, "run_one", fake_run), \
                    patch.object(benchmark, "command_output", fake_output), redirect_stdout(io.StringIO()):
                self.assertEqual(benchmark.main(), 0)
            report = json.loads((output / "summary.json").read_text())
            self.assertEqual(len(report["runs"]), 4)
            self.assertEqual(report["manifest"]["jobs"], 2)
            self.assertEqual(len(observed), 4)
            self.assertEqual(len({domain for _, domain in observed}), 2)
            self.assertFalse(collisions)


if __name__ == "__main__":
    unittest.main()
