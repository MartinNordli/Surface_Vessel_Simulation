"""Setpoint courses: scenario schema, sequencing and scoring without ROS."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "njord_sim"))
from njord_sim.configuration import guard_requirements, resolve_scenario
from njord_sim.scenario import world_xml
from njord_sim.scenario_core import load_scenario
from njord_sim.setpoint_core import SetpointCourse, SetpointObserver, commanded_yaw


def course(**setpoints):
    """A resolved setpoint scenario built from goto_square.yaml with other setpoints."""
    data = yaml.safe_load((ROOT / "scenarios/goto_square.yaml").read_text())
    if setpoints:
        data["setpoints"] = setpoints
    resolved = resolve_scenario(data)
    resolved["hull"] = {"length_m": 2.0, "beam_m": 1.0}
    return resolved


def drive(scorer, poses, start=0.0, step=0.1):
    """Feed (x, y, yaw) poses at ``step`` s intervals; returns all events."""
    events = []
    for i, pose in enumerate(poses):
        events += scorer.update(start + i * step, *pose)
    return events


class SetpointScenarioTests(unittest.TestCase):
    def test_shipped_setpoint_courses_resolve(self):
        for name in ("goto_square", "goto_retarget", "station_keeping"):
            with self.subTest(name=name):
                scenario = load_scenario(ROOT / f"scenarios/{name}.yaml")
                self.assertEqual(scenario["kind"], "setpoints")
                self.assertEqual(scenario["gates"], [])
                for item in scenario["setpoints"]:
                    for key in ("tolerance_m", "heading_tolerance_deg", "hold_s", "timeout_s"):
                        self.assertIn(key, item)
                json.dumps(scenario, allow_nan=False)
        self.assertEqual(load_scenario(ROOT / "scenarios/slalom.yaml")["kind"], "gates")

    def test_defaults_fill_and_entries_override(self):
        resolved = course(defaults={"tolerance_m": 2.0, "heading_tolerance_deg": 20.0, "hold_s": 1.0,
                                    "timeout_s": 30.0},
                          sequence=[{"name": "a", "position": [5.0, 0.0], "hold_s": 3.0},
                                    {"name": "b", "position": [5.0, 5.0], "heading_deg_enu": 90.0}])
        a, b = resolved["setpoints"]
        self.assertEqual((a["tolerance_m"], a["hold_s"], a["heading_deg_enu"]), (2.0, 3.0, None))
        self.assertEqual((b["hold_s"], b["heading_deg_enu"], b["advance_after_s"]), (1.0, 90.0, None))

    def test_invalid_setpoints_rejected(self):
        base = {"tolerance_m": 1.0, "heading_tolerance_deg": 10.0, "hold_s": 1.0, "timeout_s": 30.0}
        cases = {
            "missing setting": {"sequence": [{"name": "a", "position": [1.0, 0.0]}]},
            "duplicate name": {"defaults": base, "sequence": [{"name": "a", "position": [1.0, 0.0]},
                                                               {"name": "a", "position": [2.0, 0.0]}]},
            "3D position": {"defaults": base, "sequence": [{"name": "a", "position": [1.0, 0.0, 0.0]}]},
            "nonpositive tolerance": {"defaults": {**base, "tolerance_m": 0.0},
                                      "sequence": [{"name": "a", "position": [1.0, 0.0]}]},
            "heading tolerance": {"defaults": {**base, "heading_tolerance_deg": 181.0},
                                  "sequence": [{"name": "a", "position": [1.0, 0.0]}]},
            "last advances": {"defaults": base, "sequence": [{"name": "a", "position": [1.0, 0.0],
                                                              "advance_after_s": 3.0}]},
            "typo": {"defaults": base, "sequence": [{"name": "a", "position": [1.0, 0.0], "tolerence_m": 1}]},
            "empty": {"defaults": base, "sequence": []},
            "nan": {"defaults": base, "sequence": [{"name": "a", "position": [float("nan"), 0.0]}]},
        }
        for name, setpoints in cases.items():
            with self.subTest(name=name), self.assertRaises(ValueError):
                course(**setpoints)

    def test_course_has_exactly_one_kind_and_a_contact_model(self):
        data = yaml.safe_load((ROOT / "scenarios/goto_square.yaml").read_text())
        both = copy.deepcopy(data)
        both["gates"] = yaml.safe_load((ROOT / "scenarios/reference.yaml").read_text())["gates"]
        neither = copy.deepcopy(data)
        del neither["setpoints"]
        no_obstacle = copy.deepcopy(data)
        no_obstacle["obstacles"] = []
        jitter = copy.deepcopy(data)
        jitter["gate_y_jitter_m"] = 0.5
        for name, scenario in (("both", both), ("neither", neither), ("jitter", jitter)):
            with self.subTest(name=name), self.assertRaises(ValueError):
                resolve_scenario(scenario)
        from njord_sim.scenario_core import validate_scenario
        resolved = resolve_scenario(no_obstacle)
        resolved["hull"] = {"length_m": 2.0, "beam_m": 1.0}
        with self.assertRaisesRegex(ValueError, "obstacle"):
            validate_scenario(resolved)

    def test_world_has_only_the_obstacle_markers(self):
        world = world_xml(load_scenario(ROOT / "scenarios/goto_square.yaml"))
        self.assertIn('name="reference_buoy"', world)
        self.assertNotIn("_red", world)

    def test_guard_requirements_follow_course_kind(self):
        def required(**components):
            return guard_requirements(components, "race")["required_status"]
        self.assertEqual(required(course="setpoints"), ["navigation", "controller"])
        self.assertEqual(required(course="setpoints", controller="external"), ["navigation"])
        self.assertEqual(required(course="setpoints", autonomy="external"), ["navigation"])
        self.assertEqual(required(course="gates", controller="external"), ["navigation", "planner", "mission"])
        with self.assertRaises(ValueError):
            required(course="circle")


class SetpointCourseTests(unittest.TestCase):
    def specs(self, *items, **defaults):
        settings = {"tolerance_m": 1.0, "heading_tolerance_deg": 10.0, "hold_s": 1.0, "timeout_s": 20.0}
        settings.update(defaults)
        return course(defaults=settings, sequence=list(items))

    def test_targets_are_issued_in_order_after_the_hold(self):
        scorer = SetpointCourse(self.specs({"name": "a", "position": [2.0, 0.0], "heading_deg_enu": 0.0},
                                           {"name": "b", "position": [2.0, 2.0], "heading_deg_enu": 90.0}))
        events = scorer.update(0.0, 0.0, 0.0, 0.0)
        self.assertEqual([(e["type"], e["name"]) for e in events], [("issued", "a")])
        self.assertEqual(scorer.target(0), (2.0, 0.0, 0.0))
        events = drive(scorer, [(1.5, 0.0, 0.0)] * 10, start=1.0)  # inside from t=1.0
        self.assertEqual(events, [])
        events = scorer.update(2.0, 1.5, 0.0, 0.0)  # held 1.0 s
        self.assertEqual([(e["type"], e["name"]) for e in events], [("ended", "a"), ("issued", "b")])
        self.assertEqual(events[0]["outcome"], "reached")
        self.assertEqual([s["name"] for s in scorer.remaining()], ["b"])
        a = scorer.metrics()["setpoints"][0]
        self.assertAlmostEqual(a["time_to_reach_s"], 1.0)
        self.assertAlmostEqual(a["time_to_settle_s"], 1.0)
        self.assertAlmostEqual(a["hold_rms_distance_m"], 0.5)
        self.assertAlmostEqual(a["hold_duration_s"], 1.0)
        # Leaving the tolerance restarts the hold.
        drive(scorer, [(2.0, 1.5, math.pi / 2)] * 5, start=3.0)
        drive(scorer, [(2.0, -5.0, math.pi / 2)], start=3.5)
        self.assertEqual(scorer.status, "running")
        events = drive(scorer, [(2.0, 1.5, math.pi / 2)] * 12, start=4.0)
        self.assertEqual(scorer.status, "completed")
        metrics = scorer.metrics()
        self.assertTrue(metrics["reached_goal"])
        self.assertEqual((metrics["setpoints_reached"], metrics["setpoints_required"]), (2, 2))
        self.assertAlmostEqual(metrics["setpoints"][1]["time_to_settle_s"], 2.0)
        json.dumps(metrics, allow_nan=False)

    def test_heading_must_match_unless_free(self):
        scorer = SetpointCourse(self.specs({"name": "a", "position": [1.0, 0.0], "heading_deg_enu": 90.0}))
        drive(scorer, [(1.0, 0.0, 0.0)] * 30)
        self.assertEqual(scorer.status, "running")
        free = SetpointCourse(self.specs({"name": "a", "position": [1.0, 0.0]}))
        drive(free, [(1.0, 0.0, 0.0)] * 30)
        self.assertEqual(free.status, "completed")
        self.assertFalse(free.metrics()["setpoints"][0]["heading_scored"])

    def test_heading_error_wraps_across_pi(self):
        scorer = SetpointCourse(self.specs({"name": "a", "position": [0.0, 0.0], "heading_deg_enu": 180.0}))
        drive(scorer, [(0.0, 0.0, -math.pi + 0.05)] * 15)
        self.assertEqual(scorer.status, "completed")

    def test_timeout_fails_required_target_and_continues(self):
        scorer = SetpointCourse(self.specs({"name": "a", "position": [50.0, 0.0]},
                                           {"name": "b", "position": [0.0, 0.0]}, timeout_s=2.0))
        events = drive(scorer, [(0.0, 5.0, 0.0)] * 21)
        self.assertEqual([e["outcome"] for e in events if e["type"] == "ended"], ["timeout"])
        self.assertEqual(scorer.tracks[-1].spec["name"], "b")
        drive(scorer, [(0.0, 0.0, 0.0)] * 15, start=3.0)
        self.assertEqual(scorer.status, "setpoints_failed")
        self.assertFalse(scorer.metrics()["reached_goal"])

    def test_advance_after_replaces_target_without_requiring_it(self):
        scorer = SetpointCourse(self.specs({"name": "a", "position": [50.0, 0.0], "advance_after_s": 1.0},
                                           {"name": "b", "position": [0.0, 0.0]}))
        events = drive(scorer, [(0.0, 0.0, 0.0)] * 25)
        ended = [e for e in events if e["type"] == "ended"]
        self.assertEqual([(e["name"], e["outcome"]) for e in ended], [("a", "advanced"), ("b", "reached")])
        self.assertAlmostEqual([e for e in events if e["type"] == "issued"][1]["time_s"], 1.0)
        self.assertEqual(scorer.status, "completed")
        metrics = scorer.metrics()
        self.assertEqual((metrics["setpoints_required"], metrics["setpoints_reached"]), (1, 1))

    def test_free_heading_points_along_the_leg(self):
        self.assertAlmostEqual(commanded_yaw({"position": [0.0, 5.0], "heading_deg_enu": None}, (0.0, 0.0)),
                               math.pi / 2)
        self.assertAlmostEqual(commanded_yaw({"position": [0.0, 5.0], "heading_deg_enu": -90.0}, (0.0, 0.0)),
                               -math.pi / 2)

    def test_scenario_timeout_contact_and_invalid_data_end_the_run(self):
        scorer = SetpointCourse(self.specs({"name": "a", "position": [50.0, 0.0]}, timeout_s=1e3))
        scorer.scenario["timeout_s"] = 1.0
        drive(scorer, [(0.0, 0.0, 0.0)] * 12)
        self.assertEqual(scorer.status, "simulation_timeout")
        self.assertEqual(scorer.metrics()["setpoints"][0]["outcome"], "unfinished")
        crash = SetpointCourse(self.specs({"name": "a", "position": [50.0, 0.0]}))
        crash.update(0.0, 0.0, 0.0, 0.0)
        crash.contact(True, 0.0)
        self.assertEqual(crash.status, "collision")
        for bad in ((float("nan"), 0.0, 0.0), (0.0, float("inf"), 0.0)):
            broken = SetpointCourse(self.specs({"name": "a", "position": [50.0, 0.0]}))
            broken.update(0.0, *bad)
            self.assertEqual(broken.status, "invalid_ground_truth")
        reset = SetpointCourse(self.specs({"name": "a", "position": [50.0, 0.0]}))
        reset.update(5.0, 0.0, 0.0, 0.0)
        self.assertEqual(reset.update(5.0, 1.0, 0.0, 0.0), [])  # repeated stamp ignored
        reset.update(4.0, 0.0, 0.0, 0.0)
        self.assertEqual(reset.status, "clock_reset")
        json.dumps(reset.metrics(), allow_nan=False)

    def test_path_metrics_and_impulse(self):
        scorer = SetpointCourse(self.specs({"name": "a", "position": [4.0, 0.0]}, hold_s=0.0))
        for i, (x, y) in enumerate([(0, 0), (1, 1), (2, 2), (3, 1), (4, 0)]):
            scorer.update(float(i), x, y, 0.0, force_sum=100.0)
        a = scorer.metrics()["setpoints"][0]
        self.assertEqual(a["outcome"], "reached")
        self.assertAlmostEqual(a["path_length_m"], 4 * math.sqrt(2))
        self.assertAlmostEqual(a["max_cross_track_m"], 2.0)
        self.assertAlmostEqual(a["force_impulse_ns"], 400.0)
        self.assertAlmostEqual(a["overshoot_m"], 0.0)
        self.assertAlmostEqual(a["path_efficiency"], 4 / (4 * math.sqrt(2)))

    def test_overshoot_is_measured_past_the_target_along_the_approach(self):
        scorer = SetpointCourse(self.specs({"name": "a", "position": [10.0, 0.0]}, hold_s=1.0))
        for i, x in enumerate([0.0, 5.0, 9.5, 11.2, 10.4, 10.0, 10.0, 10.0]):
            scorer.update(float(i) * 0.5, x, 0.0, 0.0)
        self.assertAlmostEqual(scorer.metrics()["setpoints"][0]["overshoot_m"], 1.2)


class SetpointObserverTests(unittest.TestCase):
    def setUp(self):
        self.observer = SetpointObserver(course(), {"tolerance_m": 1.0, "heading_tolerance_deg": 10.0,
                                                    "hold_s": 1.0})

    def test_latest_target_wins(self):
        self.observer.update(0.0, 0.0, 0.0, 0.0)
        self.observer.issue(0.5, 10.0, 0.0, 0.0)
        drive(self.observer, [(1.0, 0.0, 0.0)] * 5, start=1.0)
        events = self.observer.issue(2.0, 0.0, 10.0, math.pi / 2, name="manual")
        self.assertEqual([(e["type"], e.get("outcome")) for e in events], [("ended", "superseded"), ("issued", None)])
        drive(self.observer, [(0.0, 10.0, math.pi / 2)] * 15, start=2.1)
        self.observer.stop()
        metrics = self.observer.metrics()
        self.assertEqual([s["outcome"] for s in metrics["setpoints"]], ["superseded", "reached"])
        self.assertEqual(metrics["setpoints"][1]["name"], "manual")
        self.assertEqual(metrics["status"], "stopped")
        json.dumps(metrics, allow_nan=False)

    def test_contacts_are_counted_without_ending_the_observation(self):
        self.observer.update(0.0, 0.0, 0.0, 0.0)
        self.observer.contact(True, 0.0)
        self.assertEqual(self.observer.status, "observing")
        self.observer.stop()
        self.assertTrue(self.observer.metrics()["collision"])


if __name__ == "__main__":
    unittest.main()
