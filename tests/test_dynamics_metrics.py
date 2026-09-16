"""Analytic acceptance tests; no ROS/Gazebo fidelity claims."""
import importlib.util
import math
from pathlib import Path
import unittest


def module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parents[1] / "validation" / (name + ".py"))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


dynamics = module("dynamics_metrics")
numerical = module("numerical_acceptance")


def samples(speed=1.0, yaw=0.0):
    return [(i * 0.25, speed * i * 0.25, 0.0, speed, yaw) for i in range(81)]


class DynamicsMetricsTests(unittest.TestCase):
    def test_stationarity_requires_coverage_and_rejects_acceleration(self):
        self.assertTrue(dynamics.stationary(samples()))
        self.assertFalse(dynamics.stationary(samples()[:10]))
        self.assertFalse(dynamics.stationary([(t, x, y, t, yaw) for t, x, y, speed, yaw in samples()]))
        self.assertFalse(dynamics.stationary(samples()[::4]))
        bad = samples()
        bad[-1] = (20, 0, 0, float("nan"), 0)
        self.assertFalse(dynamics.stationary(bad))

    def test_unsettled_speed_is_not_reported_as_steady(self):
        rows = [(t, x, y, t, yaw) for t, x, y, speed, yaw in samples()]
        report = dynamics.summarize_experiment(rows, "straight")
        self.assertFalse(report["complete"])
        self.assertNotIn("steady_speed_mps", report["metrics"])

    def test_unstopped_coast_is_incomplete_lower_bound(self):
        report = dynamics.summarize_experiment(samples(), "coast")
        self.assertFalse(report["complete"])
        self.assertIsNone(report["metrics"]["coast_distance_m"])
        self.assertEqual(report["metrics"]["observed_distance_m"], 20)
        self.assertTrue(dynamics.summarize_experiment(samples(0), "coast")["complete"])

    def test_turn_radius_and_both_signs(self):
        for yaw in (-0.25, 0.25):
            report = dynamics.summarize_experiment(samples(1, yaw), "turn_left")
            self.assertTrue(report["complete"])
            self.assertEqual(report["metrics"]["turning_radius_m"], 4)

    def test_signed_reverse_and_vertical_settling(self):
        rows = [row + (0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0) for row in samples()]
        self.assertTrue(dynamics.summarize_experiment(rows, "reverse")["complete"])
        self.assertFalse(dynamics.summarize_experiment(rows, "straight")["complete"])
        moving = [row[:5] + (0.02 * row[0],) + row[6:9] + (0.02,) + row[10:] for row in rows]
        self.assertFalse(dynamics.stationary(moving))

    def test_settled_pitched_motion_uses_world_pose_not_body_heave(self):
        # A boat translating horizontally at fixed pitch has body z velocity.
        pitch, surge = -0.1, 2.0
        body_heave = surge * math.tan(pitch)
        rows = [row + (0.0, 0.0, pitch, surge, body_heave, 0.0, 0.0) for row in samples(surge)]
        self.assertTrue(dynamics.stationary(rows))
        self.assertEqual(dynamics.pose_drift(rows, 5), 0.0)

    def test_small_bounded_vibration_does_not_imply_secular_drift(self):
        rows = [row + (0.0, 0.0, 0.0002 * math.sin(20 * row[0]), 1.0,
                       0.0, 0.0, 0.004 * math.cos(20 * row[0])) for row in samples()]
        self.assertTrue(dynamics.stationary(rows))
        # A bounded but large oscillation and a small monotonically changing
        # angle are still rejected by the original pose excursion limits.
        large = [row[:7] + (row[7] * 10,) + row[8:] for row in rows]
        drift = [row[:7] + (row[0] * 0.0002,) + row[8:] for row in rows]
        self.assertFalse(dynamics.stationary(large))
        self.assertFalse(dynamics.stationary(drift))
        self.assertAlmostEqual(dynamics.pose_drift(drift, 7), 0.0002)

    def test_hydrostatic_requires_excitation_and_restoration(self):
        rows = [row + (-0.1, 0.05 if row[0] < 5 else 0.0, 0.05 if row[0] < 5 else 0.0,
                       0.0, 0.0, 0.0, 0.0) for row in samples(0)]
        report = dynamics.summarize_experiment(rows, "hydrostatic")
        self.assertTrue(report["complete"])
        self.assertAlmostEqual(report["metrics"]["mean_z_m"], -0.1)
        rows = [row[:6] + (0.05, 0.05) + row[8:] for row in rows]
        self.assertFalse(dynamics.summarize_experiment(rows, "hydrostatic")["complete"])

    def runs(self):
        return [dict(experiment="straight", comparison_group="fixture", provenance=f"manifest-{step}-{rep}",
                     step_s=step, repetition=rep, complete=True, metrics={"speed": 1.0, "yaw": 0.0})
                for step in numerical.STEPS for rep in range(3)]

    def test_convergence_requires_three_matched_repetitions_all_steps(self):
        units = {"speed": "m/s", "yaw": "rad/s"}
        self.assertTrue(numerical.compare(self.runs(), units)["passed"])
        self.assertFalse(numerical.compare(self.runs()[1:], units)["passed"])
        runs = self.runs()
        runs[0]["complete"] = False
        self.assertFalse(numerical.compare(runs, units)["passed"])

    def test_relative_and_absolute_tolerance_and_no_cancellation(self):
        units = {"speed": "m/s", "yaw": "rad/s"}
        runs = self.runs()
        runs[3]["metrics"]["yaw"] = 0.0009
        self.assertTrue(numerical.compare(runs, units)["passed"])
        runs[3]["metrics"]["speed"] = 1.03
        runs[4]["metrics"]["speed"] = 0.97
        self.assertFalse(numerical.compare(runs, units)["passed"])
        runs[3]["metrics"]["speed"] = float("nan")
        self.assertFalse(numerical.compare(runs, units)["passed"])


if __name__ == "__main__":
    unittest.main()
