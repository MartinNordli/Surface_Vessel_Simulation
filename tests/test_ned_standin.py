"""NED/FRD stand-in controller: the conversion sent to Control & Autonomy.

Pure-Python checks of validation/ned_dp_standin.py against the simulator's own
forward/left/up helpers. They verify signs and the thruster layout, not the
simulated or real boat's behaviour.
"""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "njord_sim"))
from njord_sim.configuration import _read, thruster_table, validate_vessel
from njord_sim.control_core import allocate_thrusters

spec = importlib.util.spec_from_file_location("ned_dp_standin", ROOT / "validation/ned_dp_standin.py")
standin = importlib.util.module_from_spec(spec)
spec.loader.exec_module(standin)

VESSELS = ROOT / "njord_sim/config/vessels"
ALGORITHMS = ROOT / "njord_sim/config/algorithms.yaml"
GAINS = {'kp_position': 100.0, 'kd_position': 200.0, 'kp_yaw': 400.0, 'kd_yaw': 300.0}


class ConversionTests(unittest.TestCase):
    def test_pose_swaps_axes_and_measures_heading_clockwise_from_north(self):
        cases = {0.0: math.pi / 2, math.pi / 2: 0.0, math.pi: -math.pi / 2, -math.pi / 2: math.pi}
        for yaw_enu, psi in cases.items():
            x, y, heading = standin.ned_pose(3.0, 4.0, yaw_enu)
            self.assertEqual((x, y), (4.0, 3.0))
            self.assertAlmostEqual(abs(standin.wrap(heading - psi)), 0.0)
            self.assertTrue(-math.pi <= heading <= math.pi)

    def test_velocity_keeps_surge_and_flips_sway_and_yaw_rate(self):
        # 0.5 m/s to port and 0.2 rad/s counter-clockwise in the simulator's body frame.
        self.assertEqual(standin.frd_velocity(1.0, 0.5, 0.2), (1.0, -0.5, -0.2))

    def test_thrusters_mirror_position_and_angle(self):
        table = thruster_table(validate_vessel(_read(VESSELS / "munin_v0.yaml")))
        bow_port = standin.frd_thrusters(table, [0.0, 0.0, 0.0])[0]
        self.assertAlmostEqual(bow_port[0], 1.1)
        self.assertAlmostEqual(bow_port[1], -0.55)              # port is negative y in FRD
        self.assertAlmostEqual(math.degrees(bow_port[2]), 45.0)  # pushes forward and to starboard


class AllocationTests(unittest.TestCase):
    def setUp(self):
        self.table, self.matrix, self.limits, self.settings = standin.load(VESSELS / "munin_v0.yaml", ALGORITHMS)
        self.positions = [v for t in self.table for v in t['position_m']]
        self.axes = [v for t in self.table for v in t['axis']]

    def test_matches_the_simulator_allocation_of_the_mirrored_wrench(self):
        for wrench in [(100.0, 0.0, 0.0), (0.0, 80.0, 0.0), (0.0, 0.0, 50.0), (60.0, -40.0, 25.0)]:
            with self.subTest(wrench=wrench):
                forces = standin.thruster_forces(wrench, self.matrix, self.limits)
                # The same wrench in forward/left/up: sway and yaw moment change sign.
                reference = allocate_thrusters((wrench[0], -wrench[1], -wrench[2]), self.positions, self.axes,
                                               self.limits, self.limits)
                for force, expected in zip(forces, reference):
                    self.assertAlmostEqual(force, expected, places=6)

    def test_forces_are_scaled_together_to_the_thruster_limit(self):
        forces = standin.thruster_forces((1e5, 2e4, 0.0), self.matrix, self.limits)
        self.assertAlmostEqual(max(abs(f) for f in forces), min(self.limits))
        small = standin.thruster_forces((1e2, 2e1, 0.0), self.matrix, self.limits)
        for big, little in zip(forces, small):
            self.assertAlmostEqual(big / forces[0], little / small[0])

    def test_gains_and_limits_come_from_the_configuration_files(self):
        for key, value in GAINS.items():
            self.assertEqual(self.settings[key], _read(ALGORITHMS)['setpoint_control'][key])
        self.assertEqual(len(self.limits), 4)

    def test_rejects_a_vessel_that_cannot_push_sideways(self):
        with self.assertRaisesRegex(ValueError, 'fully actuated'):
            standin.load(VESSELS / "njord_v1.yaml", ALGORITHMS)


class ControlLawTests(unittest.TestCase):
    def test_error_is_rotated_into_the_body_frame(self):
        still = (0.0, 0.0, 0.0)
        # Heading north, target 2 m north: push forward only.
        self.assertEqual(standin.dp_wrench((0.0, 0.0, 0.0), still, (2.0, 0.0, 0.0), GAINS), (200.0, 0.0, 0.0))
        # Heading east (psi = 90 deg), target 2 m north: north is to port, so push to port (negative Y).
        x, y, n = standin.dp_wrench((0.0, 0.0, math.pi / 2), still, (2.0, 0.0, math.pi / 2), GAINS)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, -200.0)
        self.assertAlmostEqual(n, 0.0)

    def test_heading_error_wraps_and_velocity_is_damped(self):
        state, target = (0.0, 0.0, math.radians(170.0)), (0.0, 0.0, math.radians(-170.0))
        x, y, n = standin.dp_wrench(state, (1.0, -0.5, 0.1), target, GAINS)
        self.assertAlmostEqual(n, 400.0 * math.radians(20.0) - 300.0 * 0.1)  # the short way, clockwise
        self.assertAlmostEqual(x, -200.0)
        self.assertAlmostEqual(y, 100.0)

    def test_closed_loop_reaches_and_holds_an_enu_target(self):
        # Planar point mass with linear damping, integrated in the simulator's ENU and
        # forward/left/up conventions. The stand-in only sees the converted values.
        mass, inertia, damping = 200.0, 160.0, (40.0, 100.0, 50.0)
        x = y = yaw = u = v = r = 0.0
        target_enu, step = (6.0, -4.0, math.radians(120.0)), 0.05
        target = standin.ned_pose(*target_enu)
        table, matrix, limits, _ = standin.load(VESSELS / "munin_v0.yaml", ALGORITHMS)
        for _ in range(int(120.0 / step)):
            wrench = standin.dp_wrench(standin.ned_pose(x, y, yaw), standin.frd_velocity(u, v, r), target, GAINS)
            forces = standin.thruster_forces(wrench, matrix, limits)
            fx = sum(f * t['axis'][0] for f, t in zip(forces, table))
            fy = sum(f * t['axis'][1] for f, t in zip(forces, table))
            mz = sum(f * (t['position_m'][0] * t['axis'][1] - t['position_m'][1] * t['axis'][0])
                     for f, t in zip(forces, table))
            u += step * (fx - damping[0] * u) / mass
            v += step * (fy - damping[1] * v) / mass
            r += step * (mz - damping[2] * r) / inertia
            x += step * (math.cos(yaw) * u - math.sin(yaw) * v)
            y += step * (math.sin(yaw) * u + math.cos(yaw) * v)
            yaw += step * r
        self.assertLess(math.hypot(x - target_enu[0], y - target_enu[1]), 0.05)
        self.assertLess(abs(standin.wrap(yaw - target_enu[2])), math.radians(1.0))


if __name__ == "__main__":
    unittest.main()
