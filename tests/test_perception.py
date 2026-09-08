"""Independent synthetic sensor/geometry regressions; no ROS installation needed."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"njord_sim"))
from njord_sim.geometry import transform_points, rotation_matrix
from njord_sim.mapping_core import OccupancyMapper
from njord_sim.perception_core import Blob, BuoyTracker, associate_lidar, choose_gate, detect_blobs


class GeometryTests(unittest.TestCase):
    def test_pitch_and_roll_rotate_height_not_just_heading(self):
        pitch = (0., math.sin(math.pi/4), 0., math.cos(math.pi/4))
        np.testing.assert_allclose(transform_points([[2, 0, 0]], [1, 2, 3], pitch), [[1, 2, 1]], atol=1e-12)
        roll = (math.sin(math.pi/4), 0., 0., math.cos(math.pi/4))
        np.testing.assert_allclose(transform_points([[0, 2, 0]], [0, 0, 0], roll), [[0, 0, 2]], atol=1e-12)

    def test_invalid_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            rotation_matrix([0, 0, 0, 0])


class MapTests(unittest.TestCase):
    def make_map(self, inflation=0):
        return OccupancyMapper(resolution=1., size_m=20., origin=(0., 0.), inflation_m=inflation, observation_ttl_s=3.)

    def test_ray_clears_only_observed_cells(self):
        mapper = self.make_map()
        mapper.update([2.1, 2.1], [[8.1, 2.1]], [True], 1.)
        grid = mapper.grid(1.)
        self.assertTrue(np.all(grid[2, 2:8] == 0))
        self.assertEqual(grid[2, 8], 100)
        self.assertEqual(grid[3, 3], -1)
        self.assertEqual(grid[2, 9], -1)

    def test_same_cloud_hits_win_and_next_observation_can_clear(self):
        mapper = self.make_map()
        mapper.update([2.1, 2.1], [[8.1, 2.1], [10.1, 2.1]], [True, False], 1.)
        self.assertEqual(mapper.grid(1.)[2, 8], 100)
        mapper.update([2.1, 2.1], [[10.1, 2.1]], [False], 2.)
        self.assertEqual(mapper.grid(2.)[2, 8], 0)

    def test_same_stamp_scan_does_not_erase_cloud_hit(self):
        mapper = self.make_map()
        mapper.update([2.1, 2.1], [[8.1, 2.1]], [True], 1.)
        mapper.update([2.1, 2.1], [[10.1, 2.1]], [False], 1.)
        self.assertEqual(mapper.grid(1.)[2, 8], 100)

    def test_aging_restores_unknown_including_inflation(self):
        mapper = self.make_map(inflation=2.)
        mapper.update([2.1, 2.1], [[8.1, 2.1]], [True], 1.)
        self.assertEqual(mapper.grid(1.)[3, 8], 100)
        self.assertTrue(np.all(mapper.grid(4.1) == -1))

    def test_old_cloud_does_not_freshen_map(self):
        mapper = self.make_map()
        mapper.update([2.1, 2.1], [[8.1, 2.1]], [True], 2.)
        self.assertFalse(mapper.update([2.1, 2.1], [[10.1, 2.1]], [False], 1.))
        self.assertEqual(mapper.last_stamp, 2.)
        self.assertEqual(mapper.grid(2.)[2, 8], 100)

    def test_negative_origin_and_boundaries(self):
        mapper = OccupancyMapper(resolution=.5, size_m=10., origin=(-5., -5.), inflation_m=0.)
        self.assertEqual(mapper.cell([-5.1, -5]), (-1, 0))
        self.assertFalse(mapper.update([-5.1, -5.], [[0, 0]], [True], 1.))
        self.assertTrue(mapper.update([0., 0.], [[10., 0.]], [True], 1.))
        self.assertTrue(np.all(mapper.grid(1.)[10, 10:] == 0))


class PerceptionTests(unittest.TestCase):
    def test_projection_selects_buoy_depth_not_background(self):
        points = np.array([[0, 0, 10], [.1, 0, 10], [0, 0, 30], [-.1, 0, 10], [0, 0, -2]])
        intrinsic = np.array([[100, 0, 50], [0, 100, 50], [0, 0, 1]])
        output = associate_lidar([Blob("red", (45, 45, 55, 55), .9)], points, points, intrinsic)
        self.assertEqual(len(output), 1)
        self.assertEqual(output[0][0], "red")
        np.testing.assert_allclose(output[0][1], [0, 0, 10])

    def test_single_return_is_insufficient(self):
        self.assertEqual(associate_lidar([Blob("red", (0, 0, 10, 10), .9)], [[0, 0, 2]], [[0, 0, 2]], np.eye(3)), [])

    def test_tracks_need_distinct_observations_and_do_not_retimestamp(self):
        tracker = BuoyTracker(minimum_observations=3)
        observation = [("red", np.array([10., 0., 1.]), .9)]
        self.assertEqual(tracker.update(observation, 1.), [])
        self.assertEqual(tracker.update(observation, 1.), [])
        self.assertEqual(tracker.update(observation, 1.1), [])
        self.assertEqual(len(tracker.update(observation, 1.2)), 1)
        self.assertEqual(tracker.update([], 1.3), [])
        self.assertEqual(tracker.tracks[0]["stamp"], 1.2)
        self.assertEqual(tracker.update(observation, 4.), [])

    def test_colored_blobs(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("OpenCV unavailable; run full suite inside ROS image")
        image = np.zeros((100, 150, 3), dtype=np.uint8)
        cv2.rectangle(image, (10, 20), (30, 70), (0, 0, 255), -1)
        cv2.rectangle(image, (90, 20), (110, 70), (0, 255, 0), -1)
        self.assertEqual(sorted(blob.color for blob in detect_blobs(image)), ["green", "red"])

    def test_nearest_gate_red_on_left_and_passed_filter(self):
        observations = [("red", [20, 8]), ("green", [20, -8]), ("red", [50, 8]), ("green", [50, -8])]
        gate = choose_gate(observations, [0, 0])
        np.testing.assert_allclose(gate.center, [20, 0])
        np.testing.assert_allclose(gate.forward, [1, 0])
        gate = choose_gate(observations, [0, 0], passed=[[20, 0]])
        np.testing.assert_allclose(gate.center, [50, 0])
        self.assertIsNone(choose_gate([("green", [20, 8]), ("red", [20, -8])], [0, 0]))

    def test_ambiguous_pairs_refused(self):
        observations = [("red", [20, 8]), ("green", [20, -8]), ("green", [21, -9])]
        self.assertIsNone(choose_gate(observations, [0, 0]))

    def test_gate_behind_vessel_is_not_selected(self):
        self.assertIsNone(choose_gate([("red", [-10, 8]), ("green", [-10, -8])], [0, 0]))


if __name__ == "__main__":
    unittest.main()
