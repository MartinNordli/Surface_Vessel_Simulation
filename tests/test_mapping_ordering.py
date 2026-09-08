"""Cloud/scan acquisition ordering must not turn observed obstacles into free cells."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from njord_sim.mapping_core import OccupancyMapper


class MappingOrderingTests(unittest.TestCase):
    def mapper(self):
        return OccupancyMapper(resolution=1, size_m=20, origin=(0, 0), inflation_m=0,
                               observation_ttl_s=3)

    def scan(self, mapper, stamp):
        return mapper.update((1.5, 10.5), [(18.5, 10.5)], [False], stamp, stream='scan')

    def cloud(self, mapper, stamp, hit=True):
        return mapper.update((1.5, 10.5), [(8.5, 10.5)], [hit], stamp, stream='cloud')

    def test_recent_hit_wins_in_both_cross_stream_arrival_orders(self):
        results = []
        for cloud_first in (False, True):
            mapper = self.mapper()
            operations = [lambda: self.scan(mapper, 10.1), lambda: self.cloud(mapper, 10.0)]
            if cloud_first:
                operations.reverse()
            self.assertTrue(all(operation() for operation in operations))
            self.assertEqual(mapper.grid(10.2)[10, 8], 100)
            self.assertEqual(mapper.observed[10, 8], 10.0)
            self.assertEqual(mapper.observed[10, 4], 10.1)
            self.assertEqual(mapper.last_stamp, 10.1)
            results.append(mapper.grid(10.2))
        np.testing.assert_array_equal(*results)

    def test_stale_free_ray_cannot_clear_or_freshen_newer_hit(self):
        mapper = self.mapper()
        self.cloud(mapper, 10.2)
        self.assertTrue(self.scan(mapper, 10.0))
        self.assertEqual(mapper.grid(10.2)[10, 8], 100)
        self.assertEqual(mapper.observed[10, 8], 10.2)
        self.assertEqual(mapper.grid(13.21)[10, 8], -1)

    def test_old_hit_cannot_override_substantially_newer_free_observation(self):
        mapper = self.mapper()
        self.scan(mapper, 10.5)
        self.assertTrue(self.cloud(mapper, 10.0))
        self.assertEqual(mapper.grid(10.5)[10, 8], 0)
        self.assertEqual(mapper.observed[10, 8], 10.5)

    def test_new_free_observation_clears_after_hit_precedence_window(self):
        mapper = self.mapper()
        self.cloud(mapper, 10.0)
        self.scan(mapper, 10.1)
        self.assertEqual(mapper.grid(10.1)[10, 8], 100)
        self.scan(mapper, 10.31)
        self.assertEqual(mapper.grid(10.31)[10, 8], 0)
        self.assertEqual(mapper.observed[10, 8], 10.31)

    def test_each_stream_rejects_its_own_regression(self):
        mapper = self.mapper()
        self.scan(mapper, 10.1)
        self.cloud(mapper, 10.0)
        self.assertFalse(self.scan(mapper, 10.05))
        self.assertFalse(self.cloud(mapper, 9.99))
        self.assertEqual(mapper.stream_stamps, {'scan': 10.1, 'cloud': 10.0})

    def test_duplicate_cloud_does_not_extend_occupied_cell_lifetime(self):
        mapper = self.mapper()
        self.cloud(mapper, 10.0)
        self.scan(mapper, 10.2)
        self.cloud(mapper, 10.0)
        self.assertEqual(mapper.observed[10, 8], 10.0)
        self.assertEqual(mapper.grid(13.01)[10, 8], -1)


class MappingAdapterTimestampTests(unittest.TestCase):
    def adapter(self):
        try:
            from njord_sim.mapper_node import Mapper
        except ImportError:
            self.skipTest('ROS imports unavailable; no DDS context is needed for these callback checks')
        node = Mapper.__new__(Mapper)
        node.config = dict(resolution=1, size_m=20, origin=(0, 0), inflation_m=0)
        node.mapper = OccupancyMapper(**node.config)
        node.last_stamp, node.last_clock = None, None
        return node

    def test_delayed_cloud_does_not_regress_published_acquisition_stamp(self):
        node = self.adapter()
        scan_stamp = NS(sec=10, nanosec=100000000)
        node.record_stamp(scan_stamp)
        node.record_stamp(NS(sec=10, nanosec=0))
        self.assertIs(node.last_stamp, scan_stamp)

    def test_clock_reset_clears_all_streams_without_waiting_for_cloud(self):
        node = self.adapter()
        node.mapper.update((1.5, 10.5), [(8.5, 10.5)], [True], 10., stream='cloud')
        node.mapper.update((1.5, 10.5), [(18.5, 10.5)], [False], 10.1, stream='scan')
        node.last_clock, node.last_stamp = 10.1, NS(sec=10, nanosec=100000000)
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100000000))
        self.assertEqual(node.observe_clock(), 0.1)
        self.assertIsNone(node.last_stamp)
        self.assertEqual(node.mapper.stream_stamps, {})
        self.assertTrue(np.all(node.mapper.grid(0.1) == -1))


if __name__ == '__main__':
    unittest.main()
