"""Vectorized ray tracing gives exactly the cells of the reference Bresenham loop."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from njord_sim.mapping_core import OccupancyMapper, grid_line, trace_rays


def reference(start, ends, hits, size):
    """The previous per-cell loop over grid_line, as (free, hit) sets of (column, row)."""
    free, hit = set(), set()
    for end, is_hit in {(tuple(end), bool(h)) for end, h in zip(ends, hits)}:
        for cell in grid_line(start, end):
            if not (0 <= cell[0] < size and 0 <= cell[1] < size):
                break
            (hit if cell == end and is_hit else free).add(cell)
    return free-hit, hit


def as_cells(linear, size):
    return {(int(c) % size, int(c) // size) for c in linear}


class RayTraceTests(unittest.TestCase):
    def test_random_rays_match_the_reference_exactly(self):
        rng = np.random.default_rng(7)
        size = 60
        for trial in range(200):
            start = tuple(int(v) for v in rng.integers(-5, size+5, 2))
            ends = rng.integers(-40, size+40, (int(rng.integers(0, 40)), 2))
            hits = rng.random(len(ends)) < 0.5
            free, hit = trace_rays(start, ends, hits, size)
            expected_free, expected_hit = reference(start, [tuple(e) for e in ends], hits, size)
            with self.subTest(trial=trial):
                self.assertEqual(as_cells(free, size), expected_free)
                self.assertEqual(as_cells(hit, size), expected_hit)

    def test_degenerate_and_axis_aligned_rays(self):
        size = 10
        cases = [((3, 3), [(3, 3)], [True]), ((3, 3), [(3, 3)], [False]), ((0, 0), [(9, 0), (0, 9), (9, 9)], [True, False, True]),
                 ((5, 5), [(5, 5), (5, 20)], [False, True]), ((2, 7), [(-3, 7), (2, -4)], [True, True])]
        for start, ends, hits in cases:
            free, hit = trace_rays(start, ends, hits, size)
            with self.subTest(start=start, ends=ends):
                self.assertEqual((as_cells(free, size), as_cells(hit, size)), reference(start, ends, hits, size))
        free, hit = trace_rays((1, 1), np.empty((0, 2)), [], size)
        self.assertEqual((len(free), len(hit)), (0, 0))

    def test_mapper_grid_is_unchanged_for_a_full_scan(self):
        rng = np.random.default_rng(3)
        mapper = OccupancyMapper(resolution=0.5, size_m=60., origin=(-30., -30.), inflation_m=0.)
        angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
        ranges = np.where(rng.random(720) < 0.2, rng.uniform(3., 25., 720), 40.)
        ends = np.column_stack((2+ranges*np.cos(angles), 1+ranges*np.sin(angles)))
        hits = ranges < 40.
        self.assertTrue(mapper.update((2., 1.), ends, hits, 10.))
        start = mapper.cell((2., 1.))
        cells = [tuple(e) for e in np.floor((ends-mapper.origin)/mapper.resolution).astype(int)]
        expected_free, expected_hit = reference(start, cells, hits, mapper.size)
        occupied = {(c, r) for r, c in zip(*np.nonzero(mapper.values == 100))}
        observed_free = {(c, r) for r, c in zip(*np.nonzero(mapper.values == 0))}
        self.assertEqual(occupied, expected_hit)
        self.assertEqual(observed_free, expected_free)
        self.assertTrue(mapper.update((2., 1.), [], [], 10.1))  # no rays is still accepted


if __name__ == '__main__':
    unittest.main()
