#!/usr/bin/env python3
"""Median time of one OccupancyMapper.update for a 720-ray lidar scan.

Uses the default algorithms.yaml grid (0.5 m cells, 160 m) and rays of up to
80 m, with 20 % of them hits, like the shipped lidar settings. Host-only and
repeatable (fixed seed); it measures the Python mapping core, not a running
simulator. Usage: python3 validation/mapper_timing.py [--repeats N]
"""
import argparse
from pathlib import Path
import statistics
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from njord_sim.mapping_core import OccupancyMapper


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats', type=int, default=20)
    args = parser.parse_args()
    rng = np.random.default_rng(1)
    mapper = OccupancyMapper(resolution=0.5, size_m=160., origin=(-40., -40.), inflation_m=4.)
    angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
    times = []
    for k in range(args.repeats):
        ranges = np.where(rng.random(720) < 0.2, rng.uniform(3., 60., 720), 80.)
        ends = np.column_stack((10+ranges*np.cos(angles), ranges*np.sin(angles)))
        started = time.perf_counter()
        mapper.update((10., 0.), ends, ranges < 80., 1.0+0.1*k)
        times.append(time.perf_counter()-started)
    print(f'720-ray scan update: median {1e3*statistics.median(times):.1f} ms, '
          f'max {1e3*max(times):.1f} ms over {args.repeats} scans')


if __name__ == '__main__':
    main()
