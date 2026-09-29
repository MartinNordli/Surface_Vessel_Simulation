#!/usr/bin/env python3
"""Write njord_gz_plugins/test/load_vectors.csv from physics_core (run by hand).

The CSV holds wind-coefficient, wind-load and thruster-wrench cases with the
values physics_core computes. test/loads_test.cc checks the C++ header
njord/Loads.hh against it, and tests/test_njord_physics.py checks that
physics_core still reproduces it, so the two implementations cannot drift
apart unnoticed. Tests never rewrite the file; rerun this script only after an
intended change to the load models, and review the diff.

Columns: kind, then kind-specific inputs and expected outputs:
  table    angle_deg, cx, cy, cn          one row of the wind table
  coeff    angle_deg | cx, cy, cn
  wind     air_x, air_y, area_x, area_y, length | X, Y, N
  thrust   force_n, px, py, pz, ax, ay, az, cx, cy, cz | fx, fy, fz, tx, ty, tz
"""
from pathlib import Path
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'njord_sim'))
from njord_sim.physics_core import thruster_wrench, wind_coefficients, wind_load

# An irregular, asymmetric table exercises wrap-around and uneven spacing.
TABLE = [dict(angle_deg=10., cx=0.9, cy=0.1, cn=0.05), dict(angle_deg=100., cx=0.1, cy=1.1, cn=-0.1),
         dict(angle_deg=190., cx=-0.8, cy=-0.2, cn=0.02), dict(angle_deg=300., cx=0.3, cy=-0.9, cn=0.12)]


def rows():
    yield from (['table', r['angle_deg'], r['cx'], r['cy'], r['cn']] for r in TABLE)
    for angle in (-725., -30., 0., 5., 10., 55., 100., 190., 245., 299.9, 300., 330., 359.99, 360., 725.):
        yield ['coeff', angle, *wind_coefficients(angle, TABLE)]
    for air_x, air_y in ((5., 0.), (0., 5.), (-3., 4.), (2.5, -7.), (-0.4, -0.3), (0., 0.), (12., 1.)):
        for area_x, area_y, length in ((0.9, 1.8, 3.0), (2.0, 5.0, 6.0)):
            yield ['wind', air_x, air_y, area_x, area_y, length,
                   *wind_load((air_x, air_y), area_x, area_y, length, TABLE)]
    yaw = math.radians(30.)
    for force, position, axis, com in (
            (100., (-1.2, 0.6, -0.1), (1., 0., 0.), (0., 0., 0.)),
            (-250., (-1.2, -0.6, -0.1), (1., 0., 0.), (0.1, 0.2, 0.05)),
            (80., (0.8, 0.5, -0.2), (math.cos(yaw), math.sin(yaw), 0.), (0.1, 0., 0.)),
            (-40., (0.0, -0.7, 0.0), (0., 1., 0.), (-0.3, 0.1, 0.2))):
        f, t = thruster_wrench(force, position, axis, com)
        yield ['thrust', force, *position, *axis, *com, *f, *t]


def main():
    output = ROOT / 'njord_gz_plugins/test/load_vectors.csv'
    output.write_text(''.join(','.join(row[:1] + [repr(float(v)) for v in row[1:]]) + '\n' for row in rows()))
    print(f'Wrote {output}')


if __name__ == '__main__':
    main()
