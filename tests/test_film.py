"""Chase-camera geometry and GIF frame selection, without Gazebo or Pillow."""
import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


filmer = load('njord_filmer_script', 'scripts/run_filmer.py')
gif = load('njord_film_gif_script', 'scripts/make_film_gif.py')


def rotate(q, v):
    """Rotate vector ``v`` by unit quaternion ``q`` = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    # v' = v + 2w (u x v) + 2 u x (u x v), with u = (x, y, z)
    cx, cy, cz = y*vz - z*vy, z*vx - x*vz, x*vy - y*vx
    return (vx + 2*(w*cx + y*cz - z*cy), vy + 2*(w*cy + z*cx - x*cz), vz + 2*(w*cz + x*cy - y*cx))


class FilmerTests(unittest.TestCase):
    def test_look_at_points_the_camera_axis_at_the_target_without_roll(self):
        eye, target = (-15.0, -12.0, 9.0), (10.0, 3.0, 0.0)
        q = filmer.look_at(eye, target)
        self.assertAlmostEqual(sum(c*c for c in q), 1.0)
        direction = [t - e for t, e in zip(target, eye)]
        norm = math.sqrt(sum(d*d for d in direction))
        for got, want in zip(rotate(q, (1.0, 0.0, 0.0)), direction):
            self.assertAlmostEqual(got, want / norm)
        self.assertAlmostEqual(rotate(q, (0.0, 1.0, 0.0))[2], 0.0)  # level horizon

    def test_follow_filter_moves_towards_the_vessel_in_simulation_time(self):
        follow = filmer.Follow(1.0)
        self.assertEqual(follow.update((0.0, 0.0), 10.0), (0.0, 0.0))
        self.assertEqual(follow.update((10.0, 0.0), 10.0), (0.0, 0.0))  # same stamp: no change
        x, _ = follow.update((10.0, 0.0), 11.0)
        self.assertAlmostEqual(x, 10.0 * (1.0 - math.exp(-1.0)))


class FrameSelectionTests(unittest.TestCase):
    def records(self, start_moving_s, end_s, rate_hz=10):
        return [{'file': f'frame_{i:06d}.png', 'sim_time_s': i / rate_hz,
                 'vessel_xy': [max(0.0, i / rate_hz - start_moving_s), 0.0]}
                for i in range(int(end_s * rate_hz) + 1)]

    def test_skips_the_wait_and_samples_at_the_requested_speed(self):
        chosen = gif.select_frames(self.records(20.0, 80.0), speed=8.0, fps=16.0, lead_s=1.0)
        times = [r['sim_time_s'] for r in chosen]
        self.assertAlmostEqual(times[0], 19.6)  # 1 s before the vessel is over 0.5 m away (t=20.6)
        self.assertTrue(all(abs(b - a - 0.5) < 1e-9 for a, b in zip(times, times[1:])))
        self.assertGreaterEqual(times[-1], 79.5)

    def test_frames_without_a_vessel_position_are_ignored(self):
        records = self.records(0.0, 5.0)
        records[0]['vessel_xy'] = None
        self.assertNotIn(records[0], gif.select_frames(records, speed=1.0, fps=10.0))


if __name__ == '__main__':
    unittest.main()
