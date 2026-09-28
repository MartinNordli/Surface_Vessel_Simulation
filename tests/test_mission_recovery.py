"""Reference mission search and retry, checked with the scorer's own crossing rule."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from njord_sim.mission_core import ERROR, OK, SEARCH_BEARINGS_DEG, WARN, GateMission
from njord_sim.scenario_core import gate_crossing
from test_mission_heading import SETTINGS

# One gate at x = 20 m with its opening facing +x: red (port) at y = +7,
# green (starboard) at y = -7, as the scorer and the mission both see it.
GATE = {'red': (20., 7.), 'green': (20., -7.), 'radius_m': 0.5}
DETECTIONS = [('r', 'red', GATE['red']), ('g', 'green', GATE['green'])]


class Run:
    """Drive a GateMission along given positions at 10 Hz, as the node would."""

    def __init__(self, settings=SETTINGS, gates=1):
        self.mission = GateMission(gates, 0., settings)
        self.now = 10.
        self.decisions, self.path = [], []

    def at(self, x, y, yaw=0., detections=DETECTIONS):
        self.now += 0.1
        self.mission.odometry((x, y), yaw, self.now)
        self.mission.camera(self.now, self.now, detections)
        self.path.append((x, y))
        decision = self.mission.step(self.now)
        self.decisions.append(decision)
        return decision


def crossings(points, gate=GATE):
    """Scorer gate crossings (direction) along a polyline of (x, y) points."""
    found = []
    for a, b in zip(points, points[1:]):
        crossing = gate_crossing(a, b, gate)
        if crossing:
            found.append(crossing[0])
    return found


class SearchTests(unittest.TestCase):
    def test_waits_then_searches_forward_bearings_then_gives_up(self):
        run = Run()
        # No gate in view: an error (zero thrust) until search_after_s.
        decision = run.at(0., 0., detections=[])
        self.assertEqual((decision.level, decision.message), (ERROR, 'no unambiguous observed gate'))
        start = run.now
        while decision.message != 'searching' and run.now < start + 10:
            decision = run.at(0., 0., detections=[])
        self.assertEqual((decision.level, decision.message), (OK, 'searching'))
        # Searching begins search_after_s after the gate was lost, within one step.
        self.assertAlmostEqual(run.now - start, SETTINGS['search_after_s'], delta=0.1 + 1e-9)
        self.assertEqual(run.mission.phase, 'search')
        self.assertAlmostEqual(decision.goal[0], SETTINGS['search_radius_m'])
        self.assertAlmostEqual(decision.goal[1], 0.)
        # Unreached goals advance after search_goal_s, through every bearing.
        goals = [decision.goal[:2]]
        searching_since = run.now
        while run.now < searching_since + SETTINGS['search_timeout_s'] - 1:
            decision = run.at(0., 0., detections=[])
            if decision.level == OK and decision.goal[:2] != goals[-1]:
                goals.append(decision.goal[:2])
        bearings = [round(math.degrees(math.atan2(y, x))) for x, y in goals]
        self.assertEqual(bearings[:len(SEARCH_BEARINGS_DEG)], [round(b) for b in SEARCH_BEARINGS_DEG])
        # Every search goal lies ahead along the course: no backward crossing.
        self.assertTrue(all(x > 0 for x, _ in goals))
        while run.now < searching_since + SETTINGS['search_timeout_s'] + 0.5:
            decision = run.at(0., 0., detections=[])
        self.assertEqual((decision.level, decision.message), (ERROR, 'no gate found while searching'))

    def test_reached_goal_advances_and_a_seen_gate_ends_the_search(self):
        run = Run()
        while run.mission.phase != 'search':
            decision = run.at(0., 0., detections=[])
        first = decision.goal
        decision = run.at(first[0], first[1], detections=[])  # arrived: next bearing
        self.assertNotEqual(decision.goal[:2], first[:2])
        decision = run.at(first[0], first[1])
        self.assertEqual((decision.level, decision.message), (OK, 'valid'))
        self.assertEqual(run.mission.phase, 'approach')
        self.assertEqual(decision.goal[:2], (15., 0.))  # approach point before the gate

    def test_next_gate_is_selected_in_the_same_step_as_the_crossing(self):
        second = {'red': (40., 7.), 'green': (40., -7.)}
        detections = DETECTIONS + [('r2', 'red', second['red']), ('g2', 'green', second['green'])]
        run = Run(gates=2)
        for x in np.arange(10., 26.01, 0.5):
            decision = run.at(float(x), 0., detections=detections)
        self.assertEqual(len(run.mission.passed), 1)
        self.assertEqual((decision.level, decision.message), (OK, 'valid'))
        self.assertEqual(decision.goal[:2], (35., 0.))
        self.assertNotIn(WARN, [d.level for d in run.decisions])


class RetryTests(unittest.TestCase):
    def miss(self, run, y=-10.):
        """Pass the gate outside the green buoy (starboard) until the mission reacts."""
        for x in np.arange(10., 29.01, 0.5):
            decision = run.at(float(x), y)
        return decision

    def test_missed_gate_is_retried_around_the_outside_and_then_passed(self):
        run = Run()
        decision = self.miss(run)
        self.assertEqual((decision.level, decision.message), (OK, 'retrying gate'))
        retry = decision.goal[:2]
        # Beside the passed (green) buoy, retry_clearance_m outside it, on the gate line.
        self.assertAlmostEqual(retry[0], 20.)
        self.assertAlmostEqual(retry[1], -7. - SETTINGS['retry_clearance_m'])
        # Straight legs: to the retry point, on to the approach point, and
        # through the gate. The only crossing the scorer sees is forward.
        route = [run.path[-1], retry, (15., 0.), (20., 0.), (26., 0.)]
        self.assertEqual(crossings(route), [1])
        # Drive that route: the mission returns to approach and passes the gate.
        decision = run.at(*retry)
        self.assertEqual(run.mission.phase, 'approach')
        self.assertEqual(decision.goal[:2], (15., 0.))
        for x in np.arange(15., 26.01, 0.5):
            decision = run.at(float(x), 0.)
        self.assertEqual(len(run.mission.passed), 1)
        self.assertEqual(crossings(run.path), [1])

    def test_retry_side_follows_the_buoy_that_was_passed(self):
        run = Run()
        decision = self.miss(run, y=10.)  # outside the red (port) buoy
        self.assertAlmostEqual(decision.goal[1], 7. + SETTINGS['retry_clearance_m'])

    def test_retries_are_bounded_and_zero_retries_keep_the_old_error(self):
        run = Run()
        self.miss(run)
        run.at(20., -7. - SETTINGS['retry_clearance_m'])  # retry point reached
        for x in np.arange(15., 29.01, 0.5):  # misses again
            decision = run.at(float(x), -10.)
        self.assertEqual((decision.level, decision.message), (ERROR, 'gate crossing missed'))
        no_retry = Run(dict(SETTINGS, max_gate_retries=0))
        decision = self.miss(no_retry)
        self.assertEqual((decision.level, decision.message), (ERROR, 'gate crossing missed'))

    def test_estimate_jump_past_the_gate_is_not_a_miss_to_retry(self):
        # An initializing estimator can jump far away and back (seen live:
        # about 1e6 m). That is no observed crossing: the old non-latching
        # error, then normal tracking once the position is plausible again.
        run = Run()
        decision = run.at(5., 0.)
        self.assertEqual((decision.level, run.mission.phase), (OK, 'approach'))
        decision = run.at(87170., 1028456.)
        self.assertEqual((decision.level, decision.message), (ERROR, 'gate crossing missed'))
        self.assertEqual(run.mission.retries, 0)
        gate = run.mission.gate
        decision = run.at(5.2, 0.)
        # Tracking resumes on the same gate (as before search and retry existed,
        # the jump may already have advanced approach to cross).
        self.assertEqual((decision.level, decision.message), (OK, 'valid'))
        self.assertIs(run.mission.gate, gate)
        # No crossing is inferred from a jump across the line either.
        run.at(40., 0.)
        run.at(5.4, 0.)
        self.assertFalse(run.mission.crossed)
        self.assertEqual(run.mission.retries, 0)

    def test_clock_rollback_resets_search_and_retry_state(self):
        run = Run()
        self.miss(run)
        self.assertEqual(run.mission.phase, 'retry')
        run.mission.step(1.)
        self.assertEqual(run.mission.phase, 'seek')
        self.assertIsNone(run.mission.gate)
        self.assertEqual((run.mission.retries, run.mission.search), (0, None))


if __name__ == '__main__':
    unittest.main()
