"""Command guard decisions on simulation and steady time, without ROS."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'njord_sim'))
from njord_sim.constants import COMMAND_TIMEOUT_S, PROCESS_LIVENESS_S
from njord_sim.guard_core import GuardCore

STATUSES = ('navigation', 'planner', 'mission')


def core(thrusters=2, required=STATUSES, run_active=True):
    return GuardCore(thrusters, required, run_active, COMMAND_TIMEOUT_S, PROCESS_LIVENESS_S,
                     [500.0]*thrusters, [400.0]*thrusters, 450.0)


class RaceAtRealTimeFactor:
    """A controller at 20 Hz and heartbeats at 10 Hz of simulation time, the
    evaluator's run-active signal at 10 Hz of steady time; ``rtf`` simulation
    seconds pass per steady second."""

    def __init__(self, guard, rtf, command=(100.0, -50.0)):
        self.guard, self.rtf, self.command = guard, rtf, command
        self.wall, self.controller = 0.0, True
        self.next = {'command': 0.0, 'status': 0.0, 'run': 0.0}

    @property
    def sim(self):
        return 1000.0 + self.rtf*self.wall

    def advance(self, wall_s, step=0.002):
        """Deliver every due input; return the decisions seen along the way."""
        decisions = []
        end = self.wall + wall_s
        while self.wall < end:
            self.wall += step
            if self.controller and self.sim >= self.next['command']:
                for index, value in enumerate(self.command):
                    self.guard.command(index, value, self.sim, self.wall)
                self.next['command'] = self.sim + 0.05
            if self.sim >= self.next['status']:
                for key in STATUSES:
                    self.guard.status(key, True, self.sim, self.sim, self.wall)
                self.next['status'] = self.sim + 0.1
            if self.wall >= self.next['run']:
                self.guard.set_run_active(True, self.wall)
                self.next['run'] = self.wall + 0.1
            decisions.append((self.sim, self.guard.decide(self.sim, self.wall)))
        return decisions


class GuardCoreTests(unittest.TestCase):
    def test_valid_inputs_pass_at_slow_and_fast_real_time_factors(self):
        for rtf in (0.1, 0.3, 1.0, 3.0, 10.0):
            with self.subTest(rtf=rtf):
                guard = core()
                race = RaceAtRealTimeFactor(guard, rtf)
                race.advance(0.5)  # every input received at least once
                failures = [reason for _, (valid, reason) in race.advance(3.0) if not valid]
                self.assertEqual(failures, [])
                self.assertEqual(guard.forces(True), [100.0, -50.0])

    def test_stopped_controller_expires_after_the_timeout_in_simulation_time(self):
        for rtf in (0.3, 1.0, 3.0):
            with self.subTest(rtf=rtf):
                guard = core()
                race = RaceAtRealTimeFactor(guard, rtf)
                race.advance(1.0)
                last_command = guard.inputs[0][0]
                race.controller = False
                decisions = race.advance(1.0 / rtf + 0.2)
                # Valid strictly inside the timeout, invalid after it, at any RTF.
                self.assertTrue(all(valid for sim, (valid, _) in decisions
                                    if sim - last_command < COMMAND_TIMEOUT_S - 1e-9))
                expired = [(sim, reason) for sim, (valid, reason) in decisions if not valid]
                self.assertTrue(expired)
                self.assertAlmostEqual(expired[0][0] - last_command, COMMAND_TIMEOUT_S, delta=0.01 * rtf + 1e-6)
                self.assertIn('stale in simulation time', expired[0][1])

    def test_frozen_clock_expires_on_steady_liveness(self):
        guard = core()
        race = RaceAtRealTimeFactor(guard, 1.0)
        race.advance(1.0)
        sim = race.sim
        wall = race.wall
        # /clock stops: nothing advances in simulation time, and the frozen
        # simulator's publishers stop too.
        self.assertTrue(guard.decide(sim, wall + PROCESS_LIVENESS_S - 0.1)[0])
        valid, reason = guard.decide(sim, wall + PROCESS_LIVENESS_S + 0.1)
        self.assertFalse(valid)
        self.assertIn('not live', reason)
        # The same holds for data inputs when no run signal is required.
        free = core(run_active=False)
        free_race = RaceAtRealTimeFactor(free, 1.0)
        free_race.advance(1.0)
        valid, reason = free.decide(free_race.sim, free_race.wall + PROCESS_LIVENESS_S + 0.1)
        self.assertFalse(valid)
        self.assertIn('not live in steady time', reason)

    def test_run_active_missing_false_or_not_live(self):
        guard = core()
        race = RaceAtRealTimeFactor(guard, 1.0)
        race.advance(0.5)
        self.assertTrue(guard.decide(race.sim, race.wall)[0])
        guard.set_run_active(False, race.wall)
        self.assertEqual(guard.decide(race.sim, race.wall), (False, 'run not active'))
        guard.set_run_active(True, race.wall)
        self.assertFalse(guard.decide(race.sim, race.wall + PROCESS_LIVENESS_S + 0.1)[0])
        # Without the requirement, the run signal is irrelevant.
        free = core(run_active=False)
        free_race = RaceAtRealTimeFactor(free, 1.0)
        free_race.advance(0.5)
        free.set_run_active(False, free_race.wall)
        self.assertTrue(free.decide(free_race.sim, free_race.wall)[0])

    def test_only_required_heartbeats_are_checked(self):
        guard = core(required=('navigation',))
        guard.set_run_active(True, 0.0)
        for index in (0, 1):
            guard.command(index, 10.0, 5.0, 0.0)
        self.assertEqual(guard.decide(5.0, 0.0), (False, 'navigation status missing'))
        guard.status('navigation', True, 5.0, 5.0, 0.0)
        self.assertTrue(guard.decide(5.0, 0.0)[0])

    def test_invalid_and_old_inputs(self):
        guard = core(required=('navigation',), run_active=False)
        guard.status('navigation', True, 10.0, 10.0, 0.0)
        guard.command(0, float('nan'), 10.0, 0.0)
        guard.command(1, 10.0, 10.0, 0.0)
        self.assertEqual(guard.decide(10.0, 0.0), (False, 'thruster 1 command invalid'))
        guard.command(0, 10.0, 10.0, 0.0)
        self.assertTrue(guard.decide(10.0, 0.0)[0])
        # A heartbeat with old content, or an ERROR repeat of an old stamp, invalidates.
        guard.status('navigation', True, 8.0, 10.0, 0.0)
        self.assertEqual(guard.decide(10.0, 0.0), (False, 'navigation status invalid'))
        guard.status('navigation', True, 10.05, 10.05, 0.0)
        self.assertTrue(guard.decide(10.05, 0.0)[0])
        guard.status('navigation', False, 10.05, 10.05, 0.0)
        self.assertFalse(guard.decide(10.05, 0.0)[0])

    def test_clock_rollback_clears_every_input(self):
        guard = core(required=('navigation',), run_active=False)
        guard.status('navigation', True, 10.0, 10.0, 0.0)
        for index in (0, 1):
            guard.command(index, 10.0, 10.0, 0.0)
        self.assertTrue(guard.decide(10.0, 0.0)[0])
        self.assertEqual(guard.decide(2.0, 0.0), (False, 'thruster 1 command missing'))
        self.assertEqual(guard.inputs, {})

    def test_complete_set_requires_every_thruster(self):
        guard = core(thrusters=4, required=(), run_active=False)
        for index in (0, 1, 2):
            guard.command(index, 1.0, 1.0, 0.0)
        self.assertFalse(guard.take_complete_set())
        guard.command(1, 2.0, 1.01, 0.0)
        self.assertFalse(guard.take_complete_set())
        guard.command(3, 1.0, 1.02, 0.0)
        self.assertTrue(guard.take_complete_set())
        self.assertFalse(guard.take_complete_set())

    def test_forces_are_clamped_per_thruster_and_zero_when_invalid(self):
        guard = GuardCore(2, (), False, 0.5, 2.0, [300.0, 500.0], [100.0, 500.0], 450.0)
        guard.command(0, 1000.0, 1.0, 0.0)
        guard.command(1, -1000.0, 1.0, 0.0)
        self.assertEqual(guard.forces(True), [300.0, -450.0])
        guard.command(0, -1000.0, 1.0, 0.0)
        self.assertEqual(guard.forces(True), [-100.0, -450.0])
        self.assertEqual(guard.forces(False), [0.0, 0.0])

    def test_invalid_configuration(self):
        with self.assertRaises(ValueError):
            GuardCore(2, (), False, 0.5, 2.0, [1.0], [1.0, 1.0], 1.0)
        with self.assertRaises(ValueError):
            GuardCore(2, (), False, 0.0, 2.0, [1.0]*2, [1.0]*2, 1.0)


if __name__ == '__main__':
    unittest.main()
