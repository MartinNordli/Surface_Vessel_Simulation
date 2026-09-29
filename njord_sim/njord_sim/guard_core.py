"""Decision logic of the command guard, free of ROS (see command_guard_node.py).

The guard forwards per-thruster forces only while every required input is
fresh and valid. Each input is recorded with two receipt times:

* simulation time (/clock, s): a thrust command or health heartbeat is stale
  once it is older than ``timeout_s`` of simulation time. That bounds how
  long it can act on the boat, independent of the real-time factor.
* steady wall time (s): ``liveness_s`` only detects a stopped /clock or a
  dead publisher. While simulation time stands still the boat does not move,
  so this limit can be generous without letting a stale command act longer.

The run-active signal from the evaluator is a state, not data: it is
published on the evaluator's steady-time timer, so only its liveness is
checked. Invalid inputs are stored as ``None`` and make the decision invalid.
"""
import math

STATUS_MAX_AGE_S = 1.0     # oldest accepted heartbeat content, simulation s
STATUS_MAX_FUTURE_S = 0.1  # tolerated clock skew into the future, simulation s


class GuardCore:
    """Latest inputs of the guard and the decision whether thrust may pass.

    Args:
        thrusters: number of thruster commands, in vessel-file order.
        required_status: heartbeat keys that must be OK, e.g. 'navigation'.
        require_run_active: whether the evaluator's run-active signal is needed.
        timeout_s: data freshness limit in simulation seconds.
        liveness_s: process liveness limit in steady seconds.
        forward_limits, reverse_limits: per-thruster force magnitudes [N].
        max_thrust: operating force limit for every thruster [N].
    """

    def __init__(self, thrusters, required_status, require_run_active, timeout_s, liveness_s,
                 forward_limits, reverse_limits, max_thrust):
        if thrusters < 1 or len(forward_limits) != thrusters or len(reverse_limits) != thrusters:
            raise ValueError('one forward and one reverse limit per thruster is required')
        if not (timeout_s > 0 and liveness_s > 0):
            raise ValueError('timeout_s and liveness_s must be positive')
        self.thrusters = thrusters
        self.required = [*range(thrusters), *required_status]
        self.require_run_active = require_run_active
        self.timeout_s, self.liveness_s = timeout_s, liveness_s
        self.forward, self.reverse, self.max_thrust = list(forward_limits), list(reverse_limits), max_thrust
        self.reset()

    def reset(self):
        """Forget every input, e.g. after simulation time moved backwards."""
        # key -> (simulation receipt s, steady receipt s, value or None if invalid)
        self.inputs = {}
        self.status_stamps = {}
        self.run_active = (False, -math.inf)  # (value, steady receipt s)
        self.updated = set()  # thrusters commanded since the last complete set
        self.last_sim = None

    def clock(self, sim_now):
        """Record the current simulation time; clear all inputs if it went backwards."""
        if self.last_sim is not None and sim_now < self.last_sim:
            self.reset()
        self.last_sim = sim_now

    def command(self, index, value, sim_now, wall_now):
        """Record a thrust command in N for thruster ``index``; non-finite is invalid."""
        self.clock(sim_now)
        self.inputs[index] = (sim_now, wall_now, value if math.isfinite(value) else None)
        self.updated.add(index)

    def status(self, key, ok, stamp, sim_now, wall_now):
        """Record a heartbeat: ``ok`` if its level is OK, ``stamp`` its header time (s).

        The content must be at most STATUS_MAX_AGE_S old. A stamp that is not
        newer than the last accepted one cannot freshen the input, but an
        invalid repeat still invalidates it.
        """
        self.clock(sim_now)
        valid = ok and -STATUS_MAX_FUTURE_S <= sim_now - stamp <= STATUS_MAX_AGE_S
        if stamp <= self.status_stamps.get(key, -math.inf):
            if not valid:
                self.inputs[key] = (sim_now, wall_now, None)
            return
        self.status_stamps[key] = stamp
        self.inputs[key] = (sim_now, wall_now, True if valid else None)

    def set_run_active(self, value, wall_now):
        """Record the evaluator's run-active signal."""
        self.run_active = (bool(value), wall_now)

    def decide(self, sim_now, wall_now):
        """Return (valid, reason): whether thrust may pass now, and why not."""
        self.clock(sim_now)
        if self.require_run_active:
            active, received = self.run_active
            if wall_now - received > self.liveness_s:
                return False, 'run-active signal missing or not live'
            if not active:
                return False, 'run not active'
        for key in self.required:
            name = f'thruster {key + 1} command' if isinstance(key, int) else f'{key} status'
            if key not in self.inputs:
                return False, f'{name} missing'
            sim_received, wall_received, value = self.inputs[key]
            if value is None:
                return False, f'{name} invalid'
            if sim_now - sim_received > self.timeout_s:
                return False, f'{name} stale in simulation time'
            if wall_now - wall_received > self.liveness_s:
                return False, f'{name} not live in steady time'
        return True, 'ok'

    def take_complete_set(self):
        """True once every thruster was commanded since the previous complete set."""
        if len(self.updated) < self.thrusters:
            return False
        self.updated.clear()
        return True

    def forces(self, valid):
        """Forces in N: commands clamped to the limits if ``valid``, otherwise zeros."""
        if not valid:
            return [0.0] * self.thrusters
        # Clamp each thruster to [-min(max_thrust, reverse), min(max_thrust, forward)].
        return [max(-min(self.max_thrust, self.reverse[i]), min(self.max_thrust, self.forward[i], self.inputs[i][2]))
                for i in range(self.thrusters)]
