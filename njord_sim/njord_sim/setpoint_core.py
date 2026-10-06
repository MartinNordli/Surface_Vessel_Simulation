"""Setpoint sequencing and scoring without ROS or Gazebo dependencies.

Pure-Python core of the evaluator for setpoint courses (scenario kind
``setpoints``, see docs/control-autonomy-setpoints.md); evaluator_node.py
wires it to ROS. Two modes share the scoring:

- ``SetpointCourse`` (a race, ``demo``/``gui``): issues the scenario's target
  poses in order and decides when each one is reached, timed out, or replaced
  after its ``advance_after_s``. The evaluator publishes each issued target.
- ``SetpointObserver`` (``lab``): targets arrive from outside (``njord goto``,
  RViz, a team's mission node); each new one replaces the active one.

Both are fed ground-truth poses stamped in simulation time; ground truth is
used for scoring only. A target is *inside* when the boat is within
``tolerance_m`` of it and, when the heading is scored, within
``heading_tolerance_deg`` of the commanded heading. It is *reached* once the
boat has stayed inside for ``hold_s``. Hold statistics cover that final
uninterrupted inside interval.

Conventions: world ENU, metres, yaw in radians counter-clockwise from east
(degrees in the metrics), times in seconds of simulation time.
"""

import math

from .control_core import wrap
from .scenario_core import obstacles, swept_clearance

# Outcomes of one target. 'superseded' only occurs in observer mode,
# 'advanced' only for targets with advance_after_s.
OUTCOMES = ('reached', 'timeout', 'advanced', 'superseded', 'unfinished')


def commanded_yaw(setpoint, previous_xy):
    """Heading (rad) to command for ``setpoint``: its own heading, or when free
    the bearing from ``previous_xy`` (where the boat was told to come from)."""
    if setpoint.get('heading_deg_enu') is not None:
        return wrap(math.radians(setpoint['heading_deg_enu']))
    dx = setpoint['position'][0] - previous_xy[0]
    dy = setpoint['position'][1] - previous_xy[1]
    return math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else 0.0


def _segment_distance(point, start, end):
    """Distance (m) from ``point`` to the segment ``start``-``end``."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    f = 0.0 if length2 == 0 else max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length2))
    return math.dist(point, (start[0] + f * dx, start[1] + f * dy))


class SetpointTrack:
    """Score one target pose from the moment it was issued until it ends.

    ``spec`` holds ``name``, ``position``, ``tolerance_m``,
    ``heading_tolerance_deg``, ``hold_s`` and optionally ``timeout_s`` and
    ``advance_after_s`` (None disables them). ``yaw`` is the commanded heading
    in rad; ``heading_scored`` False ignores it for acceptance.
    """

    def __init__(self, index, spec, issued_at, start_xy, yaw, heading_scored, run_start):
        self.index, self.spec = index, spec
        self.issued_at, self.start_xy, self.yaw = issued_at, tuple(start_xy), yaw
        self.heading_scored, self.run_start = heading_scored, run_start
        self.required = spec.get('advance_after_s') is None
        self.outcome = None
        self.ended_at = None
        self.distance = math.dist(start_xy, spec['position'])
        self.heading_error = 0.0
        self.first_inside = None
        self.inside_since = None
        self.time_inside = 0.0
        self.min_distance = self.distance
        self.overshoot = None
        self.path_length = 0.0
        self.max_cross_track = 0.0
        self.impulse = 0.0
        self.previous_xy = tuple(start_xy)
        self._reset_hold()

    def _reset_hold(self):
        self.hold = {'time': 0.0, 'sq_distance': 0.0, 'max_distance': 0.0,
                     'sq_heading': 0.0, 'max_heading': 0.0}

    def inside(self):
        """True if the latest pose is within the position (and heading) tolerance."""
        return (self.distance <= self.spec['tolerance_m']
                and (not self.heading_scored
                     or abs(math.degrees(self.heading_error)) <= self.spec['heading_tolerance_deg']))

    def update(self, t, x, y, yaw, dt, force_sum):
        """Score one pose at simulation time ``t``; ``dt`` (s) since the previous
        one and ``force_sum`` the summed magnitude of the commanded thrusts (N)
        that acted over it. Ends the track when its outcome is decided."""
        if self.outcome is not None:
            return
        self.distance = math.dist((x, y), self.spec['position'])
        self.heading_error = wrap(yaw - self.yaw)
        self.min_distance = min(self.min_distance, self.distance)
        self.path_length += math.dist(self.previous_xy, (x, y))
        self.previous_xy = (x, y)
        self.impulse += force_sum * dt
        if self.first_inside is None:
            self.max_cross_track = max(self.max_cross_track,
                                       _segment_distance((x, y), self.start_xy, self.spec['position']))
        else:
            self.overshoot = max(self.overshoot, self.distance)
        if self.inside():
            if self.first_inside is None:
                self.first_inside, self.overshoot = t, self.distance
            if self.inside_since is None:
                self.inside_since = t
                self._reset_hold()
            else:
                self.time_inside += dt
                error_deg = abs(math.degrees(self.heading_error))
                self.hold['time'] += dt
                self.hold['sq_distance'] += self.distance ** 2 * dt
                self.hold['sq_heading'] += error_deg ** 2 * dt
            self.hold['max_distance'] = max(self.hold['max_distance'], self.distance)
            self.hold['max_heading'] = max(self.hold['max_heading'], abs(math.degrees(self.heading_error)))
            if t - self.inside_since >= self.spec['hold_s']:
                return self.end(t, 'reached')
        else:
            self.inside_since = None
        elapsed = t - self.issued_at
        if self.spec.get('advance_after_s') is not None and elapsed >= self.spec['advance_after_s']:
            self.end(t, 'advanced')
        elif self.spec.get('timeout_s') is not None and elapsed >= self.spec['timeout_s']:
            self.end(t, 'timeout')

    def end(self, t, outcome):
        """End the track at time ``t`` with ``outcome`` (one of OUTCOMES)."""
        if self.outcome is None:
            self.outcome, self.ended_at = outcome, t
            if outcome != 'reached':
                # Hold statistics only describe a completed hold.
                self.inside_since = None if not self.inside() else self.inside_since

    def metrics(self):
        """JSON-serializable result of this target; None marks 'not applicable'."""
        def rel(t):
            return None if t is None else t - self.issued_at
        hold_time = self.hold['time'] if self.inside_since is not None else 0.0
        return {
            'index': self.index, 'name': self.spec['name'],
            'target': list(self.spec['position']),
            'commanded_heading_deg_enu': math.degrees(self.yaw), 'heading_scored': self.heading_scored,
            'tolerance_m': self.spec['tolerance_m'],
            'heading_tolerance_deg': self.spec['heading_tolerance_deg'],
            'hold_s': self.spec['hold_s'], 'timeout_s': self.spec.get('timeout_s'),
            'advance_after_s': self.spec.get('advance_after_s'), 'required': self.required,
            'issued_at_s': self.issued_at - self.run_start,
            'outcome': self.outcome or 'unfinished',
            'reached': self.outcome == 'reached',
            'duration_s': rel(self.ended_at),
            'time_to_reach_s': rel(self.first_inside),
            'time_to_settle_s': rel(self.inside_since) if self.outcome == 'reached' else None,
            'final_distance_m': self.distance,
            'final_heading_error_deg': math.degrees(self.heading_error),
            'min_distance_m': self.min_distance,
            'overshoot_m': self.overshoot,
            'time_inside_s': self.time_inside,
            'hold_duration_s': hold_time,
            'hold_rms_distance_m': math.sqrt(self.hold['sq_distance'] / hold_time) if hold_time > 0 else None,
            'hold_max_distance_m': self.hold['max_distance'] if self.inside_since is not None else None,
            'hold_rms_heading_error_deg': math.sqrt(self.hold['sq_heading'] / hold_time) if hold_time > 0 else None,
            'hold_max_heading_error_deg': self.hold['max_heading'] if self.inside_since is not None else None,
            'path_length_m': self.path_length,
            'straight_line_m': math.dist(self.start_xy, self.spec['position']),
            'path_efficiency': (math.dist(self.start_xy, self.spec['position']) / self.path_length
                                if self.path_length > 1e-6 else None),
            'max_cross_track_m': self.max_cross_track,
            # Integral of the summed |thrust| commands the guard passed: an
            # actuator effort measure in N*s, not electrical energy.
            'force_impulse_ns': self.impulse,
        }


class _SetpointScoring:
    """Shared pose validation, clearance, contacts and track bookkeeping."""

    def __init__(self, scenario):
        self.scenario = scenario
        self.obstacles = obstacles(scenario)
        self.tracks = []
        self.previous = None
        self.previous_time = None
        self.start_time = None
        self.elapsed = 0.0
        self.distance = 0.0
        self.min_clearance = math.inf
        self.contact_messages = 0
        self.contact_events = 0
        self.contact_times = []
        self.status = 'running'
        self.events = []

    @property
    def active(self):
        """The track still being scored, or None."""
        return self.tracks[-1] if self.tracks and self.tracks[-1].outcome is None else None

    def contact(self, vessel_contact, stamp=None):
        """Record one contact message; ``vessel_contact`` is True if it involved the vessel."""
        self.contact_messages += 1
        if vessel_contact:
            self.contact_events += 1
            self.contact_times.append(None if stamp is None or self.start_time is None else stamp - self.start_time)

    def _accept(self, stamp, x, y, yaw):
        """Validate a pose; returns dt (s) or None when it must be ignored."""
        if not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            self._fail('invalid_ground_truth')
            return None
        if self.previous_time is not None and stamp <= self.previous_time:
            if stamp < self.previous_time:
                self._fail('clock_reset')
            return None
        if self.start_time is None:
            self.start_time = stamp
        dt = 0.0 if self.previous_time is None else stamp - self.previous_time
        pose = (x, y, yaw)
        self.min_clearance = min(self.min_clearance, swept_clearance(
            self.previous, pose, self.obstacles, self.scenario['hull']))
        if self.previous is not None:
            self.distance += math.dist(self.previous[:2], pose[:2])
        self.previous, self.previous_time = pose, stamp
        self.elapsed = stamp - self.start_time
        return dt

    def _fail(self, status):
        raise NotImplementedError

    def _event(self, kind, track, t):
        event = {'type': kind, 'index': track.index, 'name': track.spec['name'],
                 'time_s': t - (self.start_time if self.start_time is not None else t)}
        if kind == 'ended':
            event.update(outcome=track.outcome, final_distance_m=track.distance,
                         final_heading_error_deg=math.degrees(track.heading_error))
        self.events.append(event)
        return event

    def _end_active(self, t, outcome):
        track = self.active
        if track is not None:
            track.end(t, outcome)
            return [self._event('ended', track, t)]
        return []

    def base_metrics(self):
        reached = sum(t.outcome == 'reached' for t in self.tracks)
        return {
            'course': 'setpoints',
            'time_s': self.elapsed,
            'distance_travelled_m': self.distance,
            'min_clearance_m': self.min_clearance if math.isfinite(self.min_clearance) else None,
            'clearance_model': 'horizontal_conservative_rectangle_swept_0.1m_2deg',
            'geometric_overlap': self.min_clearance <= 0,
            'collision': bool(self.contact_events) if self.contact_messages else None,
            'contact_status': 'observed' if self.contact_messages else 'unavailable',
            'contact_message_count': self.contact_messages,
            'contact_event_count': self.contact_events,
            'contact_times_s': self.contact_times,
            'setpoints_issued': len(self.tracks),
            'setpoints_reached': reached,
            'setpoints': [track.metrics() for track in self.tracks],
            'setpoint_events': self.events,
        }


class SetpointCourse(_SetpointScoring):
    """Issue a scenario's targets in order and score the run.

    ``status`` starts as ``running``; once it is anything else no later
    input changes it. Final statuses:

    - ``completed``            every required target was reached
    - ``setpoints_failed``     the sequence ended with a required target missed
    - ``collision``            a contact message involved the vessel
    - ``geometric_overlap``    the swept hull envelope touched an obstacle
    - ``simulation_timeout``   the scenario's ``timeout_s`` elapsed
    - ``invalid_ground_truth`` a non-finite pose or stamp was received
    - ``clock_reset``          simulation time went backwards

    ``update`` returns the events of that sample: ``issued`` (publish this
    target now) and ``ended`` dictionaries, in order.
    """

    def __init__(self, scenario):
        super().__init__(scenario)
        if scenario.get('kind') != 'setpoints':
            raise ValueError('SetpointCourse needs a setpoint scenario')
        self.specs = scenario['setpoints']

    def _fail(self, status):
        if self.status == 'running':
            self.status = status

    def contact(self, vessel_contact, stamp=None):
        super().contact(vessel_contact, stamp)
        if vessel_contact and self.status == 'running':
            self.status = 'collision'

    def _issue(self, index, t, start_xy):
        spec = self.specs[index]
        track = SetpointTrack(index, spec, t, start_xy, commanded_yaw(spec, start_xy),
                              spec.get('heading_deg_enu') is not None, self.start_time)
        self.tracks.append(track)
        return self._event('issued', track, t)

    def remaining(self):
        """Specs of the active target and every later one (the lookahead sequence)."""
        if self.status != 'running' or not self.tracks:
            return []
        return self.specs[self.tracks[-1].index:]

    def target(self, index):
        """(x, y, yaw) of an issued target, as published on the setpoint topic."""
        track = self.tracks[index]
        return track.spec['position'][0], track.spec['position'][1], track.yaw

    def update(self, stamp, x, y, yaw, force_sum=0.0):
        """Advance the course with a ground-truth pose; returns this sample's events."""
        if self.status != 'running':
            return []
        first = self.start_time is None
        dt = self._accept(stamp, x, y, yaw)
        if dt is None:
            return []
        events = [self._issue(0, stamp, (x, y))] if first else []
        track = self.active
        track.update(stamp, x, y, yaw, dt, force_sum)
        if track.outcome is not None:
            events.append(self._event('ended', track, stamp))
            if track.index + 1 < len(self.specs):
                events.append(self._issue(track.index + 1, stamp, (x, y)))
            else:
                self.status = ('completed' if all(t.outcome == 'reached' for t in self.tracks if t.required)
                               else 'setpoints_failed')
        if self.min_clearance <= 0:
            self.status = 'geometric_overlap'
        elif self.status == 'running' and self.elapsed >= self.scenario['timeout_s']:
            self.status = 'simulation_timeout'
        if self.status != 'running':
            events += self._end_active(stamp, 'unfinished')
        return events

    def stop(self, status, stamp=None):
        """End the run from outside (watchdogs, interruption) with ``status``."""
        self._fail(status)
        self._end_active(stamp if stamp is not None else (self.previous_time or 0.0), 'unfinished')

    def metrics(self):
        """JSON-serializable result of the whole course."""
        result = self.base_metrics()
        result.update(status=self.status, reached_goal=self.status == 'completed',
                      setpoints_total=len(self.specs),
                      setpoints_required=sum(spec.get('advance_after_s') is None for spec in self.specs))
        return result


class SetpointObserver(_SetpointScoring):
    """Score targets issued from outside during a free (lab) run.

    ``acceptance`` gives ``tolerance_m``, ``heading_tolerance_deg`` and
    ``hold_s`` for every observed target. Each ``issue`` replaces the active
    target ('superseded'). A reached target stays reached; later drift is
    scored only if a new target is issued. Contacts are counted but do not
    end the observation. ``status`` stays ``observing`` until ``stop``.
    """

    def __init__(self, scenario, acceptance):
        super().__init__(scenario)
        self.acceptance = dict(acceptance)
        self.status = 'observing'

    def _fail(self, status):
        self.status = status

    def issue(self, stamp, x, y, yaw, name=None):
        """Start scoring a new target (x, y in m, yaw in rad) issued at ``stamp``.

        Returns the events: the end of the previous target, if any, and the
        new one. Before the first pose the start position is unknown, so the
        target is held until one arrives.
        """
        if self.status != 'observing' or not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            return []
        events = self._end_active(stamp, 'superseded')
        start = self.previous[:2] if self.previous is not None else (x, y)
        if self.start_time is None:
            self.start_time = stamp
        spec = {'name': name or f'setpoint_{len(self.tracks) + 1}', 'position': [x, y],
                'heading_deg_enu': math.degrees(yaw), 'advance_after_s': None, 'timeout_s': None,
                **self.acceptance}
        track = SetpointTrack(len(self.tracks), spec, stamp, start, wrap(yaw), True, self.start_time)
        self.tracks.append(track)
        events.append(self._event('issued', track, stamp))
        return events

    def update(self, stamp, x, y, yaw, force_sum=0.0):
        """Score a ground-truth pose; returns this sample's events."""
        if self.status != 'observing':
            return []
        dt = self._accept(stamp, x, y, yaw)
        if dt is None:
            return []
        track = self.active
        if track is None:
            return []
        track.update(stamp, x, y, yaw, dt, force_sum)
        return [self._event('ended', track, stamp)] if track.outcome is not None else []

    def stop(self, status='stopped', stamp=None):
        """End the observation; an unfinished target stays 'unfinished'."""
        if self.status == 'observing':
            self.status = status
        self._end_active(stamp if stamp is not None else (self.previous_time or 0.0), 'unfinished')

    def metrics(self):
        result = self.base_metrics()
        result.update(status=self.status, reached_goal=None, acceptance=self.acceptance)
        return result
