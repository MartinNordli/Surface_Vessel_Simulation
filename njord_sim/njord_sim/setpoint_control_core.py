"""Reference controller for setpoint courses, without ROS dependencies.

Drives to one target pose (x, y, heading) at a time and holds it.
``setpoint_controller_node.py`` wires it to ``/njord/setpoint``, the
navigation odometry and one thrust topic per thruster. It is a simple,
transparent baseline that shows the simulator works end to end and gives
Control & Autonomy something to compare against; it does not avoid
obstacles and its gains are untuned starting values (algorithms.yaml
``setpoint_control``).

Two modes, with hysteresis so the boat does not chatter between them:

* transit (farther than ``approach_radius_m``, or than 1.5 times that after
  station keeping started): turn towards the target with PD control on the
  bearing error and command a surge speed that ``control_core.speed_limit``
  caps so the boat can stop at the target (assumed braking deceleration
  and reaction time) and slows down while the target is off the bow.
* station keeping (inside ``approach_radius_m``): PD control on the
  position error in the body frame and on the target heading. A fully
  actuated vessel (surge, sway and yaw independent, e.g. munin_v0) uses all
  three. A two-thruster vessel cannot push sideways: it keeps pointing at
  the target until it is within ``align_radius_m`` and only then turns to
  the target heading, so it can hold its position along the heading only.

The resulting body wrench (surge N, sway N, yaw N*m) is distributed over the
thrusters by ``control_core.allocate_thrusters``, each limited to the
smaller of its vessel limit and ``max_thrust``.

Conventions: map ENU, body x forward / y left, yaw counter-clockwise from
east in rad, SI units.
"""

import math

from .control_core import allocate_thrusters, allocation_matrix, independent_rows, speed_limit, wrap

TRANSIT, STATION_KEEPING = 'transit', 'station_keeping'
# Leave station keeping only beyond this multiple of approach_radius_m.
HYSTERESIS = 1.5
# speed_limit slows the boat near obstacles below 3 m clearance; setpoint
# control has no map, so it passes a clearance that disables that limit.
NO_OBSTACLE_LIMIT_M = 3.0


class SetpointController:
    """Turn a pose, body velocity and target into thruster forces (N).

    ``settings`` holds the algorithms.yaml ``setpoint_control`` values plus
    ``max_speed`` (m/s, the speed profile) and ``max_thrust`` (N). The
    thruster layout is flattened xyz ``positions`` (m, relative to the
    centre of mass) and unit ``axes``, with per-thruster forward and reverse
    limits in N.
    """

    def __init__(self, settings, positions, axes, forward_limits, reverse_limits):
        if min(settings['approach_radius_m'], settings['align_radius_m'], settings['max_speed'],
               settings['braking_deceleration_mps2'], settings['max_thrust']) <= 0:
            raise ValueError('setpoint controller radii, speed, deceleration and thrust must be positive')
        if settings['align_radius_m'] >= settings['approach_radius_m']:
            raise ValueError('align_radius_m must be smaller than approach_radius_m')
        self.settings = dict(settings)
        self.positions, self.axes = list(positions), list(axes)
        self.forward = [min(v, settings['max_thrust']) for v in forward_limits]
        self.reverse = [min(v, settings['max_thrust']) for v in reverse_limits]
        self.fully_actuated = independent_rows(allocation_matrix(self.positions, self.axes)) == 3
        self.mode = TRANSIT

    def reset(self):
        """Forget the mode, e.g. after a new target or a clock reset."""
        self.mode = TRANSIT

    def wrench(self, pose, velocity, target):
        """Body wrench (surge N, sway N, yaw N*m) for one control step.

        ``pose`` is (x, y, yaw) in map, ``velocity`` (u, v, r) in the body
        frame (m/s, m/s, rad/s), ``target`` (x, y, yaw) in map.
        """
        s = self.settings
        x, y, yaw = pose
        u, v, r = velocity
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        if self.mode == TRANSIT and distance <= s['approach_radius_m']:
            self.mode = STATION_KEEPING
        elif self.mode == STATION_KEEPING and distance > HYSTERESIS * s['approach_radius_m']:
            self.mode = TRANSIT
        # Position error in the body frame (x forward, y left).
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
        sway_damping = -s['kd_position'] * v if self.fully_actuated else 0.0
        if self.mode == TRANSIT:
            bearing_error = wrap(math.atan2(dy, dx) - yaw)
            speed = speed_limit(s['max_speed'], bearing_error, distance, NO_OBSTACLE_LIMIT_M, u,
                                s['braking_deceleration_mps2'], s['reaction_time_s'], 0.0)
            return (s['kp_surge'] * (speed - u), sway_damping,
                    s['kp_yaw'] * bearing_error - s['kd_yaw'] * r)
        if self.fully_actuated:
            heading_error = wrap(target[2] - yaw)
            sway = s['kp_position'] * ey - s['kd_position'] * v
        else:
            # Point at the target until close enough to turn to its heading.
            aim = target[2] if distance <= s['align_radius_m'] else math.atan2(dy, dx)
            heading_error = wrap(aim - yaw)
            sway = 0.0
        return (s['kp_position'] * ex - s['kd_position'] * u, sway,
                s['kp_yaw'] * heading_error - s['kd_yaw'] * r)

    def thrusts(self, pose, velocity, target):
        """Thruster forces in N, one per thruster, for one control step."""
        return allocate_thrusters(self.wrench(pose, velocity, target), self.positions, self.axes,
                                  self.forward, self.reverse)
