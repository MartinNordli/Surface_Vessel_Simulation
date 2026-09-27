"""Independent analytic checks for isolated, explicitly simplified experiments.

These do not integrate the plugin equations and do not certify marine fidelity.
Onset acceleration assumes rest, no current/wind/restoring force, diagonal
inertia and diagonal added mass. Nonzero COM offsets require the caller to
account for rigid-body coupling before comparing acceleration.
"""
import math

ABSOLUTE = {'N': .02, 'N*m': .02, 'm/s^2': .01, 'rad/s^2': .001,
            'm': .02, 'm/s': .01, 'rad/s': .001, 'rad': .001, 's': .01}


def wrench(thrusters, forces, center_of_mass_m):
    if len(thrusters) != len(forces) or not thrusters:
        raise ValueError('one force per thruster required')
    force, moment = [0., 0., 0.], [0., 0., 0.]
    for thruster, value in zip(thrusters, forces):
        if not math.isfinite(value):
            raise ValueError('nonfinite force')
        yaw = math.radians(thruster['yaw_deg'])
        vector = [value * math.cos(yaw), value * math.sin(yaw), 0.]
        arm = [x-c for x,c in zip(thruster['position_m'], center_of_mass_m)]
        cross = [arm[1]*vector[2]-arm[2]*vector[1], arm[2]*vector[0]-arm[0]*vector[2],
                 arm[0]*vector[1]-arm[1]*vector[0]]
        force = [x+y for x,y in zip(force, vector)]
        moment = [x+y for x,y in zip(moment, cross)]
    return {'force_n': force, 'moment_nm': moment}


def diagonal_acceleration(load, mass_kg, inertia_diagonal, added_mass):
    denominators = [x+y for x,y in zip([mass_kg]*3+list(inertia_diagonal), added_mass)]
    if len(denominators) != 6 or any(not math.isfinite(x) or x <= 0 for x in denominators):
        raise ValueError('six finite positive effective mass terms required')
    return [f/m for f,m in zip(load['force_n']+load['moment_nm'], denominators)]


def response(target, initial, dt, tau):
    if not all(math.isfinite(v) for v in (target, initial, dt, tau)) or dt < 0 or tau < 0:
        raise ValueError('finite nonnegative time required')
    return target if tau == 0 else target + (initial-target)*math.exp(-dt/tau)


def acceptance(observed, expected, unit):
    floor = ABSOLUTE[unit]
    valid = all(isinstance(x, (int,float)) and not isinstance(x,bool) and math.isfinite(x)
                for x in (observed, expected))
    tolerance = max(floor, .02*abs(expected)) if valid else None
    return {'passed': valid and abs(observed-expected) <= tolerance,
            'observed': observed if valid else None, 'expected': expected if valid else None,
            'tolerance': tolerance if valid else None, 'unit': unit,
            'evidence_level': 'analytic_correctness_only'}
