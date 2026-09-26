"""SI/FLU reference calculations; independent of ROS and Gazebo.

Pure-Python mirrors of the force models used by the Njord vessel profile, so
the unit tests can check the math without building the C++ plugin:

- thruster_wrench / actuator_response  ->  NjordPhysics.cc and hydro::Response
- damping_wrench                        ->  gz-sim Hydrodynamics (xU / xUabsU terms)
- wind_coefficients                     ->  the wind table lookup in NjordPhysics.cc

Frames: FLU is the vessel body frame (x forward, y left, z up). Forces are in
newtons, torques in newton-metres, lengths in metres, velocities in m/s.
"""

import math


def thruster_wrench(force_n, position_m, axis, center_of_mass_m=(0.0, 0.0, 0.0)):
    """Return ``(force, torque)`` of one thruster about the centre of mass.

    Args:
        force_n: signed thrust in newtons along ``axis`` (negative = reverse).
        position_m: thruster position in the body frame (m).
        axis: unit thrust direction in the body frame.
        center_of_mass_m: centre of mass in the body frame (m).

    Returns body-frame force (N) and torque (N m) as 3-tuples; the torque is
    the lever arm (position - COM) crossed with the force.
    """
    force = tuple(force_n * a for a in axis)
    r = tuple(p - c for p, c in zip(position_m, center_of_mass_m))
    torque = (
        r[1] * force[2] - r[2] * force[1],
        r[2] * force[0] - r[0] * force[2],
        r[0] * force[1] - r[1] * force[0],
    )
    return force, torque


def actuator_response(
    previous, command, dt, response_time, forward, reverse, valid=True
):
    """Expired commands target zero; physical first-order residual force decays.

    Args:
        previous: thrust applied in the previous step (N).
        command: requested thrust (N); clipped to [-reverse, forward].
        dt: step length (s).
        response_time: first-order time constant tau (s); 0 means instant.
        forward, reverse: positive thrust limits (N) in each direction.
        valid: False when the command is stale; the target becomes zero.

    Returns the new applied thrust (N). A zero target does not remove force
    instantly: it decays with ``tau``, and the hull keeps its momentum.
    """
    target = (
        max(-reverse, min(forward, command))
        if valid and math.isfinite(command)
        else 0.0
    )
    # Exact discretisation of dF/dt = (target - F) / tau over one step:
    # F += (target - F) * (1 - exp(-dt/tau)); expm1 keeps precision for small dt.
    return (
        target
        if response_time == 0
        else previous + (target - previous) * (-math.expm1(-dt / response_time))
    )


def damping_wrench(relative_velocity, linear, quadratic):
    """Return the six-DOF damping wrench for a body-frame relative velocity.

    ``relative_velocity`` is (u, v, w, p, q, r) relative to the water (m/s,
    rad/s). ``linear`` and ``quadratic`` are the nonnegative damping magnitudes
    from the vessel YAML. The result always opposes motion:
    -linear*v - quadratic*|v|*v per axis, the same sign convention Gazebo gets
    from the negated xU / xUabsU parameters written by njord_model.py.
    """
    return tuple(
        -l * v - q * abs(v) * v for v, l, q in zip(relative_velocity, linear, quadratic)
    )


def wind_coefficients(angle_deg, rows):
    """Periodic linear interpolation; angles describe air velocity toward FLU.

    ``angle_deg`` is the direction the relative air flow moves toward, in the
    body frame, degrees counter-clockwise from forward (any value; wrapped to
    [0, 360)). ``rows`` are dicts with ``angle_deg``, ``cx``, ``cy``, ``cn``.
    Returns interpolated ``(cx, cy, cn)``; the table wraps from its last angle
    back to its first across 360 degrees.
    """
    rows = sorted(rows, key=lambda r: r["angle_deg"])
    angle = angle_deg % 360
    for i, a in enumerate(rows):
        b = rows[(i + 1) % len(rows)]
        start, end = a["angle_deg"], b["angle_deg"]
        # The last interval wraps around 360 degrees to the first row.
        if end <= start:
            end += 360
        value = angle if angle >= start else angle + 360
        if start <= value <= end:
            f = (value - start) / (end - start)
            return tuple(a[k] + f * (b[k] - a[k]) for k in ("cx", "cy", "cn"))
    raise ValueError("invalid wind coefficient table")
