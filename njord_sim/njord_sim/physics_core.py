"""SI/FLU reference calculations; independent of ROS and Gazebo.

Independent Python implementation of the load models in njord/Loads.hh
(thruster wrench, wind coefficients, wind load). scripts/make_load_vectors.py
writes njord_gz_plugins/test/load_vectors.csv from these functions, and both
this module and the C++ header are tested against that file, so the two
implementations cannot drift apart unnoticed. Nothing here runs during a
simulation.

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


AIR_DENSITY_KG_M3 = 1.225  # same constant as njord::loads::kAirDensity


def wind_load(air_velocity, area_x, area_y, length, rows):
    """Body-frame wind force and yaw moment ``(X, Y, N)`` in N and N m.

    ``air_velocity`` is the relative air velocity (x, y[, z]) in the body frame
    (m/s); only the horizontal part counts. With the dynamic pressure
    q = 0.5 * rho_air * |v_xy|^2 and the coefficients for the direction the
    air moves toward: X = q Ax cx, Y = q Ay cy, N = q Ay L cn.
    """
    angle = math.degrees(math.atan2(air_velocity[1], air_velocity[0]))
    cx, cy, cn = wind_coefficients(angle, rows)
    pressure = 0.5 * AIR_DENSITY_KG_M3 * (air_velocity[0] ** 2 + air_velocity[1] ** 2)
    return pressure * area_x * cx, pressure * area_y * cy, pressure * area_y * length * cn
