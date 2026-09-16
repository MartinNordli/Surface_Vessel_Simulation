"""SI/FLU reference calculations; independent of ROS and Gazebo."""

import math


def thruster_wrench(force_n, position_m, axis, center_of_mass_m=(0.0, 0.0, 0.0)):
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
    """Expired commands target zero; physical first-order residual force decays."""
    target = (
        max(-reverse, min(forward, command))
        if valid and math.isfinite(command)
        else 0.0
    )
    return (
        target
        if response_time == 0
        else previous + (target - previous) * (-math.expm1(-dt / response_time))
    )


def damping_wrench(relative_velocity, linear, quadratic):
    return tuple(
        -l * v - q * abs(v) * v for v, l, q in zip(relative_velocity, linear, quadratic)
    )


def wind_coefficients(angle_deg, rows):
    """Periodic linear interpolation; angles describe air velocity toward FLU."""
    rows = sorted(rows, key=lambda r: r["angle_deg"])
    angle = angle_deg % 360
    for i, a in enumerate(rows):
        b = rows[(i + 1) % len(rows)]
        start, end = a["angle_deg"], b["angle_deg"]
        if end <= start:
            end += 360
        value = angle if angle >= start else angle + 360
        if start <= value <= end:
            f = (value - start) / (end - start)
            return tuple(a[k] + f * (b[k] - a[k]) for k in ("cx", "cy", "cn"))
    raise ValueError("invalid wind coefficient table")
