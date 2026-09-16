"""Pure calculations for the open-loop manoeuvre report."""
import math


def summarize(samples, straight_s, turn_s):
    straight = [s for s in samples if s[0] < straight_s]
    turn = [s for s in samples if straight_s + turn_s / 2 < s[0] < straight_s + turn_s]
    coast_start = straight_s + turn_s
    coast = [s for s in samples if s[0] >= coast_start]
    if not straight or not turn or len(coast) < 2:
        raise ValueError("all manoeuvres require odometry samples")
    top_speed = max(s[3] for s in straight)
    rise = next((s[0] for s in straight if s[3] >= 0.95 * top_speed), None)
    yaw_rate = sum(s[4] for s in turn) / len(turn)
    turn_speed = sum(s[3] for s in turn) / len(turn)
    radius = turn_speed / abs(yaw_rate) if abs(yaw_rate) > 1e-6 else None
    entry_speed = coast[0][3]
    stop_index = next((i for i, s in enumerate(coast) if s[3] <= max(0.05, 0.05 * entry_speed)), None)
    observed = coast[:stop_index + 1] if stop_index is not None else coast
    distance = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(observed, observed[1:]))
    return {"top_speed_mps": top_speed, "time_to_95pct_s": rise,
            "mean_acceleration_to_95pct_mps2": (0.95 * top_speed - straight[0][3]) / (rise - straight[0][0])
            if rise is not None and rise > straight[0][0] else None,
            "turn_speed_mps": turn_speed, "steady_yaw_rate_radps": yaw_rate,
            "turning_radius_m": radius, "coast_entry_speed_mps": entry_speed,
            "coast_distance_m": distance, "stopped_within_observation": stop_index is not None,
            "coast_observation_s": observed[-1][0] - observed[0][0]}


def pose_drift(samples, column):
    """Least-squares world-position / Euler-angle slope over simulation time.

    Body heave includes forward motion at pitch; body angular rates also need
    rotation before becoming Euler rates. Pose slopes avoid both frame errors.
    Angles are unwrapped to avoid a spurious drift at the +/-pi boundary.
    """
    times = [row[0] - samples[0][0] for row in samples]
    values = [row[column] for row in samples]
    if column in (6, 7):
        for index in range(1, len(values)):
            values[index] = values[index - 1] + math.remainder(values[index] - values[index - 1], 2 * math.pi)
    mean_t, mean_value = sum(times) / len(times), sum(values) / len(values)
    denominator = sum((t - mean_t) ** 2 for t in times)
    return sum((t - mean_t) * (value - mean_value) for t, value in zip(times, values)) / denominator if denominator else 0.0


def stationary(samples, window_s=10.0, speed_tolerance=0.01,
               yaw_tolerance=0.001, max_gap_s=0.5):
    """Require a full, contiguous trailing window with bounded peak-to-peak response.

    Rows are (simulation seconds, x m, y m, speed m/s, yaw rate rad/s).
    This is an operational stationarity criterion, not calibration evidence.
    """
    if window_s <= 0 or speed_tolerance < 0 or yaw_tolerance < 0:
        raise ValueError("invalid stationarity limits")
    if len(samples) < 2:
        return False
    if any(len(s) < 5 or not all(math.isfinite(v) for v in s) for s in samples):
        return False
    if any(b[0] <= a[0] for a, b in zip(samples, samples[1:])):
        return False
    start = samples[-1][0] - window_s
    # Include the sample at/before the window boundary so coverage is explicit.
    index = max((i for i, row in enumerate(samples) if row[0] <= start), default=-1)
    if index < 0:
        return False
    tail = samples[index:]
    if any(b[0] - a[0] > max_gap_s for a, b in zip(tail, tail[1:])):
        return False
    if len(tail[0]) >= 12:
        for column, tolerance in ((5, 0.01), (6, 0.001), (7, 0.001)):
            if max(s[column] for s in tail) - min(s[column] for s in tail) > tolerance:
                return False
        # Require bounded pose AND small secular drift. A tiny bounded vibration
        # may have large instantaneous derivatives without a changing equilibrium.
        if any(abs(pose_drift(tail, column)) > tolerance
               for column, tolerance in ((5, 0.01), (6, 0.001), (7, 0.001))):
            return False
    return (max(s[3] for s in tail) - min(s[3] for s in tail) <= speed_tolerance
            and max(s[4] for s in tail) - min(s[4] for s in tail) <= yaw_tolerance)


def summarize_experiment(samples, experiment, window_s=10.0):
    """One isolated manoeuvre; missing steady response and stopping are incomplete."""
    if experiment not in {"straight", "reverse", "turn_left", "turn_right", "coast", "drift", "hydrostatic"}:
        raise ValueError("unknown experiment")
    if len(samples) < 2 or any(len(s) < 5 or not all(math.isfinite(v) for v in s) for s in samples):
        return {"complete": False, "reason": "insufficient or invalid samples", "metrics": {}}
    if any(b[0] <= a[0] or b[0] - a[0] > 0.5 for a, b in zip(samples, samples[1:])):
        return {"complete": False, "reason": "nonmonotonic or missing odometry", "metrics": {}}
    distance = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(samples, samples[1:]))
    metrics = {"observed_distance_m": distance, "peak_speed_mps": max(s[3] for s in samples),
               "observation_s": samples[-1][0] - samples[0][0]}
    stable = stationary(samples, window_s)
    tail = [s for s in samples if s[0] >= samples[-1][0] - window_s]
    if len(samples[0]) >= 12:
        metrics.update(mean_z_m=sum(s[5] for s in tail) / len(tail),
                       mean_roll_rad=sum(s[6] for s in tail) / len(tail),
                       mean_pitch_rad=sum(s[7] for s in tail) / len(tail),
                       mean_surge_mps=sum(s[8] for s in tail) / len(tail))
        metrics.update(world_vertical_drift_mps=pose_drift(tail, 5),
                       roll_drift_radps=pose_drift(tail, 6),
                       pitch_drift_radps=pose_drift(tail, 7))
        early = [s for s in samples if s[0] <= samples[0][0] + 2.0]
        if len(early) > 1:
            metrics["observed_initial_surge_acceleration_mps2"] = (early[-1][8] - early[0][8]) / (early[-1][0] - early[0][0])
    if experiment == "hydrostatic":
        if len(samples[0]) < 12:
            return {"complete": False, "reason": "hydrostatic pose and velocity channels missing", "metrics": metrics}
        excited = [column for column in (6, 7) if abs(samples[0][column]) >= 0.01]
        restored = bool(excited) and all(abs(sum(s[c] for s in tail) / len(tail)) <= 0.5 * abs(samples[0][c]) for c in excited)
        metrics["restoring_response_observed"] = restored
        complete = stable and restored
        return {"complete": complete, "reason": None if complete else "small-angle restoring response or settled float pose not established", "metrics": metrics}
    if experiment == "coast":
        stopped = stable and all(s[3] <= 0.05 and abs(s[4]) <= 0.001 for s in tail)
        metrics.update(coast_entry_speed_mps=samples[0][3],
                       coast_distance_m=distance if stopped else None,
                       stopped_within_observation=stopped)
        return {"complete": stopped, "reason": None if stopped else "stop not established; observed distance is a lower bound", "metrics": metrics}
    if stable:
        speed = sum(s[3] for s in tail) / len(tail)
        yaw = sum(s[4] for s in tail) / len(tail)
        metrics.update(steady_speed_mps=speed, steady_yaw_rate_radps=yaw)
        if experiment.startswith("turn"):
            metrics["turning_radius_m"] = speed / abs(yaw) if abs(yaw) > 0.001 else None
            stable = abs(yaw) > 0.001
    if experiment in {"straight", "reverse"}:
        direction_ok = len(samples[0]) >= 12 and (metrics["mean_surge_mps"] > 0.01 if experiment == "straight" else metrics["mean_surge_mps"] < -0.01)
        stable = stable and direction_ok
    return {"complete": stable, "reason": None if stable else "stationary response or commanded surge direction not established", "metrics": metrics}
