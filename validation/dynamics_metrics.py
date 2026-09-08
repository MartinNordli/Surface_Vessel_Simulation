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
