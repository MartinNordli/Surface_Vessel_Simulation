"""Closed-loop sanity run for the planner stack, without ROS or Gazebo.

A 3-DOF surface vessel, a simulated 2D lidar and the D* Lite planner from
njord_sim/njord_sim/dstar_lite.py, wired into the same loop shape the ROS nodes use:
scan -> occupancy grid -> plan -> line-of-sight guidance -> thrust -> motion.

The boat starts with an empty map, so every obstacle is discovered on the way
and every detour is a genuine replan. This is what you run before touching
Gazebo: if the planner cannot get a boat through here, no amount of
hydrodynamic fidelity will save it.

    python3 sandbox/headless_demo.py

Results are written to outputs/sandbox/ so a run never rewrites the committed
figures in this directory. Use --output-dir for a different destination.
"""

import argparse
import json
import math
import os
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "njord_sim"))
from njord_sim.dstar_lite import DStarLite, Grid

# ---------------------------------------------------------------------- #
# scenario
# ---------------------------------------------------------------------- #

WORLD = 200.0          # metres, square
RES = 2.0              # metres per grid cell
CELLS = int(WORLD / RES)
START = np.array([20.0, 20.0])
GOAL = np.array([180.0, 180.0])

# (x, y, radius) in metres. None of these are in the map at t=0.
OBSTACLES = [
    (60.0, 55.0, 6.0),
    (95.0, 90.0, 9.0),
    (120.0, 80.0, 5.0),
    (130.0, 140.0, 8.0),
    (75.0, 130.0, 7.0),
    (160.0, 150.0, 5.0),
    (45.0, 95.0, 6.0),
]

# vessel
MASS = 180.0
IZZ = 200.0
X_U = 40.0             # linear surge damping
N_R = 300.0            # linear yaw damping
BEAM = 1.8             # thruster separation, metres
T_MAX = 100.0          # newtons per thruster
DT = 0.05

# sensing
LIDAR_HZ = 5.0
LIDAR_RANGE = 50.0
LIDAR_BEAMS = 180
LIDAR_SIGMA = 0.15     # range noise, metres

# planning and control
INFLATION_CELLS = 3    # hull half-width plus safety margin
FOOTPRINT_CELLS = 2    # cells around the boat that are always kept free
LOOKAHEAD = 12.0
KP_PSI, KD_PSI = 900.0, 700.0
KP_U = 120.0
U_MAX = 4.0
GOAL_TOL = 5.0
TIMEOUT = 400.0


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def to_cell(p):
    return (int(p[1] / RES), int(p[0] / RES))


def to_world(cell):
    return np.array([(cell[1] + 0.5) * RES, (cell[0] + 0.5) * RES])


# ---------------------------------------------------------------------- #
# simulated sensor
# ---------------------------------------------------------------------- #

_OBS = np.array(OBSTACLES)


def lidar_scan(pos, heading, rng):
    """Analytic ray/circle intersection. Returns hit points in world frame."""
    angles = heading + np.linspace(-math.pi, math.pi, LIDAR_BEAMS, endpoint=False)
    dirs = np.stack([np.cos(angles), np.sin(angles)], axis=1)      # (B, 2)
    m = _OBS[:, :2][None, :, :] - pos[None, None, :]                # (1, O, 2)
    proj = np.einsum("bi,boi->bo", dirs, np.broadcast_to(m, (LIDAR_BEAMS, len(_OBS), 2)))
    m2 = np.sum(m**2, axis=2)                                       # (1, O)
    disc = proj**2 - (m2 - _OBS[:, 2][None, :] ** 2)
    t = np.where(disc >= 0, proj - np.sqrt(np.maximum(disc, 0.0)), np.inf)
    t = np.where(t > 0, t, np.inf)
    ranges = np.min(t, axis=1)
    hit = ranges < LIDAR_RANGE
    if not np.any(hit):
        return np.empty((0, 2))
    ranges = ranges[hit] + rng.normal(0.0, LIDAR_SIGMA, size=hit.sum())
    return pos[None, :] + dirs[hit] * ranges[:, None]


# ---------------------------------------------------------------------- #
# map
# ---------------------------------------------------------------------- #


def _disc(centre, radius_cells):
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc <= radius_cells**2:
                yield (centre[0] + dr, centre[1] + dc)


def integrate_scan(grid, hits, robot_cell):
    """Mark hits plus their inflation radius as blocked. Returns changed cells.

    The boat's own footprint is forced free every cycle. Without this, a track
    that clips the inflated ring around an obstacle walls the boat in and the
    planner correctly reports that no path exists, which looks like a planner
    bug but is a mapping bug. costmap_2d does the same thing.
    """
    changed = []
    footprint = {c for c in _disc(robot_cell, FOOTPRINT_CELLS) if grid.in_bounds(c)}

    for point in hits:
        for target in _disc(to_cell(point), INFLATION_CELLS):
            if not grid.in_bounds(target) or target in footprint:
                continue
            if grid.set_blocked(target, True):
                changed.append(target)

    for target in footprint:
        if grid.set_blocked(target, False):
            changed.append(target)
    return changed


# ---------------------------------------------------------------------- #
# guidance
# ---------------------------------------------------------------------- #


def lookahead_point(path_xy, pos):
    if len(path_xy) == 1:
        return path_xy[0]
    d = np.linalg.norm(path_xy - pos[None, :], axis=1)
    i = int(np.argmin(d))
    while i < len(path_xy) - 1 and np.linalg.norm(path_xy[i] - pos) < LOOKAHEAD:
        i += 1
    return path_xy[i]


# ---------------------------------------------------------------------- #
# main loop
# ---------------------------------------------------------------------- #


def run(seed=0, verbose=True, record_every=None):
    """record_every: capture a frame every N control steps, for sandbox/make_gif.py."""
    rng = np.random.default_rng(seed)
    grid = Grid(CELLS, CELLS)

    pos = START.copy()
    psi = math.atan2(GOAL[1] - START[1], GOAL[0] - START[0])
    u = 0.0
    r = 0.0

    planner = DStarLite(grid, to_cell(pos), to_cell(GOAL))
    planner.compute_shortest_path()

    trajectory = [pos.copy()]
    replans = 0
    plan_times = []
    min_clearance = math.inf
    t = 0.0
    next_scan = 0.0
    path_xy = np.array([to_world(c) for c in planner.path()]) if planner.path() else np.array([GOAL])
    first_plan = path_xy.copy()
    frames = []
    hits = np.empty((0, 2))
    step = 0

    while t < TIMEOUT:
        if t >= next_scan:
            next_scan += 1.0 / LIDAR_HZ
            hits = lidar_scan(pos, psi, rng)
            robot_cell = to_cell(pos)
            changed = integrate_scan(grid, hits, robot_cell)
            if changed:
                planner.update_start(robot_cell)
                planner.apply_changes(changed)
                t0 = time.perf_counter()
                planner.compute_shortest_path()
                plan_times.append(time.perf_counter() - t0)
                replans += 1
                cells = planner.path()
                if not cells:
                    if verbose:
                        print(f"  t={t:5.1f}s  no feasible path, aborting")
                    break
                path_xy = np.array([to_world(c) for c in cells])

        if record_every and step % record_every == 0:
            frames.append(
                {
                    "t": t,
                    "pos": pos.copy(),
                    "psi": psi,
                    "path": path_xy.copy(),
                    "hits": hits.copy(),
                    "blocked": set(grid.blocked),
                    "replans": replans,
                }
            )
        step += 1

        target = lookahead_point(path_xy, pos)
        psi_d = math.atan2(target[1] - pos[1], target[0] - pos[0])
        e = wrap(psi_d - psi)

        moment = KP_PSI * e - KD_PSI * r
        u_d = U_MAX * max(0.25, 1.0 - abs(e) / (math.pi / 2))
        thrust = KP_U * (u_d - u)

        left = np.clip(thrust / 2 - moment / BEAM, -T_MAX, T_MAX)
        right = np.clip(thrust / 2 + moment / BEAM, -T_MAX, T_MAX)
        thrust = left + right                    # actual, after saturation
        moment = (right - left) * BEAM / 2

        u += DT * (thrust - X_U * u) / MASS
        r += DT * (moment - N_R * r) / IZZ
        psi = wrap(psi + DT * r)
        pos = pos + DT * u * np.array([math.cos(psi), math.sin(psi)])

        trajectory.append(pos.copy())
        clearance = min(np.hypot(pos[0] - ox, pos[1] - oy) - orad for ox, oy, orad in OBSTACLES)
        min_clearance = min(min_clearance, clearance)
        t += DT

        if np.linalg.norm(pos - GOAL) < GOAL_TOL:
            break

    trajectory = np.array(trajectory)
    reached = bool(np.linalg.norm(pos - GOAL) < GOAL_TOL)
    metrics = {
        "seed": seed,
        "reached_goal": reached,
        "time_s": round(t, 2),
        "distance_travelled_m": round(float(np.sum(np.linalg.norm(np.diff(trajectory, axis=0), axis=1))), 1),
        "straight_line_m": round(float(np.linalg.norm(GOAL - START)), 1),
        "min_clearance_m": round(float(min_clearance), 2),
        "collision": bool(min_clearance < 0.0),
        "replans": replans,
        "max_plan_ms": round(max(plan_times) * 1e3, 1) if plan_times else 0.0,
        "mean_plan_ms": round(sum(plan_times) / len(plan_times) * 1e3, 1) if plan_times else 0.0,
    }
    return metrics, trajectory, first_plan, grid, frames


def plot(trajectory, first_plan, grid, path):
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    blocked = np.array([to_world(c) for c in grid.blocked]) if grid.blocked else np.empty((0, 2))
    if len(blocked):
        ax.scatter(blocked[:, 0], blocked[:, 1], s=3, c="#c9d4e0", label="discovered occupancy (inflated)")
    for ox, oy, orad in OBSTACLES:
        ax.add_patch(plt.Circle((ox, oy), orad, color="#5a6b7d", alpha=0.85))
    ax.plot(first_plan[:, 0], first_plan[:, 1], "--", c="#b0b0b0", lw=1.4, label="initial plan (empty map)")
    ax.plot(trajectory[:, 0], trajectory[:, 1], c="#1f77b4", lw=2.0, label="executed track")
    ax.plot(*START, "o", c="green", ms=9, label="start")
    ax.plot(*GOAL, "*", c="red", ms=16, label="goal")
    ax.set_xlim(0, WORLD)
    ax.set_ylim(0, WORLD)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("D* Lite closed loop, obstacles discovered by lidar en route")
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=140)


DEFAULT_OUTPUT = os.path.join(os.path.dirname(__file__), "..", "outputs", "sandbox")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Closed-loop planner run without ROS or Gazebo")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    output_dir = os.path.abspath(arguments.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    all_metrics = []
    for seed in range(5):
        metrics, trajectory, first_plan, grid, _ = run(seed)
        all_metrics.append(metrics)
        print(
            f"seed {seed}: goal={'yes' if metrics['reached_goal'] else 'NO '} "
            f"t={metrics['time_s']:6.1f}s  travelled={metrics['distance_travelled_m']:6.1f}m "
            f"(straight {metrics['straight_line_m']}m)  min clearance={metrics['min_clearance_m']:6.2f}m  "
            f"replans={metrics['replans']:3d}  worst plan={metrics['max_plan_ms']:.1f}ms"
        )
        if seed == 0:
            plot(trajectory, first_plan, grid, os.path.join(output_dir, "headless_demo.png"))

    with open(os.path.join(output_dir, "headless_metrics.json"), "w") as fh:
        json.dump(all_metrics, fh, indent=2)

    ok = all(m["reached_goal"] and not m["collision"] for m in all_metrics)
    print("\nPASS" if ok else "\nFAIL")
    sys.exit(0 if ok else 1)
