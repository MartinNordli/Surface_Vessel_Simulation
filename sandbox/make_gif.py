"""Render the sandbox closed-loop run as an animated GIF.

    python3 sandbox/make_gif.py

Shows what the boat knows, not what the world looks like: the discovered
occupancy grid grows as the lidar sweeps, the plan snaps to a new route each
time a buoy is found, and the track lags the plan because the boat cannot turn
on the spot. That lag is the interesting part of the picture.
"""

import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.dirname(__file__))
from headless_demo import GOAL, OBSTACLES, START, WORLD, run, to_world

RECORD_EVERY = 12          # control steps between frames (0.05 s each)
FPS = 20


def main(seed=0):
    metrics, trajectory, _, _, frames = run(seed, verbose=False, record_every=RECORD_EVERY)
    print(f"{len(frames)} frames, {metrics['time_s']}s simulated, {metrics['replans']} replans")

    fig, ax = plt.subplots(figsize=(6.0, 6.0))
    fig.subplots_adjust(left=0.1, right=0.97, top=0.93, bottom=0.09)

    def draw(i):
        frame = frames[i]
        ax.clear()
        ax.set_xlim(0, WORLD)
        ax.set_ylim(0, WORLD)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

        if frame["blocked"]:
            cells = np.array([to_world(c) for c in frame["blocked"]])
            ax.scatter(cells[:, 0], cells[:, 1], s=2.5, c="#c9d4e0", zorder=1)
        for ox, oy, orad in OBSTACLES:
            ax.add_patch(plt.Circle((ox, oy), orad, color="#5a6b7d", alpha=0.9, zorder=2))
        if len(frame["hits"]):
            ax.scatter(frame["hits"][:, 0], frame["hits"][:, 1], s=4, c="#e8833a", zorder=4)

        ax.plot(frame["path"][:, 0], frame["path"][:, 1], c="#9aa7b4", lw=1.3, zorder=3)
        travelled = trajectory[: (i * RECORD_EVERY) + 1]
        ax.plot(travelled[:, 0], travelled[:, 1], c="#1f77b4", lw=2.0, zorder=5)

        x, y = frame["pos"]
        ax.plot(
            x, y, marker=(3, 0, np.degrees(frame["psi"]) - 90), ms=11, c="#1f77b4", zorder=6
        )
        ax.plot(*START, "o", c="green", ms=7, zorder=6)
        ax.plot(*GOAL, "*", c="red", ms=14, zorder=6)
        ax.set_title(
            f"t = {frame['t']:5.1f} s     replans: {frame['replans']:3d}     "
            f"cells mapped: {len(frame['blocked']):4d}",
            fontsize=10,
            family="monospace",
        )

    animation = FuncAnimation(fig, draw, frames=len(frames), interval=1000 / FPS)
    out = os.path.join(os.path.dirname(__file__), "headless_demo.gif")
    animation.save(out, writer=PillowWriter(fps=FPS), dpi=100)
    print(f"wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
