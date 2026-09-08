# Njord simulator engineering instructions

## Purpose and architecture
Build a reproducible ROS 2 Jazzy / Gazebo Harmonic / VRX simulator for autonomous
surface-vessel research. WAM-V is the initial reference, not a calibrated Njord
digital twin. Keep simulation, sensor processing, planning, control and scoring
replaceable through documented ROS interfaces. Preserve the pure-Python D* Lite
core and independent A* correctness checks.

## Engineering invariants
- Use SI units, ENU world coordinates, ROS body/optical frames and timestamped TF.
- Use `/clock` and simulation time for algorithms and race scoring. Use steady
  wall time for infrastructure watchdogs. Never freshen old sensor data by merely
  republishing it with a new timestamp.
- Ground truth and scenario obstacle coordinates belong to simulation/evaluation;
  autonomous navigation must use sensor estimates. Debug truth mode is explicit.
- Drive physical thrusters in newtons; never teleport or apply kinematic velocity
  as a shortcut. Zero thrust does not instantly stop a boat.
- Invalid paths, stale inputs and failed processes must invalidate commands.
- Keep unknown, observed free and occupied map cells distinct. Do not erase nearby
  obstacles by clearing an oversized circle around the vessel.
- Report measured evidence honestly. CPU tests do not establish GPU rendering,
  marine fidelity, successful ROS integration or collision-free completion.

## Commands and verification
- Planner baseline: `python3 tests/test_dstar_lite.py`.
- Unit suite: `python3 -m unittest discover -s tests -p 'test_*.py'`.
- Container/launch/benchmark commands are maintained in README.md and scripts/.
- Run focused regressions for changed behavior, then the integrated checks needed
  for the claim. Use fixed seeds and retain config/commit provenance in metrics.
- Never rewrite tracked fixtures through tests. Put generated results in outputs/.

## Git and collaboration
Use a feature branch and small, coherent commits with relevant validation. Inspect
the diff before staging explicit paths. Do not overwrite user changes or rewrite
shared history. Exclude caches, builds, bags and generated metrics from Git.

Delegate independent substantial work to specialized agents when useful:
- `sim_platform`: containers, Gazebo/VRX assets, bridges and NVIDIA integration.
- `perception`: timestamped sensor transforms, mapping, buoy detection and fusion.
- `autonomy`: D* Lite, mission execution, guidance and command validity.
- `validation`: independent tests, dynamics, scoring and reproducibility review.

The primary agent owns shared interfaces, launch/package integration, and commits.
Assign disjoint files to workers; use worktrees if edits overlap. Agents inherit
the session model and permissions. The validation role should independently
review implementation claims and may add tests when assigned explicit ownership.

## Documentation
Document actual defaults, topic types, frames, failure behavior and limitations.
Keep third-party versions pinned and licenses/attribution intact. Never describe
WAM-V parameters or simplified camera/water rendering as measured Njord realism.
