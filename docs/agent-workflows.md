# Agent workflows

Procedures for common tasks. [AGENTS.md](../AGENTS.md) holds the rules that apply
regardless of task; this file holds the how-to. User-facing detail on running and
configuring the simulator stays in [running.md](running.md) and
[team-integration.md](team-integration.md).

## Build and run

All container work goes through the `scripts/njord` wrapper, which selects the
right Compose overlay (including WSL) and stamps run provenance.

```bash
./scripts/njord pull [tag]            # Pull the CI-tested image for this commit as njord-sim:local
./scripts/njord build                 # Build njord-sim:local with commit/digest build args
./scripts/njord check-image           # Fail unless the image was built from this checkout
./scripts/njord test                  # Full unit suite inside the container
./scripts/njord selftest              # Start simulator + autonomy, run check_runtime.py, stop
./scripts/njord demo [reference|slalom]   # Headless race, exits with the evaluator
./scripts/njord gui   [reference|slalom]  # Same with Gazebo GUI and RViz
./scripts/njord lab   [reference|slalom]  # Simulator + autonomy kept running, no evaluator
./scripts/njord benchmark [reference|slalom] [--seeds ...] [--environments ...]
./scripts/njord smoke                 # validation/check_runtime.py against a live stack
./scripts/njord doctor                # Docker, Compose, NVIDIA and image checks
```

`NJORD_CPU=1` adds `compose.cpu.yaml` (Mesa software rendering, no GPU devices) for
hosts without a usable GPU; CI uses it. It shows that sensors work, not rendering
performance. Image commands warn when the image's source digest differs from the
checkout: pull again after changing commits, or build after local edits to files
copied into the image.

Each `demo`/`gui`/`lab` run writes to a fresh `OUTPUT_HOST` under `outputs/`; the
wrapper refuses to reuse a directory that already holds a finished run.

## Verification by change area

Run the focused checks for what changed, then the integrated checks that the claim
needs. Use fixed seeds and keep config and commit provenance in the metrics.

| Change | Minimum verification |
| --- | --- |
| Planner / D* Lite | `python3 tests/test_dstar_lite.py` (independent A* comparison) |
| Any pure-Python core | `python3 -m unittest discover -s tests -p 'test_*.py'` |
| Nodes, launch, config, ROS wiring | `./scripts/njord test` (host suite skips ROS-dependent tests) |
| Sensors, TF, rendering | `./scripts/njord selftest`, or `./scripts/njord smoke` against a running `lab` stack |
| Dynamics / thruster behavior | `validation/check_dynamics.py` with only the simulator service up |
| Perception and guidance end to end | `./scripts/njord demo`, then a benchmark on matched seeds |
| Scoring, repeatability, regressions | `./scripts/njord benchmark` with explicit seeds and environments |
| Dockerfile, compose, `scripts/njord`, CI workflow | `./scripts/njord build`, `check-image`, `test`, `NJORD_CPU=1 ./scripts/njord selftest`; push and confirm the GitHub CI run |
| Closed-loop planner without ROS | `python3 sandbox/headless_demo.py` |

Host runs skip tests whose ROS or model dependencies are missing; only the
container suite is a complete pass. A CPU test result never substantiates GPU
rendering, marine fidelity, ROS integration or collision-free completion.

Detailed coverage, measured baselines and known failure modes are described under
[docs/validation.md](validation.md).

## Continuous integration

`.github/workflows/ci.yml` runs on every push, `v*` tag and fork pull request. The
`unit` job runs the host suite. The `image` job builds the image with a GHCR layer
cache, then runs `check-image`, `test` and `NJORD_CPU=1 selftest`. Only if all of
these pass does it publish `ghcr.io/martinnordli/surface_vessel_simulation` as
`sha-<commit>`, `latest` (main) and the tag name. `./scripts/njord pull` fetches
`sha-<commit>` for the checked-out commit, so every image other teams use has
passed CI.

- A green CI run shows that the image builds and the container suite passes. It
  also shows that live sensors, TF and navigation start with software rendering.
  It does not show GPU rendering, race completion or benchmark results.
- Report CI as passed only after the GitHub run has finished. Local checks are not
  a CI result.
- Keep `scripts/njord` the single entry point: CI calls the same commands as users.
  Change a command and its CI step together.

## Changing interfaces

Topic names, message types, frames, QoS and node ownership are documented in
[docs/interfaces.md](interfaces.md). Update that file in the same commit as any
interface change, and keep replaceable-algorithm boundaries intact so a team can
substitute their own controller, perception or mapping node.

## Delegating to subagents

[AGENTS.md](../AGENTS.md#subagents) says when to split work out; this is the scope
of each role in `.codex/agents/`:

- `sim_platform` — containers, Gazebo/VRX assets, bridges, NVIDIA integration.
  Owns `Dockerfile`, `compose*.yaml`, `docker/`, `njord_gz_plugins/`, `scripts/`,
  `.github/workflows/`.
- `perception` — timestamped sensor transforms, mapping, buoy detection, fusion.
  Owns `perception_*`, `mapper_node.py`, `mapping_core.py`, `sensor_adapter_node.py`.
- `autonomy` — D* Lite, mission execution, guidance, command validity. Owns
  `dstar_lite.py`, `planner_*`, `guidance_node.py`, `mission_node.py`,
  `command_guard_node.py`, `control_core.py`.
- `validation` — independent tests, dynamics, scoring, reproducibility review.
  Owns `validation/`, `tests/` and `evaluator_node.py`, and reviews the claims the
  other roles make. It adds tests only for files assigned to it.

A useful task brief for one subagent contains: read `AGENTS.md` first; the goal in
one or two sentences; the exact files it may edit; the invariants that bear on
those files; the verification command it must run and report output from; and
anything it must not touch (shared interfaces, launch wiring, commits).

Split a task by area, not by step, so each agent needs only its own slice of the
system. If two parts would edit the same file, run them in sequence or in separate
worktrees rather than in parallel.
