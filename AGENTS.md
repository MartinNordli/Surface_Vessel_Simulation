# Njord simulator — agent instructions

Reproducible ROS 2 Jazzy / Gazebo Harmonic / VRX simulator for autonomous
surface-vessel simulation. The goal is to implement a simulator that can simulate
how Njord NTNU's surface vessel will behave in a tournament before deploying it
on the water. The other teams at Njord NTNU should be able to test their algorithms
and sensors accurately through this simulator.

These instructions apply to every task. Task-specific procedure lives in the
linked guides, not here.

## Repository layout

```
.
├── AGENTS.md                 This file: rules that apply to all work
├── README.md                 Project overview and quick start
├── Dockerfile                ROS 2 Jazzy + Gazebo Harmonic + VRX image
├── compose*.yaml             Base stack plus gui / wsl / cpu / record / film / team overlays
├── .codex/agents/            Sub-agent role definitions (see docs/agent-workflows.md)
├── .github/workflows/        CI/CD: unit suite, image build, container tests, GHCR publish
├── docker/                   Image dependencies, lock file, entrypoint, VRX license
├── docs/                     Documentation
│   ├── running.md            Every run option: courses, vessels, GUI, settings, results
│   ├── configuration.md      Where every setting lives; vessel/scenario/algorithm schema
│   ├── ros2-interface.md     ROS topics, types, frames, QoS, timing rules, external nodes
│   ├── control-autonomy.md   Step-by-step guide for running C&A's nodes (scripts/ca-nodes.sh)
│   ├── codebase.md           What each directory and file does
│   ├── physics-model.md      How the physics is computed; calibration protocol
│   ├── validation.md         Tests, CI and measured evidence
│   ├── agent-workflows.md    How to build, verify, delegate and commit
│   ├── evidence/             Compact summaries of runs the docs cite (scripts/summarize_evidence.py)
│   ├── media/                Images and animations used by the docs
│   └── archive/              Earlier decision documents (platform choice, C&A questions)
├── njord_sim/                ROS 2 Python package (main simulator logic)
│   ├── config/               algorithms.yaml, localization.yaml, RViz
│   │   ├── vessels/          One file per vessel: wamv.yaml (default), njord_v1.yaml, munin_v0.yaml
│   │   └── examples/         Partial sensor override and ROS parameter examples
│   ├── launch/               simulation.launch.py, dstar_demo.launch.py
│   └── njord_sim/            Nodes (*_node.py), pure-Python cores (*_core.py),
│                             configuration.py, constants.py
├── njord_gz_plugins/         C++ Gazebo plugins: actuator watchdog, Njord physics, contact monitor
├── scenarios/                Course files: gate races (reference, slalom), dynamics,
│                             setpoint courses (goto_square, goto_retarget, station_keeping,
│                             hold_position)
├── scripts/                  `njord` CLI, C&A node runner, benchmark, runners, host setup, build metadata
├── tests/                    unittest suite (test_*.py) and ROS smoke helpers
├── validation/               Independent checks: dynamics, lidar, runtime, timeouts
├── sandbox/                  Headless demo and figures without ROS/Gazebo
└── outputs/                  Generated runs, metrics and bags (git-ignored)
```

Node logic is split so that `*_core.py` is pure Python and testable without ROS,
while `*_node.py` only wires it to topics, parameters and timing.

## Engineering invariants

- Every setting has exactly one home ([docs/configuration.md](docs/configuration.md)):
  the vessel file, the scenario, `algorithms.yaml`, or `constants.py` for fixed
  platform values. Read values from there; never hardcode a second copy in a
  node, launch file, script or another YAML file. Each vessel file is
  self-contained.
- SI units, ENU world coordinates, ROS body/optical frames, timestamped TF.
- Algorithms and scoring use `/clock` simulation time; infrastructure watchdogs use
  steady wall time. Never freshen stale sensor data by re-stamping it.
- Ground truth and scenario obstacle coordinates belong to simulation and
  evaluation only; autonomy must run on sensor estimates. Truth mode is explicit.
- Drive thrusters in newtons. Never teleport or apply kinematic velocity as a
  shortcut; zero thrust does not stop a boat instantly.
- Invalid paths, stale inputs and failed processes must invalidate commands.
- Keep unknown, observed-free and occupied map cells distinct; do not clear an
  oversized circle around the vessel.
- Keep the pure-Python D* Lite core and the independent A* correctness checks.
- Report only measured evidence. CPU tests do not establish GPU rendering, marine
  fidelity, ROS integration or collision-free completion.

## Git

- Small, contained work (a doc edit, a one-file fix) may stay on the current
  branch. Anything larger — new behavior, changes across several files or
  packages, anything that needs its own validation — gets its own branch off
  `main` first: `git checkout -b <type>/<short-topic>`, e.g. `feat/gate-timeout`
  or `docs/sensor-config`. Never commit directly to `main`.
- Inspect the diff before staging, stage explicit paths, and keep commits small
  and coherent. Say in the commit which validation backs the change.
- Never overwrite uncommitted user changes and never rewrite shared history.
- Keep caches, builds, bags and generated metrics out of Git; generated results
  go in `outputs/`, which is git-ignored. The one exception is `docs/evidence/`:
  compact summaries that documentation cites, written only by
  `scripts/summarize_evidence.py` and never edited by hand.

## Subagents

Use subagents when a task is large and splits into parts that belong to
different areas of the system — platform, perception, autonomy, validation.
Each subagent starts without this conversation's context, which is the point:
give one part to one agent with only the context that part needs, so no agent
carries irrelevant detail. A small or single-area task is cheaper to do directly.

- Give each subagent: the goal, the files it owns, the invariants that apply, the
  verification command it must run, and an instruction to read this file first.
- Assign disjoint files. If edits would overlap, use separate worktrees.
- The primary agent owns shared interfaces, launch and package integration, and
  all commits. Subagents do not commit and do not change shared interfaces.
- Verify what a subagent reports; a claim of a passing check is not the check.
- Role definitions live in `.codex/agents/`; see
  [docs/agent-workflows.md](docs/agent-workflows.md) for what each role covers.

## Evidence and documentation

- Run the checks that actually cover the change, with fixed seeds, and keep
  config and commit provenance in metrics. Never rewrite tracked fixtures from
  tests.
- Document real defaults, topic types, frames, failure behavior and limitations.
  Keep third-party versions pinned and licenses and attribution intact. Never
  present WAM-V parameters or simplified rendering as measured Njord realism.

## Detailed guides

| Topic | File |
| --- | --- |
| Build and run commands, verification per change area, subagent roles | [docs/agent-workflows.md](docs/agent-workflows.md) |
| Courses, vessels, run settings, GUI and headless runs, results | [docs/running.md](docs/running.md) |
| Where settings live, file schemas, parameter precedence | [docs/configuration.md](docs/configuration.md) |
| ROS topics, message types, frames, QoS, guard timing, external nodes | [docs/ros2-interface.md](docs/ros2-interface.md) |
| Control & Autonomy's nodes against the simulator, step by step | [docs/control-autonomy.md](docs/control-autonomy.md) |
| What each directory and file does | [docs/codebase.md](docs/codebase.md) |
| Physics computation, model limits, calibration protocol | [docs/physics-model.md](docs/physics-model.md) |
| Test coverage, CI, measured evidence, dynamics measurements | [docs/validation.md](docs/validation.md) |
