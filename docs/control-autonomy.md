# Control & Autonomy: run your nodes in the simulator

A step-by-step guide for running Control & Autonomy's (C&A) own nodes from
the `Njord2627` repository against the simulator, and watching the boat
drive to target positions (setpoints) in Gazebo and RViz.

You need: a PC with Ubuntu 24.04 (or Windows with WSL2) and an NVIDIA
graphics card, and read access to `github.com/NjordNTNU/Njord2627`.

Contents: [1 How it fits together](#1-how-it-fits-together) ·
[2 Set up the PC (once)](#2-set-up-the-pc-once) ·
[3 Run a setpoint course with windows](#3-run-a-setpoint-course-with-windows) ·
[4 Send your own targets](#4-send-your-own-targets) ·
[5 After changing your code](#5-after-changing-your-code) ·
[6 The interface](#6-the-interface) · [7 Results](#7-results) ·
[8 Troubleshooting](#8-troubleshooting) · [9 Without windows](#9-without-windows) ·
[10 What has been tested](#10-what-has-been-tested)

## 1. How it fits together

```
 Simulator                       C&A nodes (simulation mode)                            Simulator
 ─────────                       ───────────────────────────                            ─────────
 /njord/odometry ──▶ navigation_node ──▶ /vessel/state_est ──┐
   (ENU, FLU)        ENU→NED, FLU→FRD                         ├─▶ dp_node ──▶ /dp/control_signal ──▶ thrust_allocator ──▶ /thruster_1/command
 /njord/setpoint ───────────────────────────────────────────┘    (NED)       (TauBody)                                    …
   (target pose)                                                                                                          /thruster_4/command
 /clock ──▶ use_sim_time on every node                                                                                     (newtons)
```

- The **simulator** publishes the boat's state on `/njord/odometry` and the
  target on `/njord/setpoint`, in ROS conventions (ENU world, FLU body).
- **C&A's nodes** convert to NED/FRD, run the DP controller, allocate the
  force to the four thrusters and publish one force in newtons per thruster.
- The **evaluator** in the simulator issues the targets of a course one at a
  time and scores how well the boat reaches and holds each one.

The boat is `munin_v0`, a placeholder with Munin's assumed four-thruster X
layout and an uncalibrated hull. Results say how the controller behaves
against this model, not how Munin will behave.

## 2. Set up the PC (once)

Everything lives in one folder, `Njord`, next to each other:

```
~/Njord/
├── Njord2627/                   C&A's repository (your code)
├── Surface_Vessel_Simulation/   the simulator
└── ca_sim_ws/                   created by the setup: build files and Python packages
```

Build files go in `ca_sim_ws/`, so neither repository gets any.

**Step 1: check the PC.**

```bash
lsb_release -ds      # Ubuntu 24.04 (on WSL2: inside the Ubuntu window)
nvidia-smi           # a table with your graphics card
```

If `nvidia-smi` fails, install the NVIDIA driver first. On WSL2 the Windows
driver is used; do not install a driver inside WSL.

**Step 2: get both repositories.**

```bash
mkdir -p ~/Njord && cd ~/Njord
git clone https://github.com/NjordNTNU/Njord2627.git
git clone https://github.com/MartinNordli/Surface_Vessel_Simulation.git
```

**Step 3: install Docker** (once per PC).

```bash
cd ~/Njord/Surface_Vessel_Simulation
sudo bash scripts/setup-host.sh       # Docker, Compose and the NVIDIA Container Toolkit
sudo usermod -aG docker $USER         # use docker without sudo
```

Log out and in again (on WSL2: close all Ubuntu windows and run `wsl
--shutdown` in Windows), then check that `docker ps` works without `sudo`.

**Step 4: get the simulator image** (a few GB).

```bash
cd ~/Njord/Surface_Vessel_Simulation
./scripts/njord pull
./scripts/njord doctor
```

Expected: `doctor` prints the `nvidia-smi` table twice and ends with
`Image sha256:… matches this checkout`. If `pull` says `No published image`,
CI has not finished for the newest commit yet; wait, or run
`./scripts/njord build` (slow the first time).

**Step 5: build C&A's nodes.**

```bash
cd ~/Njord/Surface_Vessel_Simulation
scripts/ca-nodes.sh setup     # Python packages for thrust allocation (skadipy, scipy, …), once
scripts/ca-nodes.sh build     # colcon build of navigation, dynamic_pos, thrust_allocation
```

Expected: `Installed Python packages in /home/<you>/Njord/ca_sim_ws/pydeps`,
then `Summary: 6 packages finished`. The nodes are built inside the
simulator's ROS 2 Jazzy image, so you do not need ROS installed on the PC.

## 3. Run a setpoint course with windows

Use two terminals, both in `~/Njord/Surface_Vessel_Simulation`. Run them from
the desktop (not over SSH) so the windows can open.

**Terminal 1: start C&A's nodes.**

```bash
cd ~/Njord/Surface_Vessel_Simulation
scripts/ca-nodes.sh run
```

Expected:

```
[INFO] [dp_node]: DPNode started
[INFO] [dp_node]: DP Simulation Mode Enabled: Receiving setpoint from /njord/setpoint topic
[INFO] [thrust_allocation]: Starting thrust allocator node
```

The nodes now wait for the simulator.

**Terminal 2: start the simulator with windows.**

```bash
cd ~/Njord/Surface_Vessel_Simulation
STATE_SOURCE=truth CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml \
  ./scripts/njord gui goto_square
```

| Setting | Why |
|---|---|
| `STATE_SOURCE=truth` | `/njord/odometry` is the exact simulated state, like an ideal Seapath. Without it the nodes get a GPS/IMU estimate with very noisy velocity. |
| `CONTROLLER=external` | Turns off the simulator's own controller, so only your nodes drive |
| `VESSEL_CONFIG=…munin_v0.yaml` | The four-thruster boat your allocation is written for |
| `gui` | A measured run with the Gazebo and RViz windows |
| `goto_square` | The course: four targets on a 20 m square |

What happens:

1. Two windows open: Gazebo (the simulation) and RViz (the targets and the
   boat's track). They open on top of each other; move them apart.
2. The race starts by itself once your nodes are connected. Until then
   terminal 2 prints what it waits for, e.g.
   `Waiting for a subscriber on /njord/setpoint, a publisher on /thruster_1/command …`.
3. Terminal 1 prints `position received`, then `reference received` once a
   second.
4. Terminal 2 prints one line per target:

   ```
   [evaluator]: Setpoint 1/4 'east' issued at 0.0 s: x=20.00 y=0.00 heading=0.0 deg
   [evaluator]: Setpoint 1/4 'east' reached after 19.1 s: error 0.15 m / 0.0 deg
   [evaluator]: Setpoint 2/4 'north_east' issued at 19.1 s: x=20.00 y=20.00 heading=90.0 deg
   …
   [evaluator]: Setpoint 4/4 'home' reached after 44.5 s: error 0.02 m / 7.8 deg
   Report: …/outputs/run-<time>/report.html
   ```

5. Gazebo closes and the command ends. `echo $?` prints 0 when every target
   was reached. RViz stays open with the result until the next `gui` run
   replaces it (or `docker rm -f njord-sim-rviz-1`).

**In RViz** you see the square from above. Each target is a disc with an
arrow (the heading to hold) and a label. The active target is yellow, a
reached target turns green, a failed one red. The white line is the boat's
track. Choose **Focus Camera** (key C) and click on the boat, then scroll to
zoom in.

**In Gazebo** the camera stands still; the boat (a grey box, since
`munin_v0` has no hull model) drives away from it.

**Stop terminal 1 with Ctrl-C after each run** and start it again before the
next. A new run restarts the simulation clock at zero, and the nodes keep
their last target and timers from the previous run.

**Other courses** (same two terminals, change the last word in terminal 2):

| Course | What it tests | Pass criterion |
|---|---|---|
| `hold_position` | Hold the start pose for 60 s (no step) | 1.0 m and 10° |
| `station_keeping` | Drive 10 m east, then hold for 60 s | 1.0 m and 10° |
| `goto_square` | Four corners of a 20 m square, each with a heading | 1.5 m and 15° for 5 s |
| `goto_retarget` | The target changes before it is reached | last target, 1.5 m and 15° for 5 s |

Add `ENVIRONMENT=windy` (4 m/s wind) or `ENVIRONMENT=current` (4 m/s wind and
0.3 m/s current) in front of the command for disturbances.

## 4. Send your own targets

In `lab` there is no race: you send targets yourself and every target is
scored until the next one arrives. Use three terminals.

**Terminal 1:** `scripts/ca-nodes.sh run`, as before.

**Terminal 2:**

```bash
STATE_SOURCE=truth CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml \
  GUI=1 ./scripts/njord lab hold_position
```

**Terminal 3:** send targets once the windows are up (the boat drifts until
the first target arrives):

```bash
cd ~/Njord/Surface_Vessel_Simulation
./scripts/njord goto 0 0 0       # hold the start: x = 0, y = 0, face east
./scripts/njord goto 5 0 0       # 5 m east, face east
./scripts/njord goto 5 5 90      # 5 m north of that, face north
```

The arguments are x and y in metres in the `map` frame (x east, y north) and
the heading in degrees counter-clockwise from east. In NED terms, `goto 5 0 0`
is north 0, east 5, ψ = 90°. Each `goto` replies
`Sent /njord/setpoint to N subscriber(s)` and terminal 2 prints
`… reached after … s` when the boat has been within 1.5 m and 15° for 5 s.

You can also click targets in RViz: choose **2D Goal Pose** (key G), click
where the boat should go and drag in the direction it should face.

Press Ctrl-C in terminal 2 to end; it writes a report with one section per
target. Then stop terminal 1.

## 5. After changing your code

| You changed | Do this |
|---|---|
| C++ or Python code, a message, `CMakeLists.txt` or `package.xml` | `scripts/ca-nodes.sh build`, then start terminal 1 again |
| `DPParams.yaml` or `NavigationParams.yaml` | Just start terminal 1 again; the run reads them from your repository |
| You pulled new commits in `Njord2627` | `scripts/ca-nodes.sh build` |
| The thruster layout | Your allocation and `njord_sim/config/vessels/munin_v0.yaml` must agree (below) |

`scripts/ca-nodes.sh shell` opens a shell in the same container, with your
workspace sourced, for `ros2 topic echo` and similar.

The run starts the same nodes and parameter files as
`navigation/launch/dpSimulation.launch.py`, but one by one with
`use_sim_time:=true`, because the launch file does not set it yet. Once it
accepts `use_sim_time`, the script can use the launch file instead.

## 6. The interface

### From the simulator to your nodes

| Topic | Type | Content | QoS and rate |
|---|---|---|---|
| `/njord/odometry` | `nav_msgs/Odometry` | Pose of `base_link` in `map` (ENU); `twist` in the body frame (FLU). `header.stamp` is simulation time. | About 30 Hz. Best effort or reliable both work. |
| `/njord/setpoint` | `geometry_msgs/PoseStamped` | Target position `pose.position.x/y` and heading `pose.orientation` in `map` | Reliable, transient local. Sent when a target is issued and repeated once a second with the **same** stamp. |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time | Every node needs `use_sim_time:=true` |

### From your nodes to the simulator

| Topic | Type | Content | Rules |
|---|---|---|---|
| `/thruster_1/command` … `/thruster_4/command` | `std_msgs/Float64` | Force in newtons along the thruster's direction; negative is reverse | Send all four every cycle (about 20 Hz). If one is older than 0.5 s of simulation time, all thrust is set to zero. Limit ±500 N per thruster. No namespace. |

`ROS_DOMAIN_ID` is 42. All topics, frames and rules are in
[ros2-interface.md](ros2-interface.md).

### Conversion to NED/FRD

| Quantity | From the simulator | In NED / FRD |
|---|---|---|
| Position | `position.x` (east), `position.y` (north) | north = `position.y`, east = `position.x` |
| Heading | yaw, counter-clockwise from east | ψ = π/2 − yaw, clockwise from north, wrapped to [−π, π] |
| Velocity | `twist.linear.x`, `twist.linear.y` (to port) | u = `linear.x`, v = −`linear.y` |
| Yaw rate | `twist.angular.z`, counter-clockwise | r = −`angular.z` |

The same conversion applies to the target. Examples:

| | Simulator (ENU) | NED |
|---|---|---|
| Start pose of every course | x = 0, y = 0, yaw = 0° (facing east) | N = 0, E = 0, ψ = 90° |
| `station_keeping` target | x = 10, y = 0, heading 0° | N = 0, E = 10, ψ = 90° |

`scripts/ned_dp_standin.py` in the simulator repository is a small, tested
Python example of this conversion.

### Thrusters on `munin_v0`

The layout is assumed (X) until Naval decides. The allocation in
`thrust_allocation_node.py` uses the same layout in FRD.

| Name | Where | Simulator (FLU): [x, y] m, direction | C&A (FRD): [x, y] m, direction | Positive force pushes |
|---|---|---|---|---|
| `thruster_1` | bow, port | [1.1, 0.55], −45° | [1.1, −0.55], +45° | forward and to starboard |
| `thruster_2` | bow, starboard | [1.1, −0.55], +45° | [1.1, 0.55], −45° | forward and to port |
| `thruster_3` | stern, port | [−1.1, 0.55], +45° | [−1.1, −0.55], −45° | forward and to port |
| `thruster_4` | stern, starboard | [−1.1, −0.55], −45° | [−1.1, 0.55], +45° | forward and to starboard |

The direction is the angle of positive thrust from straight ahead:
counter-clockwise in the simulator, clockwise in FRD. To try another layout,
change `thrusters:` in `njord_sim/config/vessels/munin_v0.yaml` (no rebuild)
and the allocation to match.

## 7. Results

Every run writes `outputs/run-<time>/` in the simulator repository:

- **`report.html`**: open it in a browser. The result, a top view of the
  track and the targets, a table per target, and plots of distance, heading
  error, speed and force per thruster over time.
- **`run_metrics.json`**: all numbers. Per target (`setpoints[]`):
  `time_to_reach_s`, `final_distance_m`, `final_heading_error_deg`, hold
  precision (`hold_rms_distance_m`, `hold_max_distance_m`,
  `hold_rms_heading_error_deg`), `overshoot_m` and `force_impulse_ns`
  (∫ Σ|F| dt, a measure of control effort).
- **`timeseries.csv`**: 10 Hz pose, velocity, target error and force per
  thruster, for your own plots.

All scoring uses the simulator's ground truth.
[running.md](running.md#8-results) lists every field and status.

## 8. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| The race never starts; terminal 2 prints `Waiting for …` | Your nodes are not connected: terminal 1 not running, crashed, or publishing under another name or namespace |
| The boat does not move | `scripts/ca-nodes.sh shell`, then `ros2 topic echo /njord/guard_status --field status`: it names the missing or stale command |
| `… stale in simulation time` | A node runs without `use_sim_time:=true`, or sends less than every 0.5 s |
| The boat goes the wrong way or spins | Conversion or thruster layout differs from section 6 |
| Two publishers on a thruster topic | `CONTROLLER=external` is missing, so the simulator's own controller also runs |
| `scripts/ca-nodes.sh build` fails on a missing package | The image lacks a dependency of your code; tell the simulation group |
| `No C&A sources at …/Njord2627/munin_ws/src` | `Njord2627` is not next to the simulator; clone it there, or set `CA_REPO=/path/to/Njord2627` |
| `Image … mismatched … digest` | The simulator code changed after the image was fetched: `./scripts/njord pull` |
| No windows | Run from a terminal on the desktop; `echo $DISPLAY` must print something |

## 9. Without windows

Replace `gui` with `demo` and leave out `GUI=1`. The run, the report and the
exit code are the same:

```bash
STATE_SOURCE=truth CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml \
  ./scripts/njord demo goto_square
```

## 10. What has been tested

On 2026-10-19 on the development PC (RTX 5090, WSL2), with C&A's `master` at
c1c4589, `munin_v0`, `STATE_SOURCE=truth`, seed 1, using exactly the commands
in this guide:

| Run | Result |
|---|---|
| `gui goto_square`, calm | Completed, exit 0: 4/4 targets in 19.1 / 44.7 / 46.5 / 44.5 s; final error 0.15 m and 0.02 m, heading error 0.0–7.8° |
| `demo goto_square`, calm | Same result as with windows |
| `demo station_keeping`, calm | Completed, exit 0: reached after 10 s, held 60 s; hold 0.14 m RMS, 1.0 m max, overshoot 0.28 m |
| `demo hold_position`, `ENVIRONMENT=current` | **Failed** (`setpoints_failed`): the boat settled about 1.5 m from the target, just outside the 1.0 m tolerance, until the 200 s timeout |

Findings for C&A:

- On `goto_square` in calm water the heading ended exactly 7.8° off at all
  three targets whose heading is not east (90°, 180° and −90°), and 0.0° off
  at the target facing east. Within the 15° tolerance, but the identical
  offset without any disturbance suggests a conversion or allocation detail
  rather than tuning. Not investigated further.
- Against current the position settles about 1.5 m off target. With the
  integral gains at zero in `DPParams.yaml` (`x_ki`, `y_ki`, `psi_ki`), a
  steady offset against a constant disturbance is expected.
- `dpSimulation.launch.py` does not set `use_sim_time`. With the simulator
  every node needs it.

Also tested the same day, without windows: `lab hold_position` with three `njord goto`
targets (section 4) reached all three (5.0, 10.9 and 30.5 s), Ctrl-C wrote
the report, and `scripts/ca-nodes.sh run` stops cleanly with exit code 0 on
Ctrl-C and on `docker stop`.

Not tested: a native Linux PC (only WSL2), nodes running outside Docker or on
a second machine, and estimated state (`STATE_SOURCE=estimate`) with C&A's
nodes. Earlier tests with a stand-in controller and with ROS 2 Humble are in
[validation.md](validation.md).
