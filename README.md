# njord-sim-poc

A D* Lite planner running closed loop against a simulated boat, first in pure
Python and then in VRX / Gazebo, with validation at every level so the results
mean something.

The point of this repo is not the planner. It is that every claim it makes is
checkable: the planner is checked against an independent optimal search, the
vessel model is checked against measurable manoeuvres, the lidar is checked
against world geometry, and the closed loop is scored on ground truth across
repeated runs.

```
njord_sim/           ROS 2 (ament_python) package
  njord_sim/
    dstar_lite.py    planner core, no ROS or Gazebo dependency
    mapper_node.py   lidar point cloud -> inflated occupancy grid
    planner_node.py  persistent D* Lite search, publishes nav_msgs/Path
    guidance_node.py line-of-sight following, differential thrust
    evaluator_node.py ground truth scoring, writes JSON
  launch/dstar_demo.launch.py
tests/               planner correctness against A*
sandbox/             closed loop with no ROS and no Gazebo
validation/          checks that run against a live VRX simulation
```

## The validation ladder

Run these in order. Each level is only meaningful if the level below it
passed. Skipping a level does not save time, it moves the debugging to a place
where the cause is much harder to see.

### 0. Planner correctness, no simulator needed

```
python3 tests/test_dstar_lite.py
```

Checks D* Lite against an independent A* on several hundred random grids, and
then the property that actually matters: after the robot moves and obstacles
appear and disappear, the incrementally repaired solution must equal a
from-scratch optimal replan. Two floating point bugs in the priority queue were
found by exactly this test. Both produced a `g` value that was slightly too
low rather than a crash, so the planner reported a cost it could not achieve
and path extraction failed several cycles later, far from the cause. Neither
would have been visible in a demo video.

Expected output:

```
  static grids: 200 cases, 146 solvable, all match A*
  incremental replanning: 655 repair steps, all match a full A* replan
  block + clear + moving start: 610 repair steps, all match a full A* replan
  obstacle removal: cost dropped 25.31 -> 10.00, matches A*
  walled-off goal: reported unreachable, no path returned
```

### 1. Closed loop, no simulator needed

```
python3 sandbox/headless_demo.py
```

A 3-DOF vessel, a simulated 2D lidar and the real planner, in the same loop
shape as the ROS nodes. The boat starts with an empty map, so every obstacle is
discovered en route and every detour is a genuine replan. Runs five seeds in
about ten seconds and writes `sandbox/headless_demo.png` and
`sandbox/headless_metrics.json`. For a version you can put in a slide:

```
python3 sandbox/make_gif.py
```

writes `sandbox/headless_demo.gif`, the same run animated. It shows what the
boat knows rather than what the world looks like: the occupancy grid grows as
the lidar sweeps, the plan snaps to a new route each time a buoy is found, and
the track lags the plan because the boat cannot turn on the spot.

This is the level that catches inflation radius, lookahead distance and control
gains, and it catches them in seconds instead of in minutes of Gazebo wall
clock. If the planner cannot get a boat through here, no amount of
hydrodynamic fidelity will save it.

### 2. Vessel dynamics in VRX

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
python3 validation/check_dynamics.py
```

Prints top speed, acceleration time, steady yaw rate, turning radius and
stopping distance from three open-loop manoeuvres. These are the same three
manoeuvres you can measure on the real boat in an afternoon, and until the two
sets of numbers agree, everything downstream is a result about a simulator
rather than about Njord's boat.

The turning radius this reports is also the lower bound for the costmap
inflation radius. A boat cannot stop.

### 3. Sensor against world geometry

```
python3 validation/check_lidar.py --ros-args -p target:="[-470.0, 210.0, 1.5]"
```

Holds position, compares the closest lidar return against the distance
computed from the world file, and reports the fraction of returns falling below
the wave-rejection height. A bias larger than one cell width means the
occupancy grid is offset from the world, which looks exactly like a planner
that avoids obstacles that are not there.

### 4. Closed loop in VRX

```
colcon build --packages-select njord_sim && source install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
ros2 launch njord_sim dstar_demo.launch.py
ros2 topic pub --once /njord/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -380.0, y: 250.0}}}"
```

Watch `/njord/path` in RViz next to the Gazebo view. The evaluator writes
`run_metrics.json` on completion.

### 5. Repeats

Run level 4 across several starting positions and obstacle layouts with
`--ros-args -p run_label:=...` and collect the JSON files. One successful run
is an anecdote. A table of clearance, path efficiency and replan latency over
ten runs is evidence, and it is also the regression test that stops next year's
team from breaking this without noticing.

## Before it will run

Every topic name is a launch argument, because they are the single thing most
likely to differ between VRX releases and thruster configurations. Check them
first:

```
ros2 topic list | grep -E "thrust|points|odom"
```

The defaults assume `/wamv/thrusters/{left,right}/thrust` (`std_msgs/Float64`),
a lidar on `/wamv/sensors/lidars/lidar_wamv_sensor/points`, and ground truth
odometry on `/wamv/ground_truth/odometry`. Override whichever differ.

## Deliberate shortcuts

These are fine for a proof of concept and wrong for the real system. They are
listed here so nobody inherits them by accident.

- **Ground truth odometry is used as the navigation solution.** There is no
  state estimator. Swapping in GPS and IMU fusion will degrade path following,
  and that degradation is itself worth measuring.
- **Points are transformed with odometry plus a static sensor offset, not
  through tf2.** Exact while the odometry is ground truth, wrong the moment it
  is not.
- **The map only grows.** Nothing is cleared except the boat's own footprint,
  so a moving target vessel leaves a permanent smear. Fine for static buoys,
  not for traffic.
- **The stock VRX WAM-V is used, not Njord's hull.** Replacing it is the first
  real task, and level 2 above is how you know when it is done.
- **Control gains are hand-tuned for the WAM-V** and will need retuning.
