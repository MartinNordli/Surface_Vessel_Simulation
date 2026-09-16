# Team integration: Control Systems and Perception/CV

A step-by-step guide for testing your own ROS 2 nodes against the simulator.

The simulator exposes ROS 2 Jazzy interfaces: the cameras provide RGB and
`CameraInfo`, the lidar provides `PointCloud2` and `LaserScan`, and GPS/IMU provide
raw and noise-processed measurements. Estimated odometry and timestamped TF are
available. This tests algorithms against simulated sensors, not the physical
sensors' firmware, drivers or full optical properties. Topic names, types and
frames are listed in [interfaces.md](interfaces.md).

## 1. Build and run a reference

From the repository root, after the host setup in [running.md](running.md):

```bash
./scripts/njord build simulator
./scripts/njord test
SEED=1 ENVIRONMENT=calm PROFILE=fast ./scripts/njord demo slalom
```

Keep `outputs/run-*/run_metrics.json`, including for failed runs. `status`,
`gates_passed`, `collision`, `contact_status` and `time_s` describe the outcome.
A sensor change can make the reference algorithm fail the course; that is a test
result.

## 2. Choose which algorithms to replace

| Setting | Reference nodes left out | What the team provides |
|---|---|---|
| None | None | The full reference chain runs |
| `CONTROLLER=external` | `guidance` | Left/right thrust from your own controller |
| `PERCEPTION=external` | `perception` | Buoy detections from your own CV/fusion system |
| `MAPPING=external` | `mapper` | Your own occupancy map |
| `AUTONOMY=external` | All five: mapper, perception, mission, planner, guidance | The whole algorithm chain and its status messages |

The three single settings can be combined. The GPS/IMU adapter, localization and
command guard always run. `AUTONOMY=external` overrides the single settings.
These settings only leave out reference nodes; they do not start your packages.

## 3. Connect your own ROS 2 workspace

Use the same `ROS_DOMAIN_ID` (default 42), message definitions and Jazzy version.
All algorithm nodes must use `use_sim_time:=true`. Raw sensors use best-effort
QoS; control and published algorithm results use reliable QoS.

On a Linux/WSL host with ROS 2 Jazzy, start the nodes directly:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/team_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run <package> <node> --ros-args -p use_sim_time:=true
```

Packages from other ROS distributions must be built for Jazzy. Alternatively,
build and run inside the simulator image with the bundled workspace overlay:

```bash
export TEAM_WS_HOST=/absolute/path/to/team_ws  # workspace containing src/
export NJORD_UID=$(id -u) NJORD_GID=$(id -g)
export ROS_DOMAIN_ID=42
# Separate build/install/log directories avoid mixing with host build files.
docker compose -f compose.yaml -f compose.team.yaml run --rm team \
  colcon --log-base log-sim build --build-base build-sim --install-base install-sim
# Open a team shell; the image entrypoint has already sourced Jazzy and the simulator.
docker compose -f compose.yaml -f compose.team.yaml run --rm team bash
source /team_ws/install-sim/setup.bash
ros2 run <package> <node> --ros-args -p use_sim_time:=true
```

The team image must have the package's own dependencies installed; build a
derived image if needed. The overlay shares DDS through the host network and
mounts the workspace, but not the run's `/outputs`. The image contains the
example courses; algorithms must use the sensors and not read scenario ground
truth. For CV that needs a GPU, add a suitable GPU setup to the team service; the
base configuration does not give the team container a GPU.

## 4. Test your own controller

1. Start the team controller as in step 3. It must wait for valid, fresh inputs.
2. Start a new race in another terminal from the simulator repository:

   ```bash
   CONTROLLER=external SEED=1 ./scripts/njord demo slalom
   ```

3. Subscribe to `/njord/path`, `/njord/occupancy` and `/njord/odometry`. Publish
   `std_msgs/msg/Float64` to both `/njord/thrusters/{left,right}/thrust` in
   **newtons**, normally at 20 Hz. There is no `cmd_vel` input; a controller that
   outputs velocity setpoints needs its own layer that converts them to physical
   thrust.
4. Check that each thrust topic has exactly one publisher:

   ```bash
   docker compose exec simulator /entrypoint.sh ros2 topic info /njord/thrusters/left/thrust --verbose
   docker compose exec simulator /entrypoint.sh ros2 topic echo /njord/race_active --once
   ```

5. The controller must zero thrust itself on an empty, invalid or stale path, a
   stale map or stale odometry. The guard requires fresh planner, mission and
   navigation status plus the evaluator's race-active signal. Missing commands for
   0.5 s of wall-clock time remove thrust; the boat keeps its momentum and can drift.

A concrete connection test without your own package is to run the reference
controller as a separate process while `CONTROLLER=external` is set:

```bash
docker compose run --rm autonomy ros2 run njord_sim guidance --ros-args \
  -p use_sim_time:=true -p max_speed:=1.0
```

This command takes the place of the team controller in the test. Stop the process
after the race before starting another controller. It demonstrates the ROS
connection; the team's actual controller must be tested separately.

## 5. Test your own computer vision or map

To inspect sensors without a race ending while you develop:

```bash
AUTONOMY=external ./scripts/njord lab slalom
# Another terminal:
docker compose exec simulator /entrypoint.sh ros2 topic list
docker compose exec simulator /entrypoint.sh ros2 topic hz \
  /wamv/sensors/cameras/front_left_camera_sensor/image_raw
docker compose exec simulator /entrypoint.sh ros2 topic echo \
  /wamv/sensors/cameras/front_left_camera_sensor/camera_info --once
docker compose exec simulator /entrypoint.sh ros2 topic hz /wamv/sensors/lidars/lidar_wamv_sensor/points
docker compose exec simulator /entrypoint.sh ros2 run tf2_ros tf2_echo map wamv/front_left_camera_link_optical
```

`lab` starts the simulator, estimation and the selected reference nodes, without
an evaluator. Stop earlier races first (`docker compose down` with your Compose
configuration); use only one simulator/evaluator per ROS domain and Gazebo
partition. `lab` does not stop an evaluator that is already running. The guard
keeps thrust at zero without a race-active signal from an evaluator on the same
domain. Exit with Ctrl-C before starting a new race. This is a passive sensor
test; wind and waves can still move the boat. Measured Hz values are wall-clock
receive rates and depend on the simulator's real-time factor.

To feed your own CV results into the reference mission, planner and controller:

```bash
PERCEPTION=external SEED=1 ./scripts/njord demo slalom
```

Start the CV node before the race. Publish reliable
`vision_msgs/msg/Detection3DArray` on `/njord/buoys`, in `map`, with the original
observation time:

- Use stable, unique `detection.id` values and put the position in
  `bbox.center.position`.
- Set `results[].hypothesis.class_id` to `red` or `green` with a score of at
  least 0.35.
- The reference mission requires a processed camera frame at most 0.5 s old.
  Publish an empty detection list when a **new processed frame** contains no
  buoys; never re-stamp old detections.
- Image-space detections (`Detection2DArray`) need depth/range and a transform to
  `map` before this mission node can use them.

A runnable replacement test is the same separate command as for the controller,
but with `ros2 run njord_sim perception --ros-args -p use_sim_time:=true`.

For your own map, use `MAPPING=external` and publish `nav_msgs/msg/OccupancyGrid`
on `/njord/occupancy` in the `map` frame, with valid geometry and cells -1
unknown, 0 observed free and 100 occupied/inflated. Keep unknown as unknown.

## 6. Change sensors and algorithm parameters without rebuilding

With the default mount, `njord_sim/config/` on the host is available as `/config`
inside the containers. A bundled example reduces the cameras to 320×180 at 10 Hz,
the lidar to 360×8 at 5 Hz, GPS to 5 Hz and IMU to 50 Hz:

```bash
VESSEL_CONFIG=/config/sensors_low_bandwidth.yaml SEED=1 ./scripts/njord lab slalom
# Or a complete race:
VESSEL_CONFIG=/config/sensors_low_bandwidth.yaml SEED=1 ./scripts/njord demo slalom
```

Team files can live in a separate directory:

```bash
mkdir -p outputs/team-config
cp njord_sim/config/sensors_low_bandwidth.yaml outputs/team-config/sensors.yaml
cp njord_sim/config/team_example.yaml outputs/team-config/algorithms.yaml
# Edit the files, then start a new race:
CONFIG_HOST="$PWD/outputs/team-config" \
VESSEL_CONFIG=/config/sensors.yaml ROS_PARAMS_FILE=/config/algorithms.yaml \
SEED=1 ENVIRONMENT=calm PROFILE=fast ./scripts/njord demo slalom
```

**Sensor file.** A partial override of `vessel.yaml`. It supports resolution,
a shared camera rate, horizontal field of view in radians, camera/lidar noise,
lidar range and ray count, GPS/IMU rate and GPS/IMU orientation noise. Unknown
field names and invalid numbers are rejected. Settings apply to both cameras.
Sensor placement and rotation are changed in `sensors.xacro` and require
`./scripts/njord build simulator`, so URDF/TF and the physical model are updated
together. New sensor types need a model, bridge and any adapters; adding a name
to YAML does not add an arbitrary sensor.

**Algorithm file.** Standard ROS 2 format with node names and `ros__parameters`;
see `team_example.yaml`. It overrides default parameters and `PROFILE` for the
nodes started by the simulator's autonomy launch. `use_sim_time` always stays
enabled. External nodes get their parameters through their own launch files or
`--params-file /config/algorithms.yaml`. `max_thrust_n` sets the force limit;
`thruster_separation_m` changes the controller's mixing, not the placement of the
VRX thrusters.

**Provenance.** The simulator saves the resolved sensor configuration in
`outputs/run-*/vessel_config.yaml`; the sensor adapter and guard use this same
copy. Algorithm parameters are copied to `ros_params.yaml` when selected, and
`autonomy_config.json` stores selections, arguments and SHA256 fingerprints.
External packages must additionally archive their own commit, parameters and
dependencies.

## 7. Record and compare trials

```bash
# Native Linux; on WSL add :compose.wsl.yaml to COMPOSE_FILE.
COMPOSE_FILE=compose.yaml:compose.record.yaml \
VESSEL_CONFIG=/config/sensors_low_bandwidth.yaml \
SEED=1 ./scripts/njord demo slalom recorder
# Sensor recording without a race, until you press Ctrl-C:
COMPOSE_FILE=compose.yaml:compose.record.yaml \
AUTONOMY=external ./scripts/njord lab slalom recorder
```

The MCAP bag is stored in the new run directory. Inspect it and replay sensors with
the simulator stopped, preferably in a separate ROS domain:

```bash
export OUTPUT_HOST="$PWD/outputs/run-REPLACE-WITH-ACTUAL-DIRECTORY"
ROS_DOMAIN_ID=43 docker compose run --rm autonomy ros2 bag info /outputs/bag
ROS_DOMAIN_ID=43 docker compose run --rm autonomy ros2 bag play /outputs/bag \
  --clock --topics /tf /tf_static /njord/odometry \
  /wamv/sensors/cameras/front_left_camera_sensor/image_raw \
  /wamv/sensors/cameras/front_left_camera_sensor/camera_info \
  /wamv/sensors/cameras/front_right_camera_sensor/image_raw \
  /wamv/sensors/cameras/front_right_camera_sensor/camera_info \
  /wamv/sensors/lidars/lidar_wamv_sensor/points
unset OUTPUT_HOST
```

Start the CV node on domain 43 with `use_sim_time:=true`. During replay only the
player may publish `/clock`. The topic list selects sensors, TF and odometry for
offline CV and avoids duplicating recorded CV results. Replay is not physically
affected by new control commands; comparing controllers requires new simulated
races.

Compare configurations with the same course, seed, environment and profile, and
separate output directories. The reference chain can run as a repeatable matrix:

```bash
VESSEL_CONFIG=/config/vessel.yaml ./scripts/njord benchmark slalom \
  --seeds 1 2 3 --environments calm
VESSEL_CONFIG=/config/sensors_low_bandwidth.yaml ./scripts/njord benchmark slalom \
  --seeds 1 2 3 --environments calm
```

Benchmark picks its own ROS domain per job, so external nodes do not connect to
these runs automatically. Use single races as above, or integrate team startup
into each benchmark job before parallel external testing. Keep all failures and
timeouts when comparing results. CPU/ROS tests do not prove GPU rendering or
collision-free completion for a new configuration.
