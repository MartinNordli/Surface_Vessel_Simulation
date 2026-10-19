# Open questions for Control & Autonomy and Naval

(Archived; translated from Norwegian. Links point to the current
documentation.)

The simulator should run Control & Autonomy's (C&A) code unchanged and behave
like Munin before the boat is built. The answers below decide the interface
and the model. Enter the answer and the date under each question once it is
settled, and link to the change in the simulator if the answer required one.

Things that are not decided yet are fine: write "not decided" and what it is
waiting for.

## Already settled (answers from C&A, September 2026)

- Munin gets **four fixed thrusters**, not azimuth. The thrusters are driven
  by PWM; conversion from force to PWM is done later by C&A and Electrical, so
  the simulator takes force in newtons.
- The allocation node publishes `std_msgs/Float64` on `thruster_1/command` …
  `thruster_4/command`. The simulator now listens on `/thruster_1/command` …
  `/thruster_4/command` (see [ros2-interface.md](../ros2-interface.md)).
- A node will receive GNSS and attitude from the sensors; it is not written
  yet.
- No mass, inertia or hydrodynamics data before the boat is finished
  (February). No rosbags from Munin yet.
- Disturbances: waves and current are probably small in competition, but it
  is desirable to be able to test against them.
- Both ground truth from the simulation and simulated GNSS/IMU for their own
  estimator are relevant.
- Metrics start from the Autodrone rules: path-following error, time and
  collisions.
- Many runs faster than real time are relevant for parameter search and
  possibly RL.
- Most important this semester: normal transit with path planning through
  obstacles, and dynamic positioning (DP).

## Thrusters and actuation (Naval Lead and C&A)

Until these are answered the simulator uses the placeholder
[`munin_v0.yaml`](../../njord_sim/config/vessels/munin_v0.yaml) with an
assumed X layout.

1. **Placement:** Where are the four thrusters, as a position [x, y, z] in
   metres from an agreed origin, and in which direction does each push? Is it
   an X layout (angled thrusters), two aft plus two tunnel thrusters, or
   something else?
   - Answer:
2. **Origin:** Which point is the boat's reference point in C&A's
   mathematics: the centre of gravity, midships at the waterline, or
   something else?
   - Answer:
3. **Numbering:** Which physical thruster is `thruster_1`, `thruster_2`,
   `thruster_3` and `thruster_4`?
   - Answer:
4. **Sign and limits:** Does a positive value on `thruster_i/command` mean
   thrust in the thruster's direction, in newtons? What is the approximate
   maximum force forward and backward per thruster?
   - Answer:
5. **Namespace:** Does the allocation node run in a ROS namespace? The topic
   `thruster_1/command` is relative and becomes `/thruster_1/command` only
   without a namespace.
   - Answer (simulation group, 2026-10-19): the node on C&A's `master` runs
     without a namespace and its topics reach the simulator.
6. **Rate:** Are all four commands published in every control cycle, at the
   same rate? The simulator only passes force while all four are fresh.
   - Answer:

## Sensors and state

7. **Message types:** Which message types does the GNSS and attitude node on
   the boat get? `sensor_msgs/NavSatFix` and `sensor_msgs/Imu` (which the
   simulator sends today), or the driver's own types, e.g. heading or
   `geometry_msgs/TwistStamped` for velocity?
   - Answer (C&A, 2026-10-12): Neither. The boat gets position and heading in
     NED and velocity in BODY from a Kongsberg Seapath. A C&A node publishes
     this as `/vessel/state_est` (own type `VesselStateEst`: `header`, `u`,
     `v`, `r`, `x`, `y`, `psi`, `b`), possibly with an EKF on top. The GNSS
     and IMU topics are not used. In simulation the node subscribes to
     `/njord/odometry` and converts by itself
     ([control-autonomy.md](../control-autonomy.md)).
8. **Convention:** Do you compute in ENU and FLU (x forward, y port, z up;
   the ROS standard and what the simulator uses) or NED and FRD? Should the
   simulator offer an adapter, or do you convert yourselves?
   - Answer (C&A, 2026-10-12): NED and FRD. C&A converts in a simulator mode
     in its own nodes, so the simulator does not need their message packages.
9. **Hardware:** Which GNSS (with or without RTK) and which IMU are planned?
   Datasheets give realistic values for noise, bias and drift in the
   simulation.
   - Answer (C&A, 2026-10-12): Kongsberg Seapath. Its software and model are
     proprietary (probably an ESKF with unknown parameters), so it cannot be
     modelled. A simplified estimate is good enough for now.
10. **Truth or estimate:** Will you normally run on ground truth
    (`STATE_SOURCE=truth`), on the simulator's estimate, or on your own
    estimator from raw GNSS/IMU?
    - Answer (C&A, 2026-10-12): Not your own estimator from raw GNSS/IMU.
      Truth or the simulator's estimate, chosen with `STATE_SOURCE`. The
      simulation group recommends `STATE_SOURCE=truth` for DP: the estimate
      has very noisy velocity ([validation.md](../validation.md)).

## Control and timing

11. **Rate:** Which rate do you plan for the control loop, even roughly (10,
    20 or 50 Hz)?
    - Answer:
12. **Simulation time:** Do the nodes use `use_sim_time`? It is required for
    stamps and timers to match the simulation, and to run faster than real
    time.
    - Answer (simulation group, 2026-10-19): `dpSimulation.launch.py` on
      `master` does not set it; `scripts/ca-nodes.sh` passes it to each node.

## DP and evaluation

13. **Setpoint:** How will you receive the DP setpoint: as
    `geometry_msgs/PoseStamped` on `/njord/goal` (position and heading in
    `map`), or some other way?
    - Answer (C&A, 2026-10-12): The DP controller takes its own `DPSetpoint`
      (`header`, `x`, `y`, `psi` in NED). In simulation the DP node converts
      `geometry_msgs/PoseStamped` from `/njord/setpoint`; velocity, tolerance
      and ID are not needed in the message. The controller expects a smooth
      reference from an MPC node under development, but can hold a position
      for 60 s alone. LOS guidance is also under development and will want a
      path of waypoints.
14. **Tolerances:** Which requirements apply to DP (for example ±1 m and
    ±5°), and how long should the position be held?
    - Answer (C&A, 2026-10-12): Not decided; not important for now.
15. **Metrics:** Which Autodrone metrics are relevant: path-following error
    against which reference (planned path or the line between gates), time,
    collisions, energy use?
    - Answer:

## Later

16. **PWM to force:** When can Electrical measure the relation between PWM
    and force per thruster? It is needed if the simulator is to accept PWM
    directly.
    - Answer:
