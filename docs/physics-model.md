# Physics model

How the simulator computes the vessel's motion, where each term lives in the
code, what has been verified, and how the Njord model is meant to be
calibrated against the real boat.

> **The Njord-profile vessels are uncalibrated.** `njord_v1.yaml` and
> `munin_v0.yaml` use invented engineering values (a box hull, 180 kg,
> assumed damping and thrusters). Passing tests shows that the code computes
> what the equations say, not that the equations match Munin.

## 1. Two vessel profiles

| Profile | Vessel files | Who computes the physics |
|---|---|---|
| `wamv_reference` (default) | `vessels/wamv.yaml` | VRX's plugins ([section 8](#8-the-wam-v-profile)) |
| `njord` | `vessels/njord_v1.yaml`, `vessels/munin_v0.yaml` | Our `NjordPhysics` plugin together with gz-sim `Hydrodynamics` and the DART physics engine |

The rest of this document is about the Njord profile unless stated.

## 2. The equation of motion

The Njord profile is Fossen's 6-DOF vessel model. In each time step
(4 ms by default) DART solves

```
(M_RB + M_A) ν̇ + C_RB(ν) ν + C_A(ν_r) ν_r + D(ν_r) ν_r + g(η) = τ_thrust + τ_wind
```

where ν is the body-frame velocity and ν_r the velocity relative to the
water (ν minus current). None of our code computes acceleration directly: the
plugins compute **forces and moments** and apply them to the body; DART
integrates the motion. Inertia, coasting and overshoot therefore come from
the physics, never from a kinematic shortcut.

| Term | Computed by | Values written by |
|---|---|---|
| M_RB, mass and inertia | DART | `njord_model.py` (from the vessel file) |
| M_A, added mass | DART, via gz-physics `fluid_added_mass` | `njord_model.py` |
| C_RB, rigid-body Coriolis | DART | — |
| C_A, added-mass Coriolis | DART, corrected to relative velocity by gz-sim `Hydrodynamics` | — |
| D(ν_r), linear and quadratic damping with current | gz-sim `Hydrodynamics` | `njord_model.py` (coefficients from the vessel file) |
| g(η), buoyancy and restoring moments | `NjordPhysics.cc` with `Hydrostatics.hh` | — |
| Gravity m·g | gz-physics, applied at the centre of mass | `scenario.py` writes the world gravity from `constants.GRAVITY_MPS2` |
| τ_thrust | `NjordPhysics.cc` with `Loads.hh` | thrusters from the vessel file |
| τ_wind | `NjordPhysics.cc` with `Loads.hh` | wind table from the vessel file, wind from the scenario |

## 3. From YAML to running physics

```
vessel file + scenario + constants.py
        │  configuration.py: reads and validates everything
        ▼
njord_model.py  → vessel.sdf (link, inertia, plugins with parameters)
scenario.py     → njord_course.sdf (world, gravity, time step)
        │
        ▼
Gazebo, every time step:
  PreUpdate:  NjordPhysics  → buoyancy + thrust + wind
              Hydrodynamics → damping, current, Coriolis correction
  Update:     DART          → solves the equation of motion and integrates
  PostUpdate: sensors, odometry
```

**Where the values live.** Mass, centre of mass, inertia, geometry, damping,
added mass, wind table and thrusters: the vessel file. Wind, current, water
density, water level and time step: the scenario's `environments`. Gravity
(`GRAVITY_MPS2`) and the command timeout: `constants.py`. Air density
(1.225 kg/m³): `kAirDensity` in `Loads.hh`.

**Validation** (`configuration.py`). Everything is checked before any
process starts, so the plugin never receives invalid values:
`_validate_inertia`, `_validate_geometry`, `_validate_wind` and
`_validate_thrusters` check each section; `thruster_axis` turns `yaw_deg` into
a unit vector; `resolve_scenario` turns wind and current into ENU vectors;
`resolve_configuration` rejects waves and gusts for the Njord profile and a
hull that cannot carry its own weight.

**Model generation** (`njord_model.py`, `generate`). Writes `vessel.sdf`,
`vessel.urdf` and `bridges.yaml` into the run directory: mass, inertia and
added mass; the `NjordPhysics` parameters (buoyancy meshes, water, wind,
thrusters with position, axis, limits and response time); and the
`Hydrodynamics` parameters. Damping values are positive magnitudes in YAML
and negated here, because Gazebo uses hydrodynamic derivatives (e.g.
`xU = -40`). `disable_added_mass` stops the added mass being counted twice.

**World** (`scenario.py`). Writes the world gravity (Njord profile only), the
physics engine (`dart`) and time step, and leaves out VRX's wind and wave
plugins for the Njord profile so no force is counted twice.

## 4. The `NjordPhysics` plugin

`njord_gz_plugins/src/NjordPhysics.cc`. The file header lists every
parameter and the failure behaviour.

- **`Configure`**, once at start: reads gravity from the world and stops if
  it is missing or not pointing down; reads and validates the buoyancy meshes
  and rejects overlapping volumes; reads the thrusters and the wind table;
  subscribes to `/njord/actuator_forces`.
- **`Command`**: one force in newtons per thruster. A wrong number of values
  or a non-finite value invalidates the whole command.
- **`PreUpdate`**, every step:
  1. If simulation time goes backwards (reset), the thrusters are zeroed.
  2. **Buoyancy:** each mesh is moved to the world frame and clipped at the
     water surface. The force ρ·g·V_submerged acts straight up at the
     centroid of the submerged part; its moment about the centre of mass
     gives the restoring roll and pitch moments.
  3. **Thrusters:** a command is valid only while fresh in both simulation
     time (0.5 s) and wall time (2 s). The target is clipped to
     [−reverse limit, forward limit]; an invalid command targets zero. A
     first-order lag (`response_time_s`) shapes how fast the thruster builds
     force. Force along the axis and its moment about the centre of mass are
     added up.
  4. **Wind:** relative air velocity (wind minus the boat's velocity at the
     centre of mass), rotated into the body frame, gives X, Y and N from the
     wind table.
  5. The sum is applied at the centre of mass (`AddWorldWrench`).
  6. Telemetry for evaluation: `/njord/actuator_applied` (applied and target
     forces) and `/njord/actuator_wrench`.

## 5. Helper headers (testable without Gazebo)

**`include/njord/Hydrostatics.hh`**

- `Validate`: one mesh must be closed, convex and outward-facing. Several
  separate convex volumes (for example catamaran hulls) are separate meshes.
- `Box`: a box mesh.
- `Submerged`: volume and centroid of the submerged part. Each triangle is
  clipped against z ≤ water level (Sutherland–Hodgman), the cut edges close
  the waterplane, and volume and first moment are integrated with signed
  tetrahedra.
- `Response`: exact discretization of dF/dt = (target − F)/τ.

**`include/njord/Loads.hh`**

- `WindCoefficients`: periodic linear interpolation in the wind table. The
  angle is the direction the air moves **toward** in body axes (0° = forward,
  counter-clockwise). Literature often uses where the wind comes *from*, which
  is 180° different.
- `WindLoad`: q = ½·ρ_air·|v_xy|², X = q·A_x·c_x, Y = q·A_y·c_y,
  N = q·A_y·L·c_n.
- `ThrusterWrench`: force = axis·F, moment = (position − centre of mass) ×
  force.

## 6. Upstream: gz-sim `Hydrodynamics` and DART

- [gz-sim 8.15.0 `Hydrodynamics`](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/hydrodynamics/Hydrodynamics.cc):
  velocity relative to the current, rotated into the body frame; diagonal
  damping D = −(X_u + X_|u|u·|u|) per axis (with `munin_v0`'s values, surge
  force −(40 + 30·|u|)·u); a correction so the added-mass Coriolis term uses
  the velocity relative to the water. Its own added mass is disabled because
  DART has it.
- [gz-physics 7.8.0 DART](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/AddedMassFeatures.cc):
  added mass is put into DART's inertia tensor, DART's own gravity is turned
  off for the link, and gravity is applied as m·g at the centre of mass
  without the added mass
  ([`SimulationFeatures.cc`](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/SimulationFeatures.cc)).

The Docker build pins these versions (`docker/dependencies.lock.json`).

## 7. From controller to thrust

```
controller ──▶ /<thruster>/command (Float64, N) ──▶ command_guard_node.py
   ──▶ /njord/actuator_forces (Float32Array, N) ──▶ ros_gz_bridge ──▶ NjordPhysics::Command
```

The reference gate controller is in `guidance_node.py` (P on speed with
`kp_surge`, PD on heading with `kp_yaw` and `kd_yaw`, from
`algorithms.yaml`); `allocate_thrusters` in `control_core.py` distributes the
wrench over the thrusters. The setpoint controller is
`setpoint_control_core.py`. An external controller publishes directly on
`/<thruster>/command` ([ros2-interface.md](ros2-interface.md)).

## 8. The WAM-V profile

The WAM-V uses VRX's physics (VRX commit `03eae36`); nothing above applies to
it. Buoyancy and waves: `vrx::Surface`. Damping and added mass:
`vrx::SimpleHydrodynamics`. Wind with gusts: `vrx::USVWind`. Thrusters:
gz-sim `Thruster`, with symmetric limits set from `max_thrust_n`. Our code
for it is `vessel.py` and `ActuatorWatchdog.cc`, which gives it the same
command expiry as the Njord plugin. The VRX WAM-V has no current.
[VRX source](https://github.com/osrf/vrx/tree/03eae362bb544f630595acd53b931e4a7060dffd/vrx_gz/src).

## 9. What the Njord model supports, and its limits

Supported: separate visual, collision and buoyancy geometry (boxes or convex
triangular OBJ meshes, several disjoint buoyancy volumes); full mass, centre
of mass and inertia; clipped-volume hydrostatics; added mass and damping
relative to the current; periodic wind tables; any number (two or more) of
fixed thrusters at any angle with separate forward and reverse limits,
response lag and command expiry.

Limits:

- Flat water and constant wind only. Waves and gusts are rejected for the
  Njord profile.
- Convex OBJ only. Concave CAD must be split into disjoint convex buoyancy
  volumes. Changing geometry never rescales mass, inertia or damping.
- Thrust does not depend on boat speed (no propeller or motor model).
- The analytic fixture yaws unstably in long open-loop reverse runs; reverse
  is accepted only over 25 s.
- Numerical convergence is shown for straight, reverse, both turns, coast and
  hydrostatics at 150 N only ([validation.md](validation.md#njord-model-evidence)).
- Scoring uses a conservative rectangle around the hull, not a swept 3D
  hull.

## 10. Calibration against the real boat

Calibration has three levels of evidence, and only the last says anything
about the boat:

| Check | Establishes | Does not establish |
|---|---|---|
| Configuration, model and analytic force/moment tests | The code computes the stated equations | Gazebo execution or marine fidelity |
| Live simulator experiments at 4, 2 and 1 ms | Simulator response and numerical convergence for the inputs | Agreement with the boat |
| Pre-registered comparison with real-boat trials | Agreement within declared limits for the measured boat, load and environment | Anything outside the measured envelope |

### Isolated simulator experiments

`validation/check_dynamics.py` runs one open-loop experiment per fresh
simulator process, with only the simulator service running:

| `experiment` | What it does |
|---|---|
| `straight`, `reverse` | Equal thrust forward or backward; acceleration and top speed |
| `turn_left`, `turn_right` | 20 % thrust on one side, 100 % on the other; yaw rate and radius |
| `coast` | Accelerate to steady speed, cut thrust, measure the stopping distance |
| `drift` | Zero thrust in wind or current |
| `hydrostatic` | Initial roll and pitch of 0.05 rad; the excited angles must halve and settle |

Defaults are 300 N per thruster and a 60 s window. Every experiment starts
from rest in a fresh process (no reset or teleport) and requires a stationary
10 s window before and after; a response that does not settle is reported
incomplete, never relaxed. Commands are in [validation.md](validation.md#5-dynamics-measurements).

### Numerical acceptance

```bash
python3 scripts/dynamics_campaign.py --output outputs/dynamics-campaign --repetitions 3
python3 validation/numerical_acceptance.py outputs/comparison.json
```

Each experiment runs at 4, 2 and 1 ms with three matched repetitions, all
else fixed. Every 2 ms run must match its 1 ms run within
max(2 % of the value, 0.02 m / 0.01 m/s / 0.001 rad/s). Failures cannot
cancel in a mean. These are numerical criteria, not sea-trial accuracy.

### Measurements of the real boat

Before fitting, register a dataset manifest
(`validation/calibration/dataset_manifest.template.json`) with: raw files and
their SHA-256; instruments and calibration dates; hull, load, mass, centre of
gravity and inertia; units, frames and clock synchronization; water, wind and
current with uncertainty; commands and actual actuator observations; and a
train/holdout split by independent trials, with acceptance limits decided in
advance.

Trials to collect: mass, centre of gravity and inertia; draft, trim and heel
restoration; each thruster forward and reverse including transients; tow
resistance, acceleration and coasting; both turns, differential thrust and
zigzag; wind and current drift. Fit on the training trials only, then report
residuals on the untouched holdout trials. `validation/calibration_report.py`
checks a manifest and writes a report. Approval covers only the measured hull
revision, load and operating envelope; a change invalidates it.

Independent analytic checks of single effects are in
`validation/physical_acceptance.py` (thruster force and moment, onset
acceleration, actuator response) and `validation/hull_acceptance.py` (planar
momentum balance with drag, wind and added mass, and box flotation).

## 11. Extending the model

| Idea | Where |
|---|---|
| Thrust that falls with speed, or a motor model | The thruster loop in `NjordPhysics.cc` and a new function in `Loads.hh`; new parameters in the vessel file, `_validate_thrusters` and `njord_model.py` |
| Wind heel and pitch moments | `WindLoad` in `Loads.hh` (also K and M) and the wind step in `NjordPhysics.cc`; new coefficients in the wind table |
| Gusts for the Njord profile | `NjordPhysics.cc` (wind is constant today) and the rejection in `resolve_configuration` |
| Waves for the Njord profile | `Submerged` must clip against a wave surface instead of a plane; the rejection in `resolve_configuration` |
| Better damping or added mass | Values only, in `hydrodynamics:` of the vessel file. Needs measurements (section 10). |
| Water density, gravity | `water_density_kg_m3` in the scenario; `GRAVITY_MPS2` in `constants.py` (rebuild) |

Project rules that apply: thrusters are driven in newtons; never teleport or
set velocities directly; every value has one home; numbers in `njord_v1.yaml`
and `munin_v0.yaml` stay labelled uncalibrated until measured.
