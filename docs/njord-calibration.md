# Njord calibration and numerical evidence

The analytic hull and provisional coefficients are **uncalibrated**. Passing
mathematics, model generation, or numerical convergence checks establishes only
that level of evidence. Njord realism requires independent measurements of the
actual hull revision and loading condition in the stated operating envelope.

## Isolated simulator experiments

Run `validation/check_dynamics.py` with only the simulator service running and no
autonomy publisher. Every experiment, time step and repetition requires a newly
started simulator process, configured with the same pose and zero initial
velocity. Do not reset or teleport a moving vessel. Start the validation node
before the first odometry timestamp exceeds 5 simulation seconds (the
`fresh_start_limit_s` parameter); launch together or pause initial startup when
necessary. The tool rejects a late connection. This timestamp guard cannot prove
process identity: retain the launch log/container identity in the run manifest.

Select one `experiment` parameter: `straight`, `reverse`, `turn_left`,
`turn_right`, `coast`, `drift`, or `hydrostatic`. Defaults are 300 N per thruster and a 60 s
measurement interval. Choose thrust within the resolved physical limits and
record it; the script does not infer limits from an arbitrary manifest schema.
A turn commands 20% thrust on one side and 100% on the other. A reverse commands
negative equal thrust. Raw samples include speed magnitude and signed body surge;
straight/reverse completion additionally requires the corresponding surge sign.
The initial two-second surge change is reported as observed acceleration.

The initial zero-thrust phase needs a contiguous 10 s window with speed variation
at most 0.01 m/s and yaw-rate variation at most 0.001 rad/s. Non-drift tests also
require speed at most 0.01 m/s and absolute yaw rate at most 0.001 rad/s. Coast
first accelerates with equal thrust until the same variation limits and yaw
limit hold at speed above 0.05 m/s, then cuts thrust. This avoids measuring
coasting immediately after a turn. Under appreciable wind/current, a stationary
initial state may be impossible: use `drift` for that environment; report other
experiments incomplete rather than silently relaxing the criterion.

Steady response requires the full contiguous trailing window; timestamp gaps
above 0.5 s invalidate a measurement. A stop additionally requires speed at most
0.05 m/s and absolute yaw rate at most 0.001 rad/s throughout that window. An
unobserved stop reports `coast_distance_m: null` and observed distance as a lower
bound. The measured stop distance includes the final confirmation window;
compare runs using the same window and speed threshold. A response that does not
stabilize within the observation period is incomplete. Preparation times out
after 60 simulation seconds per phase, and odometry/wall watchdogs use monotonic
wall time. Thrust is invalidated at completion or failure; inertia remains.

Pass `manifest_path` for the resolved run manifest and `output_path` for a fresh
JSON artifact below `outputs/`. Existing result files are never overwritten.
The report stores raw timestamped samples, parameters, manifest content and its
SHA-256. A report without a manifest is an exploratory observation and must not
be used for reproducible acceptance. The legacy `dynamics_metrics.summarize`
function remains for historical report compatibility; its sequential manoeuvre
numbers do not meet this protocol.

## Numerical acceptance

The campaign runner automates fresh isolated Compose projects and saves all logs:

```bash
python3 scripts/dynamics_campaign.py --output outputs/dynamics-campaign --repetitions 3
```

Build the updated image first. Select an unused `--ros-domain` (default 137);
existing simulators on that same ROS domain would contaminate observations. The
runner invokes `scripts/njord test` and `scripts/njord simulator`, starts the
subscriber first, tears down its own simulator after each measurement, and
`--fail-fast` stops after the first incomplete trial and retains its evidence.
The runner compares the resolved configurations with only timestep and its source checksum
removed. It retains incomplete trials and fails acceptance if any are incomplete.
Run a separate `--experiments drift --environment ...` campaign for wind/current.
The complete default campaign contains 54 trials and has not been executed as
part of the pure-Python tests.

For every selected central experiment, run **4, 2 and 1 ms** with at least **three
matched repetitions**, keeping seed, model, scenario, load and commands fixed
except time step. Retain all raw results, including failures. Use straight,
reverse, both turns and coast in calm water, and separate wind/current drift
cases. The hydrostatic experiment uses initial roll and pitch of 0.05 rad with zero
thrust, and requires each excited angle to halve before reaching a stationary
trailing window. Its reported mean z is the odometry origin elevation; physical
draft must be derived from the waterline and hull geometry. Initial and terminal
pose must be expressed in an absolute world frame for restoration evidence.
Samples include z, roll, pitch, surge, heave and all angular rates; stationarity
also bounds z variation to 0.01 m, roll/pitch variation to 0.001 rad, heave to
0.01 m/s and roll/pitch drift to 0.001 rad/s. Drift is a least-squares
slope of world z or unwrapped Euler angles over the complete trailing window,
not the maximum body-frame instantaneous rate. Forward motion at fixed pitch
has nonzero body heave without world vertical drift; small bounded numerical
vibration can have high instantaneous angular rates without a changing
equilibrium. The original peak-to-peak pose limits still reject excessive
oscillation and secular movement. Raw body rates remain in the report. An unexcited hydrostatic trial is
incomplete. Sensor/contact and command-watchdog integration need separate checks.

`python3 validation/numerical_acceptance.py outputs/comparison.json` consumes:

```json
{
  "metrics": {"steady_speed_mps": "m/s", "steady_yaw_rate_radps": "rad/s"},
  "runs": [
    {
      "experiment": "straight",
      "comparison_group": "sha256-of-identical-config-excluding-timestep",
      "provenance": "sha256-of-this-run-manifest",
      "step_s": 0.004,
      "repetition": 0,
      "complete": true,
      "metrics": {"steady_speed_mps": 1.2, "steady_yaw_rate_radps": 0.0}
    }
  ]
}
```

The single-row example is intentionally incomplete and fails acceptance. Build
separate comparison inputs for experiments with different measurable quantities
(e.g. `coast_distance_m` in `m`). The caller derives `step_s` from the actual
resolved manifest and checks that each `comparison_group` differs only in time
step. The comparison utility validates presence, finite values, unique matched
repetitions, complete status and supported time steps; it cannot independently
authenticate the caller's manifest references.

Every matched 2/1 ms pair must differ by at most
`max(0.02 * abs(value_at_1ms), absolute_tolerance)`, where the absolute tolerance
is 0.02 m, 0.01 m/s, or 0.001 rad/s. Individual failures cannot cancel in a mean.
4 ms runs are mandatory evidence for the convergence trend; the specified
acceptance threshold applies to 2 versus 1 ms. Exit status 2 means incomplete or
failed numerical acceptance. These thresholds are numerical criteria, not sea
trial accuracy targets.

## Measurement datasets and holdout protocol

Before fitting, register an immutable dataset manifest containing:

- Dataset ID, UTC collection time, raw-file SHA-256 hashes and acquisition software
  revision; instrument IDs, calibration dates, sample rates and missing-data flags.
- Hull/CAD and collision/buoyancy geometry revisions, vessel configuration hash,
  loading inventory, total mass, center of gravity, inertia reference point/axes,
  battery condition, thruster/propeller revisions and measured actuator limits.
- SI units per channel; ENU world and forward-left-up body axes; sensor optical
  axes and timestamped transforms; clock source, synchronization offset/drift,
  and uncertainty for both timing and values (including systematic errors).
- Water density/level, wind and current speed and direction **toward** in ENU,
  environmental uncertainty, waves, water depth, trial area and test operator.
- Maneuver commands and actual actuator observations, initial conditions,
  stationary/stop definitions, excluded samples with reasons, and valid speed,
  loading and environmental envelope.
- Predetermined train/validation split by independent trial, acceptance tolerances
  derived from measurement uncertainty and intended use, and fit-method revision.

Collect geometry/mass/load/CG/inertia measurements; draft/trim and small heel
restoration; each thruster in forward/reverse including transients; tow resistance,
acceleration and coasting; both turns, differential thrust and zigzag; wind and
current drift with measured environment. Avoid fitting and validating on adjacent
samples from the same maneuver: reserve independent trials before fitting.

Record fitted parameter values and uncertainties, optimization choices, training
residuals and untouched validation residuals. Approve only the measured hull
revision, loading condition and operating envelope. A hull/load change invalidates
that approval until reassessed. Njord wave response is outside the initial
flat-water scope. Missing hull data, sea trials or runtime checks must remain
explicitly incomplete; do not fabricate placeholder measurements.
