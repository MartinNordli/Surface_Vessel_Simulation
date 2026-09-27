# Parameter evidence and calibration status

The Njord fixture remains **uncalibrated**. Its numerical values are engineering
inputs, not measured boat geometry or hydrodynamic coefficients. A passing test
of parameter propagation does not establish agreement with a physical vessel.

Evidence has three distinct scopes:

| Check | Establishes | Does not establish |
| --- | --- | --- |
| Configuration/model tests and analytic force/moment oracle | Input validation and mathematical correctness under stated assumptions | Gazebo execution or marine fidelity |
| Live dynamics repetitions at 4, 2 and 1 ms | Simulator response and numerical convergence for the recorded inputs | Boat calibration |
| Preregistered real-boat holdout comparisons | Agreement within declared limits for the measured boat, load and environment | Accuracy outside the measured operating envelope |

`validation/physical_acceptance.py` independently computes planar thruster force,
its three-dimensional moment about the declared center of mass, diagonal onset
acceleration under explicit rest/no-environment/no-restoring assumptions, and
first-order actuator response. The acceptance limit is the greater of 2% of the
reference magnitude and the declared SI absolute floor. Coupled inertia, offset
COM and nonzero restoring/environment loads require their own treatment; the
diagonal onset oracle must not be applied to those cases silently.

`validation/numerical_acceptance.py` requires three matched repetitions at every
step, rejects missing/nonfinite metrics and incomplete reports, and compares
each 2 ms run to its matched 1 ms run. All repetitions must pass; opposite errors
cannot cancel in an average. This is convergence evidence only.

`validation/check_dynamics.py` records raw odometry and timestamped applied-force
telemetry. Its initial acceleration metric is an observed first-two-second
average, not instantaneous acceleration. Explicit `excitation` force vectors
permit individual thruster and signed axis experiments. A complete stationary
excitation report alone does not assert the correct direction or expected
acceleration; campaign acceptance must apply those independent checks. Turn-left
and turn-right experiments require positive and negative ENU yaw respectively.
Missing telemetry for excitation and startup exceptions produce incomplete
reports; existing output artifacts are never replaced.

The live sensor check requires ten overlapping simulation seconds of advancing
camera, lidar and navigation stamps with gaps and acquisition age no greater
than 0.5 s, plus transforms at acquisition time. Image variation is spatial per
color channel, so a constant red image fails. Navigation validates all pose and
twist components, both frames and quaternion normalization. These are stream
and TF checks, not a measurement of estimator accuracy. Truth-mode navigation
and evaluator results must never be labeled estimator validation.

The calibration manifest template in
[`validation/calibration/dataset_manifest.template.json`](../validation/calibration/dataset_manifest.template.json)
intentionally has no measurements. It records boat/load revisions, units,
frames, clock synchronization uncertainty, environment, channel uncertainty,
checksummed real-boat files, disjoint fit/holdout run IDs, and a sealed
preregistration with acceptance limits. `validation/calibration_report.py`
rejects the empty template, missing files, mismatched checksums, incomplete
holdout channels and undeclared tolerances, writing a report even for invalid
input. It is a comparison scaffold: collection, independent uncertainty
assessment and preregistration audit still require actual boat data.

Execution evidence for this change is recorded in the final review and ignored
`outputs/` logs. Synthetic unit and ROS transport tests use fabricated messages;
they are not live Gazebo, GPU rendering or sea trials. No real-boat measurements
have been supplied or invented.

`validation/hull_acceptance.py` adds a controlled planar oracle. It integrates
measured actuator force/moment, configured drag relative to current, and wind
load over half-second intervals, then compares the predicted mean acceleration
to the measured velocity change using effective rigid plus added mass. It
requires ten seconds of bracketed acquisition-time telemetry, a zero COM,
diagonal inertia, upright pose and negligible off-axis motion. Unsupported
coupled trajectories fail explicitly. The hydrostatic check compares settled
height against the independently calculated displacement of a single box.
These restricted checks make mass, inertia, damping, added mass, density and
environment effects reviewable; they do not validate arbitrary six-DOF models.

Executed host evidence during this change: the full unittest suite passed 201
tests with one skipped test before the last telemetry-validation hardening.
This included actual local ROS/DDS transport with synthetic messages. The
calibration template was executed and correctly produced an incomplete report.
Live Gazebo and subsequent rebuilt-container results must be reviewed separately.

## Reviewed execution records

The following artifacts were independently read after the platform agent ran
the commands. Commands used the `scripts/njord` wrapper; no source tests are
being substituted for simulator evidence.

| Command / artifact | Observed result | Scope |
| --- | --- | --- |
| `python3 -m unittest discover -s tests -p 'test_*.py'`; `outputs/parameter-validation/host-tests.log` | 201 tests passed, one skipped at that revision | Host tests, including synthetic local ROS/DDS transport |
| `./scripts/njord test`; `outputs/platform-container-tests.log` | 210 tests passed, no skips | Rebuilt container suite, model generation and synthetic ROS transport |
| `NJORD_CPU=1 ./scripts/njord selftest`; `outputs/platform-cpu-selftest.log` | Earlier run failed: camera samples rejected, lidar/navigation lacked ten seconds of continuous coverage | Preserved failed live run; sensor publication alone did not pass |
| `NJORD_CPU=1 ./scripts/njord selftest`; `outputs/platform-cpu-selftest-v2/sensor_validation.json` and corresponding `.log` | Passed: lidar/navigation windows `[0.1,10.2]`, left/right cameras `[0.132,10.168]`; common coverage **10.036 simulation seconds** | Actual Gazebo with software rendering, advancing sensor stamps and acquisition-time TF |
| `python3 validation/calibration_report.py validation/calibration/dataset_manifest.template.json --output outputs/parameter-validation/calibration-empty-report.json` | Incomplete/failed as intended; report preserved | Negative control: no boat measurements or fit/holdout data supplied |

The successful container/live image is
`sha256:47992da717d334db98169353ba9ef354160e256150761baa566fa49d89783b27`.
The live run ID is `b46ba34ab21b4a74a21d5b7edc65eb88`; its autonomy manifest
records `state_source: estimate`. The stream checker explicitly reports
`estimator_accuracy_validated: false`. No GPU performance, race completion,
marine fidelity, numerical convergence campaign or real-boat calibration is
established by these results. Later code changes require their own matching
build and execution evidence.
