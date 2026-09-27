#!/usr/bin/env python3
"""Fresh-process 4/2/1 ms dynamics campaign through scripts/njord.

Run after building the image. Each repetition gets isolated Compose/ROS/Gazebo
identities and fresh output. Does not modify or stop an existing simulator stack.

Where to run: on the host, from a checkout, with Docker and the built image.
Nothing needs to be running. Example (see docs/njord-calibration.md):

    python3 scripts/dynamics_campaign.py --output outputs/dynamics-campaign --repetitions 3

For every experiment, each time step (4, 2, 1 ms) and each repetition, a
trial writes a copy of the scenario with that physics step, starts
``validation/check_dynamics.py`` through ``scripts/njord test`` and then a
fresh simulator through ``scripts/njord simulator``, waits for the report and
tears both down. The results are then checked with
``validation/numerical_acceptance.compare``.

Isolation: each trial has its own Compose project and GZ_PARTITION
(``dynamics-<random>``). All trials share ``--ros-domain`` (default 137),
which is not leased like the benchmark's domains: choose one no other running
simulator uses, or its topics will contaminate the measurements.

Outputs under --output (must not exist yet):
    <experiment>-<step>-<repetition>/   scenario.yaml, simulator.log, check.log,
                                        dynamics.json and the run artifacts, or
                                        failure.json for an incomplete trial
    <experiment>-runs.json              trial summaries, rewritten per trial
    <experiment>-acceptance.json        numerical_acceptance.compare result
    campaign-incomplete.json            only when --fail-fast stopped early

Exit code: 0 if every experiment passes acceptance, 2 otherwise.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import math
import shutil
import subprocess
import sys
import time
import uuid

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'validation'))
from numerical_acceptance import compare


def stop(process):
    """Stop a started njord process group: SIGTERM, then SIGKILL after 30 s.

    The whole process group is signalled (it was started with
    start_new_session=True), so scripts/njord and its docker compose child
    both receive it; ``njord simulator`` then runs ``docker compose down``.
    """
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def trial(args, scenario, experiment, step, repetition, output):
    """Run one fresh-process measurement and return its summary dict.

    Args:
        args: parsed command-line options.
        scenario: loaded dynamics scenario (copied, never modified).
        experiment: check_dynamics experiment name.
        step: physics step in seconds (0.004, 0.002 or 0.001).
        repetition: repetition index, recorded in the report.
        output: new trial directory, mounted as /outputs.

    The summary holds experiment, step_s, repetition, comparison_group,
    provenance (the run manifest SHA-256), complete, metrics and output. Any
    failure is written to failure.json and returned as an incomplete trial
    (comparison_group "incomplete") so it stays in the evidence.
    """
    output.mkdir(parents=True, exist_ok=False)
    if getattr(args, 'trial_vessel', None) is not None:
        (output / 'vessel.yaml').write_text(yaml.safe_dump(args.trial_vessel))
    for name, source in getattr(args, 'resource_files', {}).items():
        destination = output / 'resources' / name
        destination.parent.mkdir(exist_ok=True)
        shutil.copyfile(source, destination)
    scenario = copy.deepcopy(scenario)
    algorithms = copy.deepcopy(args.algorithms)
    limit = min(min(t['forward_limit_n'], t['reverse_limit_n']) for t in args.trial_vessel['thrusters'])
    original_limit = algorithms['guidance']['max_thrust']
    algorithms['guidance']['max_thrust'] = min(original_limit, limit)
    (output/'algorithms.yaml').write_text(yaml.safe_dump(algorithms))
    (output/'dependent-settings.json').write_text(json.dumps({'guidance.max_thrust': {
        'original': original_limit, 'effective': algorithms['guidance']['max_thrust'],
        'reason': 'simulator-only physical limits; autonomy is not launched'}}, indent=2)+'\n')
    if experiment.startswith('oscillator_'):
        shape = args.trial_vessel['geometry']['buoyancy']
        length, beam, height = shape['size_m']
        environment = scenario['environments'][args.environment]
        draft = args.trial_vessel['mass_kg']/(environment['water_density_kg_m3']*length*beam)
        scenario['start'][2:] = [environment['water_level_m']-shape['pose'][2]+height/2-draft, 0., 0., 0.]
        axis = experiment.removeprefix('oscillator_')
        if axis == 'heave':
            scenario['start'][2] += min(.005, draft/10, (height-draft)/10)
        else:
            scenario['start'][3 if axis == 'roll' else 4] = min(.005, draft/(10*max(length,beam)), (height-draft)/(10*max(length,beam)))
    if experiment == 'sensor_probe':
        distance = .9*args.trial_vessel['sensors']['settings']['lidar_range']
        scenario['obstacles'] = [{'name':'range_target', 'position':[distance, 0.], 'radius_m':2.}]
    if experiment == 'hydrostatic':
        # Start with 0.05 rad roll and pitch so the restoring motion is excited.
        scenario['start'][3:5] = [0.05, 0.05]
    for environment in scenario['environments'].values():
        environment['physics_step_s'] = step
    (output / 'scenario.yaml').write_text(yaml.safe_dump(scenario))
    # A random token isolates this trial's Compose project and Gazebo partition.
    token = uuid.uuid4().hex[:12]
    env = dict(os.environ, OUTPUT_HOST=str(output), SCENARIO='/outputs/scenario.yaml',
               COMPOSE_PROJECT_NAME='dynamics-' + token, GZ_PARTITION='dynamics-' + token,
               ROS_DOMAIN_ID=str(args.ros_domain), ENVIRONMENT=args.environment, SEED=str(args.seed))
    if getattr(args, 'trial_vessel', None) is not None:
        env['VESSEL_CONFIG'] = '/outputs/vessel.yaml'
        env['ALGORITHMS_CONFIG'] = '/outputs/algorithms.yaml'
    command = [str(ROOT/'scripts/njord'), 'runtime-test', 'python3', '/opt/njord/validation/check_dynamics.py',
               '--ros-args', '-p', 'use_sim_time:=true', '-p', 'experiment:=' + experiment,
               '-p', 'manifest_path:=/outputs/run_manifest.json', '-p', 'output_path:=/outputs/dynamics.json',
               '-p', f'repetition:={repetition}', '-p', f'thrust_n:={args.thrust}',
               '-p', f'duration_s:={args.duration}']
    if getattr(args, 'forces_n', None) is not None:
        command.extend(['-p', 'forces_n:=' + json.dumps(args.forces_n),
                        '-p', 'decay_s:=' + str(max(2.0, 8*max(t['response_time_s'] for t in args.trial_vessel['thrusters'])))])
    sensor = experiment == 'sensor_probe'
    if sensor:
        parameter = getattr(args, 'parameter_name', '')
        mode = 'noise' if 'noise' in parameter else 'range' if parameter.endswith('lidar_range') else 'rates'
        env.update(AUTONOMY='external', CONTROLLER='external', PERCEPTION='external', MAPPING='external')
        command = [str(ROOT/'scripts/njord'), 'runtime-test', 'python3', '/opt/njord/validation/sensor_parameter_probe.py',
                   '--output-dir', '/outputs', '--mode', mode, '--samples', '10000' if mode == 'noise' else '100',
                   '--timeout-s', str(max(args.timeout, 1800) if mode == 'noise' else args.timeout)]
        if mode == 'range':
            command += ['--minimum-observed-range-m', str(.85*args.trial_vessel['sensors']['settings']['lidar_range'])]
    sim = check = None
    try:
        with (output/'check.log').open('w') as check_log, (output/'simulator.log').open('w') as sim_log:
            if sensor:
                sim = subprocess.Popen([str(ROOT/'scripts/njord'), 'lab'], cwd=ROOT, env=env,
                                       stdout=sim_log, stderr=subprocess.STDOUT, start_new_session=True)
                ready_deadline = time.monotonic()+120
                while not (output/'run_ready.json').is_file():
                    if sim.poll() is not None or time.monotonic()>ready_deadline:
                        raise RuntimeError('sensor simulator startup failed')
                    time.sleep(.1)
                check = subprocess.Popen(command, cwd=ROOT, env=env, stdout=check_log,
                                         stderr=subprocess.STDOUT, start_new_session=True)
            else:
                # Subscribe before Gazebo starts so initial acquisition is retained.
                check = subprocess.Popen(command, cwd=ROOT, env=env, stdout=check_log,
                                         stderr=subprocess.STDOUT, start_new_session=True)
                time.sleep(2)
                if check.poll() is not None:
                    raise RuntimeError('validator exited before simulator startup; see check.log')
                sim = subprocess.Popen([str(ROOT/'scripts/njord'), 'simulator'], cwd=ROOT,
                                       env=env, stdout=sim_log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            # Infrastructure watchdog on steady wall time (--timeout, seconds).
            deadline = time.monotonic() + (max(args.timeout, 1800) if sensor and mode == "noise" else args.timeout)
            while check.poll() is None:
                if sim.poll() is not None:
                    raise RuntimeError('simulator exited before measurement completed')
                if time.monotonic() > deadline:
                    raise TimeoutError('trial wall-time limit exceeded')
                time.sleep(0.2)
        if check.returncode != 0:
            raise RuntimeError(f'validator exited {check.returncode}; retained report cannot validate failed process')
        if sensor:
            report = json.loads((output/'sensor_parameter_metrics.json').read_text())
            return dict(experiment=experiment, step_s=step, repetition=repetition,
                        comparison_group='sensor', provenance=report.get('run_manifest_sha256'),
                        complete=report.get('complete') is True and report.get('pass') is True,
                        physical_acceptance=report, metrics=report.get('metrics', {}), output=str(output))
        report = json.loads((output/'dynamics.json').read_text())
        physical = None
        hull = None
        if getattr(args, 'hull_axis', None):
            from hull_acceptance import compare as compare_hull
            hull = compare_hull(report, args.trial_vessel, scenario['environments'][args.environment], args.hull_axis)
            (output/'hull-acceptance.json').write_text(json.dumps(hull, indent=2)+'\n')
        elif experiment.startswith('oscillator_'):
            from hull_acceptance import oscillator
            hull = oscillator(report, args.trial_vessel, scenario['environments'][args.environment], experiment.removeprefix('oscillator_'))
            (output/'hull-acceptance.json').write_text(json.dumps(hull, indent=2)+'\n')
        elif experiment == 'hydrostatic':
            from hull_acceptance import hydrostatic_equilibrium
            hull = hydrostatic_equilibrium(report, args.trial_vessel, scenario['environments'][args.environment])
            (output/'hull-acceptance.json').write_text(json.dumps(hull, indent=2)+'\n')
        if experiment == 'excitation':
            physical = actuator_acceptance(report, args.trial_vessel, args.forces_n)
            (output/'actuator-acceptance.json').write_text(json.dumps(physical, indent=2)+'\n')
        resolved = json.loads((output/'resolved_configuration.json').read_text())
        actual_step = resolved['scenario']['environment']['physics_step_s']
        if actual_step != step:
            raise ValueError('resolved timestep differs from requested timestep')
        group = comparison_group(resolved, report['manifest']['content']['image_identity'])
        return dict(experiment=experiment, step_s=actual_step, repetition=repetition,
                    comparison_group=group, provenance=report['manifest']['sha256'],
                    complete=report['complete'] and (physical is None or physical['passed']) and (hull is None or hull['passed']),
                    physical_acceptance=physical, hull_acceptance=hull, metrics=report['metrics'], output=str(output))
    except (OSError, ValueError, RuntimeError, TimeoutError, KeyError) as exc:
        (output/'failure.json').write_text(json.dumps({'complete': False, 'reason': str(exc)})+'\n')
        return dict(experiment=experiment, step_s=step, repetition=repetition,
                    comparison_group='incomplete', provenance=str(output), complete=False, metrics={})
    finally:
        stop(check)
        stop(sim)


def comparison_group(resolved, image_identity):
    """Match physical inputs across timesteps, omitting only derived step fields."""
    normalized = copy.deepcopy(resolved)
    normalized['scenario']['environment'].pop('physics_step_s', None)
    normalized.pop('sensor_max_period_s', None)
    for value in normalized['scenario']['environments'].values():
        value.pop('physics_step_s', None)
    # Scenario source hash necessarily changes with timestep; normalized
    # scenario above covers its remaining contents. Keep other resource hashes.
    normalized.get("resources", {}).pop("/outputs/scenario.yaml", None)
    # Frozen scenario bytes differ only in the deliberately varied timestep.
    normalized.get('resources', {}).pop('/outputs/source_config/scenario.yaml', None)
    # Runs are only comparable when they used the same image.
    normalized["image_identity"] = image_identity
    return hashlib.sha256(json.dumps(normalized, sort_keys=True).encode()).hexdigest()


def actuator_acceptance(report, vessel, requested=None):
    """Check actual timestamped force lag and wrench using independent oracles.

    Missing telemetry is a failure. This establishes actuator implementation,
    not hull hydrodynamic fidelity or calibrated boat response.
    """
    from physical_acceptance import acceptance, response, wrench
    samples = report.get('applied_samples', [])
    wrenches = report.get('wrench_samples', [])
    motors = vessel['thrusters']
    checks = []
    transient = {i: {'buildup': 0, 'decay': 0} for i, motor in enumerate(motors)
                 if motor['response_time_s'] > 0 and requested is not None and abs(requested[i]) > 1e-6}
    if any(len(sample.get('forces_n', [])) != len(motors) or len(sample.get('targets_n', [])) != len(motors)
           or not all(math.isfinite(v) for v in sample['forces_n']+sample['targets_n']) for sample in samples):
        return {'passed': False, 'reason': 'invalid applied force vector'}
    if any(len(sample.get('force_n', [])) != 3 or len(sample.get('moment_nm', [])) != 3
           or not all(math.isfinite(v) for v in sample['force_n']+sample['moment_nm']) for sample in wrenches):
        return {'passed': False, 'reason': 'invalid wrench vector'}
    for a, b in zip(samples, samples[1:]):
        dt = b['time_s']-a['time_s']
        if not 0 < dt <= .1:
            continue
        if a['targets_n'] != b['targets_n']:
            continue
        for index, motor in enumerate(motors):
            expected = response(b['targets_n'][index], a['forces_n'][index], dt, motor['response_time_s'])
            checks.append(acceptance(b['forces_n'][index], expected, 'N'))
            if index in transient and abs(a['forces_n'][index]-b['targets_n'][index]) > .1:
                phase = 'decay' if abs(b['targets_n'][index]) < 1e-6 else 'buildup'
                transient[index][phase] += 1
    wrench_checks = []
    by_stamp = {round(sample['time_s'], 8): sample for sample in samples}
    for sample in wrenches:
        applied = by_stamp.get(round(sample['time_s'], 8))
        if applied is None:
            continue
        expected = wrench(motors, applied['forces_n'], vessel['center_of_mass_m'])
        for key, unit in (('force_n', 'N'), ('moment_nm', 'N*m')):
            wrench_checks.extend(acceptance(actual, reference, unit)
                                 for actual, reference in zip(sample[key], expected[key]))
    saturation = []
    if requested is not None:
        expected_targets = [max(-motor['reverse_limit_n'], min(motor['forward_limit_n'], force))
                            for motor, force in zip(motors, requested)]
        for sample in samples:
            if any(abs(value) > 1e-6 for value in sample['targets_n']):
                saturation.extend(acceptance(value, expected, 'N')
                                  for value, expected in zip(sample['targets_n'], expected_targets))
    transient_complete = all(min(counts.values()) >= 2 for counts in transient.values())
    return {'passed': transient_complete and bool(saturation) and all(x['passed'] for x in saturation) and bool(checks) and bool(wrench_checks) and all(x['passed'] for x in checks+wrench_checks),
            'transient_pairs': transient, 'transient_complete': transient_complete,
            'lag_samples': len(checks), 'wrench_components': len(wrench_checks),
            'saturation_components': len(saturation),
            'failed_checks': [x for x in checks+wrench_checks+saturation if not x['passed']],
            'missing': [name for name, values in [('lag', checks), ('wrench', wrench_checks), ('saturation', saturation)] if not values],
            'evidence_level': 'running_actuator_implementation_only'}


def set_parameter(document, path, value):
    """Change exactly one existing scalar leaf; never silently add a setting."""
    keys = path.split('.')
    target = document
    for key in keys[:-1]:
        target = target[int(key)] if isinstance(target, list) else target[key]
    key = int(keys[-1]) if isinstance(target, list) else keys[-1]
    previous = target[key]
    if isinstance(previous, (dict, list)) or isinstance(value, (dict, list)):
        raise ValueError('campaign parameter must identify one scalar leaf')
    target[key] = value
    return previous


def physical_variants(vessel, scenario, parameter, values, environment):
    """Baseline plus one-leaf changes, suitable for independent paired trials.

    Paths start vessel. or environment.; sequence indices use dot notation.
    The caller must evaluate measured response, not infer physical validity
    solely from numerical convergence.
    """
    if not parameter or not values:
        raise ValueError('physical campaign requires a parameter and values')
    scope, separator, path = parameter.partition('.')
    if not separator or scope not in ('vessel', 'environment'):
        raise ValueError('parameter starts vessel. or environment.')
    result = [('baseline', copy.deepcopy(vessel), copy.deepcopy(scenario), None)]
    for index, value in enumerate(values):
        variant_vessel, variant_scenario = copy.deepcopy(vessel), copy.deepcopy(scenario)
        target = variant_vessel if scope == 'vessel' else variant_scenario['environments'][environment]
        before = set_parameter(target, path, value)
        if value == before:
            raise ValueError('parameter value must differ from baseline')
        result.append((f'variant-{index}', variant_vessel, variant_scenario,
                       {'parameter': parameter, 'before': before, 'after': value}))
    return result


def excitation_vectors(vessel, thrust):
    """Individual motors and signed body surge/sway/yaw commands.

    Minimum norm allocation is solved independently from the runtime allocator.
    Commands are uniformly scaled to avoid exceeding any configured limit.
    """
    import numpy as np
    motors = vessel['thrusters']
    center = vessel['center_of_mass_m']
    columns = []
    for motor in motors:
        angle = math.radians(motor['yaw_deg'])
        x, y = math.cos(angle), math.sin(angle)
        arm = [a-b for a,b in zip(motor['position_m'], center)]
        columns.append([x, y, arm[0]*y-arm[1]*x])
    matrix = np.array(columns).T
    if np.linalg.matrix_rank(matrix) < 3:
        raise ValueError('signed surge/sway/yaw matrix requires a rank-three thruster layout')
    result = {}
    for i in range(len(motors)):
        for sign in (-1, 1):
            result[f'thruster-{i}-{sign:+d}'] = [sign*thrust if j == i else 0.0 for j in range(len(motors))]
    for axis, name in enumerate(('surge', 'sway', 'yaw')):
        for sign in (-1, 1):
            wrench = np.zeros(3)
            wrench[axis] = sign*thrust
            values = np.linalg.lstsq(matrix, wrench, rcond=None)[0]
            scale = min([1.0] + [motor['forward_limit_n' if force > 0 else 'reverse_limit_n']/abs(force)
                                for motor, force in zip(motors, values) if abs(force) > 1e-12])
            result[f'{name}-{sign:+d}'] = (values*scale).tolist()
    return result


def suite_parameters(vessel, scenario, environment):
    """One physical scalar per pair, .75 and 1.25 times a nonzero baseline.

    Zero fields cannot be multiplicatively excited and are listed separately.
    Each generated variant still goes through the normal physical validator.
    """
    paths = ['vessel.mass_kg']
    paths += ['vessel.inertia_kg_m2.' + name for name in ('ixx', 'iyy', 'izz')]
    paths += [f'vessel.hydrodynamics.{name}.{i}' for name in ('added_mass', 'linear_damping', 'quadratic_damping') for i in range(6)]
    paths += [f'vessel.thrusters.{i}.{name}' for i in range(len(vessel['thrusters']))
              for name in ('yaw_deg', 'forward_limit_n', 'reverse_limit_n', 'response_time_s')]
    paths += [f'vessel.thrusters.{i}.position_m.{j}' for i in range(len(vessel['thrusters'])) for j in range(3)]
    paths += ['environment.' + name for name in ('water_density_kg_m3', 'wind_speed_mps', 'current_speed_mps')]
    paths += ['vessel.sensors.settings.' + name for name in ('lidar_range', 'lidar_rate', 'camera_rate', 'gps_rate', 'imu_rate',
              'gps_horizontal_noise_m', 'gps_vertical_noise_m', 'imu_orientation_noise_rad', 'imu_angular_velocity_noise_rad_s', 'imu_linear_acceleration_noise_m_s2', 'lidar_noise_stddev')]
    result, zero = [], []
    for path in paths:
        scope, *keys = path.split('.')
        value = vessel if scope == 'vessel' else scenario['environments'][environment]
        for key in keys:
            value = value[int(key)] if isinstance(value, list) else value[key]
        if value == 0 and path in ('environment.wind_speed_mps', 'environment.current_speed_mps'):
            result.append((path, [.75, 1.25]))  # explicit SI excitation from still-water baseline
        elif value == 0:
            zero.append(path)
        else:
            result.append((path, [value*.75, value*1.25]))
    return result, zero


def main():
    """Run the trial matrix and exit 0 when all experiments pass acceptance."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', type=Path, default=ROOT/'scenarios/dynamics.yaml')
    parser.add_argument('--vessel', type=Path, default=ROOT/'njord_sim/config/vessels/munin_v0.yaml')
    parser.add_argument('--parameter', help='one scalar path: vessel.mass_kg, vessel.thrusters.0.yaw_deg, environment.current_speed_mps')
    parser.add_argument('--smoke', action='store_true', help='bounded forward/reverse actuator integration smoke, one repetition at4ms; no numerical convergence claim')
    parser.add_argument('--suite', action='store_true', help='all nonzero physical scalar pairs .75/1.25 plus signed four-thruster excitation')
    parser.add_argument('--controlled-fixture', action='store_true', help='explicit analytic zero-COM, planar thruster fixture for independent scalar momentum oracle')
    parser.add_argument('--excitation', action='store_true', help='individual thrusters and signed surge/sway/yaw; requires rank-three layout')
    parser.add_argument('--values', nargs='+', type=json.loads, help='JSON scalar values; baseline is always included')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--experiments', nargs='+', choices=['straight', 'reverse', 'turn_left', 'turn_right', 'coast', 'drift', 'hydrostatic', 'oscillator_heave', 'oscillator_roll', 'oscillator_pitch'],
                        default=['hydrostatic', 'straight', 'reverse', 'turn_left', 'turn_right', 'coast'])
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--environment', default='calm')
    parser.add_argument('--ros-domain', type=int, default=137)
    parser.add_argument('--thrust', type=float, default=300.0)  # N per thruster
    parser.add_argument('--duration', type=float, default=60.0)  # s, simulation time
    parser.add_argument('--timeout', type=float, default=600.0)  # s, wall time per trial
    parser.add_argument('--fail-fast', action='store_true',
                        help='stop after the first incomplete trial; retain its report and logs')
    args = parser.parse_args()
    if (args.repetitions < 3 and not args.smoke) or not 0 <= args.ros_domain <= 232:
        parser.error('at least three repetitions and a ROS domain in [0,232] required')
    from benchmark import pin_image, default_compose_files
    os.environ.setdefault('COMPOSE_FILE', default_compose_files(os.environ))
    pin_image(os.environ)
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    scenario = yaml.safe_load(args.scenario.read_text())
    if args.seed is None:
        args.seed = int(os.environ.get('SEED') or scenario['seed'])
    args.algorithms = yaml.safe_load((Path(os.environ.get('CONFIG_HOST', ROOT/'njord_sim/config'))/'algorithms.yaml').read_text())
    if args.smoke:
        args.repetitions = 1
        args.duration = 30.0
    vessel = yaml.safe_load(args.vessel.read_text())
    if args.controlled_fixture:
        vessel['center_of_mass_m'] = [0., 0., 0.]
        for motor in vessel['thrusters']:
            motor['position_m'][2] = 0.
        for name in ('ixy', 'ixz', 'iyz'):
            vessel['inertia_kg_m2'][name] = 0.
        (args.output/'controlled-fixture.yaml').write_text(yaml.safe_dump(vessel))
        for environment in scenario['environments'].values():
            environment['wind_direction_to_deg_enu'] = 0.
            environment['current_direction_to_deg_enu'] = 0.
        (args.output/'controlled-scenario.yaml').write_text(yaml.safe_dump(scenario))
    # Resolve mesh URIs before moving the vessel YAML into each run directory.
    args.resource_files = {}
    def absolute_resources(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key == 'uri' and isinstance(value, str) and '://' not in value and not Path(value).is_absolute():
                    resource = (args.vessel.parent / value).resolve()
                    name = hashlib.sha256(resource.read_bytes()).hexdigest()[:16] + '-' + resource.name
                    args.resource_files[name] = resource
                    node[key] = '/outputs/resources/' + name
                else:
                    absolute_resources(value)
        elif isinstance(node, list):
            for value in node:
                absolute_resources(value)
    absolute_resources(vessel)
    if bool(args.parameter) != bool(args.values):
        parser.error('--parameter and --values must be supplied together')
    variants = physical_variants(vessel, scenario, args.parameter, args.values, args.environment) if args.parameter else [('', vessel, scenario, None)]
    if args.suite:
        variants = [('baseline', copy.deepcopy(vessel), copy.deepcopy(scenario), None)]
        parameters, zero = suite_parameters(vessel, scenario, args.environment)
        for parameter, values in parameters:
            for name, v, s, change in physical_variants(vessel, scenario, parameter, values, args.environment)[1:]:
                variants.append((parameter + '-' + name, v, s, change))
        (args.output/'suite-coverage.json').write_text(json.dumps({
            'zero_fields_requiring_explicit_values': zero,
            'physical_acceptance': 'requires independent measured oracles; numerical convergence alone does not pass',
            'sensor_variants': 'require separate rendered sensor measurement; dynamics reports do not establish sensor fidelity'}, indent=2)+'\n')
    results = []
    for name, args.trial_vessel, variant_scenario, change in variants:
        variant_output = args.output / name
        variant_output.mkdir(exist_ok=True)
        (variant_output / 'parameter-change.json').write_text(json.dumps(change, indent=2) + '\n')
        scenario = variant_scenario
        commands = {name: ('excitation', vector) for name, vector in excitation_vectors(args.trial_vessel, args.thrust).items()} if args.excitation or args.suite else {}
        commands.update({name: (name, None) for name in args.experiments})
        if args.smoke:
            commands = {f'smoke-{sign:+d}': ('excitation', [sign*args.thrust]*len(args.trial_vessel['thrusters'])) for sign in (-1,1)}
        args.parameter_name = change['parameter'] if change else ''
        if args.suite and change:
            parameter = change['parameter']
            if parameter.startswith('vessel.sensors.'):
                commands = {'sensor_probe': ('sensor_probe', None)}
            elif parameter.endswith(('mass_kg', 'water_density_kg_m3')):
                commands = {'hydrostatic': ('hydrostatic', None), 'surge-+1': commands['surge-+1']}
            elif '.thrusters.' in parameter:
                index = int(parameter.split('.')[2])
                commands = {name: value for name, value in commands.items() if name.startswith(f'thruster-{index}-')}
            elif '.inertia_kg_m2.' in parameter or '.hydrodynamics.' in parameter:
                index = int(parameter.split('.')[-1]) if '.hydrodynamics.' in parameter else {'ixx':3,'iyy':4,'izz':5}[parameter.split('.')[-1]]
                axis = {0:'surge',1:'sway',5:'yaw'}.get(index)
                if axis is None:
                    oscillator = {2:'heave',3:'roll',4:'pitch'}[index]
                    commands = {'oscillator_'+oscillator: ('oscillator_'+oscillator, None)}
                else:
                    commands = {name: value for name,value in commands.items() if name.startswith(axis+'-')}
            else:
                commands = {'drift': ('drift', None)}
        for measurement, (experiment, args.forces_n) in commands.items():
            args.hull_axis = measurement.split('-')[0] if args.controlled_fixture and measurement.split('-')[0] in ('surge','sway','yaw') else 'surge' if args.controlled_fixture and measurement == 'drift' else None
            runs = []
            steps = (0.004,) if args.smoke or (args.suite and change) else (0.004, 0.002, 0.001)
            for step in steps:
                for repetition in range(args.repetitions):
                    output = variant_output/f'{measurement}-{step:g}-{repetition}'
                    print(f'Running {output.name}', flush=True)
                    runs.append(trial(args, scenario, experiment, step, repetition, output))
                    (variant_output/f'{measurement}-runs.json').write_text(json.dumps(runs, indent=2)+'\n')
                    if args.fail_fast and not runs[-1]['complete']:
                        (args.output/'campaign-incomplete.json').write_text(json.dumps({
                            'passed': False, 'reason': 'fail-fast: incomplete trial',
                            'trial': str(output)}, indent=2)+'\n')
                        raise SystemExit(2)
            # Metrics compared between 2 ms and 1 ms for this experiment, with units.
            metrics = {'coast_distance_m': 'm'} if experiment == 'coast' else {
                'steady_speed_mps': 'm/s', 'steady_yaw_rate_radps': 'rad/s'}
            if experiment == 'hydrostatic':
                metrics = {'mean_z_m': 'm', 'mean_roll_rad': 'rad', 'mean_pitch_rad': 'rad'}
            if experiment.startswith('oscillator_'):
                metrics = {'observed_distance_m':'m', 'peak_speed_mps':'m/s'}
            if experiment == 'excitation':
                metrics = {'observed_initial_surge_acceleration_mps2': 'm/s^2', 'steady_speed_mps': 'm/s', 'steady_yaw_rate_radps': 'rad/s'}
            if experiment.startswith('turn'):
                metrics['turning_radius_m'] = 'm'
            result = compare(runs, metrics) if len(steps) == 3 else {
                'passed': len(runs) >= (1 if args.smoke else 3) and all(run['complete'] and (run.get('physical_acceptance') or run.get('hull_acceptance')) for run in runs),
                'evidence_level': 'actuator_integration_smoke' if args.smoke else 'parameter_variant_physical_oracle',
                'runs': runs}
            results.append(result)
            (variant_output/f'{measurement}-acceptance.json').write_text(json.dumps(result, indent=2)+'\n')
    missing = []
    if args.suite and not args.controlled_fixture:
        missing = ['suite requires --controlled-fixture for independent hull acceptance']
    summary = {'numerical_passed': None if args.smoke else bool(results) and all(r['passed'] for r in results),
               'evidence_level': 'actuator_integration_smoke' if args.smoke else 'controlled_simulator_parameter_checks',
               'missing_physical_checks': missing,
               'passed': bool(results) and all(r['passed'] for r in results) and not missing}
    (args.output/'campaign-acceptance.json').write_text(json.dumps(summary, indent=2)+'\n')
    raise SystemExit(0 if summary['passed'] else 2)


if __name__ == '__main__':
    main()
