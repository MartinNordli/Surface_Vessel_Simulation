#!/usr/bin/env python3
"""Fresh-process 4/2/1 ms dynamics campaign through scripts/njord.

Run after building the image. Each repetition gets isolated Compose/ROS/Gazebo
identities and fresh output. Does not modify or stop an existing simulator stack.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'validation'))
from numerical_acceptance import compare


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def trial(args, scenario, experiment, step, repetition, output):
    output.mkdir(parents=True, exist_ok=False)
    scenario = copy.deepcopy(scenario)
    if experiment == 'hydrostatic':
        scenario['start'][3:5] = [0.05, 0.05]
    for environment in scenario['environments'].values():
        environment['physics_step_s'] = step
    (output / 'scenario.yaml').write_text(yaml.safe_dump(scenario))
    token = uuid.uuid4().hex[:12]
    env = dict(os.environ, OUTPUT_HOST=str(output), SCENARIO='/outputs/scenario.yaml',
               COMPOSE_PROJECT_NAME='dynamics-' + token, GZ_PARTITION='dynamics-' + token,
               ROS_DOMAIN_ID=str(args.ros_domain), ENVIRONMENT=args.environment, SEED=str(args.seed))
    command = [str(ROOT/'scripts/njord'), 'test', 'python3', '/opt/njord/validation/check_dynamics.py',
               '--ros-args', '-p', 'use_sim_time:=true', '-p', 'experiment:=' + experiment,
               '-p', 'manifest_path:=/outputs/run_manifest.json', '-p', 'output_path:=/outputs/dynamics.json',
               '-p', f'repetition:={repetition}', '-p', f'thrust_n:={args.thrust}',
               '-p', f'duration_s:={args.duration}']
    sim = check = None
    try:
        with (output/'check.log').open('w') as check_log, (output/'simulator.log').open('w') as sim_log:
            # Start the subscriber first so first observed clock time is fresh.
            check = subprocess.Popen(command, cwd=ROOT, env=env, stdout=check_log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(2)
            if check.poll() is not None:
                raise RuntimeError('validator exited before simulator startup; see check.log')
            sim = subprocess.Popen([str(ROOT/'scripts/njord'), 'simulator'], cwd=ROOT,
                                   env=env, stdout=sim_log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
            deadline = time.monotonic() + args.timeout
            while check.poll() is None:
                if sim.poll() is not None:
                    raise RuntimeError('simulator exited before measurement completed')
                if time.monotonic() > deadline:
                    raise TimeoutError('trial wall-time limit exceeded')
                time.sleep(0.2)
        report = json.loads((output/'dynamics.json').read_text())
        resolved = json.loads((output/'resolved_configuration.json').read_text())
        actual_step = resolved['scenario']['environment']['physics_step_s']
        if actual_step != step:
            raise ValueError('resolved timestep differs from requested timestep')
        # Compare configuration hashes with timestep removed from every profile.
        normalized = copy.deepcopy(resolved)
        normalized['scenario']['environment'].pop('physics_step_s', None)
        for value in normalized['scenario']['environments'].values():
            value.pop('physics_step_s', None)
        # Scenario source hash necessarily changes with timestep; normalized
        # scenario above covers its remaining contents. Keep other resource hashes.
        normalized.get("resources", {}).pop("/outputs/scenario.yaml", None)
        normalized["image_identity"] = report["manifest"]["content"]["image_identity"]
        group = hashlib.sha256(json.dumps(normalized, sort_keys=True).encode()).hexdigest()
        return dict(experiment=experiment, step_s=actual_step, repetition=repetition,
                    comparison_group=group, provenance=report['manifest']['sha256'],
                    complete=report['complete'], metrics=report['metrics'], output=str(output))
    except (OSError, ValueError, RuntimeError, TimeoutError, KeyError) as exc:
        (output/'failure.json').write_text(json.dumps({'complete': False, 'reason': str(exc)})+'\n')
        return dict(experiment=experiment, step_s=step, repetition=repetition,
                    comparison_group='incomplete', provenance=str(output), complete=False, metrics={})
    finally:
        stop(check)
        stop(sim)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', type=Path, default=ROOT/'scenarios/dynamics.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--experiments', nargs='+', choices=['straight', 'reverse', 'turn_left', 'turn_right', 'coast', 'drift', 'hydrostatic'],
                        default=['hydrostatic', 'straight', 'reverse', 'turn_left', 'turn_right', 'coast'])
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--environment', default='calm')
    parser.add_argument('--ros-domain', type=int, default=137)
    parser.add_argument('--thrust', type=float, default=300.0)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--fail-fast', action='store_true',
                        help='stop after the first incomplete trial; retain its report and logs')
    args = parser.parse_args()
    if args.repetitions < 3 or not 0 <= args.ros_domain <= 232:
        parser.error('at least three repetitions and a ROS domain in [0,232] required')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    scenario = yaml.safe_load(args.scenario.read_text())
    results = []
    for experiment in args.experiments:
        runs = []
        for step in (0.004, 0.002, 0.001):
            for repetition in range(args.repetitions):
                output = args.output/f'{experiment}-{step:g}-{repetition}'
                print(f'Running {output.name}', flush=True)
                runs.append(trial(args, scenario, experiment, step, repetition, output))
                (args.output/f'{experiment}-runs.json').write_text(json.dumps(runs, indent=2)+'\n')
                if args.fail_fast and not runs[-1]['complete']:
                    (args.output/'campaign-incomplete.json').write_text(json.dumps({
                        'passed': False, 'reason': 'fail-fast: incomplete trial',
                        'trial': str(output)}, indent=2)+'\n')
                    raise SystemExit(2)
        metrics = {'coast_distance_m': 'm'} if experiment == 'coast' else {
            'steady_speed_mps': 'm/s', 'steady_yaw_rate_radps': 'rad/s'}
        if experiment == 'hydrostatic':
            metrics = {'mean_z_m': 'm'}
        if experiment.startswith('turn'):
            metrics['turning_radius_m'] = 'm'
        result = compare(runs, metrics)
        results.append(result)
        (args.output/f'{experiment}-acceptance.json').write_text(json.dumps(result, indent=2)+'\n')
    raise SystemExit(0 if all(r['passed'] for r in results) else 2)


if __name__ == '__main__':
    main()
