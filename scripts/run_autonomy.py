#!/usr/bin/env python3
"""Start autonomy after the simulator publishes sanitized mission metadata.

Entry point of the Compose ``autonomy`` service (runs inside the container).
The ``simulator`` service must be running with the same OUTPUT_DIR and RUN_ID:
this script waits (up to 120 s of steady wall time, see
njord_sim.run_manifest.wait_ready) for its ``run_ready.json``, verifies the
run manifest checksums, then replaces itself with
``ros2 launch njord_sim dstar_demo.launch.py`` so signals reach the launch.

Inputs (environment): OUTPUT_DIR (default /outputs), RUN_ID (required),
AUTONOMY / CONTROLLER / PERCEPTION / MAPPING ("reference" or "external"),
STATE_SOURCE ("estimate" or "truth"), PROFILE ("fast" or "conservative"), SEED, ROS_PARAMS_FILE, ENVIRONMENT and the
provenance variables IMAGE_ID, NJORD_IMAGE_SOURCE_COMMIT,
NJORD_IMAGE_SOURCE_DIGEST and RUNNER_GIT_COMMIT. Extra command-line arguments
are passed to the launch; ``name:=value`` for one of the settings above
overrides the environment.

Reads from OUTPUT_DIR: run_ready.json, vessel_config.yaml and (if present)
public_parameters.json. Autonomy never receives scenario geometry; only the
number of gates and the manifest digest.

Writes to OUTPUT_DIR: autonomy_config.json (settings, digests and the exact
launch command) and ros_params.yaml (a snapshot of ROS_PARAMS_FILE, if set).

Exit: a traceback on an invalid setting, missing file or startup timeout;
otherwise the exit code of the launched autonomy.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
from njord_sim.run_manifest import atomic_text, wait_ready


def prepare(output, run_id, environment=None, args=()):
    """Snapshot team settings after the simulation's public startup handoff.

    Args:
        output: run output directory shared with the simulator.
        run_id: identifier the simulator's run_ready.json must carry.
        environment: mapping to read settings from (default os.environ).
        args: extra launch arguments from the command line.

    Returns the ``ros2 launch`` command as an argument list. Side effects:
    writes autonomy_config.json and, with ROS_PARAMS_FILE, ros_params.yaml.
    Raises ValueError for an unknown mode/profile or a non-integer seed.
    """
    environment = os.environ if environment is None else environment
    metadata = wait_ready(output, run_id)
    output = Path(output).resolve()
    vessel = output / 'vessel_config.yaml'
    # Never fall back to image defaults after Gazebo resolved another experiment.
    vessel_bytes = vessel.read_bytes()
    settings = {name: environment.get(name.upper(), 'reference')
                for name in ('autonomy', 'controller', 'perception', 'mapping')}
    settings.update(state_source=environment.get('STATE_SOURCE', 'estimate'),
                    profile=environment.get('PROFILE', 'fast'),
                    seed=str(metadata.get('seed', environment.get('SEED') or '1')),
                    params_file=environment.get('ROS_PARAMS_FILE', ''))
    extra_args = list(args)
    public_file = output / 'public_parameters.json'
    # Command-line name:=value arguments override the environment settings.
    for argument in extra_args:
        name, separator, value = argument.partition(':=')
        if separator and name in settings:
            settings[name] = value
    # The simulator's sensor seed wins so the recorded seed is the one used.
    if public_file.is_file():
        public = json.loads(public_file.read_text())
        settings['seed'] = str(public.get('sensor_adapter', {}).get('seed', settings['seed']))
        if '_profile' in public:
            if any(argument.startswith('profile:=') and argument.partition(':=')[2] != public['_profile'] for argument in extra_args):
                raise ValueError('profile override conflicts with resolved simulation profile')
            settings['profile'] = public['_profile']
    for name in ('autonomy', 'controller', 'perception', 'mapping'):
        if settings[name] not in ('reference', 'external'):
            raise ValueError(f'{name} must be reference or external')
    if settings['state_source'] not in ('estimate', 'truth'):
        raise ValueError('state_source must be estimate or truth')
    if settings['profile'] not in ('fast', 'conservative'):
        raise ValueError('profile must be fast or conservative')
    if 'seed' in metadata and any(argument.startswith('seed:=') and argument.partition(':=')[2] != str(metadata['seed']) for argument in extra_args):
        raise ValueError('seed override conflicts with resolved simulation seed')
    settings['seed'] = str(metadata.get('seed', settings['seed']))
    seed = int(settings['seed'])
    params_digest = None
    # Simulator freezes every YAML input before starting nodes. Never reopen
    # a caller's mutable ROS file after the handoff.
    frozen_params = output / 'source_config' / 'ros_params.yaml'
    if settings['params_file'] and not frozen_params.is_file():
        raise ValueError('Simulator did not freeze ROS_PARAMS_FILE')
    settings['params_file'] = str(frozen_params) if frozen_params.is_file() else ''
    if settings['params_file']:
        params_digest = hashlib.sha256(frozen_params.read_bytes()).hexdigest()
    localization = output / 'source_config' / 'localization.yaml'
    command = ['ros2', 'launch', 'njord_sim', 'dstar_demo.launch.py',
               *[f'{name}:={value}' for name, value in settings.items() if name != 'params_file'],
               *[argument for argument in extra_args if argument.partition(':=')[0] not in settings],
               *(['localization_config:=' + str(localization)] if localization.is_file() else []),
               'expected_gates:=' + str(metadata['expected_gates']),
               *(['public_parameters:=' + str(public_file)] if public_file.is_file() else []),
               *(['params_file:=' + settings['params_file']] if settings['params_file'] else []),
               'vessel_config:=' + str(vessel)]
    # Everything needed to reproduce or audit this autonomy run.
    provenance = {
        **settings, 'seed': seed, 'run_id': metadata['run_id'],
        'manifest_sha256': metadata.get('manifest_sha256'),
        'expected_gates': metadata['expected_gates'],
        'environment': environment.get('ENVIRONMENT', 'calm'),
        'image_identity': environment.get('IMAGE_ID', 'unknown'),
        'image_source_commit': environment.get('NJORD_IMAGE_SOURCE_COMMIT', 'unknown'),
        'image_source_digest': environment.get('NJORD_IMAGE_SOURCE_DIGEST', 'unknown'),
        'image_executable_digest': environment.get('NJORD_IMAGE_EXECUTABLE_DIGEST', 'unknown'),
        'runner_git_commit': environment.get('RUNNER_GIT_COMMIT', 'unknown'),
        'vessel_config': str(vessel),
        'vessel_config_sha256': hashlib.sha256(vessel_bytes).hexdigest(),
        'params_sha256': params_digest,
        'extra_launch_args': extra_args, 'command': command,
    }
    atomic_text(output / 'autonomy_config.json', json.dumps(provenance, indent=2, allow_nan=False) + '\n')
    return command


def main():
    """Prepare the run and exec the launch (this process is replaced)."""
    args = prepare(os.environ.get('OUTPUT_DIR', '/outputs'), os.environ.get('RUN_ID', ''),
                   args=sys.argv[1:])
    frozen = Path(os.environ.get('OUTPUT_DIR', '/outputs')) / 'frozen_config'
    if not frozen.is_dir():
        raise ValueError('Simulator did not freeze default configuration')
    os.environ['NJORD_CONFIG_DIR'] = str(frozen)
    os.execvp(args[0], args)


if __name__ == '__main__':
    main()
