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
PROFILE ("fast" or "conservative"), SEED, ROS_PARAMS_FILE, ENVIRONMENT and the
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
    settings.update(profile=environment.get('PROFILE', 'fast'),
                    seed=environment.get('SEED', '1'),
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
    for name in ('autonomy', 'controller', 'perception', 'mapping'):
        if settings[name] not in ('reference', 'external'):
            raise ValueError(f'{name} must be reference or external')
    if settings['profile'] not in ('fast', 'conservative'):
        raise ValueError('profile must be fast or conservative')
    seed = int(settings['seed'])
    params_digest = None
    # Copy the team parameter file into the run so later edits to the original
    # cannot change what this run used; the launch reads the copy.
    if settings['params_file']:
        params_bytes = Path(settings['params_file']).read_bytes()
        snapshot = output / 'ros_params.yaml'
        temporary = snapshot.with_suffix('.yaml.tmp')
        temporary.write_bytes(params_bytes)
        temporary.replace(snapshot)
        settings['params_file'] = str(snapshot)
        params_digest = hashlib.sha256(params_bytes).hexdigest()
    command = ['ros2', 'launch', 'njord_sim', 'dstar_demo.launch.py',
               *[f'{name}:={value}' for name, value in settings.items() if name != 'params_file'],
               *extra_args,
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
    os.execvp(args[0], args)


if __name__ == '__main__':
    main()
