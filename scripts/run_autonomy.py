#!/usr/bin/env python3
"""Start autonomy after the simulator publishes sanitized mission metadata."""
import hashlib
import json
import os
from pathlib import Path
import sys
from njord_sim.run_manifest import atomic_text, wait_ready


def prepare(output, run_id, environment=None, args=()):
    """Snapshot team settings after the simulation's public startup handoff."""
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
    for argument in extra_args:
        name, separator, value = argument.partition(':=')
        if separator and name in settings:
            settings[name] = value
    for name in ('autonomy', 'controller', 'perception', 'mapping'):
        if settings[name] not in ('reference', 'external'):
            raise ValueError(f'{name} must be reference or external')
    if settings['profile'] not in ('fast', 'conservative'):
        raise ValueError('profile must be fast or conservative')
    seed = int(settings['seed'])
    params_digest = None
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
               *(['params_file:=' + settings['params_file']] if settings['params_file'] else []),
               'vessel_config:=' + str(vessel)]
    provenance = {
        **settings, 'seed': seed, 'run_id': metadata['run_id'],
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
    args = prepare(os.environ.get('OUTPUT_DIR', '/outputs'), os.environ.get('RUN_ID', ''),
                   args=sys.argv[1:])
    os.execvp(args[0], args)


if __name__ == '__main__':
    main()
