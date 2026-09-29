#!/usr/bin/env python3
"""Optional MCAP recording, bound to the simulator's current run manifest.

Use the Compose record overlay. SIGINT is delivered directly to rosbag2 via
exec so it can flush metadata. Bags are never overwritten or resumed implicitly.

Entry point of the ``recorder`` service in compose.record.yaml (runs inside
the container), e.g.
``COMPOSE_FILE=compose.yaml:compose.record.yaml ./scripts/njord demo recorder``
(naming the service starts it despite its ``record`` profile). The ``simulator`` service must be running
with the same OUTPUT_DIR and RUN_ID; the recorder waits up to 120 s of steady
wall time for its run_ready.json.

Inputs (environment): OUTPUT_DIR (default /outputs), RUN_ID (required), and
IMAGE_ID, NJORD_IMAGE_SOURCE_COMMIT, NJORD_IMAGE_SOURCE_DIGEST,
RUNNER_GIT_COMMIT, PROFILE, SEED and ENVIRONMENT, which are stored as
provenance in the bag's custom data and in recording.json.

Writes to OUTPUT_DIR: bag/ (MCAP, zstd, split at 2 GiB), recorder_qos.yaml
(QoS overrides) and recording.json (topics, command and provenance). The
per-thruster command topics come from the run's public_parameters.json, so
they always match the vessel file of the run.
Fails with FileExistsError if OUTPUT_DIR/bag already exists.

The bag includes ground truth (/sim/ground_truth/odometry) for offline
evaluation only; keep it out of autonomy inputs when replaying.
"""
import json
import os
from pathlib import Path

from njord_sim.constants import (CAMERAS, GPS_RAW_TOPIC, GPS_TOPIC, GROUND_TRUTH_TOPIC, IMU_RAW_TOPIC, IMU_TOPIC,
                                 LIDAR_POINTS_TOPIC, LIDAR_SCAN_TOPIC, camera_topic)
from njord_sim.run_manifest import wait_ready


# Recorded topics: clock/TF, raw and processed sensors, the reference
# autonomy's outputs, thrust commands and evaluation-only truth/contacts.
# The per-thruster command topics are added from the run (see thruster_topics).
TOPICS = (
    '/clock', '/tf', '/tf_static', '/robot_description', '/joint_states',
    *(camera_topic(camera, name) for camera in CAMERAS for name in ('image_raw', 'camera_info')),
    LIDAR_POINTS_TOPIC, LIDAR_SCAN_TOPIC, GPS_RAW_TOPIC, GPS_TOPIC, IMU_RAW_TOPIC, IMU_TOPIC,
    '/njord/local/odometry', '/njord/gps/odometry', '/njord/odometry',
    '/njord/occupancy', '/njord/buoys', '/njord/goal', '/njord/path',
    '/njord/mission_status', '/njord/planner_status', '/njord/navigation_status',
    '/njord/plan_ms', '/njord/race_active',
    '/njord/actuator_forces', GROUND_TRUTH_TOPIC, '/njord/contacts',
)


def thruster_topics(output):
    """Command topics of the run's thrusters, from public_parameters.json."""
    public = json.loads((Path(output)/'public_parameters.json').read_text())
    return list(public['command_guard']['thruster_topics'])


def prepare(output, run_id, environment=None):
    """Resolve the handoff and save reviewable recording settings before exec.

    Args:
        output: run output directory shared with the simulator.
        run_id: identifier the simulator's run_ready.json must carry.
        environment: mapping to read provenance from (default os.environ).

    Returns the ``ros2 bag record`` command as an argument list. Side effects:
    writes recorder_qos.yaml and recording.json into ``output``.
    """
    environment = os.environ if environment is None else environment
    metadata = wait_ready(output, run_id)
    output = Path(output)
    topics = (*TOPICS, *thruster_topics(output))
    bag = output/'bag'
    if bag.exists():
        raise FileExistsError(f'Refusing to overwrite existing rosbag: {bag}')
    # Keep robot description and static transforms available for late discovery.
    qos = {topic: {'history': 'keep_last', 'depth': depth, 'reliability': 'reliable',
                   'durability': 'transient_local'}
           for topic, depth in (('/tf_static', 100), ('/robot_description', 1),
                                ('/njord/race_active', 1))}
    qos_file = output/'recorder_qos.yaml'
    qos_file.write_text(json.dumps(qos, indent=2)+'\n')  # JSON is valid YAML.
    provenance = {
        'run_id': metadata['run_id'],
        'manifest_sha256': metadata.get('manifest_sha256', 'legacy-unavailable'),
        'image_identity': environment.get('IMAGE_ID', 'unknown'),
        'image_source_commit': environment.get('NJORD_IMAGE_SOURCE_COMMIT', 'unknown'),
        'image_source_digest': environment.get('NJORD_IMAGE_SOURCE_DIGEST', 'unknown'),
        'runner_git_commit': environment.get('RUNNER_GIT_COMMIT', 'unknown'),
        'profile': environment.get('PROFILE', 'fast'),
        'state_source': environment.get('STATE_SOURCE', 'estimate'),
        'seed': environment.get('SEED', '1'),
        'environment': environment.get('ENVIRONMENT', 'calm'),
    }
    # --use-sim-time stamps the bag with /clock. Cache 64 MiB, split bag files
    # at 2 GiB, poll for new topics every 100 ms.
    args = [
        'ros2', 'bag', 'record', '--output', str(bag), '--storage', 'mcap',
        '--storage-preset-profile', 'zstd_fast', '--use-sim-time',
        '--disable-keyboard-controls', '--node-name', 'njord_recorder',
        '--polling-interval', '100', '--max-cache-size', str(64*1024*1024),
        '--max-bag-size', str(2*1024*1024*1024),
        '--qos-profile-overrides-path', str(qos_file),
        '--custom-data', *(f'{key}={value}' for key, value in provenance.items()),
        '--topics', *topics,
    ]
    (output/'recording.json').write_text(json.dumps({
        **provenance, 'topics': topics, 'storage': 'mcap', 'use_sim_time': True,
        'command': args, 'qos_overrides': qos,
    }, indent=2)+'\n')
    return args


def main():
    """Prepare the recording and exec rosbag2 (this process is replaced)."""
    args = prepare(os.environ.get('OUTPUT_DIR', '/outputs'), os.environ.get('RUN_ID', ''))
    os.execvp(args[0], args)


if __name__ == '__main__':
    main()
