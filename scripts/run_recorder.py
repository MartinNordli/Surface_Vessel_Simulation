#!/usr/bin/env python3
"""Optional MCAP recording, bound to the simulator's current run manifest.

Use the Compose record overlay. SIGINT is delivered directly to rosbag2 via
exec so it can flush metadata. Bags are never overwritten or resumed implicitly.
"""
import json
import os
from pathlib import Path

from njord_sim.run_manifest import wait_ready


CAMERAS = ('front_left_camera_sensor', 'front_right_camera_sensor')
TOPICS = (
    '/clock', '/tf', '/tf_static', '/robot_description', '/wamv/joint_states',
    *(f'/wamv/sensors/cameras/{camera}/{suffix}'
      for camera in CAMERAS for suffix in ('image_raw', 'camera_info')),
    '/wamv/sensors/lidars/lidar_wamv_sensor/points',
    '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
    '/wamv/sensors/gps/gps/fix_raw', '/wamv/sensors/gps/gps/fix',
    '/wamv/sensors/imu/imu/data_raw', '/wamv/sensors/imu/imu/data',
    '/njord/local/odometry', '/njord/gps/odometry', '/njord/odometry',
    '/njord/occupancy', '/njord/buoys', '/njord/goal', '/njord/path',
    '/njord/mission_status', '/njord/planner_status', '/njord/navigation_status',
    '/njord/plan_ms', '/njord/race_active',
    '/njord/thrusters/left/thrust', '/njord/thrusters/right/thrust',
    '/njord/actuator_forces', '/wamv/ground_truth/odometry', '/njord/contacts',
)


def prepare(output, run_id, environment=None):
    """Resolve the handoff and save reviewable recording settings before exec."""
    environment = os.environ if environment is None else environment
    metadata = wait_ready(output, run_id)
    output = Path(output)
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
        'image_identity': environment.get('IMAGE_ID', 'unknown'),
        'image_source_commit': environment.get('NJORD_IMAGE_SOURCE_COMMIT', 'unknown'),
        'image_source_digest': environment.get('NJORD_IMAGE_SOURCE_DIGEST', 'unknown'),
        'runner_git_commit': environment.get('RUNNER_GIT_COMMIT', 'unknown'),
        'profile': environment.get('PROFILE', 'fast'),
        'seed': environment.get('SEED', '1'),
        'environment': environment.get('ENVIRONMENT', 'calm'),
    }
    args = [
        'ros2', 'bag', 'record', '--output', str(bag), '--storage', 'mcap',
        '--storage-preset-profile', 'zstd_fast', '--use-sim-time',
        '--disable-keyboard-controls', '--node-name', 'njord_recorder',
        '--polling-interval', '100', '--max-cache-size', str(64*1024*1024),
        '--max-bag-size', str(2*1024*1024*1024),
        '--qos-profile-overrides-path', str(qos_file),
        '--custom-data', *(f'{key}={value}' for key, value in provenance.items()),
        '--topics', *TOPICS,
    ]
    (output/'recording.json').write_text(json.dumps({
        **provenance, 'topics': TOPICS, 'storage': 'mcap', 'use_sim_time': True,
        'command': args, 'qos_overrides': qos,
    }, indent=2)+'\n')
    return args


def main():
    args = prepare(os.environ.get('OUTPUT_DIR', '/outputs'), os.environ.get('RUN_ID', ''))
    os.execvp(args[0], args)


if __name__ == '__main__':
    main()
