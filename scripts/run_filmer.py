#!/usr/bin/env python3
"""Film a race from a chase camera, for the animation at the top of the README.

Presentation only. Entry point of the ``filmer`` service in compose.film.yaml
(runs inside the container), normally started by ``./scripts/njord film
[course]``. It waits for the simulator's run_ready.json, spawns a static
camera model into the world and moves it about 30 times per second (steady
wall time) so it stays behind and above the vessel, smoothed in simulation time.

The camera is not part of the vessel: it is spawned into the world, publishes
on Gazebo transport only (no ROS bridge), and autonomy never sees it. It
follows the vessel's ground-truth pose, which is fine for a film and is never
fed back into the race. Moving it adds a rendered sensor, so the race may run
slower than real time; simulation time stays exact.

Inputs (environment): OUTPUT_DIR (default /outputs), RUN_ID (required),
RUNNER_GIT_COMMIT and IMAGE_ID (provenance).

Writes to OUTPUT_DIR/film/: frame_NNNNNN.jpg (one per camera image),
frames.jsonl (image time, vessel position and the time of that position, per
frame) and film.json (camera settings and provenance). scripts/make_film_gif.py
turns them into the GIF. Runs until stopped; frames are written as they arrive.
Fails with FileExistsError if OUTPUT_DIR/film already exists.
"""
import json
import math
import os
from pathlib import Path
import queue
import threading
import time

WORLD = 'njord_course'   # world name in scenario.world_xml
CAMERA = 'film_camera'
IMAGE_TOPIC = '/njord/film/image'
WIDTH, HEIGHT = 960, 540  # pixels
RATE_HZ = 10.0            # camera frames per second of simulation time
FOV_RAD = 1.1             # horizontal field of view
OFFSET_M = (-15.0, -12.0, 9.0)  # camera position relative to the smoothed vessel, ENU
LEAD_M = (10.0, 3.0, 0.0)       # look-at point relative to the smoothed vessel, ENU
SMOOTHING_S = 1.5         # time constant of the camera's follow filter (simulation time)

CAMERA_SDF = f"""<sdf version='1.9'>
<model name='{CAMERA}'>
  <static>true</static>
  <link name='link'>
    <sensor name='film' type='camera'>
      <always_on>true</always_on>
      <update_rate>{RATE_HZ}</update_rate>
      <topic>{IMAGE_TOPIC}</topic>
      <camera>
        <horizontal_fov>{FOV_RAD}</horizontal_fov>
        <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
        <clip><near>0.5</near><far>600</far></clip>
      </camera>
    </sensor>
  </link>
</model>
</sdf>"""


def look_at(eye, target):
    """Quaternion (w, x, y, z) turning a camera's +x axis from ``eye`` to ``target``.

    Gazebo cameras look along +x with +z up; no roll, so the horizon stays level.
    """
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))  # positive pitch looks down
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)


def camera_pose(center):
    """Camera position and orientation for a smoothed vessel position ``center`` (x, y)."""
    base = (center[0], center[1], 0.0)
    eye = tuple(b + o for b, o in zip(base, OFFSET_M))
    return eye, look_at(eye, tuple(b + l for b, l in zip(base, LEAD_M)))


class Follow:
    """First-order low-pass on the vessel position, in simulation time."""

    def __init__(self, time_constant):
        self.time_constant = time_constant
        self.position = None
        self.stamp = None

    def update(self, position, stamp):
        """Blend in ``position`` measured at ``stamp``; repeated stamps change nothing."""
        if self.position is None:
            self.position, self.stamp = tuple(position), stamp
        elif stamp > self.stamp:
            alpha = 1.0 - math.exp(-(stamp - self.stamp) / self.time_constant)
            self.position = tuple(p + alpha * (q - p) for p, q in zip(self.position, position))
            self.stamp = stamp
        return self.position


def main():
    # Gazebo and OpenCV bindings exist only in the image; keep the helpers
    # above importable by the host tests.
    import cv2
    import numpy as np
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node
    from njord_sim.constants import GROUND_TRUTH_TOPIC, GZ_MODEL_NAME
    from njord_sim.run_manifest import wait_ready

    output = Path(os.environ.get('OUTPUT_DIR', '/outputs'))
    metadata = wait_ready(output, os.environ.get('RUN_ID', ''))
    film = output / 'film'
    film.mkdir()  # FileExistsError: never mix frames of two runs
    (film / 'film.json').write_text(json.dumps({
        'run_id': metadata['run_id'],
        'runner_git_commit': os.environ.get('RUNNER_GIT_COMMIT', 'unknown'),
        'image_identity': os.environ.get('IMAGE_ID', 'unknown'),
        'width': WIDTH, 'height': HEIGHT, 'rate_hz': RATE_HZ, 'fov_rad': FOV_RAD,
        'offset_m': OFFSET_M, 'lead_m': LEAD_M, 'smoothing_s': SMOOTHING_S,
    }, indent=2) + '\n')

    node = Node()
    lock = threading.Lock()
    state = {'vessel': None, 'stamp': None, 'frames': 0}

    def on_odometry(msg):
        with lock:
            state['vessel'] = (msg.pose.position.x, msg.pose.position.y)
            state['stamp'] = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    # Callbacks only queue the image; a separate thread encodes and writes, so
    # transport never backs up and drops frames.
    pending = queue.Queue()

    def on_image(msg):
        with lock:
            vessel, pose_stamp, index = state['vessel'], state['stamp'], state['frames']
            state['frames'] += 1
        pending.put((index, msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9, vessel, pose_stamp,
                     msg.width, msg.height, msg.data))

    def writer():
        with (film / 'frames.jsonl').open('a') as log:
            while True:
                index, stamp, vessel, pose_stamp, width, height, data = pending.get()
                rgb = np.frombuffer(data, np.uint8).reshape(height, width, 3)
                name = f'frame_{index:06d}.jpg'
                cv2.imwrite(str(film / name), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                log.write(json.dumps({'file': name, 'sim_time_s': stamp, 'vessel_xy': vessel,
                                      'vessel_time_s': pose_stamp}) + '\n')
                log.flush()

    threading.Thread(target=writer, daemon=True).start()
    node.subscribe(Odometry, GROUND_TRUTH_TOPIC, on_odometry)
    deadline = time.monotonic() + 60.0
    while state['vessel'] is None:
        if time.monotonic() > deadline:
            raise TimeoutError(f'No vessel pose on {GROUND_TRUTH_TOPIC}')
        time.sleep(0.1)

    follow = Follow(SMOOTHING_S)
    with lock:
        center = follow.update(state['vessel'], state['stamp'])
    eye, quat = camera_pose(center)
    request = EntityFactory()
    request.sdf = CAMERA_SDF
    request.pose.position.x, request.pose.position.y, request.pose.position.z = eye
    request.pose.orientation.w, request.pose.orientation.x, request.pose.orientation.y, \
        request.pose.orientation.z = quat
    ok, reply = node.request(f'/world/{WORLD}/create', request, EntityFactory, Boolean, 5000)
    if not (ok and reply.data):
        raise RuntimeError(f'Could not spawn {CAMERA} into /world/{WORLD}')
    node.subscribe(Image, IMAGE_TOPIC, on_image)
    print(f'Filming {GZ_MODEL_NAME} into {film}', flush=True)

    # Move the camera in steady wall time; the filter itself runs on sim time.
    while True:
        with lock:
            center = follow.update(state['vessel'], state['stamp'])
        eye, quat = camera_pose(center)
        pose = Pose()
        pose.name = CAMERA
        pose.position.x, pose.position.y, pose.position.z = eye
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = quat
        node.request(f'/world/{WORLD}/set_pose', pose, Pose, Boolean, 1000)
        time.sleep(0.03)


if __name__ == '__main__':
    main()
